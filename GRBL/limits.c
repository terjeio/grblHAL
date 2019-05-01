/*
  limits.c - code pertaining to limit-switches and performing the homing cycle
  Part of Grbl

  Copyright (c) 2017-2019 Terje Io
  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "grbl.h"

// Homing axis search distance multiplier. Computed by this value times the cycle travel.
#ifndef HOMING_AXIS_SEARCH_SCALAR
  #define HOMING_AXIS_SEARCH_SCALAR 1.5f // Must be > 1 to ensure limit switch will be engaged.
#endif
#ifndef HOMING_AXIS_LOCATE_SCALAR
  #define HOMING_AXIS_LOCATE_SCALAR 5.0f // Must be > 1 to ensure limit switch is cleared.
#endif

// This is the Limit Pin Change Interrupt, which handles the hard limit feature. A bouncing
// limit switch can cause a lot of problems, like false readings and multiple interrupt calls.
// If a switch is triggered at all, something bad has happened and treat it as such, regardless
// if a limit switch is being disengaged. It's impossible to reliably tell the state of a
// bouncing pin because the microcontroller does not retain any state information when
// detecting a pin change. If we poll the pins in the ISR, you can miss the correct reading if the
// switch is bouncing.
// NOTE: Do not attach an e-stop to the limit pins, because this interrupt is disabled during
// homing cycles and will not respond correctly. Upon user request or need, there may be a
// special pinout for an e-stop, but it is generally recommended to just directly connect
// your e-stop switch to the microcontroller reset pin, since it is the most correct way to do this.

ISR_CODE void limit_interrupt_handler (axes_signals_t state) // DEFAULT: Limit pin change interrupt process.
{
    // Ignore limit switches if already in an alarm state or in-process of executing an alarm.
    // When in the alarm state, Grbl should have been reset or will force a reset, so any pending
    // moves in the planner and stream input buffers are all cleared and newly sent blocks will be
    // locked out until a homing cycle or a kill lock command. Allows the user to disable the hard
    // limit setting if their limits are constantly triggering after a reset and move their axes.

    if (!(sys.state & (STATE_ALARM|STATE_ESTOP)) && !sys_rt_exec_alarm) {

      #ifdef HARD_LIMIT_FORCE_STATE_CHECK
        // Check limit pin state.
        if (state.value) {
            mc_reset(); // Initiate system kill.
            system_set_exec_alarm(Alarm_HardLimit); // Indicate hard limit critical event
        }
      #else
        mc_reset(); // Initiate system kill.
        system_set_exec_alarm(Alarm_HardLimit); // Indicate hard limit critical event
      #endif
    }
}


// Set machine positions for homed limit switches. Don't update non-homed axes.
// NOTE: settings.max_travel[] is stored as a negative value.
static void limits_set_machine_positions (uint8_t cycle_mask)
{
    uint_fast8_t idx = N_AXIS;

    if(settings.flags.homing_force_set_origin) {
        do {
            if (cycle_mask & bit(--idx))
                sys_position[idx] = 0;
        } while(idx);
    } else do {
        if (cycle_mask & bit(--idx))
            sys_position[idx] = bit_istrue(settings.homing.dir_mask.value, bit(idx))
                                 ? lroundf((settings.max_travel[idx] + settings.homing.pulloff) * settings.steps_per_mm[idx])
                                 : lroundf(-settings.homing.pulloff * settings.steps_per_mm[idx]);

    } while(idx);
}

// Homes the specified cycle axes, sets the machine position, and performs a pull-off motion after
// completing. Homing is a special motion case, which involves rapid uncontrolled stops to locate
// the trigger point of the limit switches. The rapid stops are handled by a system level axis lock
// mask, which prevents the stepper algorithm from executing step pulses. Homing motions typically
// circumvent the processes for executing motions in normal operation.
// NOTE: Only the abort realtime command can interrupt this process.
void limits_go_home (uint8_t cycle_mask)
{
    if (sys.abort)
        return;// Block if system reset has been issued.

    // Initialize plan data struct for homing motion. Spindle and coolant are disabled.
    plan_line_data_t plan_data;
    plan_line_data_t *pl_data = &plan_data;
    // Initialize variables used for homing computations.
    uint_fast8_t n_cycle = (2 * settings.homing.locate_cycles + 1);
    uint_fast8_t step_pin[N_AXIS];
    float target[N_AXIS];
    float max_travel = 0.0f;
    bool approach = true;
    float homing_rate = settings.homing.seek_rate;
    uint_fast8_t limit_state, axislock, n_active_axis;

    memset(pl_data,0,sizeof(plan_line_data_t));
    pl_data->condition.system_motion = On;
    pl_data->condition.no_feed_override = On;
    pl_data->line_number = HOMING_CYCLE_LINE_NUMBER;

    uint_fast8_t idx = N_AXIS;
    do {
        idx--;
        // Initialize step pin masks
#ifdef HAL_KINEMATICS
        step_pin[idx] = hal.kinematics.limits_get_axis_mask(idx);
#else
        step_pin[idx] = bit(idx);
#endif
        // Set target based on max_travel setting. Ensure homing switches engaged with search scalar.
        // NOTE: settings.max_travel[] is stored as a negative value.
        if (bit_istrue(cycle_mask, bit(idx)))
            max_travel = max(max_travel,(-HOMING_AXIS_SEARCH_SCALAR) * settings.max_travel[idx]);
    } while(idx);

    // Set search mode with approach at seek rate to quickly engage the specified cycle_mask limit switches.
    do {

        // Initialize and declare variables needed for homing routine.
        system_convert_array_steps_to_mpos(target, sys_position);
        axislock = 0;
        n_active_axis = 0;

        idx = N_AXIS;
        do {
            // Set target location for active axes and setup computation for homing rate.
            if (bit_istrue(cycle_mask, bit(--idx))) {
                n_active_axis++;

#ifdef HAL_KINEMATICS
                hal.kinematics.limits_set_target_pos(idx);
#else
                sys_position[idx] = 0;
#endif
                // Set target direction based on cycle mask and homing cycle approach state.
                // NOTE: This happens to compile smaller than any other implementation tried.
                if (bit_istrue(settings.homing.dir_mask.value, bit(idx)))
                    target[idx] = approach ? - max_travel : max_travel;
                else
                    target[idx] = approach ? max_travel : - max_travel;

                // Apply axislock to the step port pins active in this cycle.
                axislock |= step_pin[idx];
            }
        } while(idx);

        homing_rate *= sqrtf(n_active_axis); // [sqrt(N_AXIS)] Adjust so individual axes all move at homing rate.
        sys.homing_axis_lock.mask = axislock;

        // Perform homing cycle. Planner buffer should be empty, as required to initiate the homing cycle.
        pl_data->feed_rate = homing_rate; // Set current homing rate.
        plan_buffer_line(target, pl_data); // Bypass mc_line(). Directly plan homing motion.

        sys.step_control.flags = 0;
        sys.step_control.execute_sys_motion = On; // Set to execute homing motion and clear existing flags.
        st_prep_buffer(); // Prep and fill segment buffer from newly planned block.
        st_wake_up(); // Initiate motion

        do {

            if (approach) {
                // Check limit state. Lock out cycle axes when they change.
                limit_state = hal.limits_get_state().value;

                idx = N_AXIS;
                do {
                    idx--;
                    if ((axislock & step_pin[idx]) && (limit_state & bit(idx))) {
#ifdef HAL_KINEMATICS
                        axislock &= ~hal.kinematics.limits_get_axis_mask(idx);
#else
                        axislock &= ~bit(idx);
#endif
                    }
                } while(idx);

                sys.homing_axis_lock.mask = axislock;
            }

            st_prep_buffer(); // Check and prep segment buffer. NOTE: Should take no longer than 200us.

            // Exit routines: No time to run protocol_execute_realtime() in this loop.
            if (sys_rt_exec_state & (EXEC_SAFETY_DOOR | EXEC_RESET | EXEC_CYCLE_COMPLETE)) {

                uint8_t rt_exec = sys_rt_exec_state;

                // Homing failure condition: Reset issued during cycle.
                if (rt_exec & EXEC_RESET)
                    system_set_exec_alarm(Alarm_HomingFailReset);

                // Homing failure condition: Safety door was opened.
                if (rt_exec & EXEC_SAFETY_DOOR)
                    system_set_exec_alarm(Alarm_HomingFailDoor);

                // Homing failure condition: Limit switch still engaged after pull-off motion
                if (!approach && (hal.limits_get_state().value & cycle_mask))
                    system_set_exec_alarm(Alarm_FailPulloff);

                // Homing failure condition: Limit switch not found during approach.
                if (approach && (rt_exec & EXEC_CYCLE_COMPLETE))
                    system_set_exec_alarm(Alarm_HomingFailApproach);

                if (sys_rt_exec_alarm) {
                    mc_reset(); // Stop motors, if they are running.
                    protocol_execute_realtime();
                    return;
                } else {
                    // Pull-off motion complete. Disable CYCLE_STOP from executing.
                    system_clear_exec_state_flag(EXEC_CYCLE_COMPLETE);
                    break;
                }
            }

        } while (AXES_BITMASK & axislock);

        st_reset(); // Immediately force kill steppers and reset step segment buffer.
        hal.delay_ms(settings.homing.debounce_delay, 0); // Delay to allow transient dynamics to dissipate.

        // Reverse direction and reset homing rate for locate cycle(s).
        approach = !approach;

        // After first cycle, homing enters locating phase. Shorten search to pull-off distance.
        if (approach) {
            max_travel = settings.homing.pulloff * HOMING_AXIS_LOCATE_SCALAR;
            homing_rate = settings.homing.feed_rate;
        } else {
            max_travel = settings.homing.pulloff;
            homing_rate = settings.homing.seek_rate;
        }

    } while (n_cycle-- > 0);

    // The active cycle axes should now be homed and machine limits have been located. By
    // default, Grbl defines machine space as all negative, as do most CNCs. Since limit switches
    // can be on either side of an axes, check and set axes machine zero appropriately. Also,
    // set up pull-off maneuver from axes limit switches that have been homed. This provides
    // some initial clearance off the switches and should also help prevent them from falsely
    // triggering when hard limits are enabled or when more than one axes shares a limit pin.

    limits_set_machine_positions(cycle_mask);

#ifdef ENABLE_BACKLASH_COMPENSATION
    mc_backlash_init();
#endif
    sys.step_control.flags = 0; // Return step control to normal operation.
    sys.flags.is_homed = On;
}

// Performs a soft limit check. Called from mc_line() only. Assumes the machine has been homed,
// the workspace volume is in all negative space, and the system is in normal operation.
// NOTE: Used by jogging to limit travel within soft-limit volume.
void limits_soft_check  (float *target)
{
    if (system_check_travel_limits(target)) {
        sys.flags.soft_limit = On;
        // Force feed hold if cycle is active. All buffered blocks are guaranteed to be within
        // workspace volume so just come to a controlled stop so position is not lost. When complete
        // enter alarm mode.
        if (sys.state == STATE_CYCLE) {
            system_set_exec_state_flag(EXEC_FEED_HOLD);
            do {
                if(!protocol_execute_realtime())
                    return; // aborted!
            } while (sys.state != STATE_IDLE);
        }
        mc_reset(); // Issue system reset and ensure spindle and coolant are shutdown.
        system_set_exec_alarm(Alarm_SoftLimit); // Indicate soft limit critical event
        protocol_execute_realtime(); // Execute to enter critical event loop and system abort
    }
}
