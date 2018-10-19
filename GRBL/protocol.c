/*
  protocol.c - controls Grbl execution protocol and procedures
  Part of Grbl

  Copyright (c) 2017-2018 Terje Io
  Copyright (c) 2011-2016 Sungeun K. Jeon for Gnea Research LLC
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

// Define line flags. Includes comment type tracking and line overflow detection.
typedef union {
    uint8_t value;
    struct {
        uint8_t overflow            :1,
                comment_parentheses :1,
                comment_semicolon   :1,
                block_delete        :1,
                unassigned          :4;
    };
} line_flags_t;

typedef struct {
    char *message;
    uint_fast8_t idx;
    uint_fast8_t tracker;
    bool show;
} user_message_t;

static uint_fast16_t char_counter = 0;
static char line[LINE_BUFFER_SIZE]; // Line to be executed. Zero-terminated.
static char xcommand[LINE_BUFFER_SIZE];
static user_message_t user_message = {NULL, 0, 0, false};
static const char *msg = "(MSG,";
static void protocol_exec_rt_suspend();

typedef struct {
    float target[N_AXIS];
    float restore_target[N_AXIS];
    float retract_waypoint;
    plan_line_data_t plan_data;
} parking_data_t;

// add gcode to execute not originating from serial stream
bool protocol_enqueue_gcode (char *gcode)
{
    bool ok = xcommand[0] == '\0' &&
               (sys.state == STATE_IDLE || (sys.state & (STATE_JOG|STATE_TOOL_CHANGE))) &&
                 bit_isfalse(sys_rt_exec_state, EXEC_MOTION_CANCEL);

    if(ok && gc_state.file_run)
        ok = gc_state.modal.program_flow != ProgramFlow_Running || strncmp((char *)gcode, "$J=", 3);

    if(ok)
        strcpy(xcommand, gcode);

    return ok;
}

/*
  GRBL PRIMARY LOOP:
*/
bool protocol_main_loop()
{
    // Perform some machine checks to make sure everything is good to go.

    if (settings.limits.flags.hard_enabled && settings.limits.flags.check_at_init && hal.limits_get_state().value) {
        set_state(STATE_ALARM); // Ensure alarm state is active.
        report_feedback_message(Message_CheckLimits);
    }

    // Check for and report alarm state after a reset, error, or an initial power up.
    // NOTE: Sleep mode disables the stepper drivers and position can't be guaranteed.
    // Re-initialize the sleep state as an ALARM mode to ensure user homes or acknowledges.
    if (sys.state & (STATE_ALARM|STATE_ESTOP|STATE_SLEEP)) {
        report_feedback_message(sys.state == STATE_ESTOP ? Message_EStop : Message_AlarmLock);
        set_state(sys.state == STATE_ESTOP ? STATE_ESTOP : STATE_ALARM); // Ensure alarm state is set.
    } else {
        // Check if the safety door is open.
        set_state(STATE_IDLE);
        if (hal.system_control_get_state().safety_door_ajar) {
            bit_true(sys_rt_exec_state, EXEC_SAFETY_DOOR);
            protocol_execute_realtime(); // Enter safety door mode. Should return as IDLE state.
        }
        // All systems go!
        system_execute_startup(line); // Execute startup script.
    }

    // ---------------------------------------------------------------------------------
    // Primary loop! Upon a system abort, this exits back to main() to reset the system.
    // This is also where Grbl idles while waiting for something to do.
    // ---------------------------------------------------------------------------------

    int32_t c;
    line_flags_t line_flags = {0};
    bool nocaps = false;

    xcommand[0] = '\0';
    user_message.show = false;

    for (;;) {

        // Process one line of incoming serial data, as the data becomes available. Performs an
        // initial filtering by removing spaces and comments and capitalizing all letters.
        while((c = hal.serial_read()) != SERIAL_NO_DATA) {

            if(c == CMD_RESET) {

                xcommand[0] = '\0';
                nocaps = user_message.show = false;
                char_counter = line_flags.value = 0;

                if (sys.state == STATE_JOG) // Block all other states from invoking motion cancel.
                    system_set_exec_state_flag(EXEC_MOTION_CANCEL);

            } else if ((c == '\n') || (c == '\r')) { // End of line reached

                if(!protocol_execute_realtime()) // Runtime command check point.
                    return !sys.exit;            // Bail to calling function upon system abort

                line[char_counter] = '\0'; // Set string termination character.

              #ifdef REPORT_ECHO_LINE_RECEIVED
                report_echo_line_received(line);
              #endif

                // Direct and execute one line of formatted input, and report status of execution.
                if (line_flags.overflow) // Report line overflow error.
                    gc_state.last_error = Status_Overflow;
                else if ((line[0] == '\0' || char_counter == 0) && !user_message.show) // Empty or comment line. For syncing purposes.
                    gc_state.last_error = Status_OK;
                else if (line[0] == '$') // Grbl '$' system command
                    gc_state.last_error = system_execute_line(line);
                else if (sys.state & (STATE_ALARM|STATE_ESTOP|STATE_JOG)) // Everything else is gcode. Block if in alarm, eStop or jog mode.
                    gc_state.last_error = Status_SystemGClock;
                else  // Parse and execute g-code block.
                    gc_state.last_error = gc_execute_block(line, user_message.show ? user_message.message : NULL);

                report_status_message(gc_state.last_error);

                // Reset tracking data for next line.
                nocaps = user_message.show = false;
                char_counter = line_flags.value = 0;

            } else if (c <= (nocaps ? ' ' - 1 : ' ') || line_flags.value) {
                // Throw away all whitepace, control characters, comment characters and overflow characters.
                if(c >=  ' ' && line_flags.comment_parentheses) {
                    if(user_message.tracker == 5)
                        user_message.message[user_message.idx++] = c == ')' ? '\0' : c;
                    else if(user_message.tracker > 0 && CAPS(c) == msg[user_message.tracker])
                        user_message.tracker++;
                    else
                        user_message.tracker = 0;
                    if (c == ')') {
                        // End of '()' comment. Resume line.
                        line_flags.comment_parentheses = Off;
                        user_message.show = user_message.show || user_message.tracker == 5;
                    }
                }
            } else if (char_counter == 0 && c == '/') {
                line_flags.block_delete = sys.block_delete_enabled;
            } else if (char_counter == 0 && c == '$') {
               // Do not uppercase system commands here - will destroy passwords etc...
                nocaps = On;
                line[char_counter++] = c;
            } else if (c == '(') {
                // Enable comments flag and ignore all characters until ')' or EOL unless it is a message.
                // NOTE: This doesn't follow the NIST definition exactly, but is good enough for now.
                // In the future, we could simply remove the items within the comments, but retain the
                // comment control characters, so that the g-code parser can error-check it.
                if((line_flags.comment_parentheses = !line_flags.comment_semicolon)) {
                    if(hal.show_message) {
                        if(user_message.message == NULL)
                            user_message.message = malloc(LINE_BUFFER_SIZE);
                        if(user_message.message) {
                            user_message.idx = 0;
                            user_message.tracker = 1;
                        }
                    }
                }
            } else if (c == ';') {
                // NOTE: ';' comment to EOL is a LinuxCNC definition. Not NIST.
                line_flags.comment_semicolon = !line_flags.comment_parentheses;
            } else if (char_counter >= (LINE_BUFFER_SIZE - 1)) {
                // Detect line buffer overflow and set flag.
                line_flags.overflow = On;
            } else
                line[char_counter++] = nocaps ? c : CAPS(c);
        }

        // Handle extra command (internal stream)
        if(xcommand[0] != '\0') {

            if (xcommand[0] == '$') // Grbl '$' system command
                system_execute_line(xcommand);
            else if (sys.state & (STATE_ALARM|STATE_ESTOP|STATE_JOG)) // Everything else is gcode. Block if in alarm, eStop or jog state.
                report_status_message(Status_SystemGClock);
            else // Parse and execute g-code block.
                gc_execute_block(xcommand, NULL);

            xcommand[0] = '\0';
        }

        // If there are no more characters in the serial read buffer to be processed and executed,
        // this indicates that g-code streaming has either filled the planner buffer or has
        // completed. In either case, auto-cycle start, if enabled, any queued moves.
        protocol_auto_cycle_start();

        if(!protocol_execute_realtime() && sys.abort) // Runtime command check point.
            return !sys.exit;                         // Bail to main() program loop to reset system.

        sys.cancel = false;
    }
}


// Block until all buffered steps are executed or in a cycle state. Works with feed hold
// during a synchronize call, if it should happen. Also, waits for clean cycle end.
void protocol_buffer_synchronize ()
{
    // If system is queued, ensure cycle resumes if the auto start flag is present.
    protocol_auto_cycle_start();
    while (protocol_execute_realtime() && (plan_get_current_block() || sys.state == STATE_CYCLE));
}


// Auto-cycle start triggers when there is a motion ready to execute and if the main program is not
// actively parsing commands.
// NOTE: This function is called from the main loop, buffer sync, and mc_line() only and executes
// when one of these conditions exist respectively: There are no more blocks sent (i.e. streaming
// is finished, single commands), a command that needs to wait for the motions in the buffer to
// execute calls a buffer sync, or the planner buffer is full and ready to go.
void protocol_auto_cycle_start ()
{
    if (plan_get_current_block() != NULL) // Check if there are any blocks in the buffer.
        system_set_exec_state_flag(EXEC_CYCLE_START); // If so, execute them!
}


// This function is the general interface to Grbl's real-time command execution system. It is called
// from various check points in the main program, primarily where there may be a while loop waiting
// for a buffer to clear space or any point where the execution time from the last check point may
// be more than a fraction of a second. This is a way to execute realtime commands asynchronously
// (aka multitasking) with grbl's g-code parsing and planning functions. This function also serves
// as an interface for the interrupts to set the system realtime flags, where only the main program
// handles them, removing the need to define more computationally-expensive volatile variables. This
// also provides a controlled way to execute certain tasks without having two or more instances of
// the same task, such as the planner recalculating the buffer upon a feedhold or overrides.
// NOTE: The sys_rt_exec_state variable flags are set by any process, step or serial interrupts, pinouts,
// limit switches, or the main program.
// Returns false if aborted
bool protocol_execute_realtime ()
{
    if(protocol_exec_rt_system()) {

        if(hal.execute_realtime)
            hal.execute_realtime(sys.state);

        if (sys.suspend)
            protocol_exec_rt_suspend();

      #ifdef EMULATE_EEPROM
        if(sys.state == STATE_IDLE && settings_dirty.is_dirty && !gc_state.file_run)
            eeprom_emu_sync_physical();
      #endif
    }
    return !(sys.abort || sys.cancel);
}

// Dangerous? This fn may be called from stepper ISR and will block (possibly forever?) if a previous message is beeing displayed...
// Use a real queue or "double buffering" instead of spin lock?
void protocol_message (char *message)
{
    static volatile bool spin_lock = false;

    while(spin_lock);

    spin_lock = true;

    if(message) {
        if(sys.message)
            free(sys.message);
        sys.message = message;
    } else if(sys.message) {
        hal.show_message(sys.message);
        free(sys.message);
        sys.message = NULL;
    }

    spin_lock = false;
}

// Executes run-time commands, when required. This function primarily operates as Grbl's state
// machine and controls the various real-time features Grbl has to offer.
// NOTE: Do not alter this unless you know exactly what you are doing!
bool protocol_exec_rt_system ()
{
    uint_fast16_t rt_exec; // Temp variable to avoid calling volatile multiple times.

    // Display message if provided
    if(sys.message)
        protocol_message(NULL);

    if (sys_rt_exec_alarm && (rt_exec = system_clear_exec_alarm())) { // Enter only if any bit flag is true
        // System alarm. Everything has shutdown by something that has gone severely wrong. Report
        // the source of the error to the user. If critical, Grbl disables by entering an infinite
        // loop until system reset/abort.
        set_state((alarm_code_t)rt_exec == Alarm_EStop ? STATE_ESTOP : STATE_ALARM); // Set system alarm state
        report_alarm_message((alarm_code_t)rt_exec);
        // Halt everything upon a critical event flag. Currently hard and soft limits flag this.
        if ((alarm_code_t)rt_exec == Alarm_HardLimit || (alarm_code_t)rt_exec == Alarm_SoftLimit || (alarm_code_t)rt_exec == Alarm_EStop) {
            system_set_exec_alarm(rt_exec);
            report_feedback_message((alarm_code_t)rt_exec == Alarm_EStop ? Message_EStop : Message_CriticalEvent);
            system_clear_exec_state_flag(EXEC_RESET); // Disable any existing reset
            // Block everything, except reset and status reports, until user issues reset or power
            // cycles. Hard limits typically occur while unattended or not paying attention. Gives
            // the user and a GUI time to do what is needed before resetting, like killing the
            // incoming stream. The same could be said about soft limits. While the position is not
            // lost, continued streaming could cause a serious crash if by chance it gets executed.
            while (bit_isfalse(sys_rt_exec_state, EXEC_RESET));
            system_clear_exec_alarm();
        }

    }

    if (sys_rt_exec_state && (rt_exec = system_clear_exec_states())) { // Get and clear volatile sys_rt_exec_state atomically.

        // Execute system abort.
        if (rt_exec & EXEC_RESET) {
            sys.abort = !hal.system_control_get_state().e_stop;  // Only place this is set true.
            return !sys.abort; // Nothing else to do but exit.
        }

        if(rt_exec & EXEC_STOP) { // Experimental for now, must be verified. Do NOT move to interrupt context!
            sys.cancel = true;
            sys.step_control.flags = 0;
            // Kill spindle and coolant. TODO: Check Mach3 behaviour
            gc_state.modal.coolant.value = 0;
            gc_state.modal.spindle.value = 0;
            spindle_stop();
            hal.coolant_set_state(gc_state.modal.coolant);

            if(hal.driver_reset)
                hal.driver_reset();

            if(hal.serial_suspend_read && hal.serial_suspend_read(false))
                hal.serial_cancel_read_buffer(); // flush pending blocks (after M6)

            plan_reset();
            st_reset();
            gc_sync_position();
            plan_sync_position();
            gc_state.tool_change = false;
            set_state(STATE_IDLE);
        }

        // Execute and serial print status
        if (rt_exec & EXEC_STATUS_REPORT)
            report_realtime_status();

#ifdef PID_LOG
        // Execute and serial print PID log
        if (rt_exec & EXEC_PID_REPORT)
            report_pid_log();
#endif

        rt_exec &= ~(EXEC_STOP|EXEC_STATUS_REPORT|EXEC_PID_REPORT); // clear requests already processed

        // Let state machine handle any remaining requests
        if(rt_exec)
            update_state(rt_exec);

    }

    // Execute overrides.

    if((rt_exec = get_feed_ovr())) {

        uint_fast8_t new_f_override = sys.f_override, new_r_override = sys.r_override;

        do {

          switch(rt_exec) {

              case CMD_FEED_OVR_RESET:
                  new_f_override = DEFAULT_FEED_OVERRIDE;
                  break;

              case CMD_FEED_OVR_COARSE_PLUS:
                  new_f_override += FEED_OVERRIDE_COARSE_INCREMENT;
                  break;

              case CMD_FEED_OVR_COARSE_MINUS:
                  new_f_override -= FEED_OVERRIDE_COARSE_INCREMENT;
                  break;

              case CMD_FEED_OVR_FINE_PLUS:
                  new_f_override += FEED_OVERRIDE_FINE_INCREMENT;
                  break;

              case CMD_FEED_OVR_FINE_MINUS:
                  new_f_override -= FEED_OVERRIDE_FINE_INCREMENT;
                  break;

              case CMD_RAPID_OVR_RESET:
                  new_r_override = DEFAULT_RAPID_OVERRIDE;
                  break;

              case CMD_RAPID_OVR_MEDIUM:
                  new_r_override = RAPID_OVERRIDE_MEDIUM;
                  break;

              case CMD_RAPID_OVR_LOW:
                  new_r_override = RAPID_OVERRIDE_LOW;
                  break;
          }

        } while((rt_exec = get_feed_ovr()));

        plan_feed_override(new_f_override, new_r_override);
    }

    if((rt_exec = get_accessory_ovr())) {

        bool spindle_stop = false;
        uint_fast8_t last_s_override = sys.spindle_rpm_ovr;
        coolant_state_t coolant_state = gc_state.modal.coolant;

        do {

          switch(rt_exec) {

              case CMD_SPINDLE_OVR_RESET:
                  last_s_override = DEFAULT_SPINDLE_RPM_OVERRIDE;
                  break;

              case CMD_SPINDLE_OVR_COARSE_PLUS:
                  last_s_override += SPINDLE_OVERRIDE_COARSE_INCREMENT;
                  break;

              case CMD_SPINDLE_OVR_COARSE_MINUS:
                  last_s_override -= SPINDLE_OVERRIDE_COARSE_INCREMENT;
                  break;

              case CMD_SPINDLE_OVR_FINE_PLUS:
                  last_s_override += SPINDLE_OVERRIDE_FINE_INCREMENT;
                  break;

              case CMD_SPINDLE_OVR_FINE_MINUS:
                  last_s_override -= SPINDLE_OVERRIDE_FINE_INCREMENT;
                  break;

              case CMD_SPINDLE_OVR_STOP:
                  spindle_stop = !spindle_stop;
                  break;

              case CMD_COOLANT_MIST_OVR_TOGGLE:
                  if (hal.driver_cap.mist_control && ((sys.state == STATE_IDLE) || (sys.state & (STATE_CYCLE | STATE_HOLD)))) {
                      coolant_state.mist = !coolant_state.mist;
                  }
                  break;

              case CMD_COOLANT_FLOOD_OVR_TOGGLE:
                  if ((sys.state == STATE_IDLE) || (sys.state & (STATE_CYCLE | STATE_HOLD))) {
                      coolant_state.flood = !coolant_state.flood;
                  }
                  break;

                  default:
                      if(hal.userdefined_rt_command_execute)
                          hal.userdefined_rt_command_execute(rt_exec);
                      break;
              }

        } while((rt_exec = get_accessory_ovr()));

        spindle_set_override(last_s_override);

      // NOTE: Since coolant state always performs a planner sync whenever it changes, the current
      // run state can be determined by checking the parser state.
        if(coolant_state.value != gc_state.modal.coolant.value) {
            coolant_set_state(coolant_state); // Report counter set in coolant_set_state().
            gc_state.modal.coolant = coolant_state;
        }

        if (spindle_stop && sys.state == STATE_HOLD) {
            // Spindle stop override allowed only while in HOLD state.
            // NOTE: Report counters are set in spindle_set_state() when spindle stop is executed.
            if (!sys.spindle_stop_ovr.value)
                sys.spindle_stop_ovr.initiate = On;
            else if (sys.spindle_stop_ovr.enabled)
                sys.spindle_stop_ovr.restore = On;
        }
    }

    // End execute overrides.

    // Reload step segment buffer
    if (sys.state & (STATE_CYCLE | STATE_HOLD | STATE_SAFETY_DOOR | STATE_HOMING | STATE_SLEEP| STATE_JOG))
        st_prep_buffer();

    return !(sys.abort || sys.cancel);
}

// Handles Grbl system suspend procedures, such as feed hold, safety door, and parking motion.
// The system will enter this loop, create local variables for suspend tasks, and return to
// whatever function that invoked the suspend, such that Grbl resumes normal operation.
// This function is written in a way to promote custom parking motions. Simply use this as a
// template

static void protocol_exec_rt_suspend ()
{
    while (sys.suspend) {

        if (sys.abort)
            return;

        // If door closed keep issuing cycle start requests until resumed
        if(sys.state == STATE_SAFETY_DOOR && !hal.system_control_get_state().safety_door_ajar)
            system_set_exec_state_flag(EXEC_CYCLE_START);

        // Check for sleep conditions and execute auto-park, if timeout duration elapses.
        // Sleep is valid for both hold and door states, if the spindle or coolant are on or
        // set to be re-enabled.
        if(settings.flags.sleep_enable)
            sleep_check();

        protocol_exec_rt_system();
    }
}

// Checks for and process real-time commands in input stream.
// Called from serial input interrupt handler.
ISR_CODE bool protocol_process_realtime (char c)
{
    bool add = !sys.block_input_stream;

    switch ((unsigned char)c) {

        case CMD_STOP: // Set as true
            system_set_exec_state_flag(EXEC_STOP);
            char_counter = 0;
            hal.serial_cancel_read_buffer();
            add = false;
            break;

        case CMD_RESET: // Call motion control reset routine.
            if(!hal.system_control_get_state().e_stop)
                mc_reset();
            add = false;
            break;

        case CMD_EXIT: // Call motion control reset routine.
            mc_reset();
            sys.exit = true;
            add = false;
            break;

        case CMD_STATUS_REPORT: // Set as true
            system_set_exec_state_flag(EXEC_STATUS_REPORT);
            add = false;
            break;

        case CMD_PID_REPORT:
            system_set_exec_state_flag(EXEC_PID_REPORT);
            add = false;
            break;

        case CMD_CYCLE_START: // Set as true
            system_set_exec_state_flag(EXEC_CYCLE_START);
            // Cancel any pending tool change
            gc_state.tool_change = false;
            add = false;
            break;

        case CMD_FEED_HOLD: // Set as true
            system_set_exec_state_flag(EXEC_FEED_HOLD);
            add = false;
            break;

        case CMD_SAFETY_DOOR: // Set as true
            system_set_exec_state_flag(EXEC_SAFETY_DOOR);
            add = false;
            break;

        case CMD_JOG_CANCEL: // Cancel jogging
            char_counter = 0;
            hal.serial_cancel_read_buffer();
            break;

        case CMD_FEED_OVR_RESET:
        case CMD_FEED_OVR_COARSE_PLUS:
        case CMD_FEED_OVR_COARSE_MINUS:
        case CMD_FEED_OVR_FINE_PLUS:
        case CMD_FEED_OVR_FINE_MINUS:
        case CMD_RAPID_OVR_RESET:
        case CMD_RAPID_OVR_MEDIUM:
        case CMD_RAPID_OVR_LOW:
            enqueue_feed_ovr(c);
            break;

        default:
            if((unsigned char)c > 0x7F)
                enqueue_accessory_ovr((uint8_t)c);
            break;
    }

    return add && (unsigned char)c <= 0x7F;
}
