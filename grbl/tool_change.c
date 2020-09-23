/*
  tool_change.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Manual tool change with option for automatic touch off

  Part of GrblHAL

  Copyright (c) 2020 Terje Io

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

#include <string.h>

#include "hal.h"
#include "motion_control.h"
#include "protocol.h"
#include "report.h"
#include "tool_change.h"

// NOTE: only used when settings.homing.flags.force_set_origin is true
#ifndef LINEAR_AXIS_HOME_OFFSET
#define LINEAR_AXIS_HOME_OFFSET -1.0f
#endif

#ifndef TOOL_CHANGE_PROBE_RETRACT_DISTANCE
#define TOOL_CHANGE_PROBE_RETRACT_DISTANCE 2.0f
#endif

static bool block_cycle_start;
static volatile bool execute_posted = false;
static volatile uint32_t spin_lock = 0;
static float tool_change_position;
static tool_data_t current_tool = {0}, *next_tool = NULL;
static driver_reset_ptr driver_reset = NULL;
static plane_t plane;
static bool (*enqueue_realtime_command)(char data) = NULL;
static void (*control_interrupt_callback)(control_signals_t signals) = NULL;
static coord_data_t target = {0}, previous;

// Set tool offset on successful $TPW probe, prompt for retry on failure.
// Called via probe completed event.
static void on_probe_completed (void)
{
    if(!sys.flags.probe_succeeded)
        hal.stream.write("[MSG:Probe failed, try again.]" ASCII_EOL);
    else if(sys.tlo_reference_set.mask & bit(plane.axis_linear))
        gc_set_tool_offset(ToolLengthOffset_EnableDynamic, plane.axis_linear, sys_probe_position[plane.axis_linear] - sys.tlo_reference[plane.axis_linear]);
//    else error?
}

// Restore HAL pointers on completion or reset.
static void change_completed (void)
{
    if(enqueue_realtime_command) {
        while(spin_lock);
        hal.irq_disable();
        hal.stream.enqueue_realtime_command = enqueue_realtime_command;
        enqueue_realtime_command = NULL;
        hal.irq_enable();
    }

    if(control_interrupt_callback) {
        while(spin_lock);
        hal.irq_disable();
        hal.control_interrupt_callback = control_interrupt_callback;
        control_interrupt_callback = NULL;
        hal.irq_enable();
    }

    grbl.on_probe_completed = NULL;
    gc_state.tool_change = false;
}


// Reset claimed HAL entry points and restore previous tool if needed on soft restart.
// Called from EXEC_RESET and EXEC_STOP handlers (via HAL).
static void reset (void)
{
    if(next_tool) { //TODO: move to gc_xxx() function?
        // Restore previous tool if reset is during change
#ifdef N_TOOLS
        if((sys.report.tool = current_tool.tool != next_tool->tool))
            gc_state.tool = current_tool;
#else
        if((sys.report.tool = current_tool.tool != next_tool->tool))
            memcpy(next_tool, &current_tool, sizeof(tool_data_t));
#endif
        gc_state.tool_pending = gc_state.tool->tool;
        next_tool = NULL;
    }

    change_completed();
    driver_reset();
}

// Restore coolant and spindle status, return controlled point to original position.
static bool restore (void)
{
    plan_line_data_t plan_data = {0};

    plan_data.condition.rapid_motion = On;

    target.values[plane.axis_linear] = tool_change_position;
    mc_line(target.values, &plan_data);

    memcpy(&target, &previous, sizeof(coord_data_t));
    target.values[plane.axis_linear] = tool_change_position;
    mc_line(target.values, &plan_data);

    if(protocol_buffer_synchronize()) {

        sync_position();

        coolant_sync(gc_state.modal.coolant);
        spindle_restore(gc_state.modal.spindle, gc_state.spindle.rpm);

        previous.values[plane.axis_linear] += gc_get_offset(plane.axis_linear);
        mc_line(previous.values, &plan_data);
    }

    if(protocol_buffer_synchronize()) {
        sync_position();
        memcpy(&current_tool, next_tool, sizeof(tool_data_t));
    }

    return !ABORTED;
}

// Issue warning on cycle start event if touch off by $TPW is pending.
// Used in Manual and Manual_G59_3 modes ($341=1 or $341=2). Called from the foreground process.
static void execute_warning (uint_fast16_t state)
{
    hal.stream.write("[MSG:Perform a probe with $TPW first!]" ASCII_EOL);
}

// Execute restore position after touch off (on cycle start event).
// Used in Manual and Manual_G59_3 modes ($341=1 or $341=2). Called from the foreground process.
static void execute_restore (uint_fast16_t state)
{
    // Get current position.
    system_convert_array_steps_to_mpos(target.values, sys_position);

    bool ok = restore();

    change_completed();

    report_feedback_message(Message_None);

    if(ok)
        system_set_exec_state_flag(EXEC_CYCLE_START);
}

// Execute touch off on cycle start event from @ G59.3 position.
// Used in SemiAutomatic mode ($341=3) only. Called from the foreground process.
static void execute_probe (uint_fast16_t state)
{
    bool ok;
    coord_data_t offset;
    plan_line_data_t plan_data = {0};
    gc_parser_flags_t flags = {0};

    // G59.3 contains offsets to position of TLS.
    settings_read_coord_data(CoordinateSystem_G59_3, &offset.values);

    plan_data.condition.rapid_motion = On;

    target.values[plane.axis_0] = offset.values[plane.axis_0];
    target.values[plane.axis_1] = offset.values[plane.axis_1];

    if((ok = mc_line(target.values, &plan_data))) {

        target.values[plane.axis_linear] = offset.values[plane.axis_linear];
        ok = mc_line(target.values, &plan_data);

        plan_data.feed_rate = settings.tool_change.seek_rate;
        plan_data.condition.value = 0;
        target.values[plane.axis_linear] -= settings.tool_change.probing_distance;

        if((ok = ok && mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found))
        {
            system_convert_array_steps_to_mpos(target.values, sys_probe_position);

            // Retract a bit and perform slow probe.
            target.values[plane.axis_linear] += TOOL_CHANGE_PROBE_RETRACT_DISTANCE;
            if((ok = mc_line(target.values, &plan_data))) {
                plan_data.feed_rate = settings.tool_change.feed_rate;
                target.values[plane.axis_linear] -= (TOOL_CHANGE_PROBE_RETRACT_DISTANCE + 2.0f);
                ok = mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found;
            }
        }

        if(ok) {
            if(!(sys.tlo_reference_set.mask & bit(plane.axis_linear))) {
                sys.tlo_reference[plane.axis_linear] = sys_probe_position[plane.axis_linear];
                sys.tlo_reference_set.mask |= bit(plane.axis_linear);
                sys.report.tlo_reference = On;
                report_feedback_message(Message_ReferenceTLOEstablished);
            } else
                gc_set_tool_offset(ToolLengthOffset_EnableDynamic, plane.axis_linear,
                                    sys_probe_position[plane.axis_linear] - sys.tlo_reference[plane.axis_linear]);

            ok = restore();
        }
    }

    change_completed();

    if(ok)
        system_set_exec_state_flag(EXEC_CYCLE_START);
}

// Trap cycle start commands and redirect to foreground process
// by adding the function to be called to the realtime execution queue.
ISR_CODE static void trap_control_cycle_start (control_signals_t signals)
{
    spin_lock++;

    if(signals.cycle_start) {
        if(!execute_posted) {
            if(!block_cycle_start)
                execute_posted = protocol_enqueue_rt_command(settings.tool_change.mode == ToolChange_SemiAutomatic ? execute_probe : execute_restore);
            else
                protocol_enqueue_rt_command(execute_warning);
        }
        signals.cycle_start = Off;
    } else
        control_interrupt_callback(signals);

    spin_lock--;
}

ISR_CODE static bool trap_stream_cycle_start (char c)
{
    bool drop = false;

    spin_lock++;

    if((drop = (c == CMD_CYCLE_START || c == CMD_CYCLE_START_LEGACY))) {
        if(!execute_posted) {
            if(!block_cycle_start)
                execute_posted = protocol_enqueue_rt_command(settings.tool_change.mode == ToolChange_SemiAutomatic ? execute_probe : execute_restore);
            else
                protocol_enqueue_rt_command(execute_warning);
        }
    } else
        drop = enqueue_realtime_command(c);

    spin_lock--;

    return drop;
}

// Set next and/or current tool. Called by gcode.c on on a Tn or M61 command (via HAL).
static void tool_select (tool_data_t *tool, bool next)
{
    next_tool = tool;
    if(!next)
        memcpy(&current_tool, tool, sizeof(tool_data_t));
}

// Start a tool change sequence. Called by gcode.c on a M6 command (via HAL).
static status_code_t tool_change (parser_state_t *parser_state)
{
    if(next_tool == NULL)
        return Status_GCodeToolError;

    if(current_tool.tool == next_tool->tool)
        return Status_OK;

#ifdef TOOL_LENGTH_OFFSET_AXIS
    plane.axis_linear = TOOL_LENGTH_OFFSET_AXIS;
  #if TOOL_LENGTH_OFFSET_AXIS == X_AXIS
    plane.axis_0 = Y_AXIS;
    plane.axis_1 = Z_AXIS;
  #elif TOOL_LENGTH_OFFSET_AXIS == Y_AXIS
    plane.axis_0 = Z_AXIS;
    plane.axis_1 = X_AXIS;
  #else
    plane.axis_0 = X_AXIS;
    plane.axis_1 = Y_AXIS;
  #endif
#else
    gc_get_plane_data(&plane, parser_state->modal.plane_select);
#endif

    uint8_t homed_req = settings.tool_change.mode == ToolChange_Manual ? bit(plane.axis_linear) : (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT);

    if((sys.homed.mask & homed_req) != homed_req)
        return Status_HomingRequired;

    if(settings.tool_change.mode != ToolChange_SemiAutomatic)
        grbl.on_probe_completed = on_probe_completed;

    // Trap cycle start command and control signal.
    block_cycle_start = settings.tool_change.mode != ToolChange_SemiAutomatic;
    enqueue_realtime_command = hal.stream.enqueue_realtime_command;
    hal.stream.enqueue_realtime_command = trap_stream_cycle_start;
    control_interrupt_callback = hal.control_interrupt_callback;
    hal.control_interrupt_callback = trap_control_cycle_start;

    // Stop spindle and coolant.
    hal.spindle_set_state((spindle_state_t){0}, 0.0f);
    hal.coolant_set_state((coolant_state_t){0});

    execute_posted = false;
    parser_state->tool_change = true;

    // Save current position.
    system_convert_array_steps_to_mpos(previous.values, sys_position);

    // Establish axis assignments.

    previous.values[plane.axis_linear] -= gc_get_offset(plane.axis_linear);

    plan_line_data_t plan_data = {0};
    plan_data.condition.rapid_motion = On;

    // TODO: add?
    //if(!settings.homing.flags.force_set_origin && bit_istrue(settings.homing.dir_mask.value, bit(plane.axis_linear)))
    //    tool_change_position = ?
    //else

    tool_change_position = sys.home_position[plane.axis_linear] - settings.homing.flags.force_set_origin ? LINEAR_AXIS_HOME_OFFSET : 0.0f;

    // Rapid to home position of linear axis.
    memcpy(&target, &previous, sizeof(coord_data_t));
    target.values[plane.axis_linear] = tool_change_position;
    if(!mc_line(target.values, &plan_data))
        return Status_Reset;

    if(settings.tool_change.mode == ToolChange_Manual_G59_3) {

        // G59.3 contains offsets to tool change position.
        settings_read_coord_data(CoordinateSystem_G59_3, &target.values);

        float tmp_pos = target.values[plane.axis_linear];

        target.values[plane.axis_linear] = tool_change_position;
        if(!mc_line(target.values, &plan_data))
            return Status_Reset;

        target.values[plane.axis_linear] = tmp_pos;
        if(!mc_line(target.values, &plan_data))
            return Status_Reset;
    }

    protocol_buffer_synchronize();
    sync_position();

    // Enter tool change mode, waits for cycle start to continue.
    system_set_exec_state_flag(EXEC_TOOL_CHANGE);   // Set up program pause for manual tool change
    protocol_execute_realtime();                    // Execute...

    return Status_OK;
}

// Claim HAL tool change entry points and clear current tool offsets.
// TODO: change to survive a warm reset?
void tc_init (void)
{
    if(hal.driver_cap.atc) // Do not override driver tool change implementation!
        return;

    if(!hal.stream.suspend_read) // Tool change requires support for suspending input stream.
        return;

    sys.report.tlo_reference = sys.tlo_reference_set.mask != 0;
    sys.tlo_reference_set.mask = 0;

    gc_set_tool_offset(ToolLengthOffset_Cancel, 0, 0.0f);

    if(settings.tool_change.mode == ToolChange_Disabled) {
        hal.tool_select = NULL;
        hal.tool_change = NULL;
    } else {
        if(driver_reset == NULL) {
            driver_reset = hal.driver_reset;
            hal.driver_reset = reset;
        }
        hal.tool_select = tool_select;
        hal.tool_change = tool_change;
    }
}

// Perform a probe cycle: set tool length offset and restart job if successful.
// Note: tool length offset is set by the on_probe_completed event handler.
// Called by the $TPW system command.
status_code_t tc_probe_workpiece (void)
{
    if(!(settings.tool_change.mode == ToolChange_Manual || settings.tool_change.mode == ToolChange_Manual_G59_3) || enqueue_realtime_command == NULL)
        return Status_InvalidStatement;

    // TODO: add check for reference offset set?

    bool ok;
    gc_parser_flags_t flags = {0};
    plan_line_data_t plan_data = {0};

    // Get current position.
    system_convert_array_steps_to_mpos(target.values, sys_position);

    flags.probe_is_no_error = On;
    plan_data.feed_rate = settings.tool_change.seek_rate;

    target.values[plane.axis_linear] -= settings.tool_change.probing_distance;

    if((ok = mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found))
    {
        system_convert_array_steps_to_mpos(target.values, sys_probe_position);

        // Retract a bit and perform slow probe.
        target.values[plane.axis_linear] += TOOL_CHANGE_PROBE_RETRACT_DISTANCE;
        if((ok = mc_line(target.values, &plan_data))) {

            plan_data.feed_rate = settings.tool_change.feed_rate;
            target.values[plane.axis_linear] -= (TOOL_CHANGE_PROBE_RETRACT_DISTANCE + 2.0f);
            if((ok = mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found)) {
                // Retract a bit again so that any touch plate can be removed
                system_convert_array_steps_to_mpos(target.values, sys_probe_position);
                plan_data.feed_rate = settings.tool_change.seek_rate;
                target.values[plane.axis_linear] += TOOL_CHANGE_PROBE_RETRACT_DISTANCE * 2.0f;
                ok = mc_line(target.values, &plan_data);
            }
        }
    }

    if(ok && protocol_buffer_synchronize()) {
        sync_position();
        block_cycle_start = false;
        hal.stream.write("[MSG:Remove any touch plate and press cycle start to continue.]" ASCII_EOL);
    }

    return ok ? Status_OK : Status_GCodeToolError;
}
