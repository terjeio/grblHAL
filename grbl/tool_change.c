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

#ifndef LINEAR_AXIS_HOME_OFFSET
#define LINEAR_AXIS_HOME_OFFSET -1.0f
#endif

#ifndef TOOL_CHANGE_PROBE_RETRACT_DISTANCE
#define TOOL_CHANGE_PROBE_RETRACT_DISTANCE 2.0f
#endif

static tool_data_t *current_tool = NULL, *next_tool = NULL;
static driver_reset_ptr driver_reset = NULL;
static plane_t plane;
static bool (*enqueue_realtime_command)(char data) = NULL;
static void (*execute_realtime)(uint_fast16_t state) = NULL;
void (*control_interrupt_callback)(control_signals_t signals) = NULL;
static coord_data_t target = {0}, previous;

// Set tool offset on successful $TPW probe.
static void on_probe_completed (void)
{
    if(sys.tlo_reference_set && sys.flags.probe_succeeded)
        gc_set_tool_offset(ToolLengthOffset_EnableDynamic, plane.axis_linear, sys_probe_position[plane.axis_linear] - sys.tlo_reference);
    // else error?
}

// Restore HAL pointers on completion or reset.
static void change_completed (void)
{
    if(enqueue_realtime_command) {
        hal.stream.enqueue_realtime_command = enqueue_realtime_command;
        enqueue_realtime_command = NULL;
    }

    if(control_interrupt_callback) {
        hal.control_interrupt_callback = control_interrupt_callback;
        control_interrupt_callback = NULL;
    }

    if(execute_realtime) {
        hal.execute_realtime = execute_realtime;
        execute_realtime = NULL;
    }

    hal.on_probe_completed = NULL;
    gc_state.tool_change = false;
}

static void reset (void)
{
    sys.tlo_reference_set = false; // For now...

    change_completed();
    driver_reset();
}

static void tool_select (tool_data_t *tool, bool next)
{
    if(next)
        next_tool = tool;
    else
        current_tool = tool;
}

// Restore coolant and spindle status, return controlled point to original position.
static bool restore (void)
{
    plan_line_data_t plan_data = {0};

    coolant_sync(gc_state.modal.coolant);
    spindle_restore(gc_state.modal.spindle, gc_state.spindle.rpm);

    plan_data.condition.rapid_motion = On;

    target.values[plane.axis_linear] = LINEAR_AXIS_HOME_OFFSET;
    mc_line(target.values, &plan_data);

    memcpy(&target, &previous, sizeof(coord_data_t));
    target.values[plane.axis_linear] = LINEAR_AXIS_HOME_OFFSET;
    mc_line(target.values, &plan_data);

    previous.values[plane.axis_linear] += gc_get_offset(plane.axis_linear);
    mc_line(previous.values, &plan_data);

    if(protocol_buffer_synchronize()) {
        gc_sync_position();
        current_tool = next_tool;
    }

    return !ABORTED;
}

static void execute_restore (uint_fast16_t state)
{
    hal.execute_realtime = execute_realtime;
    execute_realtime = NULL;

    // Get current position.
    system_convert_array_steps_to_mpos(target.values, sys_position);

    bool ok = restore();

    change_completed();

    if(ok)
        system_set_exec_state_flag(EXEC_CYCLE_START);
}

static void execute_probe (uint_fast16_t state)
{
    bool ok;
    coord_data_t offset;
    plan_line_data_t plan_data = {0};
    gc_parser_flags_t flags = {0};

    hal.execute_realtime = execute_realtime;
    execute_realtime = NULL;

    // G59.3 contains offsets to position of TLS.
    settings_read_coord_data(SETTING_INDEX_G59_3, &offset.values);

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
            if(!sys.tlo_reference_set) {
                sys.tlo_reference = sys_probe_position[plane.axis_linear];
                sys.tlo_reference_set = true;
                sys.report.tlo_reference = On;
                report_feedback_message(Message_ReferenceTLOEstablished);
            } else
                gc_set_tool_offset(ToolLengthOffset_EnableDynamic, plane.axis_linear, sys_probe_position[plane.axis_linear] - sys.tlo_reference);

            ok = restore();
        }
    }

    change_completed();

    if(ok)
        system_set_exec_state_flag(EXEC_CYCLE_START);
}

// Trap cycle start commands and redirect to foreground process
// by temporarily claiming the HAL execute_realtime entry point
// in order to execute probing and spindle/coolant change.
// TODO: move to state machine with own EXEC_ bit?
ISR_CODE static void trap_control_cycle_start (control_signals_t signals)
{
    if(signals.cycle_start) {
        if(execute_realtime == NULL) {
            execute_realtime = hal.execute_realtime;
            hal.execute_realtime = settings.tool_change.mode == ToolChange_SemiAutomatic ? execute_probe : execute_restore;
        }
        signals.cycle_start = Off;
    } else
        control_interrupt_callback(signals);
}

ISR_CODE static bool trap_stream_cycle_start (char c)
{
    bool drop = false;

    if((drop = (c == CMD_CYCLE_START || c == CMD_CYCLE_START_LEGACY))) {
        if(execute_realtime == NULL) {
            execute_realtime = hal.execute_realtime;
            hal.execute_realtime = settings.tool_change.mode == ToolChange_SemiAutomatic ? execute_probe : execute_restore;
        }
    } else
        drop = enqueue_realtime_command(c);

    return drop;
}

static status_code_t tool_change (parser_state_t *gc_state)
{
    if(next_tool == NULL)
        return Status_GCodeToolError;

    if(current_tool == next_tool)
        return Status_OK;

    if(!sys.homed.mask || sys.homed.mask != sys.homing.mask)
        return Status_HomingRequired;

    if(settings.tool_change.mode != ToolChange_SemiAutomatic)
        hal.on_probe_completed = on_probe_completed;

    // Trap cycle start command and control signal.
    enqueue_realtime_command = hal.stream.enqueue_realtime_command;
    hal.stream.enqueue_realtime_command = trap_stream_cycle_start;
    control_interrupt_callback = hal.control_interrupt_callback;
    hal.control_interrupt_callback = trap_control_cycle_start;

    // Stop spindle and coolant.
    hal.spindle_set_state((spindle_state_t){0}, 0.0f);
    hal.coolant_set_state((coolant_state_t){0});

    gc_state->tool_change = true;

    // Save current position.
    system_convert_array_steps_to_mpos(previous.values, sys_position);

    // Establish axis assignments.
    // A plane change should invalidate current tool reference offset?
    gc_get_plane_data(&plane, gc_state->modal.plane_select);

    previous.values[plane.axis_linear] -= gc_get_offset(plane.axis_linear);

    plan_line_data_t plan_data = {0};
    plan_data.condition.rapid_motion = On;

    // Rapid to home position of linear axis.
    memcpy(&target, &previous, sizeof(coord_data_t));
    target.values[plane.axis_linear] = LINEAR_AXIS_HOME_OFFSET;
    if(!mc_line(target.values, &plan_data))
        return Status_Reset;

    if(settings.tool_change.mode == ToolChange_Manual_G59_3) {

        // G59.3 contains offsets to tool change position.
        settings_read_coord_data(SETTING_INDEX_G59_3, &target.values);

        float tmp_pos = target.values[plane.axis_linear];

        target.values[plane.axis_linear] = LINEAR_AXIS_HOME_OFFSET;
        if(!mc_line(target.values, &plan_data))
            return Status_Reset;

        target.values[plane.axis_linear] = tmp_pos;
        if(!mc_line(target.values, &plan_data))
            return Status_Reset;
    }

    protocol_buffer_synchronize();

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

    sys.report.tlo_reference = sys.tlo_reference_set;
    sys.tlo_reference_set = false;

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
// Used by the $TPW system command.
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
            ok = mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found;
        }
    }

    if(ok)
        hal.stream.enqueue_realtime_command(CMD_CYCLE_START);

    return ok ? Status_OK : Status_GCodeToolError;
}
