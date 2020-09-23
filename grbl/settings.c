/*
  settings.c - non-volatile storage configuration handling

  Part of GrblHAL

  Copyright (c) 2017-2020 Terje Io
  Copyright (c) 2011-2015 Sungeun K. Jeon
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

#include <math.h>
#include <string.h>
#include <stdint.h>
#include <stdbool.h>
#include <assert.h>

#include "hal.h"
#include "defaults.h"
#include "report.h"
#include "limits.h"
#include "nvs_buffer.h"
#include "tool_change.h"

#ifdef ENABLE_SPINDLE_LINEARIZATION
#include <stdio.h>
#endif

#ifndef SETTINGS_RESTORE_DEFAULTS
#define SETTINGS_RESTORE_DEFAULTS          1
#endif
#ifndef SETTINGS_RESTORE_PARAMETERS
#define SETTINGS_RESTORE_PARAMETERS        1
#endif
#ifndef SETTINGS_RESTORE_STARTUP_LINES
#define SETTINGS_RESTORE_STARTUP_LINES     1
#endif
#ifndef SETTINGS_RESTORE_BUILD_INFO
#define SETTINGS_RESTORE_BUILD_INFO        1
#endif
#ifndef SETTINGS_RESTORE_DRIVER_PARAMETERS
#define SETTINGS_RESTORE_DRIVER_PARAMETERS 1
#endif

settings_t settings;

const settings_restore_t settings_all = {
    .defaults          = SETTINGS_RESTORE_DEFAULTS,
    .parameters        = SETTINGS_RESTORE_PARAMETERS,
    .startup_lines     = SETTINGS_RESTORE_STARTUP_LINES,
    .build_info        = SETTINGS_RESTORE_BUILD_INFO,
    .driver_parameters = SETTINGS_RESTORE_DRIVER_PARAMETERS
};

const settings_t defaults = {

    .version = SETTINGS_VERSION,

    .junction_deviation = DEFAULT_JUNCTION_DEVIATION,
    .arc_tolerance = DEFAULT_ARC_TOLERANCE,
    .g73_retract = DEFAULT_G73_RETRACT,

    .flags.legacy_rt_commands = DEFAULT_LEGACY_RTCOMMANDS,
    .flags.report_inches = DEFAULT_REPORT_INCHES,
    .flags.laser_mode = DEFAULT_LASER_MODE,
    .flags.lathe_mode = DEFAULT_LATHE_MODE,
    .flags.invert_probe_pin = DEFAULT_INVERT_PROBE_PIN,
    .flags.sleep_enable = DEFAULT_SLEEP_ENABLE,
    .flags.disable_laser_during_hold = DEFAULT_DISABLE_LASER_DURING_HOLD,
    .flags.restore_after_feed_hold = DEFAULT_RESTORE_AFTER_FEED_HOLD,
    .flags.force_initialization_alarm = DEFAULT_FORCE_INITIALIZATION_ALARM,
    .flags.disable_probe_pullup = DISABLE_PROBE_PIN_PULL_UP,
    .flags.allow_probing_feed_override = ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES,

    .steppers.pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS,
    .steppers.pulse_delay_microseconds = DEFAULT_STEP_PULSE_DELAY,
    .steppers.idle_lock_time = DEFAULT_STEPPER_IDLE_LOCK_TIME,
    .steppers.step_invert.mask = DEFAULT_STEPPING_INVERT_MASK,
    .steppers.dir_invert.mask = DEFAULT_DIRECTION_INVERT_MASK,
    .steppers.enable_invert.mask = INVERT_ST_ENABLE_MASK,
    .steppers.deenergize.mask = ST_DEENERGIZE_MASK,
#if DEFAULT_HOMING_ENABLE
    .homing.flags.enabled = DEFAULT_HOMING_ENABLE,
    .homing.flags.init_lock = DEFAULT_HOMING_INIT_LOCK,
    .homing.flags.single_axis_commands = HOMING_SINGLE_AXIS_COMMANDS,
    .homing.flags.force_set_origin = HOMING_FORCE_SET_ORIGIN,
    .homing.flags.manual = DEFAULT_HOMING_ALLOW_MANUAL,
#else
    .homing.flags.value = 0,
#endif
    .homing.dir_mask.value = DEFAULT_HOMING_DIR_MASK,
    .homing.feed_rate = DEFAULT_HOMING_FEED_RATE,
    .homing.seek_rate = DEFAULT_HOMING_SEEK_RATE,
    .homing.debounce_delay = DEFAULT_HOMING_DEBOUNCE_DELAY,
    .homing.pulloff = DEFAULT_HOMING_PULLOFF,
    .homing.locate_cycles = DEFAULT_N_HOMING_LOCATE_CYCLE,
    .homing.cycle[0].mask = HOMING_CYCLE_0,
    .homing.cycle[1].mask = HOMING_CYCLE_1,
    .homing.cycle[2].mask = HOMING_CYCLE_2,

    .status_report.machine_position = DEFAULT_REPORT_BUFFER_STATE,
    .status_report.buffer_state = DEFAULT_REPORT_BUFFER_STATE,
    .status_report.line_numbers = DEFAULT_REPORT_LINE_NUMBERS,
    .status_report.feed_speed = DEFAULT_REPORT_CURRENT_FEED_SPEED,
    .status_report.pin_state = DEFAULT_REPORT_PIN_STATE,
    .status_report.work_coord_offset = DEFAULT_REPORT_WORK_COORD_OFFSET,
    .status_report.overrides = DEFAULT_REPORT_OVERRIDES,
    .status_report.probe_coordinates = DEFAULT_REPORT_PROBE_COORDINATES,
    .status_report.sync_on_wco_change = DEFAULT_REPORT_SYNC_ON_WCO_CHANGE,
    .status_report.parser_state = DEFAULT_REPORT_PARSER_STATE,
    .status_report.alarm_substate = DEFAULT_REPORT_ALARM_SUBSTATE,

    .limits.flags.hard_enabled = DEFAULT_HARD_LIMIT_ENABLE,
    .limits.flags.soft_enabled = DEFAULT_SOFT_LIMIT_ENABLE,
    .limits.flags.jog_soft_limited = DEFAULT_JOG_LIMIT_ENABLE,
    .limits.flags.check_at_init = DEFAULT_CHECK_LIMITS_AT_INIT,
    .limits.flags.two_switches = DEFAULT_LIMITS_TWO_SWITCHES_ON_AXES,
    .limits.invert.mask = INVERT_LIMIT_PIN_MASK,
    .limits.disable_pullup.mask = DISABLE_LIMIT_PINS_PULL_UP_MASK,

    .control_invert.mask = INVERT_CONTROL_PIN_MASK,
    .control_disable_pullup.mask = DISABLE_CONTROL_PINS_PULL_UP_MASK,

    .spindle.rpm_max = DEFAULT_SPINDLE_RPM_MAX,
    .spindle.rpm_min = DEFAULT_SPINDLE_RPM_MIN,
    .spindle.disable_with_zero_speed = DEFAULT_SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED,
    .spindle.invert.on = INVERT_SPINDLE_ENABLE_PIN,
    .spindle.pwm_freq = DEFAULT_SPINDLE_PWM_FREQ,
    .spindle.pwm_off_value = DEFAULT_SPINDLE_PWM_OFF_VALUE,
    .spindle.pwm_min_value = DEFAULT_SPINDLE_PWM_MIN_VALUE,
    .spindle.pwm_max_value = DEFAULT_SPINDLE_PWM_MAX_VALUE,
    .spindle.at_speed_tolerance = DEFAULT_SPINDLE_AT_SPEED_TOLERANCE,
    .spindle.ppr = DEFAULT_SPINDLE_PPR,
    .spindle.pid.p_gain = DEFAULT_SPINDLE_P_GAIN,
    .spindle.pid.i_gain = DEFAULT_SPINDLE_I_GAIN,
    .spindle.pid.d_gain = DEFAULT_SPINDLE_D_GAIN,
    .spindle.pid.i_max_error = DEFAULT_SPINDLE_I_MAX,
#if SPINDLE_NPWM_PIECES > 0
    .spindle.pwm_piece[0] = { .rpm = NAN, .start = 0.0f, .end = 0.0f },
#endif
#if SPINDLE_NPWM_PIECES > 1
    .spindle.pwm_piece[1] = { .rpm = NAN, .start = 0.0f, .end = 0.0f },
#endif
#if SPINDLE_NPWM_PIECES > 2
    .spindle.pwm_piece[2] = { .rpm = NAN, .start = 0.0f, .end = 0.0f },
#endif
#if SPINDLE_NPWM_PIECES > 3
    .spindle.pwm_piece[3] = { .rpm = NAN, .start = 0.0f, .end = 0.0f },
#endif

    .coolant_invert.flood = INVERT_COOLANT_FLOOD_PIN,
    .coolant_invert.mist = INVERT_COOLANT_MIST_PIN,

    .axis[X_AXIS].steps_per_mm = DEFAULT_X_STEPS_PER_MM,
    .axis[Y_AXIS].steps_per_mm = DEFAULT_Y_STEPS_PER_MM,
    .axis[Z_AXIS].steps_per_mm = DEFAULT_Z_STEPS_PER_MM,
    .axis[X_AXIS].max_rate = DEFAULT_X_MAX_RATE,
    .axis[Y_AXIS].max_rate = DEFAULT_Y_MAX_RATE,
    .axis[Z_AXIS].max_rate = DEFAULT_Z_MAX_RATE,
    .axis[X_AXIS].acceleration = DEFAULT_X_ACCELERATION,
    .axis[Y_AXIS].acceleration = DEFAULT_Y_ACCELERATION,
    .axis[Z_AXIS].acceleration = DEFAULT_Z_ACCELERATION,
    .axis[X_AXIS].max_travel = (-DEFAULT_X_MAX_TRAVEL),
    .axis[Y_AXIS].max_travel = (-DEFAULT_Y_MAX_TRAVEL),
    .axis[Z_AXIS].max_travel = (-DEFAULT_Z_MAX_TRAVEL),

  #ifdef A_AXIS
    .axis[A_AXIS].steps_per_mm = DEFAULT_A_STEPS_PER_MM,
    .axis[A_AXIS].max_rate = DEFAULT_A_MAX_RATE,
    .axis[A_AXIS].acceleration = DEFAULT_A_ACCELERATION,
    .axis[A_AXIS].max_travel = (-DEFAULT_A_MAX_TRAVEL),
    .homing.cycle[3].mask = HOMING_CYCLE_3,
  #endif
  #ifdef B_AXIS
    .axis[B_AXIS].steps_per_mm = DEFAULT_B_STEPS_PER_MM,
    .axis[B_AXIS].max_rate = DEFAULT_B_MAX_RATE,
    .axis[B_AXIS].acceleration = DEFAULT_B_ACCELERATION,
    .axis[B_AXIS].max_travel = (-DEFAULT_B_MAX_TRAVEL),
    .homing.cycle[4].mask = HOMING_CYCLE_4,
  #endif
  #ifdef C_AXIS
    .axis[C_AXIS].steps_per_mm = DEFAULT_C_STEPS_PER_MM,
    .axis[C_AXIS].acceleration = DEFAULT_C_ACCELERATION,
    .axis[C_AXIS].max_rate = DEFAULT_C_MAX_RATE,
    .axis[C_AXIS].max_travel = (-DEFAULT_C_MAX_TRAVEL),
    .homing.cycle[5].mask = HOMING_CYCLE_5,
  #endif

    .tool_change.mode = (toolchange_mode_t)DEFAULT_TOOLCHANGE_MODE,
    .tool_change.probing_distance = DEFAULT_TOOLCHANGE_PROBING_DISTANCE,
    .tool_change.feed_rate = DEFAULT_TOOLCHANGE_FEED_RATE,
    .tool_change.seek_rate = DEFAULT_TOOLCHANGE_SEEK_RATE,

    .parking.flags.enabled = DEFAULT_PARKING_ENABLE,
    .parking.flags.deactivate_upon_init = DEFAULT_DEACTIVATE_PARKING_UPON_INIT,
    .parking.flags.enable_override_control= DEFAULT_ENABLE_PARKING_OVERRIDE_CONTROL,
    .parking.axis = DEFAULT_PARKING_AXIS,
    .parking.target = DEFAULT_PARKING_TARGET,
    .parking.rate = DEFAULT_PARKING_RATE,
    .parking.pullout_rate = DEFAULT_PARKING_PULLOUT_RATE,
    .parking.pullout_increment = DEFAULT_PARKING_PULLOUT_INCREMENT
};

// Write build info to persistent storage
void settings_write_build_info (char *line)
{
    if(hal.nvs.type != NVS_None)
        hal.nvs.memcpy_to_with_checksum(NVS_ADDR_BUILD_INFO, (uint8_t *)line, MAX_STORED_LINE_LENGTH);
}

// Read build info from persistent storage.
bool settings_read_build_info(char *line)
{
    if (!(hal.nvs.type != NVS_None && hal.nvs.memcpy_from_with_checksum((uint8_t *)line, NVS_ADDR_BUILD_INFO, MAX_STORED_LINE_LENGTH))) {
        // Reset line with default value
        line[0] = 0; // Empty line
        settings_write_build_info(line);
        return false;
    }
    return true;
}

// Write startup line to persistent storage
void settings_write_startup_line (uint8_t idx, char *line)
{
    assert(idx < N_STARTUP_LINE);

#ifdef FORCE_BUFFER_SYNC_DURING_NVS_WRITE
    protocol_buffer_synchronize(); // A startup line may contain a motion and be executing.
#endif

    if(hal.nvs.type != NVS_None)
        hal.nvs.memcpy_to_with_checksum(NVS_ADDR_STARTUP_BLOCK + idx * (MAX_STORED_LINE_LENGTH + 1), (uint8_t *)line, MAX_STORED_LINE_LENGTH);
}

// Read startup line to persistent storage.
bool settings_read_startup_line (uint8_t idx, char *line)
{
    assert(idx < N_STARTUP_LINE);

    if (!(hal.nvs.type != NVS_None && hal.nvs.memcpy_from_with_checksum((uint8_t *)line, NVS_ADDR_STARTUP_BLOCK + idx * (MAX_STORED_LINE_LENGTH + 1), MAX_STORED_LINE_LENGTH))) {
        // Reset line with default value
        *line = '\0'; // Empty line
        settings_write_startup_line(idx, line);
        return false;
    }
    return true;
}

// Write selected coordinate data to persistent storage.
void settings_write_coord_data (coord_system_id_t idx, float (*coord_data)[N_AXIS])
{
    assert(idx <= N_CoordinateSystems);

#ifdef FORCE_BUFFER_SYNC_DURING_NVS_WRITE
    protocol_buffer_synchronize();
#endif

    if(hal.nvs.type != NVS_None)
        hal.nvs.memcpy_to_with_checksum(NVS_ADDR_PARAMETERS + idx * (sizeof(coord_data_t) + 1), (uint8_t *)coord_data, sizeof(coord_data_t));
}

// Read selected coordinate data from persistent storage.
bool settings_read_coord_data (coord_system_id_t idx, float (*coord_data)[N_AXIS])
{
    assert(idx <= N_CoordinateSystems);

    if (!(hal.nvs.type != NVS_None && hal.nvs.memcpy_from_with_checksum((uint8_t *)coord_data, NVS_ADDR_PARAMETERS + idx * (sizeof(coord_data_t) + 1), sizeof(coord_data_t)))) {
        // Reset with default zero vector
        memset(coord_data, 0, sizeof(coord_data_t));
        settings_write_coord_data(idx, coord_data);
        return false;
    }
    return true;
}

// Write selected tool data to persistent storage.
bool settings_write_tool_data (tool_data_t *tool_data)
{
#ifdef N_TOOLS
    assert(tool_data->tool > 0 && tool_data->tool <= N_TOOLS); // NOTE: idx 0 is a non-persistent entry for tools not in tool table

    if(hal.nvs.type != NVS_None)
        hal.nvs.memcpy_to_with_checksum(NVS_ADDR_TOOL_TABLE + (tool_data->tool - 1) * (sizeof(tool_data_t) + 1), (uint8_t *)tool_data, sizeof(tool_data_t));

    return true;
#else
    return false;
#endif
}

// Read selected tool data from persistent storage.
bool settings_read_tool_data (uint32_t tool, tool_data_t *tool_data)
{
#ifdef N_TOOLS
    assert(tool > 0 && tool <= N_TOOLS); // NOTE: idx 0 is a non-persistent entry for tools not in tool table

    if (!(hal.nvs.type != NVS_None && hal.nvs.memcpy_from_with_checksum((uint8_t *)tool_data, NVS_ADDR_TOOL_TABLE + (tool - 1) * (sizeof(tool_data_t) + 1), sizeof(tool_data_t)) && tool_data->tool == tool)) {
        memset(tool_data, 0, sizeof(tool_data_t));
        tool_data->tool = tool;
    }

    return tool_data->tool == tool;
#else
    return false;
#endif
}

// Read Grbl global settings from persistent storage.
bool read_global_settings ()
{
    // Check version-byte of non-volatile storage
    return hal.nvs.type != NVS_None && SETTINGS_VERSION == hal.nvs.get_byte(0) && hal.nvs.memcpy_from_with_checksum((uint8_t *)&settings, NVS_ADDR_GLOBAL, sizeof(settings_t));
}

// Write Grbl global settings and version number to persistent storage
void write_global_settings ()
{
    if(hal.nvs.type != NVS_None) {
        hal.nvs.put_byte(0, SETTINGS_VERSION);
        hal.nvs.memcpy_to_with_checksum(NVS_ADDR_GLOBAL, (uint8_t *)&settings, sizeof(settings_t));
    }
}


// Restore Grbl global settings to defaults and write to persistent storage
void settings_restore (settings_restore_t restore)
{
    uint_fast8_t idx;
    char empty_line[MAX_STORED_LINE_LENGTH];

    memset(empty_line, 0xFF, MAX_STORED_LINE_LENGTH);
    *empty_line = '\0';

    if (restore.defaults) {
        memcpy(&settings, &defaults, sizeof(settings_t));

        settings.control_invert.block_delete &= hal.driver_cap.block_delete;
        settings.control_invert.e_stop &= hal.driver_cap.e_stop;
        settings.control_invert.stop_disable &= hal.driver_cap.program_stop;
        settings.control_disable_pullup.block_delete &= hal.driver_cap.block_delete;
        settings.control_disable_pullup.e_stop &= hal.driver_cap.e_stop;
        settings.control_disable_pullup.stop_disable &= hal.driver_cap.program_stop;

        write_global_settings();
    }

    if (restore.parameters) {
        float coord_data[N_AXIS];

        memset(coord_data, 0, sizeof(coord_data));
        for (idx = 0; idx <= N_WorkCoordinateSystems; idx++)
            settings_write_coord_data((coord_system_id_t)idx, &coord_data);

        settings_write_coord_data(CoordinateSystem_G92, &coord_data); // Clear G92 offsets

#ifdef N_TOOLS
        tool_data_t tool_data;
        memset(&tool_data, 0, sizeof(tool_data_t));
        for (idx = 1; idx <= N_TOOLS; idx++) {
            tool_data.tool = idx;
            settings_write_tool_data(&tool_data);
        }
#endif
    }

    if (restore.startup_lines) {
        for (idx = 0; idx < N_STARTUP_LINE; idx++)
            settings_write_startup_line(idx, empty_line);
    }

    if (restore.build_info) {
        settings_write_build_info(empty_line);
        settings_write_build_info(BUILD_INFO);
    }

    if(restore.driver_parameters && hal.driver_settings_restore)
        hal.driver_settings_restore();

    nvs_buffer_sync_physical();
}

// A helper method to set settings from command line
status_code_t settings_store_global_setting (setting_type_t setting, char *svalue)
{
    uint_fast8_t set_idx = 0;
    float value;

    if (!read_float(svalue, &set_idx, &value)) {
        status_code_t status;
        if(hal.driver_setting && (status = hal.driver_setting(setting, NAN, svalue)) != Status_Unhandled)
            return status;
        else
            return Status_BadNumberFormat;
    }

#if COMPATIBILITY_LEVEL <= 1

    if (value < 0.0f && setting != Setting_ParkingTarget)
        return Status_NegativeValue;

#endif

    if (setting >= Setting_AxisSettingsBase && setting <= Setting_AxisSettingsMax) {
        // Store axis configuration. Axis numbering sequence set by AXIS_SETTING defines.
        // NOTE: Ensure the setting index corresponds to the report.c settings printout.
        bool found = false;
        uint_fast16_t base_idx = (uint_fast16_t)setting - (uint_fast16_t)Setting_AxisSettingsBase;
        uint_fast8_t axis_idx = base_idx % AXIS_SETTINGS_INCREMENT;

        if(axis_idx < N_AXIS) switch((base_idx - axis_idx) / AXIS_SETTINGS_INCREMENT) {

            case AxisSetting_StepsPerMM:
                #ifdef MAX_STEP_RATE_HZ
                if (value * settings.axis[axis_idx].max_rate > (MAX_STEP_RATE_HZ * 60.0f))
                    return Status_MaxStepRateExceeded;
                #endif
                found = true;
                settings.axis[axis_idx].steps_per_mm = value;
                break;

            case AxisSetting_MaxRate:
                #ifdef MAX_STEP_RATE_HZ
                if (value * settings.axis[axis_idx].steps_per_mm > (MAX_STEP_RATE_HZ * 60.0f))
                    return Status_MaxStepRateExceeded;
                #endif
                found = true;
                settings.axis[axis_idx].max_rate = value;
                break;

            case AxisSetting_Acceleration:
                found = true;
                settings.axis[axis_idx].acceleration = value * 60.0f * 60.0f; // Convert to mm/min^2 for grbl internal use.
                break;

            case AxisSetting_MaxTravel:
                found = true;
                settings.axis[axis_idx].max_travel = -value; // Store as negative for grbl internal use.
                break;

#ifdef ENABLE_BACKLASH_COMPENSATION
            case AxisSetting_Backlash:
                found = true;
                settings.axis[axis_idx].backlash = value;
                break;
#endif

            default: // for stopping compiler warning
                break;
        }

        if(!(found || (hal.driver_setting && hal.driver_setting(setting, value, svalue) == Status_OK)))
            return Status_InvalidStatement;

    } else {
        // Store non-axis Grbl settings
        uint_fast16_t int_value = (uint_fast16_t)truncf(value);
        switch(setting) {

            case Setting_PulseMicroseconds:
                if (value < 2.0f)
                    return Status_SettingStepPulseMin;
                settings.steppers.pulse_microseconds = value;
                break;

            case Setting_PulseDelayMicroseconds:
                if(value > 0.0f && !hal.driver_cap.step_pulse_delay)
                    return Status_SettingDisabled;
                settings.steppers.pulse_delay_microseconds = value;
                break;

            case Setting_StepperIdleLockTime:
                settings.steppers.idle_lock_time = int_value;
                break;

            case Setting_StepInvertMask:
                settings.steppers.step_invert.mask = int_value & AXES_BITMASK;
                break;

            case Setting_DirInvertMask:
                settings.steppers.dir_invert.mask = int_value & AXES_BITMASK;
                break;

            case Setting_InvertStepperEnable: // Reset to ensure change. Immediate re-init may cause problems.
                settings.steppers.enable_invert.mask = int_value & AXES_BITMASK;
                break;

            case Setting_LimitPinsInvertMask: // Reset to ensure change. Immediate re-init may cause problems.
                settings.limits.invert.mask = int_value & AXES_BITMASK;
                break;

            case Setting_InvertProbePin: // Reset to ensure change. Immediate re-init may cause problems.
                if(!hal.probe_configure_invert_mask)
                    return Status_SettingDisabled;
                settings.flags.invert_probe_pin = int_value != 0;
                hal.probe_configure_invert_mask(false, false);
                break;

            case Setting_StatusReportMask:
#if COMPATIBILITY_LEVEL <= 1
                settings.status_report.mask = int_value;
#else
                int_value &= 0b111;
                settings.status_report.mask = (settings.status_report.mask & ~0b111) | int_value;
#endif
                break;

            case Setting_JunctionDeviation:
                settings.junction_deviation = value;
                break;

            case Setting_ArcTolerance:
                settings.arc_tolerance = value;
                break;

            case Setting_ReportInches:
                settings.flags.report_inches = int_value != 0;
                report_init();
                system_flag_wco_change(); // Make sure WCO is immediately updated.
                break;

            case Setting_ControlInvertMask:
                settings.control_invert.mask = int_value;
                settings.control_invert.block_delete &= hal.driver_cap.block_delete;
                settings.control_invert.e_stop &= hal.driver_cap.e_stop;
                settings.control_invert.stop_disable &= hal.driver_cap.program_stop;
                settings.control_invert.probe_disconnected &= hal.driver_cap.probe_connected;
               break;

            case Setting_CoolantInvertMask:
                settings.coolant_invert.mask = int_value;
                break;

            case Setting_SpindleInvertMask:
                settings.spindle.invert.mask = int_value;
                if(settings.spindle.invert.pwm && !hal.driver_cap.spindle_pwm_invert) {
                    settings.spindle.invert.pwm = Off;
                    return Status_SettingDisabled;
                }
                break;

            case Setting_SpindleAtSpeedTolerance:
                settings.spindle.at_speed_tolerance = value;
                break;

            case Setting_ControlPullUpDisableMask:
                settings.control_disable_pullup.mask = int_value;
                settings.control_disable_pullup.block_delete &= hal.driver_cap.block_delete;
                settings.control_disable_pullup.e_stop &= hal.driver_cap.e_stop;
                settings.control_disable_pullup.stop_disable &= hal.driver_cap.program_stop;
                settings.control_invert.probe_disconnected &= hal.driver_cap.probe_connected;
                break;

            case Setting_LimitPullUpDisableMask:
                settings.limits.disable_pullup.mask = int_value;
                break;

            case Setting_ProbePullUpDisable:
                if(!hal.probe_configure_invert_mask)
                    return Status_SettingDisabled;
                settings.flags.disable_probe_pullup = int_value != 0;
                break;

            case Setting_SoftLimitsEnable:
                if (int_value && !settings.homing.flags.enabled)
                    return Status_SoftLimitError;
                settings.limits.flags.soft_enabled = int_value != 0;
                break;

            case Setting_HardLimitsEnable:
                settings.limits.flags.hard_enabled = bit_istrue(int_value, bit(0));
#if COMPATIBILITY_LEVEL <= 1
                settings.limits.flags.check_at_init = bit_istrue(int_value, bit(1));
#endif
                hal.limits_enable(settings.limits.flags.hard_enabled, false); // Change immediately. NOTE: Nice to have but could be problematic later.
                break;

            case Setting_JogSoftLimited:
                if (int_value && !settings.homing.flags.enabled)
                    return Status_SoftLimitError;
                settings.limits.flags.jog_soft_limited = int_value != 0;
                break;

            case Setting_RestoreOverrides:
                settings.flags.restore_overrides = int_value != 0;
                break;

            case Setting_IgnoreDoorWhenIdle:
                settings.flags.safety_door_ignore_when_idle = int_value != 0;
                break;

            case Setting_SleepEnable:
                settings.flags.sleep_enable = int_value != 0;
                break;

            case Setting_HoldActions:
                settings.flags.disable_laser_during_hold =  bit_istrue(int_value, bit(0));
                settings.flags.restore_after_feed_hold =  bit_istrue(int_value, bit(1));
                break;

            case Setting_ForceInitAlarm:
                settings.flags.force_initialization_alarm = int_value != 0;
                break;

            case Setting_ProbingFeedOverride:
                settings.flags.allow_probing_feed_override = int_value != 0;
                break;

            case Setting_HomingEnable:
                if (bit_istrue(int_value, bit(0))) {
#if COMPATIBILITY_LEVEL > 1
                    settings.homing.flags.enabled = On;
#else
                    settings.homing.flags.value = int_value & 0x0F;
                    settings.homing.flags.manual = bit_istrue(int_value, bit(5));
                    settings.limits.flags.two_switches = bit_istrue(int_value, bit(4));
#endif
                } else {
                    settings.homing.flags.value = 0;
                    settings.limits.flags.soft_enabled = Off; // Force disable soft-limits.
                    settings.limits.flags.jog_soft_limited = Off;
                }
                break;

            case Setting_HomingDirMask:
                settings.homing.dir_mask.value = int_value & AXES_BITMASK;
                break;

            case Setting_HomingFeedRate:
                settings.homing.feed_rate = value;
                break;

            case Setting_HomingSeekRate:
                settings.homing.seek_rate = value;
                break;

            case Setting_HomingDebounceDelay:
                settings.homing.debounce_delay = int_value;
                break;

            case Setting_HomingPulloff:
                settings.homing.pulloff = value;
                break;

            case Setting_EnableLegacyRTCommands:
                settings.flags.legacy_rt_commands = value != 0;
                break;

            case Setting_HomingLocateCycles:
                settings.homing.locate_cycles = int_value < 1 ? 1 :(int_value > 127 ? 127 : int_value);
                break;

            case Setting_HomingCycle_1:
            case Setting_HomingCycle_2:
            case Setting_HomingCycle_3:
            case Setting_HomingCycle_4:
            case Setting_HomingCycle_5:
            case Setting_HomingCycle_6:
                settings.homing.cycle[setting - Setting_HomingCycle_1].mask = int_value;
                limits_set_homing_axes();
                break;

            case Setting_G73Retract:
                settings.g73_retract = value;
                break;

            case Setting_PWMFreq:
                settings.spindle.pwm_freq = value;
                break;

            case Setting_RpmMax:
                settings.spindle.rpm_max = value;
                break;

            case Setting_RpmMin:
                settings.spindle.rpm_min = value;
                break;

            case Setting_Mode:
                switch(int_value) {
                    case 1:
                        if(!hal.driver_cap.variable_spindle)
                            return Status_SettingDisabledLaser;
                        settings.flags.laser_mode = On;
                        settings.flags.lathe_mode = Off;
                        break;

                     case 2:
                        settings.flags.laser_mode = Off;
                        settings.flags.lathe_mode = On;
                        break;

                     default:
                        settings.flags.laser_mode = Off;
                        settings.flags.lathe_mode = Off;
                        break;
                }
                if(!settings.flags.lathe_mode)
                    gc_state.modal.diameter_mode = false;
                break;

            case Setting_ParkingEnable:
                settings.parking.flags.value = bit_istrue(int_value, bit(0)) ? (int_value & 0x07) : 0;
                break;

            case Setting_ParkingAxis:
                settings.parking.axis = int_value;
                break;

            case Setting_ParkingPulloutIncrement:
                settings.parking.pullout_increment = value;
                break;

            case Setting_ParkingPulloutRate:
                settings.parking.pullout_rate = value;
                break;

            case Setting_ParkingTarget:
                settings.parking.target = value;
                break;

            case Setting_ParkingFastRate:
                settings.parking.rate = value;
                break;

            case Setting_PWMOffValue:
                settings.spindle.pwm_off_value = value;
                break;

            case Setting_PWMMinValue:
                settings.spindle.pwm_min_value = value;
                break;

            case Setting_PWMMaxValue:
                settings.spindle.pwm_max_value = value;
                break;

            case Setting_StepperDeenergizeMask:
                settings.steppers.deenergize.mask = int_value & AXES_BITMASK;
                break;

            case Setting_SpindlePPR:
                settings.spindle.ppr = int_value;
                break;

#ifdef ENABLE_SPINDLE_LINEARIZATION

            case Setting_LinearSpindlePiece1:
            case Setting_LinearSpindlePiece2:
            case Setting_LinearSpindlePiece3:
            case Setting_LinearSpindlePiece4:
                {
                    uint32_t idx = setting - Setting_LinearSpindlePiece1;
                    float rpm, start, end;

                    if(svalue[0] == '0' && svalue[1] == '\0') {
                        settings.spindle.pwm_piece[idx].rpm = NAN;
                        settings.spindle.pwm_piece[idx].start =
                        settings.spindle.pwm_piece[idx].end = 0.0f;
                    } else if(sscanf(svalue, "%f,%f,%f", &rpm, &start, &end) == 3) {
                        settings.spindle.pwm_piece[idx].rpm = rpm;
                        settings.spindle.pwm_piece[idx].start = start;
                        settings.spindle.pwm_piece[idx].end = end;
                    } else
                        return Status_InvalidStatement;
                }
                break;
#endif

#ifdef SPINDLE_RPM_CONTROLLED

            case Setting_SpindlePGain:
                settings.spindle.pid.p_gain = value;
                break;

            case Setting_SpindleIGain:
                settings.spindle.pid.i_gain = value;
                break;

            case Setting_SpindleDGain:
                settings.spindle.pid.d_gain = value;
                break;

            case Setting_SpindleMaxError:
                settings.spindle.pid.max_error = value;
                break;

            case Setting_SpindleIMaxError:
                settings.spindle.pid.i_max_error = value;
                break;

#endif

            case Setting_PositionPGain:
                settings.position.pid.p_gain = value;
                break;

            case Setting_PositionIGain:
                settings.position.pid.i_gain = value;
                break;

            case Setting_PositionDGain:
                settings.position.pid.d_gain = value;
                break;

            case Setting_PositionIMaxError:
                settings.position.pid.i_max_error = value;
                break;

            case Setting_ToolChangeMode:
                if(!hal.driver_cap.atc && hal.stream.suspend_read && int_value <= ToolChange_SemiAutomatic) {
                    settings.tool_change.mode = (toolchange_mode_t)int_value;
                    tc_init();
                } else
                    return Status_InvalidStatement;
                break;

            case Setting_ToolChangeProbingDistance:
                if(!hal.driver_cap.atc)
                    settings.tool_change.probing_distance = value;
                else
                    return Status_InvalidStatement;
                break;

            case Setting_ToolChangeFeedRate:
                settings.tool_change.feed_rate = value;
                break;

            case Setting_ToolChangeSeekRate:
                settings.tool_change.seek_rate = value;
                break;

            default:;
                status_code_t status = hal.driver_setting ? hal.driver_setting(setting, value, svalue) : Status_Unhandled;
                return status == Status_Unhandled ? Status_InvalidStatement : status;
        }
    }

    write_global_settings();
#ifdef ENABLE_BACKLASH_COMPENSATION
    mc_backlash_init();
#endif
    hal.settings_changed(&settings);

    return Status_OK;
}

// Initialize the config subsystem
void settings_init() {
    if(!read_global_settings()) {
        settings_restore_t settings = settings_all;
        settings.defaults = 1; // Ensure global settings get restored
        grbl.report.status_message(Status_SettingReadFail);
        settings_restore(settings); // Force restore all non-volatile storage data.
        report_init();
#if COMPATIBILITY_LEVEL <= 1
        report_grbl_settings(true);
#else
        report_grbl_settings(false);
#endif
    } else {
        memset(&tool_table, 0, sizeof(tool_data_t)); // First entry is for tools not in tool table
#ifdef N_TOOLS
        uint_fast8_t idx;
        for (idx = 1; idx <= N_TOOLS; idx++)
            settings_read_tool_data(idx, &tool_table[idx]);
#endif
        report_init();
#ifdef ENABLE_BACKLASH_COMPENSATION
        mc_backlash_init();
#endif
        hal.settings_changed(&settings);
        if(hal.probe_configure_invert_mask) // Initialize probe invert mask.
            hal.probe_configure_invert_mask(false, false);
    }
}
