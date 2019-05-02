/*
  settings.c - eeprom configuration handling
  Part of Grbl

  Copyright (c) 2017-2019 Terje Io
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

#include "grbl.h"

settings_t settings;

const settings_t defaults = {

    .version = SETTINGS_VERSION,

    .stream = (stream_setting_t)DEFAULT_STREAM,
    .junction_deviation = DEFAULT_JUNCTION_DEVIATION,
    .arc_tolerance = DEFAULT_ARC_TOLERANCE,
    .g73_retract = DEFAULT_G73_RETRACT,

    .legacy_rt_commands = DEFAULT_LEGACY_RTCOMMANDS,

    .flags.report_inches = DEFAULT_REPORT_INCHES,
    .flags.laser_mode = DEFAULT_LASER_MODE,
    .flags.lathe_mode = DEFAULT_LATHE_MODE,
    .flags.invert_probe_pin = DEFAULT_INVERT_PROBE_PIN,
    .flags.sleep_enable = DEFAULT_SLEEP_ENABLE,
    .flags.disable_laser_during_hold = DEFAULT_DISABLE_LASER_DURING_HOLD,
    .flags.force_initialization_alarm = DEFAULT_FORCE_INITIALIZATION_ALARM,
    .flags.disable_probe_pullup = DISABLE_PROBE_PIN_PULL_UP,
    .flags.allow_probing_feed_override = ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES,
    .flags.limits_two_switches_on_axes = LIMITS_TWO_SWITCHES_ON_AXES,
    .flags.homing_single_axis_commands = HOMING_SINGLE_AXIS_COMMANDS,
    .flags.homing_force_set_origin = HOMING_FORCE_SET_ORIGIN,
    .flags.force_buffer_sync_on_wco_change = FORCE_BUFFER_SYNC_DURING_WCO_CHANGE,

    .steppers.pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS,
    .steppers.pulse_delay_microseconds = DEFAULT_STEP_PULSE_DELAY,
    .steppers.idle_lock_time = DEFAULT_STEPPER_IDLE_LOCK_TIME,
    .steppers.step_invert.mask = DEFAULT_STEPPING_INVERT_MASK,
    .steppers.dir_invert.mask = DEFAULT_DIRECTION_INVERT_MASK,
    .steppers.enable_invert.mask = INVERT_ST_ENABLE_MASK,
    .steppers.deenergize.mask = ST_DEENERGIZE_MASK,

    .homing.flags.enabled = DEFAULT_HOMING_ENABLE,
    .homing.flags.init_lock = DEFAULT_HOMING_INIT_LOCK,
    .homing.dir_mask.value = DEFAULT_HOMING_DIR_MASK,
    .homing.feed_rate = DEFAULT_HOMING_FEED_RATE,
    .homing.seek_rate = DEFAULT_HOMING_SEEK_RATE,
    .homing.debounce_delay = DEFAULT_HOMING_DEBOUNCE_DELAY,
    .homing.pulloff = DEFAULT_HOMING_PULLOFF,
    .homing.locate_cycles = DEFAULT_N_HOMING_LOCATE_CYCLE,
    .homing.cycle[X_AXIS] = 0, // indexing references of these are a bit of a misnomer...
    .homing.cycle[Y_AXIS] = 0,
    .homing.cycle[Z_AXIS] = 0,

    .status_report.buffer_state = REPORT_FIELD_BUFFER_STATE,
    .status_report.line_numbers = REPORT_FIELD_LINE_NUMBERS,
    .status_report.feed_speed = REPORT_FIELD_CURRENT_FEED_SPEED,
    .status_report.pin_state = REPORT_FIELD_PIN_STATE,
    .status_report.work_coord_offset = REPORT_FIELD_WORK_COORD_OFFSET,
    .status_report.overrrides = REPORT_FIELD_OVERRIDES,

    .limits.flags.hard_enabled = DEFAULT_HARD_LIMIT_ENABLE,
    .limits.flags.soft_enabled = DEFAULT_SOFT_LIMIT_ENABLE,
    .limits.flags.check_at_init = DEFAULT_CHECK_LIMITS_AT_INIT,
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
    .spindle.ppr = DEFAULT_SPINDLE_PPR,
    .spindle.pid.p_gain = DEFAULT_SPINDLE_P_GAIN,
    .spindle.pid.i_gain = DEFAULT_SPINDLE_I_GAIN,
    .spindle.pid.d_gain = DEFAULT_SPINDLE_D_GAIN,
    .spindle.pid.i_max_error = DEFAULT_SPINDLE_I_MAX,

    .steps_per_mm[X_AXIS] = DEFAULT_X_STEPS_PER_MM,
    .steps_per_mm[Y_AXIS] = DEFAULT_Y_STEPS_PER_MM,
    .steps_per_mm[Z_AXIS] = DEFAULT_Z_STEPS_PER_MM,
    .max_rate[X_AXIS] = DEFAULT_X_MAX_RATE,
    .max_rate[Y_AXIS] = DEFAULT_Y_MAX_RATE,
    .max_rate[Z_AXIS] = DEFAULT_Z_MAX_RATE,
    .acceleration[X_AXIS] = DEFAULT_X_ACCELERATION,
    .acceleration[Y_AXIS] = DEFAULT_Y_ACCELERATION,
    .acceleration[Z_AXIS] = DEFAULT_Z_ACCELERATION,
    .max_travel[X_AXIS] = (-DEFAULT_X_MAX_TRAVEL),
    .max_travel[Y_AXIS] = (-DEFAULT_Y_MAX_TRAVEL),
    .max_travel[Z_AXIS] = (-DEFAULT_Z_MAX_TRAVEL),

  #ifdef A_AXIS
    .steps_per_mm[A_AXIS] = DEFAULT_A_STEPS_PER_MM,
    .max_rate[A_AXIS] = DEFAULT_A_MAX_RATE,
    .acceleration[A_AXIS] = DEFAULT_A_ACCELERATION,
    .max_travel[A_AXIS] = (-DEFAULT_A_MAX_TRAVEL),
    .homing.cycle[A_AXIS] = 0,
  #endif
  #ifdef B_AXIS
    .steps_per_mm[B_AXIS] = DEFAULT_B_STEPS_PER_MM,
    .max_rate[B_AXIS] = DEFAULT_B_MAX_RATE,
    .acceleration[B_AXIS] = DEFAULT_B_ACCELERATION,
    .max_travel[B_AXIS] = (-DEFAULT_B_MAX_TRAVEL),
    .homing.cycle[B_AXIS] = 0,
  #endif
  #ifdef C_AXIS
    .steps_per_mm[C_AXIS] = DEFAULT_C_STEPS_PER_MM,
    .acceleration[C_AXIS] = DEFAULT_C_ACCELERATION,
    .max_rate[C_AXIS] = DEFAULT_C_MAX_RATE,
    .max_travel[C_AXIS] = (-DEFAULT_C_MAX_TRAVEL),
    .homing.cycle[C_AXIS] = 0,
  #endif

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
    if(hal.eeprom.type != EEPROM_None)
        hal.eeprom.memcpy_to_with_checksum(EEPROM_ADDR_BUILD_INFO, (uint8_t *)line, MAX_STORED_LINE_LENGTH);
}

// Read build info from persistent storage.
bool settings_read_build_info(char *line)
{
    if (!(hal.eeprom.type != EEPROM_None && hal.eeprom.memcpy_from_with_checksum((uint8_t *)line, EEPROM_ADDR_BUILD_INFO, MAX_STORED_LINE_LENGTH))) {
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

#ifdef FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE
    protocol_buffer_synchronize(); // A startup line may contain a motion and be executing.
#endif

    if(hal.eeprom.type != EEPROM_None)
        hal.eeprom.memcpy_to_with_checksum(EEPROM_ADDR_STARTUP_BLOCK + idx * (MAX_STORED_LINE_LENGTH + 1), (uint8_t *)line, MAX_STORED_LINE_LENGTH);
}

// Read startup line to persistent storage.
bool settings_read_startup_line (uint8_t idx, char *line)
{
    assert(idx < N_STARTUP_LINE);

    if (!(hal.eeprom.type != EEPROM_None && hal.eeprom.memcpy_from_with_checksum((uint8_t *)line, EEPROM_ADDR_STARTUP_BLOCK + idx * (MAX_STORED_LINE_LENGTH + 1), MAX_STORED_LINE_LENGTH))) {
        // Reset line with default value
        *line = '\0'; // Empty line
        settings_write_startup_line(idx, line);
        return false;
    }
    return true;
}

// Write selected coordinate data to persistent storage.
void settings_write_coord_data (uint8_t idx, float (*coord_data)[N_AXIS])
{
    assert(idx <= SETTING_INDEX_NCOORD);

#ifdef FORCE_BUFFER_SYNC_DURING_EEPROM_WRITE
    protocol_buffer_synchronize();
#endif

    if(hal.eeprom.type != EEPROM_None)
        hal.eeprom.memcpy_to_with_checksum(EEPROM_ADDR_PARAMETERS + idx * (sizeof(coord_data_t) + 1), (uint8_t *)coord_data, sizeof(coord_data_t));
}

// Read selected coordinate data from persistent storage.
bool settings_read_coord_data (uint8_t idx, float (*coord_data)[N_AXIS])
{
    assert(idx <= SETTING_INDEX_NCOORD);

    if (!(hal.eeprom.type != EEPROM_None && hal.eeprom.memcpy_from_with_checksum((uint8_t *)coord_data, EEPROM_ADDR_PARAMETERS + idx * (sizeof(coord_data_t) + 1), sizeof(coord_data_t)))) {
        // Reset with default zero vector
        clear_coord_data(coord_data);
        memset(coord_data, 0, sizeof(coord_data_t));
        settings_write_coord_data(idx, coord_data);
        return false;
    }
    return true;
}

// Write selected tool data to persistent storage.
bool settings_write_tool_data (uint8_t idx, tool_data_t *tool_data)
{
#ifdef N_TOOLS
    assert(idx > 0 && idx <= N_TOOLS); // NOTE: idx 0 is a non-persistent entry for tools not in tool table

    idx--;
    if(hal.eeprom.type != EEPROM_None)
        hal.eeprom.memcpy_to_with_checksum(EEPROM_ADDR_TOOL_TABLE + idx * (sizeof(tool_data_t) + 1), (uint8_t *)tool_data, sizeof(tool_data_t));

    return true;
#else
    return false;
#endif
}

// Read selected tool data from persistent storage.
bool settings_read_tool_data (uint8_t idx, tool_data_t *tool_data)
{
#ifdef N_TOOLS
    assert(idx > 0 && idx <= N_TOOLS); // NOTE: idx 0 is a non-persistent entry for tools not in tool table

    idx--;
    if (!(hal.eeprom.type != EEPROM_None && hal.eeprom.memcpy_from_with_checksum((uint8_t *)tool_data, EEPROM_ADDR_TOOL_TABLE + idx * (sizeof(tool_data_t) + 1), sizeof(tool_data_t))))
        memset(tool_data, 0, sizeof(tool_data_t));

    return tool_data->tool == idx + 1;
#else
    return false;
#endif
}

// Read Grbl global settings from persistent storage.
bool read_global_settings ()
{
    // Check version-byte of eeprom
    return hal.eeprom.type != EEPROM_None && SETTINGS_VERSION == hal.eeprom.get_byte(0) && hal.eeprom.memcpy_from_with_checksum((uint8_t *)&settings, EEPROM_ADDR_GLOBAL, sizeof(settings_t));
}

// Write Grbl global settings and version number to persistent storage
void write_global_settings ()
{
    if(hal.eeprom.type != EEPROM_None) {
        hal.eeprom.put_byte(0, SETTINGS_VERSION);
        hal.eeprom.memcpy_to_with_checksum(EEPROM_ADDR_GLOBAL, (uint8_t *)&settings, sizeof(settings_t));
    }
}


// Restore Grbl global settings to defaults and write to persistent storage
void settings_restore (uint8_t restore_flag) {

    if (restore_flag & SETTINGS_RESTORE_DEFAULTS) {
        memcpy(&settings, &defaults, sizeof(settings_t));

        // Sanity check for default stream assignment
        if(settings.stream != StreamSetting_Serial)
          switch(settings.stream) {

            case StreamSetting_Bluetooth:
                if(!hal.driver_cap.bluetooth)
                    settings.stream = StreamSetting_Serial;
                break;

            case StreamSetting_Ethernet:
                if(!hal.driver_cap.ethernet)
                    settings.stream = StreamSetting_Serial;
                break;

            case StreamSetting_WiFi:
                if(!hal.driver_cap.wifi)
                    settings.stream = StreamSetting_Serial;
                break;

            default:
                settings.stream = StreamSetting_Serial;
        }

        write_global_settings();
    }

    if (restore_flag & SETTINGS_RESTORE_PARAMETERS) {
        uint_fast8_t idx;
        float coord_data[N_AXIS];

        memset(coord_data, 0, sizeof(coord_data));
        for (idx = 0; idx <= SETTING_INDEX_NCOORD; idx++)
            settings_write_coord_data(idx, &coord_data);

        settings_write_coord_data(SETTING_INDEX_G92, &coord_data); // Clear G92 offsets

#ifdef N_TOOLS
        tool_data_t tool_data;
        memset(&tool_data, 0, sizeof(tool_data_t));
        for (idx = 1; idx <= N_TOOLS; idx++)
            settings_write_tool_data(idx, &tool_data);
#endif
    }

    if (hal.eeprom.type != EEPROM_None && (restore_flag & SETTINGS_RESTORE_STARTUP_LINES)) {
      #if N_STARTUP_LINE > 0
        hal.eeprom.put_byte(EEPROM_ADDR_STARTUP_BLOCK, 0);
      #endif
      #if N_STARTUP_LINE > 1
        hal.eeprom.put_byte(EEPROM_ADDR_STARTUP_BLOCK + (MAX_STORED_LINE_LENGTH + 1), 0);
      #endif
    }

    if ((restore_flag & SETTINGS_RESTORE_BUILD_INFO) && hal.eeprom.type != EEPROM_None)
        hal.eeprom.put_byte(EEPROM_ADDR_BUILD_INFO , 0);

    if((restore_flag & SETTINGS_RESTORE_DRIVER_PARAMETERS) && hal.driver_settings_restore)
        hal.driver_settings_restore(restore_flag);

    eeprom_emu_sync_physical();
}

// A helper method to set settings from command line
status_code_t settings_store_global_setting (uint_fast16_t parameter, char *svalue)
{
    uint_fast8_t set_idx = 0;
    float value;

    if (!read_float(svalue, &set_idx, &value))
        return hal.driver_setting && hal.driver_setting(parameter, NAN, svalue) ? Status_OK : Status_BadNumberFormat;

    if (svalue[set_idx] != '\0')
        return Status_InvalidStatement;

    if (value < 0.0f)
        return Status_NegativeValue;

    if ((setting_type_t)parameter >= Setting_AxisSettingsBase && (setting_type_t)parameter <= Setting_AxisSettingsMax) {
        // Store axis configuration. Axis numbering sequence set by AXIS_SETTING defines.
        // NOTE: Ensure the setting index corresponds to the report.c settings printout.
        bool found = false;
        uint_fast16_t base_idx = (uint_fast16_t)parameter - (uint_fast16_t)Setting_AxisSettingsBase;
        uint_fast8_t axis_idx = base_idx % AXIS_SETTINGS_INCREMENT;

        if(axis_idx < N_AXIS) switch((base_idx - axis_idx) / AXIS_SETTINGS_INCREMENT) {

            case AxisSetting_StepsPerMM:
                #ifdef MAX_STEP_RATE_HZ
                if (value * settings.max_rate[axis_idx] > (MAX_STEP_RATE_HZ * 60.0f))
                    return Status_MaxStepRateExceeded;
                #endif
                found = true;
                settings.steps_per_mm[axis_idx] = value;
                break;

            case AxisSetting_MaxRate:
                #ifdef MAX_STEP_RATE_HZ
                if (value * settings.steps_per_mm[axis_idx] > (MAX_STEP_RATE_HZ * 60.0f))
                    return Status_MaxStepRateExceeded;
                #endif
                found = true;
                settings.max_rate[axis_idx] = value;
                break;

            case AxisSetting_Acceleration:
                found = true;
                settings.acceleration[axis_idx] = value * 60.0f * 60.0f; // Convert to mm/min^2 for grbl internal use.
                break;

            case AxisSetting_MaxTravel:
                found = true;
                settings.max_travel[axis_idx] = -value; // Store as negative for grbl internal use.
                break;

#ifdef ENABLE_BACKLASH_COMPENSATION
            case AxisSetting_Backlash:
                found = true;
                settings.backlash[axis_idx] = value;
                break;
#endif

            default: // for stopping compiler warning
                break;
        }

        if(!(found || (hal.driver_setting && hal.driver_setting(parameter, value, svalue))))
            return Status_InvalidStatement;

    } else {
        // Store non-axis Grbl settings
        uint8_t int_value = (uint8_t)truncf(value);
        switch((setting_type_t)parameter) {

            case Setting_PulseMicroseconds:
                if (int_value < 3)
                    return Status_SettingStepPulseMin;
                settings.steppers.pulse_microseconds = int_value;
                break;

            case Setting_PulseDelayMicroseconds:
                if(int_value > 0 && !hal.driver_cap.step_pulse_delay)
                    return Status_SettingDisabled;
                settings.steppers.pulse_delay_microseconds = int_value;
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
                hal.probe_configure_invert_mask(false);
                break;

            case Setting_StatusReportMask:
                settings.status_report.mask = int_value;
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

            case Setting_ControlPullUpDisableMask:
                settings.control_disable_pullup.mask = int_value & 0x0F;
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

            case Setting_JogSoftLimited:
                if (int_value && !settings.homing.flags.enabled)
                    return Status_SoftLimitError;
                settings.limits.flags.jog_soft_limited = int_value != 0;
                break;

            case Setting_HardLimitsEnable:
                settings.limits.flags.hard_enabled = int_value != 0;
                hal.limits_enable(settings.limits.flags.hard_enabled, false); // Change immediately. NOTE: Nice to have but could be problematic later.
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

            case Setting_DisableLaserDuringHold:
                settings.flags.disable_laser_during_hold = int_value != 0;
                break;

            case Setting_ForceInitAlarm:
                settings.flags.force_initialization_alarm = int_value != 0;
                break;

            case Setting_CheckLimitsAtInit:
                settings.limits.flags.check_at_init = int_value != 0;
                break;

            case Setting_HomingInitLock:
                settings.homing.flags.init_lock = int_value != 0;
                break;

            case Setting_HomingEnable:
                if (!(settings.homing.flags.enabled = int_value != 0)) {
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
                settings.legacy_rt_commands = value != 0;
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
                settings.homing.cycle[parameter - Setting_HomingCycle_1].mask = int_value;
                limits_set_homing_axes();
                break;

            case Setting_G73Retract:
                settings.g73_retract = value;
                break;

            case Setting_RpmMax:
                settings.spindle.rpm_max = value;
                break;

            case Setting_RpmMin:
                settings.spindle.rpm_min = value;
                break;

            case Setting_LaserMode:
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
                break;

            case Setting_PWMFreq:
                settings.spindle.pwm_freq = value;
                break;

/* disabled for now - no clear use case for this value, and original code handling this is badly implemented (IMO)
            case Setting_PWMOffValue:
                settings.spindle.pwm_off_value = value;
                break;
*/

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

            case Settings_Stream:

                switch((stream_setting_t)int_value) {

                    case StreamSetting_Serial:
                        // Always available?
                        break;

                    case StreamSetting_Bluetooth:
                        if(!hal.driver_cap.bluetooth)
                            return Status_InvalidStatement;
                        break;

                    case StreamSetting_Ethernet:
                        if(!hal.driver_cap.ethernet)
                            return Status_InvalidStatement;
                        break;

                    case StreamSetting_WiFi:
                        if(!hal.driver_cap.wifi)
                            return Status_InvalidStatement;
                        break;

                    default:
                        return Status_InvalidStatement;
                }

                settings.stream = (stream_setting_t)int_value;
                break;

            default:
                if(hal.driver_setting && hal.driver_setting(parameter, value, svalue))
                    return Status_OK;
                else
                    return Status_InvalidStatement;
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
        hal.report.status_message(Status_SettingReadFail);
        settings_restore(SETTINGS_RESTORE_ALL); // Force restore all EEPROM data.
        report_init();
        report_grbl_settings();
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
    }

    // Set these here until they get a setting id assigned (otherwhise an EEPROM reset is required...)
    settings.flags.allow_probing_feed_override = ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES;
    settings.flags.limits_two_switches_on_axes = LIMITS_TWO_SWITCHES_ON_AXES;
    settings.flags.homing_single_axis_commands = HOMING_SINGLE_AXIS_COMMANDS;
    settings.flags.homing_force_set_origin = HOMING_FORCE_SET_ORIGIN;
    settings.flags.force_buffer_sync_on_wco_change = FORCE_BUFFER_SYNC_DURING_WCO_CHANGE;
}
