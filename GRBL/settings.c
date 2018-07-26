/*
  settings.c - eeprom configuration handling
  Part of Grbl

  Copyright (c) 2017-2018 Terje Io
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

    .pulse_microseconds = DEFAULT_STEP_PULSE_MICROSECONDS,
    .pulse_delay_microseconds = DEFAULT_STEP_PULSE_DELAY,
    .stepper_idle_lock_time = DEFAULT_STEPPER_IDLE_LOCK_TIME,
    .step_invert.mask = DEFAULT_STEPPING_INVERT_MASK,
    .dir_invert.mask = DEFAULT_DIRECTION_INVERT_MASK,
    .status_report.mask = DEFAULT_STATUS_REPORT_MASK,
    .junction_deviation = DEFAULT_JUNCTION_DEVIATION,
    .arc_tolerance = DEFAULT_ARC_TOLERANCE,

    .homing_dir_mask = DEFAULT_HOMING_DIR_MASK,
    .homing_feed_rate = DEFAULT_HOMING_FEED_RATE,
    .homing_seek_rate = DEFAULT_HOMING_SEEK_RATE,
    .homing_debounce_delay = DEFAULT_HOMING_DEBOUNCE_DELAY,
    .homing_pulloff = DEFAULT_HOMING_PULLOFF,
    .homing_locate_cycles = N_HOMING_LOCATE_CYCLE,
    .g73_retract = DEFAULT_G73_RETRACT,

    .flags.report_inches = DEFAULT_REPORT_INCHES,
    .flags.laser_mode = DEFAULT_LASER_MODE,
    .flags.hard_limit_enable = DEFAULT_HARD_LIMIT_ENABLE,
    .flags.homing_enable = DEFAULT_HOMING_ENABLE,
    .flags.soft_limit_enable = DEFAULT_SOFT_LIMIT_ENABLE,
    .flags.invert_probe_pin = DEFAULT_INVERT_PROBE_PIN,

    .status_report.buffer_state = REPORT_FIELD_BUFFER_STATE,
    .status_report.line_numbers = REPORT_FIELD_LINE_NUMBERS,
    .status_report.feed_speed = REPORT_FIELD_CURRENT_FEED_SPEED,
    .status_report.pin_state = REPORT_FIELD_PIN_STATE,
    .status_report.work_coord_offset = REPORT_FIELD_WORK_COORD_OFFSET,
    .status_report.overrrides = REPORT_FIELD_OVERRIDES,

    .stepper_enable_invert.mask = INVERT_ST_ENABLE_MASK,
    .stepper_deenergize.mask = ST_DEENERGIZE_MASK,

    .limit_invert.mask = INVERT_LIMIT_PIN_MASK,
    .control_invert.mask = INVERT_CONTROL_PIN_MASK,
    .limit_disable_pullup.mask = DISABLE_LIMIT_PINS_PULL_UP_MASK,
    .flags.disable_probe_pullup = DISABLE_PROBE_PIN_PULL_UP,
    .control_disable_pullup.mask = DISABLE_CONTROL_PINS_PULL_UP_MASK,

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
    .homing_cycle[X_AXIS] = 0, // indexing references of these are a bit of a misnomer...
    .homing_cycle[Y_AXIS] = 0,
    .homing_cycle[Z_AXIS] = 0,

  #ifdef A_AXIS
    .steps_per_mm[A_AXIS] = DEFAULT_A_STEPS_PER_MM,
    .max_rate[A_AXIS] = DEFAULT_A_MAX_RATE,
    .acceleration[A_AXIS] = DEFAULT_A_ACCELERATION,
    .max_travel[A_AXIS] = (-DEFAULT_A_MAX_TRAVEL),
    .homing_cycle[A_AXIS] = 0,
  #endif
  #ifdef B_AXIS
    .steps_per_mm[B_AXIS] = DEFAULT_B_STEPS_PER_MM,
    .max_rate[B_AXIS] = DEFAULT_B_MAX_RATE,
    .acceleration[B_AXIS] = DEFAULT_B_ACCELERATION,
    .max_travel[B_AXIS] = (-DEFAULT_B_MAX_TRAVEL),
    .homing_cycle[B_AXIS] = 0,
  #endif
  #ifdef C_AXIS
    .steps_per_mm[C_AXIS] = DEFAULT_C_STEPS_PER_MM,
    .acceleration[C_AXIS] = DEFAULT_C_ACCELERATION,
    .max_rate[C_AXIS] = DEFAULT_C_MAX_RATE,
    .max_travel[C_AXIS] = (-DEFAULT_C_MAX_TRAVEL),
    .homing_cycle[C_AXIS] = 0,
  #endif

    .rpm_max = DEFAULT_SPINDLE_RPM_MAX,
    .rpm_min = DEFAULT_SPINDLE_RPM_MIN,

    .spindle_invert.on = INVERT_SPINDLE_ENABLE_PIN,
    .spindle_pwm_freq = DEFAULT_SPINDLE_PWM_FREQ,
    .spindle_pwm_off_value = DEFAULT_SPINDLE_PWM_OFF_VALUE,
    .spindle_pwm_min_value = DEFAULT_SPINDLE_PWM_MIN_VALUE,
    .spindle_pwm_max_value = DEFAULT_SPINDLE_PWM_MAX_VALUE,
    .spindle_ppr = DEFAULT_SPINDLE_PPR,
    .spindle_P_gain = 1.0f,
    .spindle_I_gain = 0.0f,
    .spindle_D_gain = 100.0f
};

// Write build info to persistent storage
void settings_write_build_info (char *line)
{
    if(hal.eeprom.type != EEPROM_None) {
        hal.eeprom.memcpy_to_with_checksum(EEPROM_ADDR_BUILD_INFO, (uint8_t *)line, MAX_STORED_LINE_LENGTH);
      #ifdef EMULATE_EEPROM
        if(hal.eeprom.type == EEPROM_Emulated)
            settings_dirty.build_info = settings_dirty.is_dirty = true;
      #endif
    }
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

    if(hal.eeprom.type != EEPROM_None) {
        hal.eeprom.memcpy_to_with_checksum(EEPROM_ADDR_STARTUP_BLOCK + idx * (MAX_STORED_LINE_LENGTH + 1), (uint8_t *)line, MAX_STORED_LINE_LENGTH);
      #ifdef EMULATE_EEPROM
        if(hal.eeprom.type == EEPROM_Emulated) {
            bit_true(settings_dirty.startup_lines, bit(idx));
            settings_dirty.is_dirty = true;
        }
      #endif
    }
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

    if(hal.eeprom.type != EEPROM_None) {
        hal.eeprom.memcpy_to_with_checksum(EEPROM_ADDR_PARAMETERS + idx * (sizeof(coord_data_t) + 1), (uint8_t *)coord_data, sizeof(coord_data_t));
      #ifdef EMULATE_EEPROM
        if(hal.eeprom.type == EEPROM_Emulated) {
            bit_true(settings_dirty.coord_data, bit(idx));
            settings_dirty.is_dirty = true;
        }
      #endif
    }
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
    if(hal.eeprom.type != EEPROM_None) {
        hal.eeprom.memcpy_to_with_checksum(EEPROM_ADDR_TOOL_TABLE + idx * (sizeof(tool_data_t) + 1), (uint8_t *)tool_data, sizeof(tool_data_t));
 #ifdef EMULATE_EEPROM
        if(hal.eeprom.type == EEPROM_Emulated) {
            bit_true(settings_dirty.tool_data, bit(idx));
            settings_dirty.is_dirty = true;
        }
 #endif
    }

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
      #ifdef EMULATE_EEPROM
        if(hal.eeprom.type == EEPROM_Emulated)
            settings_dirty.global_settings = settings_dirty.is_dirty = true;
      #endif
    }
}


// Restore Grbl global settings to defaults and write to persistent storage
void settings_restore (uint8_t restore_flag) {

    if (restore_flag & SETTINGS_RESTORE_DEFAULTS) {
        memcpy(&settings, &defaults, sizeof(settings_t));
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
}

// A helper method to set settings from command line
status_code_t settings_store_global_setting (uint_fast16_t parameter, float value) {

    if (value < 0.0f)
        return Status_NegativeValue;

    if ((setting_type_t)parameter >= Setting_AxisSettingsBase && (setting_type_t)parameter <= Setting_AxisSettingsMax) {
        // Store axis configuration. Axis numbering sequence set by AXIS_SETTING defines.
        // NOTE: Ensure the setting index corresponds to the report.c settings printout.
        uint_fast8_t set_idx = 0, axis_idx = (uint_fast8_t)parameter - (uint_fast8_t)Setting_AxisSettingsBase;

        while (set_idx < AXIS_N_SETTINGS) {

            if (axis_idx < N_AXIS) {
            // Valid axis setting found.
                switch ((axis_setting_type_t)set_idx) {

                    case AxisSetting_StepsPerMM:
                        #ifdef MAX_STEP_RATE_HZ
                        if (value * settings.max_rate[axis_idx] > (MAX_STEP_RATE_HZ * 60.0f))
                            return Status_MaxStepRateExceeded;
                        #endif
                        settings.steps_per_mm[axis_idx] = value;
                        break;

                    case AxisSetting_MaxRate:
                        #ifdef MAX_STEP_RATE_HZ
                        if (value * settings.steps_per_mm[axis_idx] > (MAX_STEP_RATE_HZ * 60.0f))
                            return Status_MaxStepRateExceeded;
                        #endif
                        settings.max_rate[axis_idx] = value;
                        break;

                    case AxisSetting_Acceleration:
                        settings.acceleration[axis_idx] = value * 60.0f * 60.0f; // Convert to mm/min^2 for grbl internal use.
                        break;

                    case AxisSetting_MaxTravel:
                        settings.max_travel[axis_idx] = -value; // Store as negative for grbl internal use.
                        break;

                    default: // for stopping compiler warning
                        break;

                }
                break; // Exit while-loop after setting has been configured and proceed to the EEPROM write call.

            } else {
                set_idx++;
                // If axis index greater than N_AXIS or setting index greater than number of axis settings, error out.
                if ((axis_idx < AXIS_SETTINGS_INCREMENT) || (set_idx == AXIS_N_SETTINGS)) {
                    if(hal.driver_setting && hal.driver_setting(parameter, value))
                        return Status_OK;
                    else
                        return Status_InvalidStatement;
                }
                axis_idx -= AXIS_SETTINGS_INCREMENT;
            }
        }
    } else {
        // Store non-axis Grbl settings
        uint8_t int_value = (uint8_t)truncf(value);
        switch((setting_type_t)parameter) {

            case Setting_PulseMicroseconds:
                if (int_value < 3)
                    return Status_SettingStepPulseMin;
                settings.pulse_microseconds = int_value;
                break;

            case Setting_PulseDelayMicroseconds:
                if(int_value > 0 && !hal.driver_cap.step_pulse_delay)
                    return Status_SettingDisabled;
                settings.pulse_delay_microseconds = int_value;
                break;

            case Setting_StepperIdleLockTime:
                settings.stepper_idle_lock_time = int_value;
                break;

            case Setting_StepInvertMask:
                settings.step_invert.mask = int_value & AXES_BITMASK;
                break;

            case Setting_DirInvertMask:
                settings.dir_invert.mask = int_value & AXES_BITMASK;
                break;

            case Setting_InvertStepperEnable: // Reset to ensure change. Immediate re-init may cause problems.
                settings.stepper_enable_invert.mask = int_value & AXES_BITMASK;
                break;

            case Setting_LimitPinsInvertMask: // Reset to ensure change. Immediate re-init may cause problems.
                settings.limit_invert.mask = int_value & AXES_BITMASK;
                break;

            case Setting_InvertProbePin: // Reset to ensure change. Immediate re-init may cause problems.
                settings.flags.invert_probe_pin = int_value !=0;
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
                system_flag_wco_change(); // Make sure WCO is immediately updated.
                break;

            case Setting_ControlInvertMask:
                settings.control_invert.mask = int_value;
                break;

            case Setting_CoolantInvertMask:
                settings.coolant_invert.mask = int_value;
                break;

            case Setting_SpindleInvertMask:
                settings.spindle_invert.mask = int_value;
                break;

            case Setting_ControlPullUpDisableMask:
                settings.control_disable_pullup.mask = int_value & 0x0F;
                break;

            case Setting_LimitPullUpDisableMask:
                settings.limit_disable_pullup.mask = int_value;
                break;

            case Setting_ProbePullUpDisable:
                settings.flags.disable_probe_pullup = int_value != 0;
                break;

            case Setting_SoftLimitsEnable:
                if (int_value && !settings.flags.homing_enable)
                    return Status_SoftLimitError;
                settings.flags.soft_limit_enable = int_value != 0;
                break;

            case Setting_HardLimitsEnable:
                settings.flags.hard_limit_enable = int_value != 0;
                hal.limits_enable(settings.flags.hard_limit_enable); // Change immediately. NOTE: Nice to have but could be problematic later.
                break;

            case Setting_HomingEnable:
                settings.flags.homing_enable = int_value != 0;
                if (!int_value)
                    settings.flags.soft_limit_enable = 0; // Force disable soft-limits.
                break;

            case Setting_HomingDirMask:
                settings.homing_dir_mask = int_value & AXES_BITMASK;
                break;

            case Setting_HomingFeedRate:
                settings.homing_feed_rate = value;
                break;

            case Setting_HomingSeekRate:
                settings.homing_seek_rate = value;
                break;

            case Setting_HomingDebounceDelay:
                settings.homing_debounce_delay = int_value;
                break;

            case Setting_HomingPulloff:
                settings.homing_pulloff = value;
                break;

            case Setting_HomingLocateCycles:
                settings.homing_locate_cycles = int_value < 1 ? 1 :(int_value > 127 ? 127 : int_value);
                break;

            case Setting_HomingCycle_1:
            case Setting_HomingCycle_2:
            case Setting_HomingCycle_3:
            case Setting_HomingCycle_4:
            case Setting_HomingCycle_5:
            case Setting_HomingCycle_6:
                settings.homing_cycle[parameter - Setting_HomingCycle_1] = int_value;
                break;

            case Setting_G73Retract:
                settings.g73_retract = value;
                break;

            case Setting_RpmMax:
                settings.rpm_max = value;
                // spindle_init();
                break; // Re-initialize spindle rpm calibration

            case Setting_RpmMin:
                settings.rpm_min = value;
                // spindle_init();
                break; // Re-initialize spindle rpm calibration

            case Setting_LaserMode:
                if(hal.driver_cap.variable_spindle)
                    settings.flags.laser_mode = int_value != 0;
                else
                    return Status_SettingDisabledLaser;
                break;

            case Setting_PWMFreq:
                settings.spindle_pwm_freq = value;
//              spindle_init();
                break; // Re-initialize spindle pwm calibration

            case Setting_PWMOffValue:
                settings.spindle_pwm_off_value = value;
//              spindle_init();
                break; // Re-initialize spindle pwm calibration

            case Setting_PWMMinValue:
                settings.spindle_pwm_min_value = value;
//              spindle_init();
                break; // Re-initialize spindle pwm calibration

            case Setting_PWMMaxValue:
                settings.spindle_pwm_max_value = value;
//              spindle_init();
                break; // Re-initialize spindle pwm calibration

            case Setting_StepperDeenergizeMask:
                settings.stepper_deenergize.mask = int_value & AXES_BITMASK;
                break;

            case Setting_SpindlePPR:
                settings.spindle_ppr = int_value;
                break;

            case Setting_SpindlePGain:
                settings.spindle_P_gain = value;
                break;

            case Setting_SpindleIGain:
                settings.spindle_I_gain = value;
                break;

            case Setting_SpindleDGain:
                settings.spindle_D_gain = value;
                break;

            default:
                if(hal.driver_setting && hal.driver_setting(parameter, value))
                    return Status_OK;
                else
                    return Status_InvalidStatement;
        }
    }

    write_global_settings();
    hal.settings_changed(&settings);

    return Status_OK;
}

// Initialize the config subsystem
void settings_init() {
    if(!read_global_settings()) {
        report_status_message(Status_SettingReadFail);
        settings_restore(SETTINGS_RESTORE_ALL); // Force restore all EEPROM data.
        report_grbl_settings();
    } else {
        memset(&tool_table, 0, sizeof(tool_data_t)); // First entry is for tools not in tool table
#ifdef N_TOOLS
        uint_fast8_t idx;
        for (idx = 1; idx <= N_TOOLS; idx++)
            settings_read_tool_data(idx, &tool_table[idx]);
#endif
        hal.settings_changed(&settings);
    }
}
