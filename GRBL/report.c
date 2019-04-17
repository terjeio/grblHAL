/*
  report.c - reporting and messaging methods
  Part of Grbl

  Copyright (c) 2017-2019 Terje Io
  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

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

/*
  This file functions as the primary feedback interface for Grbl. Any outgoing data, such
  as the protocol status messages, feedback messages, and status reports, are stored here.
  For the most part, these functions primarily are called from protocol.c methods. If a
  different style feedback is desired (i.e. JSON), then a user can change these following
  methods to accomodate their needs.
*/

#include <stdarg.h>

#include "grbl.h"

static char buf[(STRLEN_COORDVALUE + 1) * N_AXIS];
static char *(*get_axis_values)(float *axis_values);
static char *(*get_rate_value)(float value);

// Append a number of strings to the static buffer
// NOTE: do NOT use for several int/float conversions as these share the same underlying buffer!
char *appendbuf (int argc, ...)
{
    char c, *s = buf, *arg;

    va_list list;
    va_start(list, argc);

    while(argc--) {
        arg = va_arg(list, char *);
        do {
            c = *s++ = *arg++;
        } while(c);
        s--;
    }

    va_end(list);

    return buf;
}

// Convert axis position values to null terminated string (mm).
static char *get_axis_values_mm (float *axis_values)
{
    uint_fast32_t idx;

    buf[0] = '\0';

    for (idx = 0; idx < N_AXIS; idx++) {
        if(gc_state.modal.diameter_mode && idx == X_AXIS)
            strcat(buf, ftoa(axis_values[idx] * 2.0f, N_DECIMAL_COORDVALUE_MM));
        else
            strcat(buf, ftoa(axis_values[idx], N_DECIMAL_COORDVALUE_MM));
        if (idx < (N_AXIS - 1))
            strcat(buf, ",");
    }

    return buf;
}

// Convert axis position values to null terminated string (inch).
static char *get_axis_values_inches (float *axis_values)
{
    uint_fast32_t idx;

    buf[0] = '\0';

    for (idx = 0; idx < N_AXIS; idx++) {
        strcat(buf, ftoa(axis_values[idx] * INCH_PER_MM, N_DECIMAL_COORDVALUE_INCH));
        if (idx < (N_AXIS - 1))
            strcat(buf, ",");
    }

    return buf;
}

// Convert rate value to null terminated string (mm).
static char *get_rate_value_mm (float value)
{
    return ftoa(value, N_DECIMAL_RATEVALUE_MM);
}

// Convert rate value to null terminated string (mm).
static char *get_rate_value_inch (float value)
{
    return ftoa(value * INCH_PER_MM, N_DECIMAL_RATEVALUE_INCH);
}

// Convert axes signals bits to string representation
// NOTE: returns pointer to null terminator!
static inline char *axis_signals_tostring (char *buf, axes_signals_t signals)
{
    if(signals.x)
        *buf++ = 'X';

    if(signals.y)
        *buf++ = 'Y';

    if (signals.z)
        *buf++ = 'Z';

#ifdef A_AXIS
    if (signals.a)
        *buf++ = 'A';
#endif

#ifdef B_AXIS
    if (signals.b)
        *buf++ = 'B';
#endif

#ifdef C_AXIS
    if (signals.c)
        *buf++ = 'C';
#endif

    *buf = '\0';

    return buf;
}

void report_init (void)
{
    get_axis_values = settings.flags.report_inches ? get_axis_values_inches : get_axis_values_mm;
    get_rate_value = settings.flags.report_inches ? get_rate_value_inch : get_rate_value_mm;
}

// Handles the primary confirmation protocol response for streaming interfaces and human-feedback.
// For every incoming line, this method responds with an 'ok' for a successful command or an
// 'error:'  to indicate some error event with the line or some critical system error during
// operation. Errors events can originate from the g-code parser, settings module, or asynchronously
// from a critical error, such as a triggered hard limit. Interface should always monitor for these
// responses.
void report_status_message (status_code_t status_code)
{
    switch(status_code) {

        case Status_OK: // STATUS_OK
            hal.stream.write("ok\r\n");
            break;

        default:
            hal.stream.write(appendbuf(3, "error:", uitoa((uint32_t)status_code), "\r\n"));
            break;
    }
}


// Prints alarm messages.
void report_alarm_message (alarm_code_t alarm_code)
{
    hal.stream.write_all(appendbuf(3, "ALARM:", uitoa((uint32_t)alarm_code), "\r\n"));
    hal.delay_ms(500, 0); // Force delay to ensure message clears output stream buffer.
}

// Prints feedback messages. This serves as a centralized method to provide additional
// user feedback for things that are not of the status/alarm message protocol. These are
// messages such as setup warnings, switch toggling, and how to exit alarms.
// NOTE: For interfaces, messages are always placed within brackets. And if silent mode
// is installed, the message number codes are less than zero.
void report_feedback_message(message_code_t message_code)
{
    hal.stream.write_all("[MSG:");

    switch(message_code) {

        case Message_CriticalEvent:
            hal.stream.write_all("Reset to continue");
            break;

        case Message_AlarmLock:
            hal.stream.write_all("'$H'|'$X' to unlock");
            break;

        case Message_AlarmUnlock:
            hal.stream.write_all("Caution: Unlocked");
            break;

        case Message_Enabled:
            hal.stream.write_all("Enabled");
            break;

        case Message_Disabled:
            hal.stream.write_all("Disabled");
            break;

        case Message_SafetyDoorAjar:
            hal.stream.write_all("Check Door");
            break;

        case Message_CheckLimits:
            hal.stream.write_all("Check Limits");
            break;

        case Message_ProgramEnd:
            hal.stream.write_all("Pgm End");
            break;

        case Message_RestoreDefaults:
            hal.stream.write_all("Restoring defaults");
            break;

        case Message_SpindleRestore:
            hal.stream.write_all("Restoring spindle");
            break;

        case Message_SleepMode:
            hal.stream.write_all("Sleeping");
            break;

        case Message_EStop:
            hal.stream.write_all("Emergency stop");
            break;

        default:
            if(hal.driver_feedback_message)
                hal.driver_feedback_message(hal.stream.write_all);
            break;
    }

    hal.stream.write_all("]\r\n");
}


// Welcome message
void report_init_message (void)
{
    hal.stream.write("\r\nGrblHAL " GRBL_VERSION " ['$' for help]\r\n");
}

// Grbl help message
void report_grbl_help (void)
{
    hal.stream.write("[HLP:$$ $# $G $I $N $x=val $Nx=line $J=line $SLP $C $X $H $B ~ ! ? ctrl-x]\r\n");
}


// Grbl settings print out.

void report_uint_setting (setting_type_t n, uint32_t val)
{
    hal.stream.write(appendbuf(3, "$", uitoa((uint32_t)n), "="));
    hal.stream.write(appendbuf(2, uitoa(val), "\r\n"));
}


void report_float_setting (setting_type_t n, float val, uint8_t n_decimal)
{
    hal.stream.write(appendbuf(3, "$", uitoa((uint32_t)n), "="));
    hal.stream.write(appendbuf(2, ftoa(val, n_decimal), "\r\n"));
}


void report_grbl_settings (void)
{
    // Print Grbl settings.
    report_uint_setting(Setting_PulseMicroseconds, settings.steppers.pulse_microseconds);
    report_uint_setting(Setting_StepperIdleLockTime, settings.steppers.idle_lock_time);
    report_uint_setting(Setting_StepInvertMask, settings.steppers.step_invert.mask);
    report_uint_setting(Setting_DirInvertMask, settings.steppers.dir_invert.mask);
    report_uint_setting(Setting_InvertStepperEnable, settings.steppers.enable_invert.mask);
    report_uint_setting(Setting_LimitPinsInvertMask, settings.limits.invert.mask);
    if(hal.probe_configure_invert_mask)
        report_uint_setting(Setting_InvertProbePin, settings.flags.invert_probe_pin);
    report_uint_setting(Setting_StatusReportMask, settings.status_report.mask);
    report_float_setting(Setting_JunctionDeviation, settings.junction_deviation, N_DECIMAL_SETTINGVALUE);
    report_float_setting(Setting_ArcTolerance, settings.arc_tolerance, N_DECIMAL_SETTINGVALUE);
    report_uint_setting(Setting_ReportInches, settings.flags.report_inches);
    report_uint_setting(Setting_ControlInvertMask, settings.control_invert.mask);
    report_uint_setting(Setting_CoolantInvertMask, settings.coolant_invert.mask);
    report_uint_setting(Setting_SpindleInvertMask, settings.spindle.invert.mask);
    report_uint_setting(Setting_ControlPullUpDisableMask, settings.control_disable_pullup.mask);
    report_uint_setting(Setting_LimitPullUpDisableMask, settings.limits.disable_pullup.mask);
    if(hal.probe_configure_invert_mask)
        report_uint_setting(Setting_ProbePullUpDisable, settings.flags.disable_probe_pullup);
    report_uint_setting(Setting_SoftLimitsEnable, settings.limits.flags.soft_enabled);
    report_uint_setting(Setting_HardLimitsEnable, settings.limits.flags.hard_enabled);
    report_uint_setting(Setting_HomingEnable, settings.homing.flags.enabled);
    report_uint_setting(Setting_HomingDirMask, settings.homing.dir_mask);
    report_float_setting(Setting_HomingFeedRate, settings.homing.feed_rate, N_DECIMAL_SETTINGVALUE);
    report_float_setting(Setting_HomingSeekRate, settings.homing.seek_rate, N_DECIMAL_SETTINGVALUE);
    report_uint_setting(Setting_HomingDebounceDelay, settings.homing.debounce_delay);
    report_float_setting(Setting_HomingPulloff, settings.homing.pulloff, N_DECIMAL_SETTINGVALUE);
    report_float_setting(Setting_G73Retract, settings.g73_retract, N_DECIMAL_SETTINGVALUE);
    report_uint_setting(Setting_PulseDelayMicroseconds, settings.steppers.pulse_delay_microseconds);
    report_float_setting(Setting_RpmMax, settings.spindle.rpm_max, N_DECIMAL_RPMVALUE);
    report_float_setting(Setting_RpmMin, settings.spindle.rpm_min, N_DECIMAL_RPMVALUE);
    report_uint_setting(Setting_LaserMode, hal.driver_cap.variable_spindle ? settings.flags.laser_mode : 0);
    report_float_setting(Setting_PWMFreq, settings.spindle.pwm_freq, N_DECIMAL_SETTINGVALUE);
//    report_float_setting(Setting_PWMOffValue, settings.spindle.pwm_off_value, N_DECIMAL_SETTINGVALUE);
    report_float_setting(Setting_PWMMinValue, settings.spindle.pwm_min_value, N_DECIMAL_SETTINGVALUE);
    report_float_setting(Setting_PWMMaxValue, settings.spindle.pwm_max_value, N_DECIMAL_SETTINGVALUE);
    report_uint_setting(Setting_StepperDeenergizeMask, settings.steppers.deenergize.mask);
    if(hal.driver_cap.spindle_sync || hal.driver_cap.spindle_pid)
        report_uint_setting(Setting_SpindlePPR, settings.spindle.ppr);
    report_uint_setting(Setting_LatheMode, settings.flags.lathe_mode);

    report_uint_setting(Setting_HomingLocateCycles, settings.homing.locate_cycles);

    uint_fast8_t idx;

    for(idx = 0 ; idx < N_AXIS ; idx++)
        report_uint_setting((setting_type_t)(Setting_HomingCycle_1 + idx), settings.homing.cycle[idx]);

    if(hal.driver_settings_report)
        hal.driver_settings_report(false, (axis_setting_type_t)0, 0);

    report_uint_setting(Setting_RestoreOverrides, settings.flags.restore_overrides);
    report_uint_setting(Setting_IgnoreDoorWhenIdle, settings.flags.safety_door_ignore_when_idle);
    report_uint_setting(Setting_SleepEnable, settings.flags.sleep_enable);
    report_uint_setting(Setting_DisableLaserDuringHold, settings.flags.disable_laser_during_hold);
    report_uint_setting(Setting_ForceInitAlarm, settings.flags.force_initialization_alarm);
    report_uint_setting(Setting_CheckLimitsAtInit, settings.limits.flags.check_at_init);
    report_uint_setting(Setting_HomingInitLock, settings.homing.flags.init_lock);
    report_uint_setting(Settings_Stream, (uint32_t)settings.stream);

    if(hal.driver_cap.spindle_pid) {
        report_float_setting(Setting_SpindlePGain, settings.spindle.pid.p_gain, N_DECIMAL_SETTINGVALUE);
        report_float_setting(Setting_SpindleIGain, settings.spindle.pid.i_gain, N_DECIMAL_SETTINGVALUE);
        report_float_setting(Setting_SpindleDGain, settings.spindle.pid.d_gain, N_DECIMAL_SETTINGVALUE);
        report_float_setting(Setting_SpindleMaxError, settings.spindle.pid.max_error, N_DECIMAL_SETTINGVALUE);
        report_float_setting(Setting_SpindleIMaxError, settings.spindle.pid.i_max_error, N_DECIMAL_SETTINGVALUE);
    }

    if(hal.driver_cap.spindle_sync) {
        report_float_setting(Setting_PositionPGain, settings.position.pid.p_gain, N_DECIMAL_SETTINGVALUE);
        report_float_setting(Setting_PositionIGain, settings.position.pid.i_gain, N_DECIMAL_SETTINGVALUE);
        report_float_setting(Setting_PositionDGain, settings.position.pid.d_gain, N_DECIMAL_SETTINGVALUE);
        report_float_setting(Setting_PositionIMaxError, settings.position.pid.i_max_error, N_DECIMAL_SETTINGVALUE);
    }

    // Print axis settings
    uint_fast8_t set_idx, val = (uint_fast8_t)Setting_AxisSettingsBase;
    uint_fast8_t max_set = hal.driver_settings_report ? AXIS_SETTINGS_INCREMENT : AXIS_N_SETTINGS;
    for (set_idx = 0; set_idx < max_set; set_idx++) {

        for (idx = 0; idx < N_AXIS; idx++) {

            switch ((axis_setting_type_t)set_idx) {

                case AxisSetting_StepsPerMM:
                    report_float_setting((setting_type_t)(val + idx), settings.steps_per_mm[idx], N_DECIMAL_SETTINGVALUE);
                    break;

                case AxisSetting_MaxRate:
                    report_float_setting((setting_type_t)(val + idx), settings.max_rate[idx], N_DECIMAL_SETTINGVALUE);
                    break;

                case AxisSetting_Acceleration:
                    report_float_setting((setting_type_t)(val + idx), settings.acceleration[idx] / (60.0f * 60.0f), N_DECIMAL_SETTINGVALUE);
                    break;

                case AxisSetting_MaxTravel:
                    report_float_setting((setting_type_t)(val + idx), -settings.max_travel[idx], N_DECIMAL_SETTINGVALUE);
                    break;

                default:
                    if(hal.driver_settings_report)
                        hal.driver_settings_report(true, (axis_setting_type_t)set_idx, idx);
                    break;
            }
        }
        val += AXIS_SETTINGS_INCREMENT;
    }
}


// Prints current probe parameters. Upon a probe command, these parameters are updated upon a
// successful probe or upon a failed probe with the G38.3 without errors command (if supported).
// These values are retained until Grbl is power-cycled, whereby they will be re-zeroed.
void report_probe_parameters (void)
{
    // Report in terms of machine position.
    float print_position[N_AXIS];
    system_convert_array_steps_to_mpos(print_position, sys_probe_position);
    hal.stream.write("[PRB:");
    hal.stream.write(get_axis_values(print_position));
    hal.stream.write(sys.probe_succeeded ? ":1" : ":0");
    hal.stream.write("]\r\n");
}


// Prints Grbl NGC parameters (coordinate offsets, probing, tool table)
void report_ngc_parameters (void)
{
    float coord_data[N_AXIS];
    uint_fast8_t idx, g5x;

    if(gc_state.modal.scaling_active) {
        hal.stream.write("[G51:");
        hal.stream.write(get_axis_values(gc_get_scaling()));
        hal.stream.write("]\r\n");
    }

    for (idx = 0; idx < SETTING_INDEX_NCOORD; idx++) {

        if (!(settings_read_coord_data(idx, &coord_data))) {
            hal.report.status_message(Status_SettingReadFail);
            return;
        }

        hal.stream.write("[G");

        switch (idx) {

            case SETTING_INDEX_G28:
                hal.stream.write("28");
                break;
            case SETTING_INDEX_G30:
                hal.stream.write("30");
                break;
            default:
                g5x = idx + 54;
                hal.stream.write(uitoa((uint32_t)(g5x > 59 ? 59 : g5x)));
                if(g5x > 59) {
                    hal.stream.write(".");
                    hal.stream.write(uitoa((uint32_t)(g5x - 59)));
                }
                break; // G54-G59
        }
        hal.stream.write(":");
        hal.stream.write(get_axis_values(coord_data));
        hal.stream.write("]\r\n");
    }

    // Print G92,G92.1 which are not persistent in memory
    hal.stream.write("[G92:");
    hal.stream.write(get_axis_values(gc_state.g92_coord_offset));
    hal.stream.write("]\r\n");
#ifdef N_TOOLS
    for (idx = 1; idx <= N_TOOLS; idx++) {
        hal.stream.write("[T");
        hal.stream.write(uitoa((uint32_t)idx));
        hal.stream.write(":");
        hal.stream.write(get_axis_values(tool_table[idx].offset));
        hal.stream.write("]\r\n");
    }
#endif
    // Print tool length offset value
    hal.stream.write("[TLO:");
    hal.stream.write(get_axis_values(gc_state.tool_length_offset));
    hal.stream.write("]\r\n");

    report_probe_parameters(); // Print probe parameters. Not persistent in memory.
}


// Print current gcode parser mode state
void report_gcode_modes (void)
{
    hal.stream.write("[GC:G");
    if (gc_state.modal.motion >= MotionMode_ProbeToward) {
        hal.stream.write("38.");
        hal.stream.write(uitoa((uint32_t)(gc_state.modal.motion - (MotionMode_ProbeToward - 2))));
    } else
        hal.stream.write(uitoa((uint32_t)gc_state.modal.motion));

    uint8_t g5x = gc_state.modal.coord_system.idx + 54;
    hal.stream.write(" G");
    hal.stream.write(uitoa((uint32_t)(g5x > 59 ? 59 : g5x)));
    if(g5x > 59) {
        hal.stream.write(".");
        hal.stream.write(uitoa((uint32_t)(g5x - 59)));
    }

    if(settings.flags.lathe_mode)
        hal.stream.write(gc_state.modal.diameter_mode ? " G7" : " G8");

    hal.stream.write(" G");
    hal.stream.write(uitoa((uint32_t)(gc_state.modal.plane_select + 17)));

    hal.stream.write(gc_state.modal.units_imperial ? " G20" : " G21");

    hal.stream.write(gc_state.modal.distance_incremental ? " G91" : " G90");

    hal.stream.write(" G");
    hal.stream.write(uitoa((uint32_t)(94 - gc_state.modal.feed_mode)));

    if(settings.flags.lathe_mode && hal.driver_cap.variable_spindle)
        hal.stream.write(gc_state.modal.spindle_rpm_mode == SpindleSpeedMode_RPM ? " G97" : " G96");

    hal.stream.write(gc_state.canned.return_mode == CCReturnMode_RPos ? " G99" : " G98");

    hal.stream.write(gc_state.modal.scaling_active ? " G51" : " G50");

    if(gc_state.modal.scaling_active) {
        axis_signals_tostring(buf, gc_get_g51_state());
        hal.stream.write(":");
        hal.stream.write(buf);
    }

    if (gc_state.modal.program_flow) {

        switch (gc_state.modal.program_flow) {

            case ProgramFlow_Paused:
                hal.stream.write(" M0");
                break;

            case ProgramFlow_OptionalStop:
                hal.stream.write(" M1");
                break;

            case ProgramFlow_CompletedM2:
                hal.stream.write(" M2");
                break;

            case ProgramFlow_CompletedM30:
                hal.stream.write(" M30");
                break;

            default:
                break;
        }
    }

    hal.stream.write(gc_state.modal.spindle.on ? (gc_state.modal.spindle.ccw ? " M4" : " M3") : " M5");

    if(gc_state.tool_change)
        hal.stream.write(" M6");

    if (gc_state.modal.coolant.value) {

        if (gc_state.modal.coolant.mist)
             hal.stream.write(" M7");

        if (gc_state.modal.coolant.flood)
            hal.stream.write(" M8");

    } else
        hal.stream.write(" M9");

    if (sys.override.control.feed_rate_disable)
        hal.stream.write(" M50");

    if (sys.override.control.spindle_rpm_disable)
        hal.stream.write(" M51");

    if (sys.override.control.feed_hold_disable)
        hal.stream.write(" M53");

    if (settings.parking.flags.enable_override_control && sys.override.control.parking_disable)
        hal.stream.write(" M56");

    hal.stream.write(appendbuf(2, " T", uitoa((uint32_t)gc_state.tool->tool)));

    hal.stream.write(appendbuf(2, " F", get_rate_value(gc_state.feed_rate)));

    if(hal.driver_cap.variable_spindle)
        hal.stream.write(appendbuf(2, " S", ftoa(gc_state.spindle.rpm, N_DECIMAL_RPMVALUE)));

    hal.stream.write("]\r\n");
}

// Prints specified startup line
void report_startup_line (uint8_t n, char *line)
{
    hal.stream.write(appendbuf(3, "$N", uitoa((uint32_t)n), "="));
    hal.stream.write(uitoa((uint32_t)n));
    hal.stream.write("=");
    hal.stream.write(line);
    hal.stream.write("\r\n");
}

void report_execute_startup_message (char *line, status_code_t status_code)
{
    hal.stream.write(">");
    hal.stream.write(line);
    hal.stream.write(":");
    hal.report.status_message(status_code);
}

// Prints build info line
void report_build_info (char *line)
{

    hal.stream.write("[VER:" GRBL_VERSION "(");
    hal.stream.write(hal.info ? hal.info : "HAL");
    hal.stream.write(")." GRBL_VERSION_BUILD ":");
    hal.stream.write(line);
    hal.stream.write("]\r\n");

    // Generate compile-time build option list

    char *append = &buf[5];

    strcpy(buf, "[OPT:");

    if(hal.driver_cap.variable_spindle)
        *append++ = 'V';

    *append++ = 'N';

    if(hal.driver_cap.mist_control)
        *append++ = 'M';

#ifdef COREXY
    *append++ = 'C';
#endif

    if(settings.parking.flags.enabled)
        *append++ = 'P';

    if(settings.flags.homing_force_set_origin)
        *append++ = 'Z';

    if(settings.flags.homing_single_axis_commands)
        *append++ = 'H';

    if(settings.flags.limits_two_switches_on_axes)
        *append++ = 'T';

    if(settings.flags.allow_probing_feed_override)
        *append++ = 'A';

    if(settings.spindle.disable_with_zero_speed)
        *append++ = '0';

    if(hal.driver_cap.software_debounce)
        *append++ = 'S';

    if(settings.parking.flags.enable_override_control)
        *append++ = 'R';

    if(!settings.homing.flags.init_lock)
        *append++ = 'L';

    if(hal.driver_cap.safety_door)
        *append++ = '+';

  #ifndef ENABLE_RESTORE_EEPROM_WIPE_ALL // NOTE: Shown when disabled.
    *append++ = '*';
  #endif

  #ifndef ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS // NOTE: Shown when disabled.
    *append++ = '$';
  #endif

  #ifndef ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS // NOTE: Shown when disabled.
    *append++ = '#';
  #endif

  #ifndef ENABLE_BUILD_INFO_WRITE_COMMAND // NOTE: Shown when disabled.
    *append++ = 'I';
  #endif

    if(!settings.flags.force_buffer_sync_on_wco_change) // NOTE: Shown when disabled.
        *append++ = 'W';

    *append++ = ',';
    *append = '\0';
    hal.stream.write(buf);

    // NOTE: Compiled values, like override increments/max/min values, may be added at some point later.
    hal.stream.write(uitoa((uint32_t)(BLOCK_BUFFER_SIZE - 1)));
    hal.stream.write(",");
    hal.stream.write(uitoa(hal.rx_buffer_size));
    hal.stream.write(",");
    hal.stream.write(uitoa((uint32_t)N_AXIS));

    hal.stream.write("]\r\n");

    strcpy(buf, "[NEWOPT:");

    if(hal.driver_cap.sd_card)
        strcat(buf, "SD,");

    if(hal.driver_cap.bluetooth)
        strcat(buf, "BT,");

    if(hal.driver_cap.ethernet)
        strcat(buf, "ETH,");

    if(hal.driver_cap.wifi)
        strcat(buf, "WIFI,");

    if(settings.flags.lathe_mode)
        strcat(buf, "LATHE,");

#ifdef N_TOOLS
    strcat(buf, "ATC;");
    strcat(buf, uitoa((uint32_t)N_TOOLS));
    strcat(buf, ",");
#else
    if(hal.stream.suspend_read)
        strcat(buf, "TC,"); // Manual tool change supported (M6)
#endif

    if(hal.driver_cap.spindle_sync)
        strcat(buf, "SS,");

#ifdef PID_LOG
    strcat(buf, "PID,");
#endif

    append = &buf[strlen(buf) - 1];
    if(*append == ',')
        *append = '\0';

    if(*append != ':') {
        hal.stream.write(buf);
        hal.stream.write("]\r\n");
    }

    if(hal.report_options)
        hal.report_options();
}


// Prints the character string line Grbl has received from the user, which has been pre-parsed,
// and has been sent into protocol_execute_line() routine to be executed by Grbl.
void report_echo_line_received (char *line)
{
    hal.stream.write("[echo: ");
    hal.stream.write(line);
    hal.stream.write("]\r\n");
}


 // Prints real-time data. This function grabs a real-time snapshot of the stepper subprogram
 // and the actual location of the CNC machine. Users may change the following function to their
 // specific needs, but the desired real-time data report must be as short as possible. This is
 // requires as it minimizes the computational overhead and allows grbl to keep running smoothly,
 // especially during g-code programs with fast, short line segments and high frequency reports (5-20Hz).
void report_realtime_status (void)
{
    int32_t current_position[N_AXIS]; // Copy current state of the system position variable
    float print_position[N_AXIS];

    memcpy(current_position, sys_position, sizeof(sys_position));
    system_convert_array_steps_to_mpos(print_position, current_position);

    // Report current machine state and sub-states
    hal.stream.write_all("<");

    switch (sys.state) {

        case STATE_IDLE:
            hal.stream.write_all("Idle");
            break;

        case STATE_CYCLE:
            hal.stream.write_all("Run");
            break;

        case STATE_HOLD:
            hal.stream.write_all(appendbuf(2, "Hold:", uitoa((uint32_t)(sys.holding_state - 1))));
            break;

        case STATE_JOG:
            hal.stream.write_all("Jog");
            break;

        case STATE_HOMING:
            hal.stream.write_all("Home");
            break;

        case STATE_ESTOP:
        case STATE_ALARM:
            hal.stream.write_all("Alarm");
            break;

        case STATE_CHECK_MODE:
            hal.stream.write_all("Check");
            break;

        case STATE_SAFETY_DOOR:
            hal.stream.write_all(appendbuf(2, "Door:", uitoa((uint32_t)sys.parking_state)));
            break;

        case STATE_SLEEP:
            hal.stream.write_all("Sleep");
            break;

        case STATE_TOOL_CHANGE:
            hal.stream.write_all("Tool");
            break;
    }

    uint_fast8_t idx;
    float wco[N_AXIS];
    if (!settings.status_report.machine_position || sys.report.wco_counter == 0) {
        for (idx = 0; idx < N_AXIS; idx++) {
            // Apply work coordinate offsets and tool length offset to current position.
            wco[idx] = gc_state.modal.coord_system.xyz[idx] + gc_state.g92_coord_offset[idx] + gc_state.tool_length_offset[idx];
            if (!settings.status_report.machine_position)
                print_position[idx] -= wco[idx];
        }
    }

    // Report position
    hal.stream.write_all(settings.status_report.machine_position ? "|MPos:" : "|WPos:");
    hal.stream.write_all(get_axis_values(print_position));

    // Returns planner and output stream buffer states.

    if (settings.status_report.buffer_state) {
        hal.stream.write_all("|Bf:");
        hal.stream.write_all(uitoa((uint32_t)plan_get_block_buffer_available()));
        hal.stream.write_all(",");
        hal.stream.write_all(uitoa(hal.stream.get_rx_buffer_available()));
    }

    if(settings.status_report.line_numbers) {
        // Report current line number
        plan_block_t *cur_block = plan_get_current_block();
        if (cur_block != NULL && cur_block->line_number > 0)
            hal.stream.write_all(appendbuf(2, "|Ln:", uitoa((uint32_t)cur_block->line_number)));
    }

    // Report realtime feed speed
    if(settings.status_report.feed_speed) {
        if(hal.driver_cap.variable_spindle) {
            hal.stream.write_all(appendbuf(2, "|FS:", get_rate_value(st_get_realtime_rate())));
            hal.stream.write_all(appendbuf(2, ",", ftoa(sys.spindle_rpm, N_DECIMAL_RPMVALUE)));
            if(hal.spindle_get_data /* && sys.mpg_mode */)
                hal.stream.write_all(appendbuf(2, ",", ftoa(hal.spindle_get_data(SpindleData_RPM).rpm, N_DECIMAL_RPMVALUE)));
        } else
            hal.stream.write_all(appendbuf(2, "|F:", get_rate_value(st_get_realtime_rate())));
    }

    if(settings.status_report.pin_state) {

        axes_signals_t lim_pin_state = (axes_signals_t)hal.limits_get_state();
        control_signals_t ctrl_pin_state = hal.system_control_get_state();
        bool prb_pin_state = hal.probe_get_state && hal.probe_get_state();

        if (lim_pin_state.value | ctrl_pin_state.value | prb_pin_state | sys.block_delete_enabled) {

            char *append = &buf[4];

            strcpy(buf, "|Pn:");

            if (prb_pin_state)
                *append++ = 'P';

            if (lim_pin_state.value)
                append = axis_signals_tostring(append, lim_pin_state);

            if (ctrl_pin_state.value) {
                if (ctrl_pin_state.safety_door_ajar)
                    *append++ = 'D';
                if (ctrl_pin_state.reset)
                    *append++ = 'R';
                if (ctrl_pin_state.feed_hold)
                    *append++ = 'H';
                if (ctrl_pin_state.cycle_start)
                    *append++ = 'S';
                if (ctrl_pin_state.e_stop)
                    *append++ = 'E';
                if (ctrl_pin_state.block_delete && sys.block_delete_enabled)
                    *append++ = 'B';
                if (ctrl_pin_state.stop_disable)
                    *append++ = 'T';
            }
            *append = '\0';
            hal.stream.write_all(buf);
        }
    }

    sys.report.flags.add_report = sys.report.override_counter <= 0;

    if(settings.status_report.work_coord_offset) {

        if (sys.report.wco_counter > 0)
            sys.report.wco_counter--;
        else {
            sys.report.wco_counter = sys.state & (STATE_HOMING|STATE_CYCLE|STATE_HOLD|STATE_JOG|STATE_SAFETY_DOOR)
                                      ? (REPORT_WCO_REFRESH_BUSY_COUNT - 1) // Reset counter for slow refresh
                                      : (REPORT_WCO_REFRESH_IDLE_COUNT - 1);
            sys.report.flags.add_report = Off; // Set override on next report.
            hal.stream.write_all("|WCO:");
            hal.stream.write_all(get_axis_values(wco));
        }
    }

    if(settings.status_report.overrrides) {

        if (sys.report.override_counter > 0)
            sys.report.override_counter--;
        else if(sys.report.flags.add_report) {

            hal.stream.write_all(appendbuf(2, "|Ov:", uitoa((uint32_t)sys.override.feed_rate)));
            hal.stream.write_all(appendbuf(2, ",", uitoa((uint32_t)sys.override.rapid_rate)));
            hal.stream.write_all(appendbuf(2, ",", uitoa((uint32_t)sys.override.spindle_rpm)));

            spindle_state_t sp_state = hal.spindle_get_state();
            coolant_state_t cl_state = hal.coolant_get_state();
            if (sp_state.on || cl_state.value || gc_state.tool_change || sys.report.override_counter < 0) {

                char *append = &buf[3];

                strcpy(buf, "|A:");

                if (sp_state.on)
                    *append++ = sp_state.ccw ? 'C' : 'S';

                if (cl_state.flood)
                    *append++ = 'F';

                if (cl_state.mist)
                    *append++ = 'M';

                if(gc_state.tool_change)
                    *append++ = 'T';

                *append = '\0';
                hal.stream.write_all(buf);
            }

            sys.report.override_counter = sys.state & (STATE_HOMING|STATE_CYCLE|STATE_HOLD|STATE_JOG|STATE_SAFETY_DOOR)
                                      ? (REPORT_OVERRIDE_REFRESH_BUSY_COUNT - 1) // Reset counter for slow refresh
                                      : (REPORT_OVERRIDE_REFRESH_IDLE_COUNT - 1);

        }
    } else if(gc_state.tool_change)
        hal.stream.write_all("|A:T");

    if(sys.report.flags.scaling) {
        axis_signals_tostring(buf, gc_get_g51_state());
        hal.stream.write_all("|Sc:");
        hal.stream.write_all(buf);
        sys.report.flags.scaling = Off;
    }

    if(sys.report.flags.mpg_mode) {
        hal.stream.write_all(sys.mpg_mode ? "|MPG:1" : "|MPG:0");
        sys.report.flags.mpg_mode = Off;
    }

    if(hal.driver_rt_report)
        hal.driver_rt_report(hal.stream.write_all);

    sys.report.flags.add_report = false;

    hal.stream.write_all(">\r\n");
}


void report_pid_log (void)
{
#ifdef PID_LOG
    uint_fast16_t idx = 0;

    hal.stream.write("[PID:");
    hal.stream.write(ftoa(sys.pid_log.setpoint, N_DECIMAL_PIDVALUE));
    hal.stream.write(",");
    hal.stream.write(ftoa(sys.pid_log.t_sample, N_DECIMAL_PIDVALUE));
    hal.stream.write(",2|"); // 2 is number of values per sample!

    if(sys.pid_log.idx) do {
        hal.stream.write(ftoa(sys.pid_log.target[idx], N_DECIMAL_PIDVALUE));
        hal.stream.write(",");
        hal.stream.write(ftoa(sys.pid_log.actual[idx], N_DECIMAL_PIDVALUE));
        idx++;
        if(idx != sys.pid_log.idx)
            hal.stream.write(",");
    } while(idx != sys.pid_log.idx);

    hal.stream.write("]\r\n");
    hal.report.status_message(Status_OK);
#else
    hal.report.status_message(Status_GcodeUnsupportedCommand);
#endif
}
