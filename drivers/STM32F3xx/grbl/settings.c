/*
  settings.c - non-volatile storage configuration handling

  Part of grblHAL

  Copyright (c) 2017-2021 Terje Io
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
#include "limits.h"
#include "nvs_buffer.h"
#include "tool_change.h"
#ifdef ENABLE_BACKLASH_COMPENSATION
#include "motion_control.h"
#endif
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
    .flags.sleep_enable = DEFAULT_SLEEP_ENABLE,
#if DEFAULT_LASER_MODE
    .mode = Mode_Laser,
    .flags.disable_laser_during_hold = DEFAULT_DISABLE_LASER_DURING_HOLD,
#else
    .flags.disable_laser_during_hold = 0,
  #if DEFAULT_LATHE_MODE
    .mode = Mode_Lathe,
  #endif
#endif
    .flags.restore_after_feed_hold = DEFAULT_RESTORE_AFTER_FEED_HOLD,
    .flags.force_initialization_alarm = DEFAULT_FORCE_INITIALIZATION_ALARM,

    .probe.disable_probe_pullup = DISABLE_PROBE_PIN_PULL_UP,
    .probe.allow_feed_override = ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES,
    .probe.invert_probe_pin = DEFAULT_INVERT_PROBE_PIN,

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
    .homing.flags.override_locks = DEFAULT_HOMING_OVERRIDE_LOCKS,
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
    .homing.dual_axis.fail_length_percent = DUAL_AXIS_HOMING_FAIL_AXIS_LENGTH_PERCENT,
    .homing.dual_axis.fail_distance_min = DUAL_AXIS_HOMING_FAIL_DISTANCE_MIN,
    .homing.dual_axis.fail_distance_max = DUAL_AXIS_HOMING_FAIL_DISTANCE_MAX,

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
    .spindle.flags.pwm_action = DEFAULT_SPINDLE_PWM_ACTION,
    .spindle.invert.on = INVERT_SPINDLE_ENABLE_PIN,
    .spindle.invert.ccw = INVERT_SPINDLE_CCW_PIN,
    .spindle.invert.pwm = INVERT_SPINDLE_PWM_PIN,
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
    .axis[X_AXIS].max_rate = DEFAULT_X_MAX_RATE,
    .axis[X_AXIS].acceleration = DEFAULT_X_ACCELERATION,
    .axis[X_AXIS].max_travel = (-DEFAULT_X_MAX_TRAVEL),
    .axis[X_AXIS].dual_axis_offset = 0.0f,
#ifdef ENABLE_BACKLASH_COMPENSATION
    .axis[X_AXIS].backlash = 0.0f,
#endif

    .axis[Y_AXIS].steps_per_mm = DEFAULT_Y_STEPS_PER_MM,
    .axis[Y_AXIS].max_rate = DEFAULT_Y_MAX_RATE,
    .axis[Y_AXIS].max_travel = (-DEFAULT_Y_MAX_TRAVEL),
    .axis[Y_AXIS].acceleration = DEFAULT_Y_ACCELERATION,
    .axis[Y_AXIS].dual_axis_offset = 0.0f,
#ifdef ENABLE_BACKLASH_COMPENSATION
    .axis[Y_AXIS].backlash = 0.0f,
#endif

    .axis[Z_AXIS].steps_per_mm = DEFAULT_Z_STEPS_PER_MM,
    .axis[Z_AXIS].max_rate = DEFAULT_Z_MAX_RATE,
    .axis[Z_AXIS].acceleration = DEFAULT_Z_ACCELERATION,
    .axis[Z_AXIS].max_travel = (-DEFAULT_Z_MAX_TRAVEL),
    .axis[Z_AXIS].dual_axis_offset = 0.0f,
#ifdef ENABLE_BACKLASH_COMPENSATION
    .axis[Z_AXIS].backlash = 0.0f,
#endif

#ifdef A_AXIS
    .axis[A_AXIS].steps_per_mm = DEFAULT_A_STEPS_PER_MM,
    .axis[A_AXIS].max_rate = DEFAULT_A_MAX_RATE,
    .axis[A_AXIS].acceleration = DEFAULT_A_ACCELERATION,
    .axis[A_AXIS].max_travel = (-DEFAULT_A_MAX_TRAVEL),
    .axis[A_AXIS].dual_axis_offset = 0.0f,
#ifdef ENABLE_BACKLASH_COMPENSATION
    .axis[A_AXIS].backlash = 0.0f,
#endif
    .homing.cycle[3].mask = HOMING_CYCLE_3,
#endif

#ifdef B_AXIS
    .axis[B_AXIS].steps_per_mm = DEFAULT_B_STEPS_PER_MM,
    .axis[B_AXIS].max_rate = DEFAULT_B_MAX_RATE,
    .axis[B_AXIS].acceleration = DEFAULT_B_ACCELERATION,
    .axis[B_AXIS].max_travel = (-DEFAULT_B_MAX_TRAVEL),
    .axis[B_AXIS].dual_axis_offset = 0.0f,
#ifdef ENABLE_BACKLASH_COMPENSATION
    .axis[B_AXIS].backlash = 0.0f,
#endif
    .homing.cycle[4].mask = HOMING_CYCLE_4,
#endif

#ifdef C_AXIS
    .axis[C_AXIS].steps_per_mm = DEFAULT_C_STEPS_PER_MM,
    .axis[C_AXIS].acceleration = DEFAULT_C_ACCELERATION,
    .axis[C_AXIS].max_rate = DEFAULT_C_MAX_RATE,
    .axis[C_AXIS].max_travel = (-DEFAULT_C_MAX_TRAVEL),
    .axis[C_AXIS].dual_axis_offset = 0.0f,
#ifdef ENABLE_BACKLASH_COMPENSATION
    .axis[C_AXIS].backlash = 0.0f,
#endif
    .homing.cycle[5].mask = HOMING_CYCLE_5,
#endif

    .tool_change.mode = (toolchange_mode_t)DEFAULT_TOOLCHANGE_MODE,
    .tool_change.probing_distance = DEFAULT_TOOLCHANGE_PROBING_DISTANCE,
    .tool_change.feed_rate = DEFAULT_TOOLCHANGE_FEED_RATE,
    .tool_change.seek_rate = DEFAULT_TOOLCHANGE_SEEK_RATE,
    .tool_change.pulloff_rate = DEFAULT_TOOLCHANGE_PULLOFF_RATE,

    .parking.flags.enabled = DEFAULT_PARKING_ENABLE,
    .parking.flags.deactivate_upon_init = DEFAULT_DEACTIVATE_PARKING_UPON_INIT,
    .parking.flags.enable_override_control= DEFAULT_ENABLE_PARKING_OVERRIDE_CONTROL,
    .parking.axis = DEFAULT_PARKING_AXIS,
    .parking.target = DEFAULT_PARKING_TARGET,
    .parking.rate = DEFAULT_PARKING_RATE,
    .parking.pullout_rate = DEFAULT_PARKING_PULLOUT_RATE,
    .parking.pullout_increment = DEFAULT_PARKING_PULLOUT_INCREMENT
};

static const setting_group_detail_t setting_group_detail [] = {
    { Group_Root, Group_Root, "Root"},
    { Group_Root, Group_General, "General"},
    { Group_Root, Group_ControlSignals, "Control signals"},
    { Group_Root, Group_Limits, "Limits"},
    { Group_Limits, Group_Limits_DualAxis, "Dual axis"},
    { Group_Root, Group_Coolant, "Coolant"},
    { Group_Root, Group_Spindle, "Spindle"},
    { Group_Spindle, Group_Spindle_Sync, "Spindle sync"},
    { Group_Root, Group_Toolchange, "Tool change"},
    { Group_Root, Group_Homing, "Homing"},
    { Group_Root, Group_Probing, "Probing"},
    { Group_Root, Group_Parking, "Parking"},
    { Group_Root, Group_Jogging, "Jogging"},
    { Group_Root, Group_Stepper, "Stepper"},
    { Group_Root, Group_MotorDriver, "Stepper driver"},
    { Group_Root, Group_Axis, "Axis"},
    { Group_Axis, Group_XAxis, "X-axis"},
    { Group_Axis, Group_YAxis, "Y-axis"},
    { Group_Axis, Group_ZAxis, "Z-axis"},
#ifdef A_AXIS
    { Group_Axis, Group_AAxis, "A-axis"},
#endif
#ifdef B_AXIS
    { Group_Axis, Group_BAxis, "B-axis"},
#endif
#ifdef C_AXIS
    { Group_Axis, Group_CAxis, "C-axis"}
#endif
};

static status_code_t store_driver_setting (setting_id_t setting, float value, char *svalue);
static status_code_t set_probe_invert (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_report_mask (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_report_inches (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_control_invert (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_spindle_invert (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_control_disable_pullup (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_probe_disable_pullup (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_soft_limits_enable (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_hard_limits_enable (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_jog_soft_limited (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_homing_enable (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_enable_legacy_rt_commands (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_homing_cycle (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_mode (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_parking_enable (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_restore_overrides (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_ignore_door_when_idle (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_sleep_enable (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_hold_actions (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_force_initialization_alarm (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_probe_allow_feed_override (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_tool_change_mode (setting_id_t id, uint_fast16_t int_value);
static status_code_t set_tool_change_probing_distance (setting_id_t id, float value);
#ifdef ENABLE_SPINDLE_LINEARIZATION
static status_code_t set_linear_piece (setting_id_t id, char *svalue);
static char *get_linear_piece (setting_id_t id);
#endif
static status_code_t set_axis_setting (setting_id_t setting, float value);
static float get_float (setting_id_t setting);
static uint32_t get_int (setting_id_t id);

static const setting_detail_t setting_detail[] = {
    { Setting_PulseMicroseconds, Group_Stepper, "Step pulse time", "microseconds", Format_Decimal, "#0.0", "2.0", NULL, Setting_IsLegacy, &settings.steppers.pulse_microseconds },
    { Setting_StepperIdleLockTime, Group_Stepper, "Step idle delay", "milliseconds", Format_Int16, "####0", NULL, "65535", Setting_IsLegacy, &settings.steppers.idle_lock_time },
    { Setting_StepInvertMask, Group_Stepper, "Step pulse invert", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsLegacy, &settings.steppers.step_invert.mask },
    { Setting_DirInvertMask, Group_Stepper, "Step direction invert", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsLegacy, &settings.steppers.dir_invert.mask },
    { Setting_InvertStepperEnable, Group_Stepper, "Invert step enable pin(s)", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsLegacy, &settings.steppers.enable_invert.mask },
    { Setting_LimitPinsInvertMask, Group_Limits, "Invert limit pins", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsLegacy, &settings.limits.invert.mask },
    { Setting_InvertProbePin, Group_Probing, "Invert probe pin", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsLegacyFn, set_probe_invert, get_int },
    { Setting_SpindlePWMBehaviour, Group_Spindle, "Disable spindle with zero speed", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtended, &settings.spindle.flags.mask },
//    { Setting_SpindlePWMBehaviour, Group_Spindle, "Spindle enable vs. speed behaviour", NULL, Format_RadioButtons, "No action,Disable spindle with zero speed,Enable spindle with all speeds", NULL, NULL, Setting_IsExtended, &settings.spindle.flags.mask },
#if COMPATIBILITY_LEVEL <= 1
    { Setting_StatusReportMask, Group_General, "Status report options", NULL, Format_Bitfield, "Position in machine coordinate,Buffer state,Line numbers,Feed & speed,Pin state,Work coordinate offset,Overrides,Probe coordinates,Buffer sync on WCO change,Parser state,Alarm substatus,Run substatus", NULL, NULL, Setting_IsExtendedFn, set_report_mask, get_int },
#else
    { Setting_StatusReportMask, Group_General, "Status report options", NULL, Format_Bitfield, "Position in machine coordinate,Buffer state", NULL, NULL, Setting_IsLegacyFn, set_report_mask, get_int },
#endif
    { Setting_JunctionDeviation, Group_General, "Junction deviation", "mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &settings.junction_deviation },
    { Setting_ArcTolerance, Group_General, "Arc tolerance", "mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &settings.arc_tolerance },
    { Setting_ReportInches, Group_General, "Report in inches", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsLegacyFn, set_report_inches, get_int },
    { Setting_ControlInvertMask, Group_ControlSignals, "Invert control pins", NULL, Format_Bitfield, "Reset,Feed hold,Cycle start,Safety door,Block delete,Optional stop,EStop,Probe connected,Motor fault", NULL, NULL, Setting_IsExpandedFn, set_control_invert, get_int },
    { Setting_CoolantInvertMask, Group_Coolant, "Invert coolant pins", NULL, Format_Bitfield, "Flood,Mist", NULL, NULL, Setting_IsExtended, &settings.coolant_invert.mask },
    { Setting_SpindleInvertMask, Group_Spindle, "Invert spindle signals", NULL, Format_Bitfield, "Spindle on,Spindle CCW,Invert PWM", NULL, NULL, Setting_IsExtendedFn, set_spindle_invert, get_int },
    { Setting_ControlPullUpDisableMask, Group_ControlSignals, "Pullup disable control pins", NULL, Format_Bitfield, "Reset,Feed hold,Cycle start,Safety door,Block delete,Optional stop,EStop,Probe connected,Motor fault", NULL, NULL, Setting_IsExtendedFn, set_control_disable_pullup, get_int },
    { Setting_LimitPullUpDisableMask, Group_Limits, "Pullup disable limit pins", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtended, &settings.limits.disable_pullup.mask },
    { Setting_ProbePullUpDisable, Group_Probing, "Pullup disable probe pin", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_probe_disable_pullup, get_int },
    { Setting_SoftLimitsEnable, Group_Limits, "Soft limits enable", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsLegacyFn, set_soft_limits_enable, get_int },
#if COMPATIBILITY_LEVEL <= 1
    { Setting_HardLimitsEnable, Group_Limits, "Hard limits enable", NULL, Format_XBitfield, "Enable,Strict mode", NULL, NULL, Setting_IsExpandedFn, set_hard_limits_enable, get_int },
#else
    { Setting_HardLimitsEnable, Group_Limits, "Hard limits enable", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsLegacyFn, set_hard_limits_enable, get_int },
#endif
#if COMPATIBILITY_LEVEL <= 1
    { Setting_HomingEnable, Group_Homing, "Homing cycle", NULL, Format_XBitfield, "Enable,Enable single axis commands,Homing on startup required,Set machine origin to 0,Two switches shares one input pin,Allow manual,Override locks,Keep homed status on reset", NULL, NULL, Setting_IsExpandedFn, set_homing_enable, get_int },
#else
    { Setting_HomingEnable, Group_Homing, "Homing cycle enable", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsLegacyFn, set_homing_enable, get_int },
#endif
    { Setting_HomingDirMask, Group_Homing, "Homing direction invert", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsLegacy, &settings.homing.dir_mask.value },
    { Setting_HomingFeedRate, Group_Homing, "Homing locate feed rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsLegacy, &settings.homing.feed_rate },
    { Setting_HomingSeekRate, Group_Homing, "Homing search seek rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsLegacy, &settings.homing.seek_rate },
    { Setting_HomingDebounceDelay, Group_Homing, "Homing switch debounce delay", "milliseconds", Format_Int16, "##0", NULL, NULL, Setting_IsLegacy, &settings.homing.debounce_delay },
    { Setting_HomingPulloff, Group_Homing, "Homing switch pull-off distance", "mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &settings.homing.pulloff },
    { Setting_G73Retract, Group_General, "G73 Retract distance", "mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsExtended, &settings.g73_retract },
    { Setting_PulseDelayMicroseconds, Group_Stepper, "Pulse delay", "microseconds", Format_Decimal, "#0.0", NULL, "10", Setting_IsExtended, &settings.steppers.pulse_delay_microseconds },
    { Setting_RpmMax, Group_Spindle, "Maximum spindle speed", "RPM", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &settings.spindle.rpm_max },
    { Setting_RpmMin, Group_Spindle, "Minimum spindle speed", "RPM", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacy, &settings.spindle.rpm_min },
    { Setting_Mode, Group_General, "Mode of operation", NULL, Format_RadioButtons, "Normal,Laser mode,Lathe mode", NULL, NULL, Setting_IsLegacyFn, set_mode, get_int },
    { Setting_PWMFreq, Group_Spindle, "Spindle PWM frequency", "Hz", Format_Decimal, "#####0", NULL, NULL, Setting_IsExtended, &settings.spindle.pwm_freq },
    { Setting_PWMOffValue, Group_Spindle, "Spindle PWM off value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &settings.spindle.pwm_off_value },
    { Setting_PWMMinValue, Group_Spindle, "Spindle PWM min value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &settings.spindle.pwm_min_value },
    { Setting_PWMMaxValue, Group_Spindle, "Spindle PWM max value", "percent", Format_Decimal, "##0.0", NULL, "100", Setting_IsExtended, &settings.spindle.pwm_max_value },
    { Setting_StepperDeenergizeMask, Group_Stepper, "Steppers deenergize", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtended, &settings.steppers.deenergize.mask },
    { Setting_SpindlePPR, Group_Spindle, "Spindle pulses per revolution (PPR)", NULL, Format_Int16, "###0", NULL, NULL, Setting_IsExtended, &settings.spindle.ppr },
    { Setting_EnableLegacyRTCommands, Group_General, "Enable legacy RT commands", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_enable_legacy_rt_commands, get_int },
    { Setting_JogSoftLimited, Group_Jogging, "Limit jog commands", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_jog_soft_limited, get_int },
    { Setting_ParkingEnable, Group_Parking, "Parking cycle", NULL, Format_XBitfield, "Enable,Enable parking override control,Deactivate upon init", NULL, NULL, Setting_IsExtendedFn, set_parking_enable, get_int },
    { Setting_ParkingAxis, Group_Parking, "Parking axis", NULL, Format_RadioButtons, "X,Y,Z", NULL, NULL, Setting_IsExtended, &settings.parking.axis },
    { Setting_HomingLocateCycles, Group_Homing, "Homing passes", NULL, Format_Int8, "##0", "1", "128", Setting_IsExtended, &settings.homing.locate_cycles },
    { Setting_HomingCycle_1, Group_Homing, "Axes homing, first pass", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtendedFn, set_homing_cycle, get_int },
    { Setting_HomingCycle_2, Group_Homing, "Axes homing, second pass", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtendedFn, set_homing_cycle, get_int },
    { Setting_HomingCycle_3, Group_Homing, "Axes homing, third pass", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtendedFn, set_homing_cycle, get_int },
#ifdef A_AXIS
    { Setting_HomingCycle_4, Group_Homing, "Axes homing, fourth pass", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtendedFn, set_homing_cycle, get_int },
#endif
#ifdef B_AXIS
    { Setting_HomingCycle_5, Group_Homing, "Axes homing, fifth pass", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtendedFn, set_homing_cycle, get_int },
#endif
#ifdef C_AXIS
    { Setting_HomingCycle_6, Group_Homing, "Axes homing, sixth pass", NULL, Format_AxisMask, NULL, NULL, NULL, Setting_IsExtendedFn, set_homing_cycle, get_int },
#endif
    { Setting_ParkingPulloutIncrement, Group_Parking, "Parking pull-out distance", "mm", Format_Decimal, "###0.0", NULL, NULL, Setting_IsExtended, &settings.parking.pullout_increment },
    { Setting_ParkingPulloutRate, Group_Parking, "Parking pull-out rate", "mm/min", Format_Decimal, "###0.0", NULL, NULL, Setting_IsExtended, &settings.parking.pullout_rate },
    { Setting_ParkingTarget, Group_Parking, "Parking target", "mm", Format_Decimal, "-###0.0", "-100000", NULL, Setting_IsExtended, &settings.parking.target },
    { Setting_ParkingFastRate, Group_Parking, "Parking fast rate", "mm/min", Format_Decimal, "###0.0", NULL, NULL, Setting_IsExtended, &settings.parking.rate },
    { Setting_RestoreOverrides, Group_General, "Restore overrides", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_restore_overrides, get_int },
    { Setting_IgnoreDoorWhenIdle, Group_Parking, "Ignore door when idle", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_ignore_door_when_idle, get_int },
    { Setting_SleepEnable, Group_General, "Sleep enable", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_sleep_enable, get_int },
    { Setting_HoldActions, Group_General, "Feed hold actions", NULL, Format_Bitfield, "Disable laser during hold,Restore spindle and coolant state on resume", NULL, NULL, Setting_IsExtendedFn, set_hold_actions, get_int },
    { Setting_ForceInitAlarm, Group_General, "Force init alarm", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_force_initialization_alarm, get_int },
    { Setting_ProbingFeedOverride, Group_Probing, "Probing feed override", NULL, Format_Bool, NULL, NULL, NULL, Setting_IsExtendedFn, set_probe_allow_feed_override, get_int },
#ifdef ENABLE_SPINDLE_LINEARIZATION
    { Setting_LinearSpindlePiece1, Group_Spindle, "Spindle linearisation, first point", NULL, Format_String, "x30", NULL, "30", Setting_IsExtendedFn, set_linear_piece, get_linear_piece },
    { Setting_LinearSpindlePiece2, Group_Spindle, "Spindle linearisation, second point", NULL, Format_String, "x30", NULL, "30", Setting_IsExtendedFn, set_linear_piece, get_linear_piece },
    { Setting_LinearSpindlePiece3, Group_Spindle, "Spindle linearisation, third point", NULL, Format_String, "x30", NULL, "30", Setting_IsExtendedFn, set_linear_piece, get_linear_piece },
    { Setting_LinearSpindlePiece4, Group_Spindle, "Spindle linearisation, fourth point", NULL, Format_String, "x30", NULL, "30", Setting_IsExtendedFn, set_linear_piece, get_linear_piece },
#endif
    { Setting_SpindlePGain, Group_Spindle_ClosedLoop, "Spindle P-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.spindle.pid.p_gain },
    { Setting_SpindleIGain, Group_Spindle_ClosedLoop, "Spindle I-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.spindle.pid.i_gain },
    { Setting_SpindleDGain, Group_Spindle_ClosedLoop, "Spindle D-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.spindle.pid.d_gain },
    { Setting_SpindleMaxError, Group_Spindle_ClosedLoop, "Spindle PID max error", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.spindle.pid.max_error },
    { Setting_SpindleIMaxError, Group_Spindle_ClosedLoop, "Spindle PID max I error", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.spindle.pid.i_max_error },
    { Setting_PositionPGain, Group_Spindle_Sync, "Spindle sync P-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.position.pid.p_gain },
    { Setting_PositionIGain, Group_Spindle_Sync, "Spindle sync I-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.position.pid.i_gain },
    { Setting_PositionDGain, Group_Spindle_Sync, "Spindle sync D-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.position.pid.d_gain },
    { Setting_PositionIMaxError, Group_Spindle_Sync, "Spindle sync PID max I error", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_IsExtended, &settings.position.pid.i_max_error },
    { Setting_AxisStepsPerMMBase, Group_Axis0, "?-axis travel resolution", "step/mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacyFn, set_axis_setting, get_float },
    { Setting_AxisMaxRateBase, Group_Axis0, "?-axis maximum rate", "mm/min", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacyFn, set_axis_setting, get_float },
    { Setting_AxisAccelerationBase, Group_Axis0, "?-axis acceleration", "mm/sec^2", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacyFn, set_axis_setting, get_float },
    { Setting_AxisMaxTravelBase, Group_Axis0, "?-axis maximum travel", "mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsLegacyFn, set_axis_setting, get_float },
#ifdef ENABLE_BACKLASH_COMPENSATION
    { Setting_AxisBacklashBase, Group_Axis0, "?-axis backlash compensation", "mm", Format_Decimal, "#####0.000", NULL, NULL, Setting_IsExtendedFn, set_axis_setting, get_float },
#endif
    { Setting_AxisAutoSquareOffsetBase, Group_Axis0, "?-axis dual axis offset", "mm", Format_Decimal, "-0.000", "-2", "2", Setting_IsExtendedFn, set_axis_setting, get_float },
/*    { Setting_AdminPassword, Group_General, "Admin Password", NULL, Format_Password, "x(32)", NULL, "32" },
    { Setting_UserPassword, Group_General, "User Password", NULL, Format_Password, "x(32)", NULL, "32" }, */
    { Setting_SpindleAtSpeedTolerance, Group_Spindle, "Spindle at speed tolerance", "percent", Format_Decimal, "##0.0", NULL, NULL, Setting_IsExtended, &settings.spindle.at_speed_tolerance },
    { Setting_ToolChangeMode, Group_Toolchange, "Tool change mode", NULL, Format_RadioButtons, "Normal,Manual touch off,Manual touch off @ G59.3,Automatic touch off @ G59.3,Ignore M6", NULL, NULL, Setting_IsExtendedFn, set_tool_change_mode, get_int },
    { Setting_ToolChangeProbingDistance, Group_Toolchange, "Tool change probing distance", "mm", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsExtendedFn, set_tool_change_probing_distance, get_float },
    { Setting_ToolChangeFeedRate, Group_Toolchange, "Tool change locate feed rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsExtended, &settings.tool_change.feed_rate },
    { Setting_ToolChangeSeekRate, Group_Toolchange, "Tool change search seek rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsExtended, &settings.tool_change.seek_rate },
    { Setting_ToolChangePulloffRate, Group_Toolchange, "Tool change probe pull-off rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsExtended, &settings.tool_change.pulloff_rate },
    { Setting_DualAxisLengthFailPercent, Group_Limits_DualAxis, "Dual axis length fail", "percent", Format_Decimal, "##0.0", "0", "100", Setting_IsExtended, &settings.homing.dual_axis.fail_length_percent },
    { Setting_DualAxisLengthFailMin, Group_Limits_DualAxis, "Dual axis length fail min", "mm/min", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsExtended, &settings.homing.dual_axis.fail_distance_min },
    { Setting_DualAxisLengthFailMax, Group_Limits_DualAxis, "Dual axis length fail max", "mm/min", Format_Decimal, "#####0.0", NULL, NULL, Setting_IsExtended, &settings.homing.dual_axis.fail_distance_max }
};

static status_code_t store_driver_setting (setting_id_t setting, float value, char *svalue)
{
    status_code_t status = hal.driver_settings.set ? hal.driver_settings.set(setting, value, svalue) : Status_Unhandled;

    if(status == Status_OK) {
 //       hal.nvs.memcpy_to_nvs(hal.nvs.driver_area.address, hal.nvs.driver_area.mem_address, hal.nvs.driver_area.size, true);
        if(hal.driver_settings.changed)
            hal.driver_settings.changed(&settings);
    }

    return status == Status_Unhandled ? Status_InvalidStatement : status;
}

static status_code_t set_probe_invert (setting_id_t id, uint_fast16_t int_value)
{
    if(!hal.probe.configure)
        return Status_SettingDisabled;

    settings.probe.invert_probe_pin = int_value != 0;
    hal.probe.configure(false, false);

    return Status_OK;
}

static status_code_t set_report_mask (setting_id_t id, uint_fast16_t int_value)
{
#if COMPATIBILITY_LEVEL <= 1
    settings.status_report.mask = int_value;
#else
    int_value &= 0b11;
    settings.status_report.mask = (settings.status_report.mask & ~0b11) | int_value;
#endif

    return Status_OK;
}

static status_code_t set_report_inches (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.report_inches = int_value != 0;
    report_init();
    system_flag_wco_change(); // Make sure WCO is immediately updated.

    return Status_OK;
}

static status_code_t set_control_invert (setting_id_t id, uint_fast16_t int_value)
{
    settings.control_invert.mask = int_value;
    settings.control_invert.block_delete &= hal.driver_cap.block_delete;
    settings.control_invert.e_stop &= hal.driver_cap.e_stop;
    settings.control_invert.stop_disable &= hal.driver_cap.program_stop;
    settings.control_invert.probe_disconnected &= hal.driver_cap.probe_connected;

    return Status_OK;
}

static status_code_t set_spindle_invert (setting_id_t id, uint_fast16_t int_value)
{
    settings.spindle.invert.mask = int_value;
    if(settings.spindle.invert.pwm && !hal.driver_cap.spindle_pwm_invert) {
        settings.spindle.invert.pwm = Off;
        return Status_SettingDisabled;
    }

    return Status_OK;
}

static status_code_t set_control_disable_pullup (setting_id_t id, uint_fast16_t int_value)
{
    settings.control_disable_pullup.mask = int_value;
    settings.control_disable_pullup.block_delete &= hal.driver_cap.block_delete;
    settings.control_disable_pullup.e_stop &= hal.driver_cap.e_stop;
    settings.control_disable_pullup.stop_disable &= hal.driver_cap.program_stop;
    settings.control_invert.probe_disconnected &= hal.driver_cap.probe_connected;

    return Status_OK;
}

static status_code_t set_probe_disable_pullup (setting_id_t id, uint_fast16_t int_value)
{
    if(!hal.probe.configure)
        return Status_SettingDisabled;

    settings.probe.disable_probe_pullup = int_value != 0;

    return Status_OK;
}

static status_code_t set_soft_limits_enable (setting_id_t id, uint_fast16_t int_value)
{
    if (int_value && !settings.homing.flags.enabled)
        return Status_SoftLimitError;

    settings.limits.flags.soft_enabled = int_value != 0;

    return Status_OK;
}

static status_code_t set_hard_limits_enable (setting_id_t id, uint_fast16_t int_value)
{
    settings.limits.flags.hard_enabled = bit_istrue(int_value, bit(0));
#if COMPATIBILITY_LEVEL <= 1
    settings.limits.flags.check_at_init = bit_istrue(int_value, bit(1));
#endif
    hal.limits.enable(settings.limits.flags.hard_enabled, false); // Change immediately. NOTE: Nice to have but could be problematic later.

    return Status_OK;
}

static status_code_t set_jog_soft_limited (setting_id_t id, uint_fast16_t int_value)
{
    if (int_value && !settings.homing.flags.enabled)
        return Status_SoftLimitError;

    settings.limits.flags.jog_soft_limited = int_value != 0;

    return Status_OK;
}

static status_code_t set_homing_enable (setting_id_t id, uint_fast16_t int_value)
{
    if (bit_istrue(int_value, bit(0))) {
#if COMPATIBILITY_LEVEL > 1
        settings.homing.flags.enabled = On;
#else
        settings.homing.flags.value = int_value & 0x0F;
        settings.limits.flags.two_switches = bit_istrue(int_value, bit(4));
        settings.homing.flags.manual = bit_istrue(int_value, bit(5));
        settings.homing.flags.override_locks = bit_istrue(int_value, bit(6));
        settings.homing.flags.keep_on_reset = bit_istrue(int_value, bit(7));
#endif
    } else {
        settings.homing.flags.value = 0;
        settings.limits.flags.soft_enabled = Off; // Force disable soft-limits.
        settings.limits.flags.jog_soft_limited = Off;
    }

    return Status_OK;
}

static status_code_t set_enable_legacy_rt_commands (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.legacy_rt_commands = int_value != 0;

    return Status_OK;
}

static status_code_t set_homing_cycle (setting_id_t id, uint_fast16_t int_value)
{
    settings.homing.cycle[id - Setting_HomingCycle_1].mask = int_value;
    limits_set_homing_axes();

    return Status_OK;
}

static status_code_t set_mode (setting_id_t id, uint_fast16_t int_value)
{
    switch((machine_mode_t)int_value) {

        case Mode_Standard:
           settings.flags.disable_laser_during_hold = 0;
           gc_state.modal.diameter_mode = false;
           break;

        case Mode_Laser:
            if(!hal.driver_cap.variable_spindle)
                return Status_SettingDisabledLaser;
            if(settings.mode != Mode_Laser)
                settings.flags.disable_laser_during_hold = DEFAULT_DISABLE_LASER_DURING_HOLD;
            gc_state.modal.diameter_mode = false;
            break;

         case Mode_Lathe:
            settings.flags.disable_laser_during_hold = 0;
            break;

         default: // Mode_Standard
            return Status_InvalidStatement;
    }

    return Status_OK;
}

static status_code_t set_parking_enable (setting_id_t id, uint_fast16_t int_value)
{
    settings.parking.flags.value = bit_istrue(int_value, bit(0)) ? (int_value & 0x07) : 0;

    return Status_OK;
}

static status_code_t set_restore_overrides (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.restore_overrides = int_value != 0;;

    return Status_OK;
}

static status_code_t set_ignore_door_when_idle (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.safety_door_ignore_when_idle = int_value != 0;

    return Status_OK;
}

static status_code_t set_sleep_enable (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.sleep_enable = int_value != 0;

    return Status_OK;
}

static status_code_t set_hold_actions (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.disable_laser_during_hold = bit_istrue(int_value, bit(0));
    settings.flags.restore_after_feed_hold = bit_istrue(int_value, bit(1));

    return Status_OK;
}

static status_code_t set_force_initialization_alarm (setting_id_t id, uint_fast16_t int_value)
{
    settings.flags.force_initialization_alarm = int_value != 0;

    return Status_OK;
}

static status_code_t set_probe_allow_feed_override (setting_id_t id, uint_fast16_t int_value)
{
    settings.probe.allow_feed_override = int_value != 0;

    return Status_OK;
}

static status_code_t set_tool_change_mode (setting_id_t id, uint_fast16_t int_value)
{
    if(!hal.driver_cap.atc && hal.stream.suspend_read && int_value <= ToolChange_Ignore) {
#if COMPATIBILITY_LEVEL > 1
        if((toolchange_mode_t)int_value == ToolChange_Manual_G59_3 || (toolchange_mode_t)int_value == ToolChange_SemiAutomatic)
            return Status_InvalidStatement;
#endif
        settings.tool_change.mode = (toolchange_mode_t)int_value;
        tc_init();
    } else
        return Status_InvalidStatement;

    return Status_OK;
}

static status_code_t set_tool_change_probing_distance (setting_id_t id, float value)
{
    if(hal.driver_cap.atc)
        return Status_InvalidStatement;

    settings.tool_change.probing_distance = value;

    return Status_OK;
}

#ifdef ENABLE_SPINDLE_LINEARIZATION

static status_code_t set_linear_piece (setting_id_t id, char *svalue)
{
    uint32_t idx = id - Setting_LinearSpindlePiece1;
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

    return Status_OK;
}

static char *get_linear_piece (setting_id_t id)
{
    static char buf[20];

    uint32_t idx = id - Setting_LinearSpindlePiece1;

    if(isnan(settings.spindle.pwm_piece[idx].rpm))
        strcpy(buf, ftoa(settings.spindle.pwm_piece[idx].rpm, N_DECIMAL_RPMVALUE));
    else {
        sprintf(buf, "$%d=%f,%f,%f" ASCII_EOL, (setting_id_t)(Setting_LinearSpindlePiece1 + idx), settings.spindle.pwm_piece[idx].rpm, settings.spindle.pwm_piece[idx].start, settings.spindle.pwm_piece[idx].end);
        hal.stream.write(buf);
    }

    return buf;
}

#endif

static status_code_t set_axis_setting (setting_id_t setting, float value)
{
    if (setting >= Setting_AxisSettingsBase && setting <= Setting_AxisSettingsMax) {
        // Store axis configuration. Axis numbering sequence set by AXIS_SETTING defines.
        // NOTE: Ensure the setting index corresponds to the report.c settings printout.
        bool found = false;
        uint_fast16_t base_idx = (uint_fast16_t)setting - (uint_fast16_t)Setting_AxisSettingsBase;
        uint_fast8_t axis_idx = base_idx % AXIS_SETTINGS_INCREMENT;

        if(axis_idx < N_AXIS) switch((base_idx - axis_idx) / AXIS_SETTINGS_INCREMENT) {

            case AxisSetting_StepsPerMM:
                if (hal.max_step_rate && value * settings.axis[axis_idx].max_rate > (float)hal.max_step_rate * 60.0f)
                    return Status_MaxStepRateExceeded;
                found = true;
                settings.axis[axis_idx].steps_per_mm = value;
                break;

            case AxisSetting_MaxRate:
                if (hal.max_step_rate && value * settings.axis[axis_idx].steps_per_mm > (float)hal.max_step_rate * 60.0f)
                    return Status_MaxStepRateExceeded;
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

            case AxisSetting_Backlash:
                found = true;
    #ifdef ENABLE_BACKLASH_COMPENSATION
                settings.axis[axis_idx].backlash = value;
                break;
    #else
                return Status_SettingDisabled;
    #endif

            case AxisSetting_AutoSquareOffset:
                found = true;
                if(hal.stepper.get_auto_squared && bit_istrue(hal.stepper.get_auto_squared().mask, bit(axis_idx)))
                    settings.axis[axis_idx].dual_axis_offset = value;
                else
                    return Status_SettingDisabled;
                break;

            default: // for stopping compiler warning
                break;
        }

        if(!found)
            return store_driver_setting(setting, value, "");

    } else if(setting > Setting_AxisSettingsMax && setting <= Setting_AxisSettingsMax2)
        return store_driver_setting(setting, value, "");

    return Status_OK;
}

static float get_float (setting_id_t setting)
{
    float value = 0.0f;

    if (setting >= Setting_AxisSettingsBase && setting <= Setting_AxisSettingsMax) {

        uint_fast16_t base_idx = (uint_fast16_t)setting - (uint_fast16_t)Setting_AxisSettingsBase;
        uint_fast8_t axis_idx = base_idx % AXIS_SETTINGS_INCREMENT;

        if(axis_idx < N_AXIS) switch((base_idx - axis_idx) / AXIS_SETTINGS_INCREMENT) {

            case AxisSetting_StepsPerMM:
                value = settings.axis[axis_idx].steps_per_mm;
                break;

            case AxisSetting_MaxRate:
                value = settings.axis[axis_idx].max_rate;
                break;

            case AxisSetting_Acceleration:
                value = settings.axis[axis_idx].acceleration  / (60.0f * 60.0f); // Convert to mm/min^2 for grbl internal use.
                break;

            case AxisSetting_MaxTravel:
                value = -settings.axis[axis_idx].max_travel; // Store as negative for grbl internal use.
                break;

#ifdef ENABLE_BACKLASH_COMPENSATION
            case AxisSetting_Backlash:
                value = settings.axis[axis_idx].backlash;
                break;
#endif

            case AxisSetting_AutoSquareOffset:
                value = settings.axis[axis_idx].dual_axis_offset;
                break;

            default: // for stopping compiler warning
                break;
        }
    } else switch(setting) {

        case Setting_ToolChangeProbingDistance:
            value = settings.tool_change.probing_distance;
            break;

        default:
            break;
    }

    return value;
}

static uint32_t get_int (setting_id_t id)
{
    uint32_t value = 0;

    switch(id) {

        case Setting_Mode:
            value = settings.mode;
            break;

        case Setting_InvertProbePin:
            value = settings.probe.invert_probe_pin;
            break;

        case Setting_StatusReportMask:
            value = settings.status_report.mask;
            break;

        case Setting_ReportInches:
            value = settings.flags.report_inches;
            break;

        case Setting_ControlInvertMask:
            value = settings.control_invert.mask;
            break;

        case Setting_SpindleInvertMask:
            value = settings.spindle.invert.mask;
            break;

        case Setting_ControlPullUpDisableMask:
            value = settings.control_disable_pullup.mask;
            break;

        case Setting_ProbePullUpDisable:
            value = settings.probe.disable_probe_pullup;
            break;

        case Setting_SoftLimitsEnable:
            value = settings.limits.flags.soft_enabled;
            break;

        case Setting_HardLimitsEnable:
            value = ((settings.limits.flags.hard_enabled & bit(0)) ? bit(0) | (settings.limits.flags.check_at_init ? bit(1) : 0) : 0);
            break;

        case Setting_JogSoftLimited:
            value = settings.limits.flags.jog_soft_limited;
            break;

        case Setting_HomingEnable:
            value = (settings.homing.flags.value & 0x0F) |
                     (settings.limits.flags.two_switches ? bit(4) : 0) |
                      (settings.homing.flags.manual ? bit(5) : 0) |
                       (settings.homing.flags.override_locks ? bit(6) : 0) |
                        (settings.homing.flags.keep_on_reset ? bit(7) : 0);
            break;

        case Setting_EnableLegacyRTCommands:
            value = settings.flags.legacy_rt_commands;
            break;

        case Setting_ParkingEnable:
            value = settings.parking.flags.value;
            break;

        case Setting_HomingCycle_1:
        case Setting_HomingCycle_2:
        case Setting_HomingCycle_3:
        case Setting_HomingCycle_4:
        case Setting_HomingCycle_5:
        case Setting_HomingCycle_6:
            value = settings.homing.cycle[id - Setting_HomingCycle_1].mask;
            break;

        case Setting_RestoreOverrides:
            value = settings.flags.restore_overrides;
            break;

        case Setting_IgnoreDoorWhenIdle:
            value = settings.flags.safety_door_ignore_when_idle;
            break;

        case Setting_SleepEnable:
            value = settings.flags.sleep_enable;
            break;

        case Setting_HoldActions:
            value = (settings.flags.disable_laser_during_hold ? bit(0) : 0) | (settings.flags.restore_after_feed_hold ? bit(1) : 0);
            break;

        case Setting_ForceInitAlarm:
            value = settings.flags.force_initialization_alarm;
            break;

        case Setting_ProbingFeedOverride:
            value = settings.probe.allow_feed_override;
            break;

        case Setting_ToolChangeMode:
            value = settings.tool_change.mode;
            break;

        default:
            break;
    }

    return value;
}

static setting_details_t details = {
    .groups = setting_group_detail,
    .n_groups = sizeof(setting_group_detail) / sizeof(setting_group_detail_t),
    .settings = setting_detail,
    .n_settings = sizeof(setting_detail) / sizeof(setting_detail_t)
};

setting_details_t *settings_get_details (void)
{
    details.on_report_settings = grbl.on_report_settings;

    return &details;
}

// Write build info to persistent storage
void settings_write_build_info (char *line)
{
    if(hal.nvs.type != NVS_None)
        hal.nvs.memcpy_to_nvs(NVS_ADDR_BUILD_INFO, (uint8_t *)line, sizeof(stored_line_t), true);
}

// Read build info from persistent storage.
bool settings_read_build_info(char *line)
{
    if (!(hal.nvs.type != NVS_None && hal.nvs.memcpy_from_nvs((uint8_t *)line, NVS_ADDR_BUILD_INFO, sizeof(stored_line_t), true) == NVS_TransferResult_OK)) {
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
        hal.nvs.memcpy_to_nvs(NVS_ADDR_STARTUP_BLOCK + idx * (sizeof(stored_line_t) + NVS_CRC_BYTES), (uint8_t *)line, sizeof(stored_line_t), true);
}

// Read startup line to persistent storage.
bool settings_read_startup_line (uint8_t idx, char *line)
{
    assert(idx < N_STARTUP_LINE);

    if (!(hal.nvs.type != NVS_None && hal.nvs.memcpy_from_nvs((uint8_t *)line, NVS_ADDR_STARTUP_BLOCK + idx * (sizeof(stored_line_t) + NVS_CRC_BYTES), sizeof(stored_line_t), true) == NVS_TransferResult_OK)) {
        // Reset line with default value
        *line = '\0'; // Empty line
        settings_write_startup_line(idx, line);
        return false;
    }
    return true;
}

// Write selected coordinate data to persistent storage.
void settings_write_coord_data (coord_system_id_t id, float (*coord_data)[N_AXIS])
{
    assert(id <= N_CoordinateSystems);

#ifdef FORCE_BUFFER_SYNC_DURING_NVS_WRITE
    protocol_buffer_synchronize();
#endif

    if(hal.nvs.type != NVS_None)
        hal.nvs.memcpy_to_nvs(NVS_ADDR_PARAMETERS + id * (sizeof(coord_data_t) + NVS_CRC_BYTES), (uint8_t *)coord_data, sizeof(coord_data_t), true);
}

// Read selected coordinate data from persistent storage.
bool settings_read_coord_data (coord_system_id_t id, float (*coord_data)[N_AXIS])
{
    assert(id <= N_CoordinateSystems);

    if (!(hal.nvs.type != NVS_None && hal.nvs.memcpy_from_nvs((uint8_t *)coord_data, NVS_ADDR_PARAMETERS + id * (sizeof(coord_data_t) + NVS_CRC_BYTES), sizeof(coord_data_t), true) == NVS_TransferResult_OK)) {
        // Reset with default zero vector
        memset(coord_data, 0, sizeof(coord_data_t));
        settings_write_coord_data(id, coord_data);
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
        hal.nvs.memcpy_to_nvs(NVS_ADDR_TOOL_TABLE + (tool_data->tool - 1) * (sizeof(tool_data_t) + NVS_CRC_BYTES), (uint8_t *)tool_data, sizeof(tool_data_t), true);

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

    if (!(hal.nvs.type != NVS_None && hal.nvs.memcpy_from_nvs((uint8_t *)tool_data, NVS_ADDR_TOOL_TABLE + (tool - 1) * (sizeof(tool_data_t) + NVS_CRC_BYTES), sizeof(tool_data_t), true) == NVS_TransferResult_OK && tool_data->tool == tool)) {
        memset(tool_data, 0, sizeof(tool_data_t));
        tool_data->tool = tool;
    }

    return tool_data->tool == tool;
#else
    return false;
#endif
}

// Read Grbl global settings from persistent storage.
// Checks version-byte of non-volatile storage and global settings copy.
bool read_global_settings ()
{
    bool ok = hal.nvs.type != NVS_None && SETTINGS_VERSION == hal.nvs.get_byte(0) && hal.nvs.memcpy_from_nvs((uint8_t *)&settings, NVS_ADDR_GLOBAL, sizeof(settings_t), true) == NVS_TransferResult_OK;

    return ok && settings.version == SETTINGS_VERSION;
}


// Write Grbl global settings and version number to persistent storage
void settings_write_global (void)
{
    if(hal.nvs.type != NVS_None) {
        hal.nvs.put_byte(0, SETTINGS_VERSION);
        hal.nvs.memcpy_to_nvs(NVS_ADDR_GLOBAL, (uint8_t *)&settings, sizeof(settings_t), true);
    }
}


// Restore Grbl global settings to defaults and write to persistent storage
void settings_restore (settings_restore_t restore)
{
    uint_fast8_t idx;
    stored_line_t empty_line;

    memset(empty_line, 0xFF, sizeof(stored_line_t));
    *empty_line = '\0';

    if (restore.defaults) {
        memcpy(&settings, &defaults, sizeof(settings_t));

        settings.control_invert.block_delete &= hal.driver_cap.block_delete;
        settings.control_invert.e_stop &= hal.driver_cap.e_stop;
        settings.control_invert.stop_disable &= hal.driver_cap.program_stop;
        settings.control_disable_pullup.block_delete &= hal.driver_cap.block_delete;
        settings.control_disable_pullup.e_stop &= hal.driver_cap.e_stop;
        settings.control_disable_pullup.stop_disable &= hal.driver_cap.program_stop;
        settings.spindle.invert.ccw &= hal.driver_cap.spindle_dir;
        settings.spindle.invert.pwm &= hal.driver_cap.spindle_pwm_invert;

        settings_write_global();
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

    if(restore.driver_parameters && hal.driver_settings.restore) {
        hal.driver_settings.restore();
        hal.nvs.memcpy_to_nvs(hal.nvs.driver_area.address, hal.nvs.driver_area.mem_address, hal.nvs.driver_area.size, false);
    }

    nvs_buffer_sync_physical();
}

bool settings_is_group_available (setting_group_t group)
{
    bool available = false;

    switch(group) {

        case Group_Probing:
            available = hal.probe.get_state != NULL;
            break;

        case Group_Encoders:
        case Group_Encoder0:
            available = hal.encoder.get_n_encoders && hal.encoder.get_n_encoders() > 0;
            break;

        case Group_Spindle_Sync:
            available = hal.driver_cap.spindle_sync;
            break;

        case Group_Spindle_ClosedLoop:
            available = hal.driver_cap.spindle_pid;
            break;

        case Group_Limits_DualAxis:
            available = hal.stepper.get_auto_squared != NULL;
            break;

        case Group_General:
        case Group_Homing:
        case Group_Jogging:
        case Group_Limits:
        case Group_ControlSignals:
        case Group_Spindle:
        case Group_Axis:
        case Group_XAxis:
        case Group_YAxis:
        case Group_ZAxis:
        #ifdef A_AXIS
        case Group_AAxis:
        #endif
        #ifdef B_AXIS
        case Group_BAxis:
        #endif
        #ifdef C_AXIS
        case Group_CAxis:
        #endif
            available = true;
            break;

        default:
            {
                uint_fast16_t idx;
                setting_details_t *details = settings_get_details();
                do {
                    if(details->settings) {
                        for(idx = 0; idx < details->n_settings; idx++) {
                            if(details->settings[idx].group != group)
                                available = true;
                        }
                    }
                    details = !available && details->on_report_settings ? details->on_report_settings() : NULL;
                } while(details);
            }
            break;
    }

    return available;
}

inline static setting_id_t normalize_id (setting_id_t id)
{
    if((id > Setting_AxisSettingsBase && id <= Setting_AxisSettingsMax) ||
       (id > Setting_AxisSettingsBase2 && id <= Setting_AxisSettingsMax2))
        id -= id % AXIS_SETTINGS_INCREMENT;
    else if(id > Setting_EncoderSettingsBase && id <= Setting_EncoderSettingsMax)
        id = Setting_EncoderSettingsBase + (id % ENCODER_SETTINGS_INCREMENT);

    return id;
}

const setting_detail_t *setting_get_details (setting_id_t id)
{
    uint_fast16_t idx;
    setting_details_t *details = settings_get_details();

    id = normalize_id(id);

    do {
        for(idx = 0; idx < details->n_settings; idx++) {
            if(details->settings[idx].id == id)
                return &details->settings[idx];
        }
        details = details->on_report_settings ? details->on_report_settings() : NULL;
    } while(details);

    return NULL;
}

bool settings_is_setting_available (setting_id_t id, setting_group_t group)
{
    bool available = settings_is_group_available(group);

    if(available) switch(normalize_id(id)) {

        case Setting_SpindlePWMBehaviour:
            available = hal.driver_cap.variable_spindle;
            break;

        case Setting_SpindlePPR:
            available = hal.driver_cap.spindle_sync || hal.driver_cap.spindle_pid;
            break;

        case Setting_SpindleAtSpeedTolerance:
            available = hal.driver_cap.spindle_at_speed;
            break;

        case Setting_RpmMax:
        case Setting_RpmMin:
        case Setting_PWMFreq:
        case Setting_PWMOffValue:
        case Setting_PWMMinValue:
        case Setting_PWMMaxValue:
            available = hal.driver_cap.variable_spindle;
            break;

        case Setting_AxisStepperCurrentBase:
        case Setting_AxisMicroStepsBase:
            if(grbl.on_report_settings) {
                uint_fast16_t idx;
                setting_details_t *details = grbl.on_report_settings();
                do {
                    if(details->settings) {
                        for(idx = 0; idx < details->n_settings; idx++) {

                            if(details->settings[idx].group != group)
                                available = true;

                        }
                    }
                    details = !available && details->on_report_settings ? details->on_report_settings() : NULL;
                } while(details);
            }
            break;

        case Setting_AxisBacklashBase:
#ifndef ENABLE_BACKLASH_COMPENSATION
            available = false;
#endif
            break;

        case Setting_AxisAutoSquareOffsetBase:
            available = hal.stepper.get_auto_squared != NULL;
            break;

        default:
            available = setting_get_details(id) != NULL;
            break;
    }

    return available;
}

static status_code_t validate_value (const setting_detail_t *setting, float value)
{
    float val;
    uint_fast8_t set_idx = 0;

    if(setting->min_value) {
        if(!read_float((char *)setting->min_value, &set_idx, &val))
            return Status_BadNumberFormat;

        if(value < val)
            return Status_SettingValueOutOfRange;

    } else if(value < 0.0f)
        return Status_NegativeValue;

    if (setting->max_value) {
        set_idx = 0;

        if(!read_float((char *)setting->max_value, &set_idx, &val))
            return Status_BadNumberFormat;

        if(value > val)
            return Status_SettingValueOutOfRange;
    }

    return Status_OK;
}

static uint32_t strnumentries (const char *s, const char delimiter)
{
    if(s == NULL || *s == '\0')
        return 0;

    char *p = (char *)s;
    uint32_t entries = 1;

    while((p = strchr(p, delimiter))) {
        p++;
        entries++;
    }

    return entries;
}

setting_datatype_t setting_datatype_to_external (setting_datatype_t datatype)
{
    switch(datatype) {

        case Format_Int8:
        case Format_Int16:
            datatype = Format_Integer;
            break;

        default:
            break;
    }

    return datatype;
}

inline static bool setting_is_string (setting_datatype_t  datatype)
{
    return datatype == Format_String || datatype == Format_Password || datatype == Format_IPv4;
}

inline static bool setting_is_core (setting_type_t  type)
{
    return !(type == Setting_NonCore || type == Setting_NonCoreFn);
}

status_code_t setting_validate_me (const setting_detail_t *setting, float value, char *svalue)
{
    status_code_t status = Status_OK;

    switch(setting->datatype) {

        case Format_Bool:
            if(!(value == 0.0f || value == 1.0f))
                status = Status_SettingValueOutOfRange;
            break;

        case Format_Bitfield:
        case Format_XBitfield:
            if(!(isintf(value) && (uint32_t)value < (1UL << strnumentries(setting->format, ','))))
                status = Status_SettingValueOutOfRange;
            break;

        case Format_RadioButtons:
            if(!(isintf(value) && (uint32_t)value < strnumentries(setting->format, ',')))
                status = Status_SettingValueOutOfRange;
            break;

        case Format_AxisMask:
            if(!(isintf(value) && (uint32_t)value < (1 << N_AXIS)))
                status = Status_SettingValueOutOfRange;
            break;

        case Format_Int8:
        case Format_Int16:
        case Format_Integer:
        case Format_Decimal:
            status = validate_value(setting, value);
            if(setting->datatype == Format_Integer && status == Status_OK && !isintf(value))
                status = Status_BadNumberFormat;
            break;

        case Format_String:
        case Format_Password:
            {
                uint_fast16_t len = strlen(svalue);
                status = validate_value(setting, (float)len);
            }
            break;

        case Format_IPv4:
            // handled by driver or plugin, dependent on network library
            break;
    }

    return status;
}

status_code_t setting_validate (setting_id_t id, float value, char *svalue)
{
    const setting_detail_t *setting = setting_get_details(id);

    // If no details available setting could nevertheless be a valid setting id.
    return setting == NULL ? Status_OK : setting_validate_me(setting, value, svalue);
}

// A helper method to set settings from command line
status_code_t settings_store_global_setting (setting_id_t id, char *svalue)
{
    uint_fast8_t set_idx = 0;
    float value = NAN;
    status_code_t status = Status_OK;
    const setting_detail_t *setting = setting_get_details(id);

    // Trim leading spaces
    while(*svalue == ' ')
        svalue++;

    if(setting) {
        if(!setting_is_string(setting->datatype) && !read_float(svalue, &set_idx, &value) && setting_is_core(setting->type))
            return Status_BadNumberFormat;

        if((status = setting_validate_me(setting, value, svalue)) != Status_OK) {
            if(setting == Setting_PulseMicroseconds && status == Status_SettingValueOutOfRange)
                status =  Status_SettingStepPulseMin;

            return status;
        }
    }

    if(!setting || setting->value == NULL) {
        if((status = store_driver_setting(id, value, svalue)) == Status_InvalidStatement)
            status = Status_BadNumberFormat;

        return status;
    }

    uint_fast16_t int_value = (uint_fast16_t)truncf(value);

    switch(setting->type) {

        case Setting_NonCore:
        case Setting_IsExtended:
        case Setting_IsLegacy:
        case Setting_IsExpanded:
            status = Status_OK;
            switch(setting->datatype) {

                case Format_Decimal:
                    *((float *)(setting->value)) = value;
                    break;

                case Format_String:
                case Format_Password:
                    strcpy(((char *)(setting->value)), svalue);
                    break;

                case Format_AxisMask:
                    int_value &= AXES_BITMASK;
                    // no break
                case Format_Int8:
                case Format_Bool:
                case Format_Bitfield:
                case Format_XBitfield:
                case Format_RadioButtons:
                    *((uint8_t *)(setting->value)) = int_value;
                    break;

                case Format_Int16:
                    *((uint16_t *)(setting->value)) = int_value;
                    break;

                case Format_Integer:
                    *((uint32_t *)(setting->value)) = int_value;
                    break;

                default:
                    status = Status_BadNumberFormat;
                    break;
            }
            break;

        case Setting_NonCoreFn:
        case Setting_IsExtendedFn:
        case Setting_IsLegacyFn:
        case Setting_IsExpandedFn:
            switch(setting->datatype) {

                case Format_Decimal:
                    status = ((setting_set_float_ptr)(setting->value))(id, value);
                    break;

                case Format_String:
                case Format_Password:
                case Format_IPv4:
                    status = ((setting_set_string_ptr)(setting->value))(id, svalue);
                    break;

                default:
                    status = ((setting_set_int_ptr)(setting->value))(id, int_value);
                    break;
            }
            break;
    }

    if(status == Status_OK) {
        settings_write_global();
      #ifdef ENABLE_BACKLASH_COMPENSATION
        mc_backlash_init();
      #endif
        hal.settings_changed(&settings);
    }

    return status;
}

// Initialize the config subsystem
void settings_init() {
    if(!read_global_settings()) {
        settings_restore_t settings = settings_all;
        settings.defaults = 1; // Ensure global settings get restored
        if(hal.nvs.type != NVS_None)
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
        if(hal.probe.configure) // Initialize probe invert mask.
            hal.probe.configure(false, false);
    }

    if(hal.nvs.driver_area.address != 0 && hal.driver_settings.load) {
        hal.driver_settings.load();
        if(hal.driver_settings.changed)
            hal.driver_settings.changed(&settings);
    }
}
