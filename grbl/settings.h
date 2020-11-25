/*
  settings.h - non-volatile storage configuration handling

  Part of grblHAL

  Copyright (c) 2017-2020 Terje Io
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

#ifndef _SETTINGS_H_
#define _SETTINGS_H_

#include "config.h"
#include "system.h"
#include "plugins.h"

// Version of the persistent storage data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of non-volatile storage
#define SETTINGS_VERSION 19  // NOTE: Check settings_reset() when moving to next version.

// Define axis settings numbering scheme. Starts at Setting_AxisSettingsBase, every INCREMENT, over N_SETTINGS.
#define AXIS_SETTINGS_INCREMENT  10 // Must be greater than the number of axis settings TODO: change to 100 to allow for a logical wider range of parameters?

// Define encoder settings numbering scheme. Starts at Setting_EncoderSettingsBase, every INCREMENT, over N_SETTINGS.
// Not referenced by the core.
#define ENCODER_N_SETTINGS_MAX 5 // NOTE: This is the maximum number of encoders allowed.
#define ENCODER_SETTINGS_INCREMENT 10

typedef enum {
    AxisSetting_StepsPerMM = 0,
    AxisSetting_MaxRate = 1,
    AxisSetting_Acceleration = 2,
    AxisSetting_MaxTravel = 3,
    AxisSetting_StepperCurrent = 4, // TODO: move to axis settings for driver?, 200+
    AxisSetting_MicroSteps = 5,     // TODO: move to axis settings for driver?, 200+
    AxisSetting_Backlash = 6,
    AxisSetting_AutoSquareOffset = 7,
    AxisSetting_NumSettings
} axis_setting_type_t;

typedef enum {
    Setting_PulseMicroseconds = 0,
    Setting_StepperIdleLockTime = 1,
    Setting_StepInvertMask = 2,
    Setting_DirInvertMask = 3,
    Setting_InvertStepperEnable = 4,
    Setting_LimitPinsInvertMask = 5,
    Setting_InvertProbePin = 6,
    Setting_StatusReportMask = 10,
    Setting_JunctionDeviation = 11,
    Setting_ArcTolerance = 12,
    Setting_ReportInches = 13,
    Setting_ControlInvertMask = 14,
    Setting_CoolantInvertMask = 15,
    Setting_SpindleInvertMask = 16,
    Setting_ControlPullUpDisableMask = 17,
    Setting_LimitPullUpDisableMask = 18,
    Setting_ProbePullUpDisable = 19,
    Setting_SoftLimitsEnable = 20,
    Setting_HardLimitsEnable = 21,
    Setting_HomingEnable = 22,
    Setting_HomingDirMask = 23,
    Setting_HomingFeedRate = 24,
    Setting_HomingSeekRate = 25,
    Setting_HomingDebounceDelay = 26,
    Setting_HomingPulloff = 27,
    Setting_G73Retract = 28,
    Setting_PulseDelayMicroseconds = 29,
    Setting_RpmMax = 30,
    Setting_RpmMin = 31,
    Setting_Mode = 32,
    Setting_PWMFreq = 33,
    Setting_PWMOffValue = 34,
    Setting_PWMMinValue = 35,
    Setting_PWMMaxValue = 36,
    Setting_StepperDeenergizeMask = 37,
    Setting_SpindlePPR = 38,
    Setting_EnableLegacyRTCommands = 39,
    Setting_JogSoftLimited = 40,
    Setting_ParkingEnable = 41,
    Setting_ParkingAxis = 42,

    Setting_HomingLocateCycles = 43,
    Setting_HomingCycle_1 = 44,
    Setting_HomingCycle_2 = 45,
    Setting_HomingCycle_3 = 46,
    Setting_HomingCycle_4 = 47,
    Setting_HomingCycle_5 = 48,
    Setting_HomingCycle_6 = 49,
// Optional driver implemented settings for jogging
    Setting_JogStepSpeed = 50,
    Setting_JogSlowSpeed = 51,
    Setting_JogFastSpeed = 52,
    Setting_JogStepDistance = 53,
    Setting_JogSlowDistance = 54,
    Setting_JogFastDistance = 55,
//
    Setting_ParkingPulloutIncrement = 56,
    Setting_ParkingPulloutRate = 57,
    Setting_ParkingTarget = 58,
    Setting_ParkingFastRate = 59,

    Setting_RestoreOverrides = 60,
    Setting_IgnoreDoorWhenIdle = 61,
    Setting_SleepEnable = 62,
    Setting_HoldActions = 63,
    Setting_ForceInitAlarm = 64,
    Setting_ProbingFeedOverride = 65,
// Optional driver implemented settings for piecewise linear spindle PWM algorithm
    Setting_LinearSpindlePiece1 = 66,
    Setting_LinearSpindlePiece2 = 67,
    Setting_LinearSpindlePiece3 = 68,
    Setting_LinearSpindlePiece4 = 69,
//

// Optional driver implemented settings for additional streams
    Setting_NetworkServices = 70,
    Setting_BlueToothDeviceName = 71,
    Setting_BlueToothServiceName = 72,
    Setting_WifiMode = 73,
    Setting_WiFi_STA_SSID = 74,
    Setting_WiFi_STA_Password = 75,
    Setting_WiFi_AP_SSID = 76,
    Setting_WiFi_AP_Password = 77,
    Setting_Wifi_AP_Country = 78,
    Setting_Wifi_AP_Channel = 79,
//

// Optional settings for closed loop spindle speed control
    Setting_SpindlePGain = 80,
    Setting_SpindleIGain = 81,
    Setting_SpindleDGain = 82,
    Setting_SpindleDeadband = 83,
    Setting_SpindleMaxError = 84,
    Setting_SpindleIMaxError = 85,
    Setting_SpindleDMaxError = 86,

// Optional settings for closed loop spindle synchronized motion
    Setting_PositionPGain = 90,
    Setting_PositionIGain = 91,
    Setting_PositionDGain = 92,
    Setting_PositionDeadband = 93,
    Setting_PositionMaxError = 94,
    Setting_PositionIMaxError = 95,
    Setting_PositionDMaxError = 96,
//

// Reserving settings values >= 100 for axis settings. Up to 259. TODO: Or should that be 299...
    Setting_AxisSettingsBase = 100,     // Reserved for core settings, up to 100 + AXIS_SETTINGS_INCREMENT * N_AXIS
    Setting_AxisSettingsMax = 199,
    Setting_AxisSettingsBase2 = 200,    // Reserved for driver settings, up to 200 + AXIS_SETTINGS_INCREMENT * N_AXIS
    Setting_AxisSettingsMax2 = 255,
//

// Optional driver implemented settings

    // Normally used for Ethernet or WiFi Station
    Setting_Hostname = 300,
    Setting_IpMode = 301,
    Setting_IpAddress = 302,
    Setting_Gateway = 303,
    Setting_NetMask = 304,
    Setting_TelnetPort = 305,
    Setting_HttpPort = 306,
    Setting_WebSocketPort = 307,

    // Normally used for WiFi Access Point
    Setting_Hostname2 = 310,
    Setting_IpMode2 = 311,
    Setting_IpAddress2 = 312,
    Setting_Gateway2 = 313,
    Setting_NetMask2 = 314,
    Setting_TelnetPort2 = 315,
    Setting_HttpPort2 = 316,
    Setting_WebSocketPort2 = 317,

    Setting_Hostname3 = 320,
    Setting_IpMode3 = 321,
    Setting_IpAddress3 = 322,
    Setting_Gateway3 = 323,
    Setting_NetMask3 = 324,
    Setting_TelnetPort3 = 325,
    Setting_HttpPort3 = 326,
    Setting_WebSocketPort3 = 327,

    Setting_AdminPassword = 330,
    Setting_UserPassword = 331,

    Setting_TrinamicDriver = 338,
    Setting_TrinamicHoming = 339,

    Setting_SpindleAtSpeedTolerance = 340,
    Setting_ToolChangeMode = 341,
    Setting_ToolChangeProbingDistance = 342,
    Setting_ToolChangeFeedRate = 343,
    Setting_ToolChangeSeekRate = 344,
    Setting_ToolChangePulloffRate = 345,

    Setting_DualAxisLengthFailPercent = 347,
    Setting_DualAxisLengthFailMin = 348,
    Setting_DualAxisLengthFailMax = 349,

    Setting_THC_Mode = 350,
    Setting_THC_Delay = 351,
    Setting_THC_Threshold = 352,
    Setting_THC_PGain = 353,
    Setting_THC_IGain = 354,
    Setting_THC_DGain = 355,
    Setting_THC_VADThreshold = 356,
    Setting_THC_VoidOverride = 357,
    Setting_Arc_FailTimeout = 358,
    Setting_Arc_RetryDelay = 359,
    Setting_Arc_MaxRetries = 360,
    Setting_Arc_VoltageScale = 361,
    Setting_Arc_VoltageOffset = 362,
    Setting_Arc_HeightPerVolt = 363,
    Setting_Arc_OkHighVoltage = 364,
    Setting_Arc_OkLowVoltage = 365,

    Settings_IoPort_InvertIn  = 370,
    Settings_IoPort_Pullup_Disable = 371,
    Settings_IoPort_InvertOut = 372,
    Settings_IoPort_OD_Enable = 373,
    Settings_ModBus_BaudRate = 374,
    Settings_ModBus_RXTimeout = 375,

    Setting_EncoderSettingsBase = 400, // NOTE: Reserving settings values >= 400 for encoder settings. Up to 449.
    Setting_EncoderSettingsMax = 449,

    // Reserved for user plugins - do NOT use for public plugins
    Setting_UserDefined_0 = 450,
    Setting_UserDefined_1 = 451,
    Setting_UserDefined_2 = 452,
    Setting_UserDefined_3 = 453,
    Setting_UserDefined_4 = 454,
    Setting_UserDefined_5 = 455,
    Setting_UserDefined_6 = 456,
    Setting_UserDefined_7 = 457,
    Setting_UserDefined_8 = 458,
    Setting_UserDefined_9 = 459,

    Setting_SettingsMax,
    Setting_SettingsAll = Setting_SettingsMax,

    // Calculated base values for stepper settings
    Setting_AxisStepsPerMMBase       = Setting_AxisSettingsBase + AxisSetting_StepsPerMM * AXIS_SETTINGS_INCREMENT,
    Setting_AxisMaxRateBase          = Setting_AxisSettingsBase + AxisSetting_MaxRate * AXIS_SETTINGS_INCREMENT,
    Setting_AxisAccelerationBase     = Setting_AxisSettingsBase + AxisSetting_Acceleration * AXIS_SETTINGS_INCREMENT,
    Setting_AxisMaxTravelBase        = Setting_AxisSettingsBase + AxisSetting_MaxTravel * AXIS_SETTINGS_INCREMENT,
    Setting_AxisStepperCurrentBase   = Setting_AxisSettingsBase + AxisSetting_StepperCurrent * AXIS_SETTINGS_INCREMENT,
    Setting_AxisMicroStepsBase       = Setting_AxisSettingsBase + AxisSetting_MicroSteps * AXIS_SETTINGS_INCREMENT,
    Setting_AxisBacklashBase         = Setting_AxisSettingsBase + AxisSetting_Backlash * AXIS_SETTINGS_INCREMENT,
    Setting_AxisAutoSquareOffsetBase = Setting_AxisSettingsBase + AxisSetting_AutoSquareOffset * AXIS_SETTINGS_INCREMENT,

    // Calculated base values for encoder settings
    Setting_EncoderModeBase           = Setting_EncoderSettingsBase + Setting_EncoderMode * ENCODER_SETTINGS_INCREMENT,
    Setting_EncoderCPRBase            = Setting_EncoderSettingsBase + Setting_EncoderCPR * ENCODER_SETTINGS_INCREMENT,
    Setting_EncoderCPDBase            = Setting_EncoderSettingsBase + Setting_EncoderCPD * ENCODER_SETTINGS_INCREMENT,
    Setting_EncoderDblClickWindowBase = Setting_EncoderSettingsBase + Setting_EncoderDblClickWindow * ENCODER_SETTINGS_INCREMENT
} setting_type_t;

typedef union {
    uint8_t mask;
    struct {
        uint8_t defaults          :1,
                parameters        :1,
                startup_lines     :1,
                build_info        :1,
                driver_parameters :1,
                unassigned        :3;

    };
} settings_restore_t;

extern const settings_restore_t settings_all;

typedef char stored_line_t[MAX_STORED_LINE_LENGTH];

typedef union {
    uint16_t value;
    struct {
        uint16_t report_inches                :1,
                 restore_overrides            :1,
                 safety_door_ignore_when_idle :1,
                 sleep_enable                 :1,
                 disable_laser_during_hold    :1,
                 force_initialization_alarm   :1,
                 legacy_rt_commands           :1,
                 restore_after_feed_hold      :1,
                 unassigned                   :8;
    };
} settingflags_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t invert_probe_pin         :1,
                disable_probe_pullup     :1,
                invert_connected_pin     :1,
                disable_connected_pullup :1,
                allow_feed_override      :1,
                enable_protection        :1,
                unassigned               :2;
    };
} probeflags_t;

typedef union {
    uint16_t mask;
    struct {
        uint16_t machine_position   :1,
                 buffer_state       :1,
                 line_numbers       :1,
                 feed_speed         :1,
                 pin_state          :1,
                 work_coord_offset  :1,
                 overrides          :1,
                 probe_coordinates  :1,
                 sync_on_wco_change :1,
                 parser_state       :1,
                 alarm_substate     :1,
                 run_substate       :1,
                 unassigned         :4;
    };
} reportmask_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t enabled                 :1,
                deactivate_upon_init    :1,
                enable_override_control :1,
                unassigned              :5;
    };
} parking_setting_flags_t;

typedef struct {
    parking_setting_flags_t flags;
    uint8_t axis;               // Define which axis that performs the parking motion
    float target;               // Parking axis target. In mm, as machine coordinate [-max_travel,0].
    float rate;                 // Parking fast rate after pull-out in mm/min.
    float pullout_rate;         // Pull-out/plunge slow feed rate in mm/min.
    float pullout_increment;    // Spindle pull-out and plunge distance in mm. Incremental distance.
} parking_settings_t;

typedef struct {
    float p_gain;
    float i_gain;
    float d_gain;
    float p_max_error;
    float i_max_error;
    float d_max_error;
    float deadband;
    float max_error;
} pid_values_t;

typedef struct {
    float rpm_max;
    float rpm_min;
    float pwm_freq;
    float pwm_period;
    float pwm_off_value;
    float pwm_min_value;
    float pwm_max_value;
    float at_speed_tolerance;
    pwm_piece_t pwm_piece[SPINDLE_NPWM_PIECES];
    pid_values_t pid;
    uint16_t ppr; // Spindle encoder pulses per revolution
    spindle_state_t invert;
    bool disable_with_zero_speed;
} spindle_settings_t;

typedef struct {
    pid_values_t pid;
} position_pid_t; // Used for synchronized motion

typedef union {
    uint8_t value;
    struct {
        uint8_t enabled              :1,
                single_axis_commands :1,
                init_lock            :1,
                force_set_origin     :1,
                manual               :1,
                override_locks       :1,
                unassigned           :2;
    };
} homing_settings_flags_t;

typedef struct {
    float fail_length_percent; // DUAL_AXIS_HOMING_FAIL_AXIS_LENGTH_PERCENT
    float fail_distance_max;   // DUAL_AXIS_HOMING_FAIL_DISTANCE_MAX
    float fail_distance_min;   // DUAL_AXIS_HOMING_FAIL_DISTANCE_MIN
} homing_dual_axis_t;

typedef struct {
    float feed_rate;
    float seek_rate;
    float pulloff;
    axes_signals_t dir_mask;
    uint8_t locate_cycles;
    uint16_t debounce_delay;
    homing_settings_flags_t flags;
    axes_signals_t cycle[N_AXIS];
    homing_dual_axis_t dual_axis;
} homing_settings_t;

typedef struct {
    axes_signals_t step_invert;
    axes_signals_t dir_invert;
    axes_signals_t enable_invert;
    axes_signals_t deenergize;
    float pulse_microseconds;
    float pulse_delay_microseconds;
    uint16_t idle_lock_time; // If value = 255, steppers do not disable.
} stepper_settings_t;

typedef struct {
    float steps_per_mm;
    float max_rate;
    float acceleration;
    float max_travel;
    float dual_axis_offset;
#ifdef ENABLE_BACKLASH_COMPENSATION
    float backlash;
#endif
} axis_settings_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t hard_enabled     :1,
                soft_enabled     :1,
                check_at_init    :1,
                jog_soft_limited :1,
                two_switches     :1,
                unassigned       :3;
    };
} limit_settings_flags_t;

typedef struct {
    limit_settings_flags_t flags;
    axes_signals_t invert;
    axes_signals_t disable_pullup;
} limit_settings_t;

typedef union {
    uint8_t value;
    uint8_t mask;
    struct {
        uint8_t bit0 :1,
                bit1 :1,
                bit2 :1,
                bit3 :1,
                bit4 :1,
                bit5 :1,
                bit6 :1,
                bit7 :1;
    };
} ioport_bus_t;

typedef struct {
    ioport_bus_t invert_in;
    ioport_bus_t pullup_disable_in;
    ioport_bus_t invert_out;
    ioport_bus_t od_enable_out;
} ioport_signals_t;

typedef enum {
    Mode_Standard = 0,
    Mode_Laser,
    Mode_Lathe
} machine_mode_t;

typedef enum {
    ToolChange_Disabled = 0,
    ToolChange_Manual,
    ToolChange_Manual_G59_3,
    ToolChange_SemiAutomatic,
    ToolChange_Ignore
} toolchange_mode_t;

typedef struct {
    float feed_rate;
    float seek_rate;
    float pulloff_rate;
    float probing_distance;
    toolchange_mode_t mode;
} tool_change_settings_t;

// Global persistent settings (Stored from byte persistent storage_ADDR_GLOBAL onwards)
typedef struct {
    // Settings struct version
    uint32_t version;
    float junction_deviation;
    float arc_tolerance;
    float g73_retract;
    machine_mode_t mode;
    tool_change_settings_t tool_change;
    axis_settings_t axis[N_AXIS];
    control_signals_t control_invert;
    control_signals_t control_disable_pullup;
    coolant_state_t coolant_invert;
    spindle_settings_t spindle;
    stepper_settings_t steppers;
    reportmask_t status_report; // Mask to indicate desired report data.
    settingflags_t flags;       // Contains default boolean settings
    probeflags_t probe;
    homing_settings_t homing;
    limit_settings_t limits;
    parking_settings_t parking;
    position_pid_t position;    // Used for synchronized motion
    ioport_signals_t ioport;
} settings_t;

typedef enum {
    Group_Root = 0,
    Group_General,
    Group_Homing,
    Group_Jogging,
    Group_Limits,
    Group_Limits_DualAxis,
    Group_ControlSignals,
    Group_Spindle,
    Group_Spindle_Sync,
    Group_Spindle_ClosedLoop,
    Group_Coolant,
    Group_Networking,
    Group_Networking_Wifi,
    Group_Toolchange,
    Group_Plasma,
    Group_Encoder,
    Group_Bluetooth,
    Group_AuxPorts,
    Group_Stepper,
    Group_Probing,
    Group_Parking,
    Group_MotorDriver,
    Group_ModBus,
    Group_UserSettings,
    Group_Axis,
// NOTE: axis groups MUST be sequential AND last
    Group_XAxis,
    Group_YAxis,
    Group_ZAxis,
#ifdef A_AXIS
    Group_AAxis,
#endif
#ifdef B_AXIS
    Group_BAxis,
#endif
#ifdef C_AXIS
    Group_CAxis,
#endif
    Group_All = Group_Root
} setting_group_t;

typedef enum {
    Format_Bool = 0,
    Format_Bitfield,
    Format_XBitfield,
    Format_RadioButtons,
    Format_AxisMask,
    Format_Integer,
    Format_Decimal,
    Format_String,
    Format_Password,
    Format_IPv4
} setting_format_t;

typedef struct {
    setting_group_t parent;
    setting_group_t id;
    const char *name;
} setting_group_detail_t;

typedef struct {
    setting_type_t id;
    setting_group_t group;
    const char *name;
    const char *unit;
    setting_format_t type;
    const char *format;
    const char *min_value;
    const char *max_value;
} setting_detail_t;

typedef struct setting_details {
    const uint8_t n_groups;
    const setting_group_detail_t *groups;
    const uint8_t n_settings;
    const setting_detail_t *settings;
    struct setting_details *(*on_report_settings)(void);
} setting_details_t;

static const setting_group_detail_t setting_group_detail [] = {
    { Group_Root, Group_Root, "Root"},
    { Group_Root, Group_General, "General"},
    { Group_Root, Group_Stepper, "Stepper"},
    { Group_Root, Group_Homing, "Homing"},
    { Group_Root, Group_Probing, "Probing"},
    { Group_Root, Group_Parking, "Parking"},
    { Group_Root, Group_Jogging, "Jogging"},
    { Group_Root, Group_Limits, "Limits"},
    { Group_Limits, Group_Limits_DualAxis, "Dual axis"},
    { Group_Root, Group_ControlSignals, "Control signals"},
    { Group_Root, Group_Spindle, "Spindle"},
    { Group_Root, Group_Coolant, "Coolant"},
    { Group_Spindle, Group_Spindle_Sync, "Spindle sync"},
    { Group_Root, Group_Networking, "Networking"},
    { Group_Networking, Group_Networking_Wifi, "WiFi"},
    { Group_Root, Group_Bluetooth, "Bluetooth"},
    { Group_Root, Group_AuxPorts, "Aux ports"},
    { Group_Root, Group_Toolchange, "Toolchange"},
    { Group_Root, Group_Plasma, "Plasma"},
    { Group_Root, Group_Encoder, "Encoder"},
    { Group_Root, Group_MotorDriver, "Motor driver"},
    { Group_Root, Group_Axis, "Axis"},
    { Group_Axis, Group_XAxis, "Xaxis"},
    { Group_Axis, Group_YAxis, "Yaxis"},
    { Group_Axis, Group_ZAxis, "Zaxis"},
#ifdef A_AXIS
    { Group_Axis, Group_AAxis, "Aaxis"},
#endif
#ifdef B_AXIS
    { Group_Axis, Group_BAxis, "Baxis"},
#endif
#ifdef C_AXIS
    { Group_Axis, Group_CAxis, "Caxis"}
#endif
};

static const setting_detail_t setting_detail[] = {
    { Setting_PulseMicroseconds, Group_Stepper, "Step pulse time", "microseconds", Format_Decimal, "#0.0", "2.0", NULL },
    { Setting_StepperIdleLockTime, Group_Stepper, "Step idle delay", "milliseconds", Format_Integer, "####0", NULL, "65535" },
    { Setting_StepInvertMask, Group_Stepper, "Step pulse invert", NULL, Format_AxisMask, NULL, NULL, NULL },
    { Setting_DirInvertMask, Group_Stepper, "Step direction invert", NULL, Format_AxisMask, NULL, NULL, NULL },
    { Setting_InvertStepperEnable, Group_Stepper, "Invert step enable pin(s)", NULL, Format_AxisMask, NULL, NULL, NULL },
    { Setting_LimitPinsInvertMask, Group_Limits, "Invert limit pins", NULL, Format_AxisMask, NULL, NULL, NULL },
    { Setting_InvertProbePin, Group_Probing, "Invert probe pin", NULL, Format_Bool, NULL, NULL, NULL },
    { Setting_StatusReportMask, Group_General, "Status report options", NULL, Format_Bitfield, "Position in machine coordinate,Buffer state,Line numbers,Feed & speed,Pin state,Work coordinate offset,Overrides,Probe coordinates,Buffer sync on WCO change,Parser state,Alarm substatus,Run substatus", NULL, NULL },
    { Setting_JunctionDeviation, Group_General, "Junction deviation", "mm", Format_Decimal, "#####0.000", NULL, NULL },
    { Setting_ArcTolerance, Group_General, "Arc tolerance", "mm", Format_Decimal, "#####0.000", NULL, NULL },
    { Setting_ReportInches, Group_General, "Report in inches", NULL, Format_Bool, NULL, NULL, NULL },
    { Setting_ControlInvertMask, Group_ControlSignals, "Invert control pins", NULL, Format_Bitfield, "Reset,Feed hold,Cycle start,Safety door,Block delete,Optional stop,EStop,Probe connected", NULL, NULL },
    { Setting_CoolantInvertMask, Group_Coolant, "Invert coolant pins", NULL, Format_Bitfield, "Flood,Mist", NULL, NULL },
    { Setting_SpindleInvertMask, Group_Spindle, "Invert spindle signals", NULL, Format_Bitfield, "Spindle on,Spindle CCW,Invert PWM", NULL, NULL },
    { Setting_ControlPullUpDisableMask, Group_ControlSignals, "Pullup disable control pins", NULL, Format_Bitfield, "Reset,Feed hold,Cycle start,Safety door,Block delete,Optional stop,EStop,Probe connected", NULL, NULL },
    { Setting_LimitPullUpDisableMask, Group_Limits, "Pullup disable limit pins", NULL, Format_AxisMask, NULL, NULL, NULL },
    { Setting_ProbePullUpDisable, Group_Probing, "Pullup disable probe pin", NULL, Format_Bool, NULL, NULL, NULL },
    { Setting_SoftLimitsEnable, Group_Limits, "Soft limits enable", NULL, Format_Bool, NULL, NULL, NULL },
    { Setting_HardLimitsEnable, Group_Limits, "Hard limits enable", NULL, Format_XBitfield, "Enable,Strict mode", NULL, NULL },
    { Setting_HomingEnable, Group_Homing, "Homing cycle", NULL, Format_XBitfield, "Enable,Enable single axis commands,Homing on startup required,Set machine origin to 0,Two switches shares one input pin,Allow manual,Override locks", NULL, NULL },
    { Setting_HomingDirMask, Group_Homing, "Homing direction invert", NULL, Format_AxisMask, NULL, NULL, NULL },
    { Setting_HomingFeedRate, Group_Homing, "Homing locate feed rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL },
    { Setting_HomingSeekRate, Group_Homing, "Homing search seek rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL },
    { Setting_HomingDebounceDelay, Group_Homing, "Homing switch debounce delay", "milliseconds", Format_Integer, "##0", NULL, NULL },
    { Setting_HomingPulloff, Group_Homing, "Homing switch pull-off distance", "mm", Format_Decimal, "#####0.000", NULL, NULL },
    { Setting_G73Retract, Group_General, "G73 Retract distance", "mm", Format_Decimal, "#####0.000", NULL, NULL },
    { Setting_PulseDelayMicroseconds, Group_Stepper, "Pulse delay", "microseconds", Format_Decimal, "#0.0", NULL, "10" },
    { Setting_RpmMax, Group_Spindle, "Maximum spindle speed", "RPM", Format_Decimal, "#####0.000", NULL, NULL },
    { Setting_RpmMin, Group_Spindle, "Minimum spindle speed", "RPM", Format_Decimal, "#####0.000", NULL, NULL },
    { Setting_Mode, Group_General, "Mode of operation", NULL, Format_RadioButtons, "Normal,Laser mode,Lathe mode", NULL, NULL },
    { Setting_PWMFreq, Group_Spindle, "Spindle PWM frequency", "Hz", Format_Decimal, "#####0", NULL, NULL },
    { Setting_PWMOffValue, Group_Spindle, "Spindle PWM off value", "percent", Format_Decimal, "##0.0", NULL, "100" },
    { Setting_PWMMinValue, Group_Spindle, "Spindle PWM min value", "percent", Format_Decimal, "##0.0", NULL, "100" },
    { Setting_PWMMaxValue, Group_Spindle, "Spindle PWM max value", "percent", Format_Decimal, "##0.0", NULL, "100" },
    { Setting_StepperDeenergizeMask, Group_Stepper, "Steppers deenergize", NULL, Format_AxisMask, NULL, NULL, NULL },
    { Setting_SpindlePPR, Group_Spindle, "Spindle pulses per revolution (PPR)", NULL, Format_Integer, "###0", NULL, NULL },
    { Setting_EnableLegacyRTCommands, Group_General, "Enable legacy RT commands", NULL, Format_Bool, NULL, NULL, NULL },
    { Setting_JogSoftLimited, Group_Jogging, "Limit jog commands", NULL, Format_Bool, NULL, NULL, NULL },
    { Setting_ParkingEnable, Group_Parking, "Parking cycle", NULL, Format_XBitfield, "Enable,Enable parking override control,Deactivate upon init", NULL, NULL },
    { Setting_ParkingAxis, Group_Parking, "Parking axis", NULL, Format_RadioButtons, "X,Y,Z", NULL, NULL },
    { Setting_HomingLocateCycles, Group_Homing, "Homing passes", NULL, Format_Integer, "##0", "1", "128" },
    { Setting_HomingCycle_1, Group_Homing, "Axes homing, first pass", NULL, Format_AxisMask, NULL, NULL, NULL },
    { Setting_HomingCycle_2, Group_Homing, "Axes homing, second pass", NULL, Format_AxisMask, NULL, NULL, NULL },
    { Setting_HomingCycle_3, Group_Homing, "Axes homing, third pass", NULL, Format_AxisMask, NULL, NULL, NULL },
#ifdef A_AXIS
    { Setting_HomingCycle_4, Group_Homing, "Axes homing, fourth pass", NULL, Format_AxisMask, NULL, NULL, NULL },
#endif
#ifdef B_AXIS
    { Setting_HomingCycle_5, Group_Homing, "Axes homing, fifth pass", NULL, Format_AxisMask, NULL, NULL, NULL },
#endif
#ifdef C_AXIS
    { Setting_HomingCycle_6, Group_Homing, "Axes homing, sixth pass", NULL, Format_AxisMask, NULL, NULL, NULL },
#endif
    { Setting_JogStepSpeed, Group_Jogging, "Step jog speed", "mm/min", Format_Decimal, "###0.0", NULL, NULL },
    { Setting_JogSlowSpeed, Group_Jogging, "Slow jog speed", "mm/min", Format_Decimal, "###0.0", NULL, NULL },
    { Setting_JogFastSpeed, Group_Jogging, "Fast jog speed", "mm/min", Format_Decimal, "###0.0", NULL, NULL },
    { Setting_JogStepDistance, Group_Jogging, "Step jog distance", "mm", Format_Decimal, "#0.000", NULL, NULL },
    { Setting_JogSlowDistance, Group_Jogging, "Slow jog distance", "mm", Format_Decimal, "###0.0", NULL, NULL },
    { Setting_JogFastDistance, Group_Jogging, "Fast jog distance", "mm", Format_Decimal, "###0.0", NULL, NULL },
    { Setting_ParkingPulloutIncrement, Group_Parking, "Parking pull-out distance", "mm", Format_Decimal, "###0.0", NULL, NULL },
    { Setting_ParkingPulloutRate, Group_Parking, "Parking pull-out rate", "mm/min", Format_Decimal, "###0.0", NULL, NULL },
    { Setting_ParkingTarget, Group_Parking, "Parking target", "mm", Format_Decimal, "-###0.0", "-100000", NULL },
    { Setting_ParkingFastRate, Group_Parking, "Parking fast rate", "mm/min", Format_Decimal, "###0.0", NULL, NULL },
    { Setting_RestoreOverrides, Group_General, "Restore overrides", NULL, Format_Bool, NULL, NULL, NULL },
    { Setting_IgnoreDoorWhenIdle, Group_Parking, "Ignore door when idle", NULL, Format_Bool, NULL, NULL, NULL },
    { Setting_SleepEnable, Group_General, "Sleep enable", NULL, Format_Bool, NULL, NULL, NULL },
    { Setting_HoldActions, Group_General, "Feed hold actions", NULL, Format_Bitfield, "Disable laser during hold,Restore spindle and coolant state on resume", NULL, NULL },
    { Setting_ForceInitAlarm, Group_General, "Force init alarm", NULL, Format_Bool, NULL, NULL, NULL },
    { Setting_ProbingFeedOverride, Group_Probing, "Probing feed override", NULL, Format_Bool, NULL, NULL, NULL },
    { Setting_NetworkServices, Group_Networking, "Network Services", NULL, Format_Bitfield, "Telnet,Websocket,HTTP,DNS,MDNS,SSDP", NULL, NULL },
    { Setting_BlueToothDeviceName, Group_AuxPorts, "Bluetooth device", NULL, Format_String, "x(32)", NULL, "32" },
    { Setting_BlueToothServiceName, Group_AuxPorts, "Bluetooth service", NULL, Format_String, "x(32)", NULL, "32" },
    { Setting_WifiMode, Group_Networking_Wifi, "WiFi Mode", NULL, Format_RadioButtons, "Off,Station,Access Point,Access Point/Station", NULL, NULL },
    { Setting_WiFi_STA_SSID, Group_Networking_Wifi, "WiFi Station (STA) SSID", NULL, Format_String, "x(64)", NULL, "64" },
    { Setting_WiFi_STA_Password, Group_Networking_Wifi, "WiFi Station (STA) Password", NULL, Format_Password, "x(32)", NULL, "32" },
    { Setting_WiFi_AP_SSID, Group_Networking_Wifi, "WiFi Access Point (AP) SSID", NULL, Format_String, "x(64)", NULL, "64" },
    { Setting_WiFi_AP_Password, Group_Networking_Wifi, "WiFi Access Point (AP) Password", NULL, Format_Password, "x(32)", NULL, "32" },
    { Setting_SpindlePGain, Group_Spindle_ClosedLoop, "Spindle P-gain", NULL, Format_Decimal, "###0.000", NULL, NULL },
    { Setting_SpindleIGain, Group_Spindle_ClosedLoop, "Spindle I-gain", NULL, Format_Decimal, "###0.000", NULL, NULL },
    { Setting_SpindleDGain, Group_Spindle_ClosedLoop, "Spindle D-gain", NULL, Format_Decimal, "###0.000", NULL, NULL },
    { Setting_SpindleMaxError, Group_Spindle_ClosedLoop, "Spindle PID max error", NULL, Format_Decimal, "###0.000", NULL, NULL },
    { Setting_SpindleIMaxError, Group_Spindle_ClosedLoop, "Spindle PID max I error", NULL, Format_Decimal, "###0.000", NULL, NULL },
    { Setting_PositionPGain, Group_Spindle_Sync, "Spindle sync P-gain", NULL, Format_Decimal, "###0.000", NULL, NULL },
    { Setting_PositionIGain, Group_Spindle_Sync, "Spindle sync I-gain", NULL, Format_Decimal, "###0.000", NULL, NULL },
    { Setting_PositionDGain, Group_Spindle_Sync, "Spindle sync D-gain", NULL, Format_Decimal, "###0.000", NULL, NULL },
    { Setting_PositionIMaxError, Group_Spindle_Sync, "Spindle sync PID max I error", NULL, Format_Decimal, "###0.000", NULL, NULL },
    { Setting_AxisStepsPerMMBase, Group_Axis, "?-axis travel resolution", "step/mm", Format_Decimal, "#####0.000", NULL, NULL },
    { Setting_AxisMaxRateBase, Group_Axis, "?-axis maximum rate", "mm/min", Format_Decimal, "#####0.000", NULL, NULL },
    { Setting_AxisAccelerationBase, Group_Axis, "?-axis acceleration", "mm/sec^2", Format_Decimal, "#####0.000", NULL, NULL },
    { Setting_AxisMaxTravelBase, Group_Axis, "?-axis maximum travel", "mm", Format_Decimal, "#####0.000", NULL, NULL },
    { Setting_AxisStepperCurrentBase, Group_Axis, "?-axis motor current", "mA", Format_Integer, "###0", NULL, NULL },
    { Setting_AxisMicroStepsBase, Group_Axis, "?-axis microsteps", "steps", Format_Integer, "###0", NULL, NULL },
#ifdef ENABLE_BACKLASH_COMPENSATION
    { Setting_AxisBacklashBase, Group_Axis, "?-axis backlash compensation", "mm", Format_Decimal, "#####0.000", NULL, NULL },
#endif
    { Setting_AxisAutoSquareOffsetBase, Group_Axis, "?-axis dual axis offset", "mm", Format_Decimal, "-0.000", "-2", "2" },
    { Setting_Hostname, Group_Networking, "Hostname", NULL, Format_String, "x(64)", NULL, "64" },
    { Setting_IpMode, Group_Networking, "IP Mode", NULL, Format_RadioButtons, "Static,DHCP,AutoIP", NULL, NULL },
    { Setting_IpAddress, Group_Networking, "IP Address", NULL, Format_IPv4, NULL, NULL, NULL },
    { Setting_Gateway, Group_Networking, "Gateway", NULL, Format_IPv4, NULL, NULL, NULL },
    { Setting_NetMask, Group_Networking, "Netmask", NULL, Format_IPv4, NULL, NULL, NULL },
    { Setting_TelnetPort, Group_Networking, "Telnet Port", NULL, Format_Integer, "####0", "1", "65535" },
    { Setting_HttpPort, Group_Networking, "HTTP Port", NULL, Format_Integer, "####0", "1", "65535" },
    { Setting_WebSocketPort, Group_Networking, "Websocket Port", NULL, Format_Integer, "####0", "1", "65535" },
    { Setting_AdminPassword, Group_General, "Admin Password", NULL, Format_Password, "x(32)", NULL, "32" },
    { Setting_UserPassword, Group_General, "User Password", NULL, Format_Password, "x(32)", NULL, "32" },
    { Setting_TrinamicDriver, Group_MotorDriver, "Trinamic driver", NULL, Format_AxisMask, NULL, NULL, NULL },
    { Setting_TrinamicHoming, Group_MotorDriver, "Sensorless homing", NULL, Format_AxisMask, NULL, NULL, NULL },
    { Setting_SpindleAtSpeedTolerance, Group_Spindle, "Spindle at speed tolerance", "percent", Format_Decimal, "##0.0", NULL, NULL },
    { Setting_ToolChangeMode, Group_Toolchange, "Tool change mode", NULL, Format_RadioButtons, "Normal,Manual touch off,Manual touch off @ G59.3,Automatic touch off @ G59.3,Ignore M6", NULL, NULL },
    { Setting_ToolChangeProbingDistance, Group_Toolchange, "Tool change probing distance", "mm", Format_Decimal, "#####0.0", NULL, NULL },
    { Setting_ToolChangeFeedRate, Group_Toolchange, "Tool change locate feed rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL },
    { Setting_ToolChangeSeekRate, Group_Toolchange, "Tool change search seek rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL },
    { Setting_ToolChangePulloffRate, Group_Toolchange, "Tool change probe pull-off rate", "mm/min", Format_Decimal, "#####0.0", NULL, NULL },
    { Setting_DualAxisLengthFailPercent, Group_Limits_DualAxis, "Dual axis length fail", "percent", Format_Decimal, "##0.0", "0", "100" },
    { Setting_DualAxisLengthFailMin, Group_Limits_DualAxis, "Dual axis length fail min", "mm/min", Format_Decimal, "#####0.0", NULL, NULL },
    { Setting_DualAxisLengthFailMax, Group_Limits_DualAxis, "Dual axis length fail max", "mm/min", Format_Decimal, "#####0.0", NULL, NULL },
    { Setting_THC_Mode, Group_Plasma, "Plasma mode", NULL, Format_RadioButtons, "Off,Up/down,Voltage", NULL, NULL },
    { Setting_THC_Delay, Group_Plasma, "Plasma THC delay", "s", Format_Decimal, "#0.0", NULL, NULL },
    { Setting_THC_Threshold, Group_Plasma, "Plasma THC threshold", "V", Format_Decimal, "#0.00", NULL, NULL },
    { Setting_THC_PGain, Group_Plasma, "Plasma THC P-gain", NULL, Format_Decimal, "###0.000", NULL, NULL },
    { Setting_THC_IGain, Group_Plasma, "Plasma THC I-gain", NULL, Format_Decimal, "###0.000", NULL, NULL },
    { Setting_THC_DGain, Group_Plasma, "Plasma THC D-gain", NULL, Format_Decimal, "###0.000", NULL, NULL },
    { Setting_THC_VADThreshold, Group_Plasma, "Plasma THC VAD threshold", "percent", Format_Integer, "#0", NULL, NULL },
    { Setting_THC_VoidOverride, Group_Plasma, "Plasma THC Void override", "percent", Format_Integer, "#0", NULL, NULL },
    { Setting_Arc_FailTimeout, Group_Plasma, "Plasma Arc fail timeout", "seconds", Format_Decimal, "#0.0", NULL, NULL },
    { Setting_Arc_RetryDelay, Group_Plasma, "Plasma Arc retry delay", "seconds", Format_Decimal, "#0.0", NULL, NULL },
    { Setting_Arc_MaxRetries, Group_Plasma, "Plasma Arc max retries", NULL, Format_Integer, "#0", NULL, NULL },
    { Setting_Arc_VoltageScale, Group_Plasma, "Plasma Arc voltage scale", NULL, Format_Decimal, "###0.000", NULL, NULL },
    { Setting_Arc_VoltageOffset, Group_Plasma, "Plasma Arc voltage offset", NULL, Format_Decimal, "###0.000", NULL, NULL },
    { Setting_Arc_HeightPerVolt, Group_Plasma, "Plasma Arc height per volt", "mm", Format_Decimal, "###0.000", NULL, NULL },
    { Setting_Arc_OkHighVoltage, Group_Plasma, "Plasma Arc ok high volts", "V", Format_Decimal, "###0.000", NULL, NULL },
    { Setting_Arc_OkLowVoltage, Group_Plasma, "Plasma Arc ok low volts", "V", Format_Decimal, "###0.000", NULL, NULL },
    { Settings_IoPort_InvertIn, Group_AuxPorts, "Invert I/O Port inputs", NULL, Format_Bitfield, "Port 0,Port 1,Port 2,Port 3,Port 4,Port 5,Port 6,Port 7", NULL, NULL },
    { Settings_IoPort_Pullup_Disable, Group_AuxPorts, "I/O Port inputs pullup disable", NULL, Format_Bitfield, "Port 0,Port 1,Port 2,Port 3,Port 4,Port 5,Port 6,Port 7", NULL, NULL },
    { Settings_IoPort_InvertOut, Group_AuxPorts, "Invert I/O Port outputs", NULL, Format_Bitfield, "Port 0,Port 1,Port 2,Port 3,Port 4,Port 5,Port 6,Port 7", NULL, NULL },
    { Settings_IoPort_OD_Enable, Group_AuxPorts, "I/O Port outputs as open drain", NULL, Format_Bitfield, "Port 0,Port 1,Port 2,Port 3,Port 4,Port 5,Port 6,Port 7", NULL, NULL },
    { Setting_EncoderModeBase, Group_Encoder, "Encoder mode", NULL, Format_RadioButtons, "Universal,Feed rate override,Rapid rate override,Spindle RPM override", NULL, NULL },
    { Setting_EncoderCPRBase, Group_Encoder, "Encoder counts per revolution (CPR)", NULL, Format_Integer, "###0", "1", NULL },
    { Setting_EncoderCPDBase, Group_Encoder, "Encoder counts per detent (CPD)", NULL, Format_Integer, "#0", "1", NULL },
    { Setting_EncoderDblClickWindowBase, Group_Encoder, "Encoder double click sensitivity", "ms", Format_Integer, "##0", "100", "900" }
};

extern settings_t settings;

// Initialize the configuration subsystem (load settings from persistent storage)
void settings_init();

// Helper function to clear and restore persistent storage defaults
void settings_restore(settings_restore_t restore_flags);

// A helper method to set new settings from command line
status_code_t settings_store_global_setting(setting_type_t setting, char *svalue);

// Writes the protocol line variable as a startup line in persistent storage
void settings_write_startup_line(uint8_t idx, char *line);

// Reads an persistent storage startup line to the protocol line variable
bool settings_read_startup_line(uint8_t idx, char *line);

// Writes build info user-defined string
void settings_write_build_info(char *line);

// Reads build info user-defined string
bool settings_read_build_info(char *line);

// Writes selected coordinate data to persistent storage
void settings_write_coord_data(coord_system_id_t id, float (*coord_data)[N_AXIS]);

// Reads selected coordinate data from persistent storage
bool settings_read_coord_data(coord_system_id_t id, float (*coord_data)[N_AXIS]);

// Writes selected tool data to persistent storage
bool settings_write_tool_data (tool_data_t *tool_data);

// Read selected tool data from persistent storage
bool settings_read_tool_data (uint32_t tool, tool_data_t *tool_data);

bool settings_is_group_available (setting_group_t group);
bool settings_is_setting_available (setting_type_t setting, setting_group_t group);

#endif
