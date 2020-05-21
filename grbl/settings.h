/*
  settings.h - eeprom configuration handling
  Part of Grbl

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

#ifndef settings_h
#define settings_h

#include "config.h"
#include "system.h"

// Version of the persistent storage data. Will be used to migrate existing data from older versions of Grbl
// when firmware is upgraded. Always stored in byte 0 of eeprom
#define SETTINGS_VERSION 15  // NOTE: Check settings_reset() when moving to next version.

// Define persistent storage memory address location values for Grbl settings and parameters
// NOTE: 1KB persistent storage is the minimum required. The upper half is reserved for parameters and
// the startup script. The lower half contains the global settings and space for future
// developments.
#define EEPROM_ADDR_GLOBAL         1U
#define EEPROM_ADDR_PARAMETERS     512U
#define EEPROM_ADDR_BUILD_INFO     942U
#define EEPROM_ADDR_STARTUP_BLOCK  (EEPROM_ADDR_BUILD_INFO - 1 - N_STARTUP_LINE * (MAX_STORED_LINE_LENGTH + 1))
#ifdef N_TOOLS
#define EEPROM_ADDR_TOOL_TABLE     (EEPROM_ADDR_PARAMETERS - 1 - N_TOOLS * (sizeof(tool_data_t) + 1))
#endif

// Define persistent storage address indexing for coordinate parameters
#if COMPATIBILITY_LEVEL <= 1
#define N_COORDINATE_SYSTEM 9  // Number of supported work coordinate systems (from index 1)
#else
#define N_COORDINATE_SYSTEM 6  // Number of supported work coordinate systems (from index 1)
#endif
#define SETTING_INDEX_NCOORD (N_COORDINATE_SYSTEM + 2)  // Total number of coordinate system stored (from index 0)
// NOTE: Work coordinate indices are (0=G54, 1=G55, ... , 6=G59)
#define SETTING_INDEX_G28    N_COORDINATE_SYSTEM        // Home position 1
#define SETTING_INDEX_G30    (N_COORDINATE_SYSTEM + 1)  // Home position 2
#if N_COORDINATE_SYSTEM >= 9
#define SETTING_INDEX_G59_3  (N_COORDINATE_SYSTEM - 1)  // G59.3 position
#endif
#define SETTING_INDEX_G92    (N_COORDINATE_SYSTEM + 2)  // Coordinate offset

typedef enum {
    SettingIndex_G54 = 0,
    SettingIndex_G55,
    SettingIndex_G56,
    SettingIndex_G57,
    SettingIndex_G58,
    SettingIndex_G59,
#if N_COORDINATE_SYSTEM >= 9
    SettingIndex_G59_1,
    SettingIndex_G59_2,
    SettingIndex_G59_3,
#endif
    SettingIndex_G28,   // Home position 1
    SettingIndex_G30,   // Home position 2
    SettingIndex_G92,   // Coordinate offset
    SettingIndex_NCoord
} setting_coord_system_t;

#define N_COORDINATE_SYSTEMS (SettingIndex_NCoord - 3)  // Number of supported work coordinate systems (from index 1)

// Define Grbl axis settings numbering scheme. Starts at Setting_AxisSettingsBase, every INCREMENT, over N_SETTINGS.
#ifdef ENABLE_BACKLASH_COMPENSATION
#define AXIS_N_SETTINGS          6
#else
#define AXIS_N_SETTINGS          4
#endif
#define AXIS_SETTINGS_INCREMENT  10  // Must be greater than the number of axis settings TODO: change to 100 to allow for a logical wider range of parameters?

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
#if COMPATIBILITY_LEVEL <= 1
    Setting_ControlInvertMask = 14,
    Setting_CoolantInvertMask = 15,
    Setting_SpindleInvertMask = 16,
    Setting_ControlPullUpDisableMask = 17,
    Setting_LimitPullUpDisableMask = 18,
    Setting_ProbePullUpDisable = 19,
#endif
    Setting_SoftLimitsEnable = 20,
    Setting_HardLimitsEnable = 21,
    Setting_HomingEnable = 22,
    Setting_HomingDirMask = 23,
    Setting_HomingFeedRate = 24,
    Setting_HomingSeekRate = 25,
    Setting_HomingDebounceDelay = 26,
    Setting_HomingPulloff = 27,
#if COMPATIBILITY_LEVEL <= 1
    Setting_G73Retract = 28,
    Setting_PulseDelayMicroseconds = 29,
#endif
    Setting_RpmMax = 30,
    Setting_RpmMin = 31,
    Setting_Mode = 32,
#if COMPATIBILITY_LEVEL <= 1 // TODO: 33-36 is defined in grbl-LPC, add separate level or always include?
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
#endif

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

    Setting_AxisSettingsBase = 100, // NOTE: Reserving settings values >= 100 for axis settings. Up to 255.
    Setting_AxisSettingsMax = 255,

// Optional driver implemented settings
    Setting_TrinamicDriver = 256,
    Setting_TrinamicHoming = 257,

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

    Setting_SettingsMax
//
} setting_type_t;

typedef enum {
    AxisSetting_StepsPerMM = 0,
    AxisSetting_MaxRate = 1,
    AxisSetting_Acceleration = 2,
    AxisSetting_MaxTravel = 3,
    AxisSetting_StepperCurrent = 4,
    AxisSetting_MicroSteps = 5,
    AxisSetting_Backlash = 6
    /*
    AxisSetting_P_Gain = 7,
    AxisSetting_I_Gain = 8,
    AxisSetting_D_Gain = 9,
    AxisSetting_I_MaxError = 10
    */
} axis_setting_type_t;

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

typedef union {
    uint16_t value;
    struct {
        uint16_t report_inches                  :1,
                laser_mode                      :1,
                invert_probe_pin                :1,
                disable_probe_pullup            :1,
                restore_overrides               :1,
                safety_door_ignore_when_idle    :1,
                sleep_enable                    :1,
                disable_laser_during_hold       :1,
                force_initialization_alarm      :1,
                report_parser_state             :1, // TODO: move reportmask_t on next SETTINGS_VERSION change
                allow_probing_feed_override     :1,
                report_alarm_substate           :1, // TODO: move reportmask_t on next SETTINGS_VERSION change
                restore_after_feed_hold         :1,
                unassigned1                     :1,
                force_buffer_sync_on_wco_change :1, // TODO: move reportmask_t on next SETTINGS_VERSION change
                lathe_mode                      :1;
    };
} settingflags_t;

typedef union {
    uint8_t mask;
    struct {
        uint8_t machine_position  :1,
                buffer_state      :1,
                line_numbers      :1,
                feed_speed        :1,
                pin_state         :1,
                work_coord_offset :1,
                overrides         :1,
                probe_coordinates :1;
        // TODO: expand to 16 bit and move some report settings from settingflags_t here on next SETTINGS_VERSION change.
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
                unassigned           :3;
    };
} homing_settings_flags_t;

typedef struct {
    float feed_rate;
    float seek_rate;
    float pulloff;
    axes_signals_t dir_mask;
    uint8_t locate_cycles;
    uint16_t debounce_delay;
    homing_settings_flags_t flags;
    axes_signals_t cycle[N_AXIS];
} homing_settings_t;

typedef struct {
    axes_signals_t step_invert;
    axes_signals_t dir_invert;
    axes_signals_t enable_invert;
    axes_signals_t deenergize;
    uint8_t pulse_microseconds;
    uint8_t pulse_delay_microseconds;
    uint8_t idle_lock_time; // If max value 255, steppers do not disable.
} stepper_settings_t;

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

// Global persistent settings (Stored from byte persistent storage_ADDR_GLOBAL onwards)
typedef struct {
    // Settings struct version
    uint32_t version;
    // Axis settings
    float steps_per_mm[N_AXIS];
    float max_rate[N_AXIS];
    float acceleration[N_AXIS];
    float max_travel[N_AXIS];
#ifdef ENABLE_BACKLASH_COMPENSATION
    float backlash[N_AXIS];
#endif
    float junction_deviation;
    float arc_tolerance;
    float g73_retract;
    bool legacy_rt_commands;
    control_signals_t control_invert;
    control_signals_t control_disable_pullup;
    coolant_state_t coolant_invert;
    spindle_settings_t spindle;
    stepper_settings_t steppers;
    reportmask_t status_report; // Mask to indicate desired report data.
    settingflags_t flags;       // Contains default boolean settings
    homing_settings_t homing;
    limit_settings_t limits;
    parking_settings_t parking;
    position_pid_t position;    // Used for synchronized motion
} settings_t;

// Setting structs that may be used by drivers

typedef struct {
    float fast_speed;
    float slow_speed;
    float step_speed;
    float fast_distance;
    float slow_distance;
    float step_distance;
} jog_settings_t;

typedef enum {
    IpMode_Static = 0,
    IpMode_DHCP,
    IpMode_AutoIP
} ip_mode_t;

typedef union {
    uint8_t mask;
    struct {
        uint8_t telnet     :1,
                websocket  :1,
                http       :1,
                dns        :1,
                mdns       :1,
                ssdp       :1,
                unassigned :2;
    };
} network_services_t;

typedef char ssid_t[65];
typedef char password_t[33];
typedef char hostname_t[33];

typedef struct {
    char ip[16];
    char gateway[16];
    char mask[16];
    hostname_t hostname;
    uint16_t telnet_port;
    uint16_t websocket_port;
    uint16_t http_port;
    ip_mode_t ip_mode;
    network_services_t services;
} network_settings_t;

typedef enum {
    WiFiMode_NULL = 0,
    WiFiMode_STA,
    WiFiMode_AP,
    WiFiMode_APSTA
} grbl_wifi_mode_t;

typedef struct {
    ssid_t ssid;
    password_t password;
    char country[4];
    uint8_t channel;
    network_settings_t network;
} wifi_ap_settings_t;

typedef struct {
    ssid_t ssid;
    password_t password;
    network_settings_t network;
} wifi_sta_settings_t;

typedef struct {
    char device_name[33];
    char service_name[33];
} bluetooth_settings_t;

// End of setting structs that may be used by drivers

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
void settings_write_coord_data(uint8_t idx, float (*coord_data)[N_AXIS]);

// Reads selected coordinate data from persistent storage
bool settings_read_coord_data(uint8_t idx, float (*coord_data)[N_AXIS]);

// Writes selected tool data to persistent storage
bool settings_write_tool_data (tool_data_t *tool_data);

// Read selected tool data from persistent storage
bool settings_read_tool_data (uint8_t tool, tool_data_t *tool_data);

#endif
