/*
  gcode.h - rs274/ngc parser.

  Part of GrblHAL

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

#ifndef _GCODE_H_
#define _GCODE_H_

#include "nuts_bolts.h"
#include "coolant_control.h"
#include "spindle_control.h"

// Define Grbl status codes. Valid values (0-255)
typedef enum {
    Status_OK = 0,
    Status_ExpectedCommandLetter = 1,
    Status_BadNumberFormat = 2,
    Status_InvalidStatement = 3,
    Status_NegativeValue = 4,
    Status_SettingDisabled = 5,
    Status_SettingStepPulseMin = 6,
    Status_SettingReadFail = 7,
    Status_IdleError = 8,
    Status_SystemGClock = 9,
    Status_SoftLimitError = 10,
    Status_Overflow = 11,
    Status_MaxStepRateExceeded = 12,
    Status_CheckDoor = 13,
    Status_LineLengthExceeded = 14,
    Status_TravelExceeded = 15,
    Status_InvalidJogCommand = 16,
    Status_SettingDisabledLaser = 17,
    Status_Reset = 18,
    Status_NonPositiveValue = 19,

    Status_GcodeUnsupportedCommand = 20,
    Status_GcodeModalGroupViolation = 21,
    Status_GcodeUndefinedFeedRate = 22,
    Status_GcodeCommandValueNotInteger = 23,
    Status_GcodeAxisCommandConflict = 24,
    Status_GcodeWordRepeated = 25,
    Status_GcodeNoAxisWords = 26,
    Status_GcodeInvalidLineNumber = 27,
    Status_GcodeValueWordMissing = 28,
    Status_GcodeUnsupportedCoordSys = 29,
    Status_GcodeG53InvalidMotionMode = 30,
    Status_GcodeAxisWordsExist = 31,
    Status_GcodeNoAxisWordsInPlane = 32,
    Status_GcodeInvalidTarget = 33,
    Status_GcodeArcRadiusError = 34,
    Status_GcodeNoOffsetsInPlane = 35,
    Status_GcodeUnusedWords = 36,
    Status_GcodeG43DynamicAxisError = 37,
    Status_GcodeIllegalToolTableEntry = 38,
    Status_GcodeValueOutOfRange = 39,
    Status_GcodeToolChangePending = 40,
    Status_GcodeSpindleNotRunning = 41,
    Status_GcodeIllegalPlane = 42,
    Status_GcodeMaxFeedRateExceeded = 43,
    Status_GcodeRPMOutOfRange = 44,
    Status_LimitsEngaged = 45,
    Status_HomingRequired = 46,
    Status_GCodeToolError = 47,
    Status_ValueWordConflict = 48,

    Status_EStop = 50,
    Status_Unhandled = 59, // For internal use only

// Some error codes as defined in bdring's ESP32 port
    Status_SDMountError = 60,
    Status_SDReadError = 61,
    Status_SDFailedOpenDir = 62,
    Status_SDDirNotFound = 63,
    Status_SDFileEmpty = 64,

    Status_BTInitError = 70
} status_code_t;


// Define modal group internal numbers for checking multiple command violations and tracking the
// type of command that is called in the block. A modal group is a group of g-code commands that are
// mutually exclusive, or cannot exist on the same line, because they each toggle a state or execute
// a unique motion. These are defined in the NIST RS274-NGC v3 g-code standard, available online,
// and are similar/identical to other g-code interpreters by manufacturers (Haas,Fanuc,Mazak,etc).
// NOTE: Modal group define values must be sequential and starting from zero.
typedef enum {
    ModalGroup_G0 = 0,  // [G4,G10,G28,G28.1,G30,G30.1,G53,G92,G92.1] Non-modal
    ModalGroup_G1,      // [G0,G1,G2,G3,G33,G38.2,G38.3,G38.4,G38.5,G76,G80] Motion
    ModalGroup_G2,      // [G17,G18,G19] Plane selection
    ModalGroup_G3,      // [G90,G91] Distance mode
    ModalGroup_G4,      // [G91.1] Arc IJK distance mode
    ModalGroup_G5,      // [G93,G94,G95] Feed rate mode
    ModalGroup_G6,      // [G20,G21] Units
    ModalGroup_G7,      // [G40] Cutter radius compensation mode. G41/42 NOT SUPPORTED.
    ModalGroup_G8,      // [G43,G43.1,G49] Tool length offset
    ModalGroup_G10,     // [G98,G99] Return mode in canned cycles
    ModalGroup_G11,     // [G50,G51] Scaling
    ModalGroup_G12,     // [G54,G55,G56,G57,G58,G59] Coordinate system selection
    ModalGroup_G13,     // [G61] Control mode
    ModalGroup_G14,     // [G96,G97] Spindle Speed Mode
    ModalGroup_G15,     // [G7,G8] Lathe Diameter Mode

    ModalGroup_M4,      // [M0,M1,M2,M30] Stopping
    ModalGroup_M6,      // [M6] Tool change
    ModalGroup_M7,      // [M3,M4,M5] Spindle turning
    ModalGroup_M8,      // [M7,M8,M9] Coolant control
    ModalGroup_M9,      // [M49,M50,M51,M53,M56] Override control
    ModalGroup_M10,     // User defined M commands
} modal_group_t;

// Define parameter word mapping.
typedef enum {
    Word_E = 0,
    Word_F,
    Word_H,
    Word_I,
    Word_J,
    Word_K,
    Word_L,
    Word_N,
    Word_P,
    Word_R,
    Word_S,
    Word_T,
    Word_X,
    Word_Y,
    Word_Z,
    Word_Q,
#if N_AXIS > 3
    Word_A,
    Word_B,
    Word_C,
#endif
    Word_D
} parameter_word_t;

#if N_AXIS == 3
#define AXIS_WORDS_MASK ((1 << Word_X)|(1 << Word_Y)|(1 << Word_Z))
#elif N_AXIS == 4
#define AXIS_WORDS_MASK ((1 << Word_X)|(1 << Word_Y)|(1 << Word_Z)|(1 << Word_A))
#elif N_AXIS == 5
#define AXIS_WORDS_MASK ((1 << Word_X)|(1 << Word_Y)|(1 << Word_Z)|(1 << Word_A)|(1 << Word_B))
#else
#define AXIS_WORDS_MASK ((1 << Word_X)|(1 << Word_Y)|(1 << Word_Z)|(1 << Word_A)|(1 << Word_B)|(1 << Word_C))
#endif

typedef union {
    parameter_word_t parameter;
    modal_group_t group;
} word_bit_t;

// Define command actions for within execution-type modal groups (motion, stopping, non-modal). Used
// internally by the parser to know which command to execute.
// NOTE: Some values are assigned specific values to make g-code state reporting and parsing
// compile a litte smaller. Although not
// ideal, just be careful with values that state 'do not alter' and check both report.c and gcode.c
// to see how they are used, if you need to alter them.

// Modal Group G0: Non-modal actions
typedef enum {
    NonModal_NoAction = 0,                  // (Default: Must be zero)
    NonModal_Dwell = 4,                     // G4 (Do not alter value)
    NonModal_SetCoordinateData = 10,        // G10 (Do not alter value)
    NonModal_GoHome_0 = 28,                 // G28 (Do not alter value)
    NonModal_SetHome_0 = 38,                // G28.1 (Do not alter value)
    NonModal_GoHome_1 = 30,                 // G30 (Do not alter value)
    NonModal_SetHome_1 = 40,                // G30.1 (Do not alter value)
    NonModal_AbsoluteOverride = 53,         // G53 (Do not alter value)
    NonModal_SetCoordinateOffset = 92,      // G92 (Do not alter value)
    NonModal_ResetCoordinateOffset = 102,   // G92.1 (Do not alter value)
    NonModal_ClearCoordinateOffset = 112,   // G92.2 (Do not alter value)
    NonModal_RestoreCoordinateOffset = 122  // G92.3 (Do not alter value)
} non_modal_t;

// Modal Group G1: Motion modes
typedef enum {
    MotionMode_Seek = 0,                    // G0 (Default: Must be zero)
    MotionMode_Linear = 1,                  // G1 (Do not alter value)
    MotionMode_CwArc = 2,                   // G2 (Do not alter value)
    MotionMode_CcwArc = 3,                  // G3 (Do not alter value)
    MotionMode_CubicSpline = 5,             // G5 (Do not alter value)
    MotionMode_SpindleSynchronized = 33,    // G33 (Do not alter value)
    MotionMode_DrillChipBreak = 73,         // G73 (Do not alter value)
    MotionMode_Threading = 76,              // G76 (Do not alter value)
    MotionMode_CannedCycle81 = 81,          // G81 (Do not alter value)
    MotionMode_CannedCycle82 = 82,          // G82 (Do not alter value)
    MotionMode_CannedCycle83 = 83,          // G83 (Do not alter value)
    MotionMode_CannedCycle85 = 85,          // G85 (Do not alter value)
    MotionMode_CannedCycle86 = 86,          // G86 (Do not alter value)
    MotionMode_CannedCycle89 = 89,          // G89 (Do not alter value)
    MotionMode_ProbeToward = 140,           // G38.2 (Do not alter value)
    MotionMode_ProbeTowardNoError = 141,    // G38.3 (Do not alter value)
    MotionMode_ProbeAway = 142,             // G38.4 (Do not alter value)
    MotionMode_ProbeAwayNoError = 143,      // G38.5 (Do not alter value)
    MotionMode_None = 80                    // G80 (Do not alter value)
} motion_mode_t;

// Modal Group G2: Plane select
typedef enum {
    PlaneSelect_XY = 0, // G17 (Default: Must be zero)
    PlaneSelect_ZX = 1, // G18 (Do not alter value)
    PlaneSelect_YZ = 2  // G19 (Do not alter value)
} plane_select_t;

// Modal Group G4: Arc IJK distance mode
//#define DISTANCE_ARC_MODE_INCREMENTAL 0 // G91.1 (Default: Must be zero)

// Modal Group M4: Program flow
typedef enum {
    ProgramFlow_Running = 0,        // (Default: Must be zero)
    ProgramFlow_Paused  = 3,        // M0
    ProgramFlow_OptionalStop = 1,   // M1
    ProgramFlow_CompletedM2 = 2,    // M2 (Do not alter value)
    ProgramFlow_CompletedM30 = 30,  // M30 (Do not alter value)
    ProgramFlow_CompletedM60 = 60   // M60 (Do not alter value)
} program_flow_t;

// Modal Group G5: Feed rate mode
typedef enum {
    FeedMode_UnitsPerMin = 0,   // G94 (Default: Must be zero)
    FeedMode_InverseTime = 1,   // G93 (Do not alter value)
    FeedMode_UnitsPerRev = 2    // G95 (Do not alter value)
} feed_mode_t;

// Modal Group G10: Canned cycle return mode
typedef enum {
    CCRetractMode_Previous = 0,  // G98 (Default: Must be zero)
    CCRetractMode_RPos = 1       // G99 (Do not alter value)
} cc_retract_mode_t;

// Modal Group G7: Cutter radius compensation mode
//#define CUTTER_COMP_DISABLE 0 // G40 (Default: Must be zero)

// Modal Group G13: Control mode
//#define CONTROL_MODE_EXACT_PATH 0 // G61 (Default: Must be zero)

// Modal Group G8: Tool length offset
typedef enum {
    ToolLengthOffset_Cancel = 0,         // G49 (Default: Must be zero)
    ToolLengthOffset_Enable = 1,         // G43
    ToolLengthOffset_EnableDynamic = 2,  // G43.1
    ToolLengthOffset_ApplyAdditional = 3 // G43.2
} tool_offset_mode_t;

// Modal Group M9: Override control
typedef enum {
    Override_Parking = 56,          // M56
    Override_FeedHold = 53,         // M53
    Override_FeedSpeed = 49,        // M49
    Override_FeedRate = 50,         // M50
    Override_SpindleSpeed = 51      // M51
} override_mode_t;

// Modal Group G12: Active work coordinate system
// N/A: Stores coordinate system value (54-59) to change to.

// Modal Group G14: Spindle Speed Mode
typedef enum {
    SpindleSpeedMode_RPM = 0,  // G96 (Default: Must be zero)
    SpindleSpeedMode_CSS = 1   // G97 (Do not alter value)
} spindle_rpm_mode_t;


typedef struct output_command {
    bool is_digital;
    bool is_executed;
    uint8_t port;
    int32_t value;
    struct output_command *next;
} output_command_t;

typedef enum {
    WaitMode_Immediate = 0,
    WaitMode_Rise,
    WaitMode_Fall,
    WaitMode_High,
    WaitMode_Low,
    WaitMode_Max // Used for validation only
} wait_mode_t;

// Modal Group M10: User defined M commands
// NOTE: Not used by core, may be used by driver code
typedef enum {
    UserMCode_Ignore = 0,
    Trinamic_DebugReport = 122,
    Trinamic_StepperCurrent = 906,
    Trinamic_ModeToggle = 569,
    Trinamic_ReportPrewarnFlags = 911,
    Trinamic_ClearPrewarnFlags = 912,
    Trinamic_HybridThreshold = 913,
    Trinamic_HomingSensivity = 914
} user_mcode_t;

// Define g-code parser position updating flags
typedef enum {
    GCUpdatePos_Target = 0,
    GCUpdatePos_System,
    GCUpdatePos_None
} pos_update_t;

// Define probe cycle exit states and assign proper position updating.
typedef enum {
    GCProbe_Found = GCUpdatePos_System,
    GCProbe_Abort = GCUpdatePos_None,
    GCProbe_FailInit = GCUpdatePos_None,
    GCProbe_FailEnd = GCUpdatePos_Target,
  #ifdef SET_CHECK_MODE_PROBE_TO_START
    GCProbe_CheckMode = GCUpdatePos_None
  #else
    GCProbe_CheckMode = GCUpdatePos_Target
  #endif
} gc_probe_t;

// Define gcode parser flags for handling special cases.

typedef union {
    uint16_t value;
    struct {
        uint16_t jog_motion          :1,
                 canned_cycle_change :1, // Use motion_mode_changed?
                 arc_is_clockwise    :1,
                 probe_is_away       :1,
                 probe_is_no_error   :1,
                 spindle_force_sync  :1,
                 laser_disable       :1,
                 laser_is_motion     :1,
                 set_coolant         :1,
                 motion_mode_changed :1,
                 reserved            :6;
    };
} gc_parser_flags_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t feed_rate_disable   :1,
                feed_hold_disable   :1,
                spindle_rpm_disable :1,
                parking_disable     :1,
                reserved            :3,
                sync                :1;
    };
} gc_override_flags_t;

typedef union {
    float values[N_AXIS];
    struct {
        float x;
        float y;
        float z;
#ifdef A_AXIS
        float a;
#endif
#ifdef B_AXIS
        float b;
#endif
#ifdef C_AXIS
        float c;
#endif
    };
} coord_data_t;

typedef enum {
    CoordinateSystem_G54 = 0,
    CoordinateSystem_G55,
    CoordinateSystem_G56,
    CoordinateSystem_G57,
    CoordinateSystem_G58,
    CoordinateSystem_G59,
#if COMPATIBILITY_LEVEL <= 1
    CoordinateSystem_G59_1,
    CoordinateSystem_G59_2,
    CoordinateSystem_G59_3,
#endif
    N_WorkCoordinateSystems,
    CoordinateSystem_G28 = N_WorkCoordinateSystems,
    CoordinateSystem_G30,
    CoordinateSystem_G92,
    N_CoordinateSystems
} coord_system_id_t;

typedef struct {
    float xyz[N_AXIS];
    coord_system_id_t id;
} coord_system_t;

typedef struct {
    uint8_t axis_0;
    uint8_t axis_1;
    uint8_t axis_linear;
} plane_t;

// NOTE: When this struct is zeroed, the above defines set the defaults for the system.
typedef struct {
    motion_mode_t motion;                // {G0,G1,G2,G3,G38.2,G80}
    feed_mode_t feed_mode;               // {G93,G94}
    bool units_imperial;                 // {G20,G21}
    bool distance_incremental;           // {G90,G91}
    bool diameter_mode;                  // {G7,G8} Lathe diameter mode.
    // uint8_t distance_arc;             // {G91.1} NOTE: Don't track. Only default supported.
    plane_select_t plane_select;         // {G17,G18,G19}
    // uint8_t cutter_comp;              // {G40} NOTE: Don't track. Only default supported.
    tool_offset_mode_t tool_offset_mode; // {G43,G43.1,G49}
    coord_system_t coord_system;         // {G54,G55,G56,G57,G58,G59,G59.1,G59.2,G59.3}
    // uint8_t control;                  // {G61} NOTE: Don't track. Only default supported.
    program_flow_t program_flow;         // {M0,M1,M2,M30}
    coolant_state_t coolant;             // {M7,M8,M9}
    spindle_state_t spindle;             // {M3,M4,M5}
    gc_override_flags_t override_ctrl;   // {M48,M49,M50,M51,M53,M56}
    spindle_rpm_mode_t spindle_rpm_mode; // {G96,G97}
    cc_retract_mode_t retract_mode;      // {G98,G99}
    bool scaling_active;                 // {G50,G51}
    bool canned_cycle_active;
    float spline_pq[2];                  // {G5}
} gc_modal_t;

typedef struct {
    float d;                   // Max spindle RPM in Constant Surface Speed Mode (G96)
    float e;                   // Thread taper length (G76)
    float f;                   // Feed
    float ijk[3];              // I,J,K Axis arc offsets
    float k;                   // G33 distance per revolution
    float p;                   // G10 or dwell parameters
    float q;                   // User defined M-code parameter (G82 peck drilling, not supported)
    float r;                   // Arc radius or retract position
    float s;                   // Spindle speed
    float xyz[N_AXIS];         // X,Y,Z Translational axes
    coord_system_t coord_data; // Coordinate data
    int32_t n;                 // Line number
    uint32_t h;                // Tool number
    uint32_t t;                // Tool selection
    uint8_t l;                 // G10 or canned cycles parameters
} gc_values_t;

typedef struct {
    float xyz[3];
    float delta;
    float dwell;
    float prev_position;
    float retract_position; // Canned cycle retract position
    bool rapid_retract;
    bool spindle_off;
    cc_retract_mode_t retract_mode;
    bool change;
} gc_canned_t;

typedef enum {
    Taper_None = 0,
    Taper_Entry,
    Taper_Exit,
    Taper_Both
} gc_taper_type;

typedef struct {
    float pitch;
    float z_final;
    float peak;
    float initial_depth;
    float depth;
    float depth_degression;
    float main_taper_height;
    float end_taper_length;
    float infeed_angle;
    float cut_direction;
    uint_fast16_t spring_passes;
    gc_taper_type end_taper_type;
} gc_thread_data;

typedef struct {
    float offset[N_AXIS];
    float radius;
    uint32_t tool;
} tool_data_t;

// Data used for Constant Surface Speed Mode calculations
typedef struct {
    float surface_speed;    // Surface speed in millimeters/min
    float target_rpm;       // Target RPM at end of movement
    float max_rpm;          // Maximum spindle RPM
    float tool_offset;      // Tool offset
    uint_fast8_t axis;      // Linear (tool) axis
    bool active;
} css_data_t;

typedef struct {
    float rpm;      // RPM
    css_data_t css; // Data used for Constant Surface Speed Mode calculations
} spindle_t;

typedef struct {
    gc_modal_t modal;
    gc_canned_t canned;
    spindle_t spindle;                  // RPM
    float feed_rate;                    // Millimeters/min
    float distance_per_rev;             // Millimeters/rev
    float position[N_AXIS];             // Where the interpreter considers the tool to be at this point in the code
    int32_t line_number;                // Last line number sent
    uint32_t tool_pending;              // Tool to be selected on next M6
    bool file_run;                      // Tracks % command
    bool is_laser_ppi_mode;
    bool is_rpm_rate_adjusted;
    bool tool_change;
    status_code_t last_error;           // last return value from parser
    // The following variables are not cleared upon warm restart when COMPATIBILITY_LEVEL <= 1
    float g92_coord_offset[N_AXIS];     // Retains the G92 coordinate offset (work coordinates) relative to
                                        // machine zero in mm. Persistent and loaded from non-volatile storage
                                        // on boot when COMPATIBILITY_LEVEL <= 1
    float tool_length_offset[N_AXIS];   // Tracks tool length offset when enabled
    tool_data_t *tool;                  // Tracks tool number and tool offset
} parser_state_t;

typedef struct {
    float xyz[N_AXIS]; // Center point
    float ijk[N_AXIS]; // Scaling factors
} scale_factor_t;

extern parser_state_t gc_state;
#ifdef N_TOOLS
extern tool_data_t tool_table[N_TOOLS + 1];
#else
extern tool_data_t tool_table;
#endif

typedef struct {
    non_modal_t non_modal_command;
    override_mode_t override_command; // TODO: add to non_modal above?
    user_mcode_t user_mcode;
    bool user_mcode_sync;
    gc_modal_t modal;
    gc_values_t values;
    output_command_t output_command;
} parser_block_t;

// Initialize the parser
void gc_init(bool cold_start);

// Execute one block of rs275/ngc/g-code
status_code_t gc_execute_block(char *block, char *message);

// Sets g-code parser position in mm. Input in steps. Called by the system abort and hard
// limit pull-off routines.
#define gc_sync_position() system_convert_array_steps_to_mpos (gc_state.position, sys_position)

// Sets g-code parser and planner position in mm.
#define sync_position() plan_sync_position(); system_convert_array_steps_to_mpos (gc_state.position, sys_position)

void gc_set_laser_ppimode (bool on);

// Gets axes scaling state.
axes_signals_t gc_get_g51_state (void);
float *gc_get_scaling (void);

// Get current axis offset.
float gc_get_offset (uint_fast8_t idx);

void gc_set_tool_offset (tool_offset_mode_t mode, uint_fast8_t idx, int32_t offset);
plane_t *gc_get_plane_data (plane_t *plane, plane_select_t select);

#endif
