/*
  config.h - compile time configuration
  Part of Grbl

  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
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

// This file contains compile-time configurations for Grbl's internal system. For the most part,
// users will not need to directly modify these, but they are here for specific needs, i.e.
// performance tuning or adjusting to non-typical machines.

// IMPORTANT: Any changes here requires a full re-compiling of the source code to propagate them.

#ifndef config_h
#define config_h

#include "grbl.h"

// Define CPU pin map and default settings.
// NOTE: OEMs can avoid the need to maintain/update the defaults.h and cpu_map.h files and use only
// one configuration file by placing their specific defaults and pin map at the bottom of this file.
// If doing so, simply comment out these two defines and see instructions below.
#define DEFAULTS_GENERIC
#define CPU_MAP_GENERIC

// Serial baud rate
// #define BAUD_RATE 230400
#define BAUD_RATE 115200

// Define realtime command special characters. These characters are 'picked-off' directly from the
// serial read data stream and are not passed to the grbl line execution parser. Select characters
// that do not and must not exist in the streamed g-code program. ASCII control characters may be
// used, if they are available per user setup. Also, extended ASCII codes (>127), which are never in
// g-code programs, maybe selected for interface programs.
// NOTE: If changed, manually update help message in report.c.

#define CMD_EXIT 0x03 // ctrl-C
#define CMD_RESET 0x18 // ctrl-X
#define CMD_STOP 0x19 // ctrl-Y
#define CMD_STATUS_REPORT '?'
#define CMD_CYCLE_START '~'
#define CMD_FEED_HOLD '!'
#define CMD_PID_REPORT 0xA2

// NOTE: All override realtime commands must be in the extended ASCII character set, starting
// at character value 128 (0x80) and up to 255 (0xFF). If the normal set of realtime commands,
// such as status reports, feed hold, reset, and cycle start, are moved to the extended set
// space, protocol.c's protocol_process_realtime() will need to be modified to accomodate the change.
#define CMD_SAFETY_DOOR 0x84
#define CMD_JOG_CANCEL  0x85
//#define CMD_DEBUG_REPORT 0x86 // Only when DEBUG enabled, sends debug report in '{}' braces.
#define CMD_FEED_OVR_RESET 0x90         // Restores feed override value to 100%.
#define CMD_FEED_OVR_COARSE_PLUS 0x91
#define CMD_FEED_OVR_COARSE_MINUS 0x92
#define CMD_FEED_OVR_FINE_PLUS 0x93
#define CMD_FEED_OVR_FINE_MINUS 0x94
#define CMD_RAPID_OVR_RESET 0x95        // Restores rapid override value to 100%.
#define CMD_RAPID_OVR_MEDIUM 0x96
#define CMD_RAPID_OVR_LOW 0x97
// #define CMD_RAPID_OVR_EXTRA_LOW 0x98 // *NOT SUPPORTED*
#define CMD_SPINDLE_OVR_RESET 0x99      // Restores spindle override value to 100%.
#define CMD_SPINDLE_OVR_COARSE_PLUS 0x9A
#define CMD_SPINDLE_OVR_COARSE_MINUS 0x9B
#define CMD_SPINDLE_OVR_FINE_PLUS 0x9C
#define CMD_SPINDLE_OVR_FINE_MINUS 0x9D
#define CMD_SPINDLE_OVR_STOP 0x9E
#define CMD_COOLANT_FLOOD_OVR_TOGGLE 0xA0
#define CMD_COOLANT_MIST_OVR_TOGGLE 0xA1

// Constant surface speed (CSS) is for lathes, when enabled this adds support for G96/G97 as per linuxcnc specs.
// A compliant HAL driver is required as the final RPM calculations are left to the driver to handle.
// #define CONSTANT_SURFACE_SPEED_OPTION // Uncomment to enable

// If homing is enabled, homing init lock sets Grbl into an alarm state upon power up. This forces
// the user to perform the homing cycle (or override the locks) before doing anything else. This is
// mainly a safety feature to remind the user to home, since position is unknown to Grbl.
//#define HOMING_INIT_LOCK // Comment to disable

// After homing, Grbl will set by default the entire machine space into negative space, as is typical
// for professional CNC machines, regardless of where the limit switches are located. Uncomment this
// define to force Grbl to always set the machine origin at the homed location despite switch orientation.
// #define HOMING_FORCE_SET_ORIGIN // Uncomment to enable.

// Number of homing cycles performed after when the machine initially jogs to limit switches.
// This help in preventing overshoot and should improve repeatability. This value should be one or
// greater.
#define N_HOMING_LOCATE_CYCLE 1 // Integer (1-128)

// Number of blocks Grbl executes upon startup. These blocks are stored in EEPROM, where the size
// and addresses are defined in settings.h. With the current settings, up to 2 startup blocks may
// be stored and executed in order. These startup blocks would typically be used to set the g-code
// parser state depending on user preferences.
#define N_STARTUP_LINE 2 // Integer (1-2)

// Number of floating decimal points printed by Grbl for certain value types. These settings are
// determined by realistic and commonly observed values in CNC machines. For example, position
// values cannot be less than 0.001mm or 0.0001in, because machines can not be physically more
// precise this. So, there is likely no need to change these, but you can if you need to here.
// NOTE: Must be an integer value from 0 to ~4. More than 4 may exhibit round-off errors.
#define N_DECIMAL_COORDVALUE_INCH 4 // Coordinate or position value in inches
#define N_DECIMAL_COORDVALUE_MM   3 // Coordinate or position value in mm
#define N_DECIMAL_RATEVALUE_INCH  1 // Rate or velocity value in in/min
#define N_DECIMAL_RATEVALUE_MM    0 // Rate or velocity value in mm/min
#define N_DECIMAL_SETTINGVALUE    3 // Decimals for floating point setting values
#define N_DECIMAL_RPMVALUE        0 // RPM value in rotations per min.
#define N_DECIMAL_PIDVALUE        3 // PID value.

// If your machine has two limits switches wired in parallel to one axis, you will need to enable
// this feature. Since the two switches are sharing a single pin, there is no way for Grbl to tell
// which one is enabled. This option only effects homing, where if a limit is engaged, Grbl will
// alarm out and force the user to manually disengage the limit switch. Otherwise, if you have one
// limit switch for each axis, don't enable this option. By keeping it disabled, you can perform a
// homing cycle while on the limit switch and not have to move the machine off of it.
// #define LIMITS_TWO_SWITCHES_ON_AXES

// Upon a successful probe cycle, this option provides immediately feedback of the probe coordinates
// through an automatically generated message. If disabled, users can still access the last probe
// coordinates through Grbl '$#' print parameters.
#define MESSAGE_PROBE_COORDINATES // Enabled by default. Comment to disable.

// for testing... open is ignored when cycle starts
//#define SAFETY_DOOR_IGNORE_WHEN_IDLE

// After the safety door switch has been toggled and restored, this setting sets the power-up delay
// between restoring the spindle and coolant and resuming the cycle.
#define SAFETY_DOOR_SPINDLE_DELAY 4.0f // Float (seconds)
#define SAFETY_DOOR_COOLANT_DELAY 1.0f // Float (seconds)

// Enable CoreXY kinematics. Use ONLY with CoreXY machines.
// IMPORTANT: If homing is enabled, you must reconfigure the homing cycle #defines above to
// #define HOMING_CYCLE_0 X_AXIS_BIT and #define HOMING_CYCLE_1 Y_AXIS_BIT
// NOTE: This configuration option alters the motion of the X and Y axes to principle of operation
// defined at (http://corexy.com/theory.html). Motors are assumed to positioned and wired exactly as
// described, if not, motions may move in strange directions. Grbl requires the CoreXY A and B motors
// have the same steps per mm internally.
// #define COREXY // Default disabled. Uncomment to enable.

// By default, Grbl sets all input pins to normal-low operation with their internal pull-up resistors
// enabled. This simplifies the wiring for users by requiring only a normally closed (NC) switch connected
// to ground. It is not recommended to use normally-open (NO) switches as this increases the risk
// of electrical noise spuriously triggering the inputs. If normally-open (NO) switches are used the
// logic of the input signals should be be inverted with the invert settings below.
// The following options disable the internal pull-up resistors, and switches must be now connect to Vcc
// instead of ground.
// WARNING: When the pull-ups are disabled, this might require additional wiring with pull-down resistors!
//          Please check driver code and/or documentation.
// #define DISABLE_LIMIT_PINS_PULL_UP_MASK AXES_BITMASK
// #define DISABLE_LIMIT_PINS_PULL_UP_MASK (X_AXIS_BIT|Y_AXIS_BIT)
// #define DISABLE_CONTROL_PINS_PULL_UP_MASK SIGNAL_MASK
// #define DISABLE_CONTROL_PINS_PULL_UP_MASK (SIGNAL_SAFETYDOOR|SIGNAL_RESET)
// #define DISABLE_PROBE_PIN_PULL_UP

// Inverts logic of the stepper enable signal(s).
// NOTE: Not universally available for individual axes - check driver documentation.
//       Specify at least X_AXIS_BIT if a common enable signal is used.
// #define INVERT_ST_ENABLE_MASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT) // Default disabled. Uncomment to enable.
// Mask to be OR'ed with stepper disable signal(s). Axes configured will not be disabled.
// NOTE: Not universally available for individual axes - check driver documentation.
//       Specify at least X_AXIS_BIT if a common enable signal is used.
// #define ST_DEENERGIZE_MASK (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT) // Default disabled. Uncomment to enable.

// Inverts logic of the input signals based on a mask. This essentially means you are using
// normally-open (NO) switches on the specified pins, rather than the default normally-closed (NC) switches.
// NOTE: The first option will invert all control pins. The second option is an example of
// inverting only a few pins. See system.h and/or nuts_bolts.h for other signal definitions.
// #define INVERT_CONTROL_PIN_MASK SIGNAL_MASK // Default disabled. Uncomment to enable.
// #define INVERT_CONTROL_PIN_MASK (SIGNAL_SAFETYDOOR|SIGNAL_RESET) // Default disabled. Uncomment to enable.
// #define INVERT_LIMIT_PIN_MASK AXES_BITMASK // Default disabled. Uncomment to enable. Uncomment to enable.
// #define INVERT_LIMIT_PIN_MASK (X_AXIS_BIT|Y_AXIS_BIT) // Default disabled. Uncomment to enable.
// For inverting the probe pin use DEFAULT_INVERT_PROBE_PIN in defaults.h

// When Grbl powers-cycles or is hard reset with the MCU reset button, Grbl boots up with no ALARM
// by default. This is to make it as simple as possible for new users to start using Grbl. When homing
// is enabled and a user has installed limit switches, Grbl will boot up in an ALARM state to indicate
// Grbl doesn't know its position and to force the user to home before proceeding. This option forces
// Grbl to always initialize into an ALARM state regardless of homing or not. This option is more for
// OEMs and LinuxCNC users that would like this power-cycle behavior.
// #define FORCE_INITIALIZATION_ALARM // Default disabled. Uncomment to enable.

// At power-up or a reset, Grbl will check the limit switch states to ensure they are not active
// before initialization. If it detects a problem and the hard limits setting is enabled, Grbl will
// simply message the user to check the limits and enter an alarm state, rather than idle. Grbl will
// not throw an alarm message.
#define CHECK_LIMITS_AT_INIT

// ---------------------------------------------------------------------------------------
// ADVANCED CONFIGURATION OPTIONS:

// Enables code for debugging purposes. Not for general use and always in constant flux.
// #define DEBUG // Uncomment to enable. Default disabled.

// Configure rapid, feed, and spindle override settings. These values define the max and min
// allowable override values and the coarse and fine increments per command received. Please
// note the allowable values in the descriptions following each define.
#define DEFAULT_FEED_OVERRIDE           100 // 100%. Don't change this value.
#define MAX_FEED_RATE_OVERRIDE          200 // Percent of programmed feed rate (100-255). Usually 120% or 200%
#define MIN_FEED_RATE_OVERRIDE           10 // Percent of programmed feed rate (1-100). Usually 50% or 1%
#define FEED_OVERRIDE_COARSE_INCREMENT   10 // (1-99). Usually 10%.
#define FEED_OVERRIDE_FINE_INCREMENT      1 // (1-99). Usually 1%.

#define DEFAULT_RAPID_OVERRIDE  100 // 100%. Don't change this value.
#define RAPID_OVERRIDE_MEDIUM    50 // Percent of rapid (1-99). Usually 50%.
#define RAPID_OVERRIDE_LOW       25 // Percent of rapid (1-99). Usually 25%.
// #define RAPID_OVERRIDE_EXTRA_LOW 5 // *NOT SUPPORTED* Percent of rapid (1-99). Usually 5%.

#define DEFAULT_SPINDLE_RPM_OVERRIDE      100 // 100%. Don't change this value.
#define MAX_SPINDLE_RPM_OVERRIDE          200 // Percent of programmed spindle speed (100-255). Usually 200%.
#define MIN_SPINDLE_RPM_OVERRIDE           10 // Percent of programmed spindle speed (1-100). Usually 10%.
#define SPINDLE_OVERRIDE_COARSE_INCREMENT  10 // (1-99). Usually 10%.
#define SPINDLE_OVERRIDE_FINE_INCREMENT     1 // (1-99). Usually 1%.

// When a M2 or M30 program end command is executed, most g-code states are restored to their defaults.
// This compile-time option includes the restoring of the feed, rapid, and spindle speed override values
// to their default values at program end.
#define RESTORE_OVERRIDES_AFTER_PROGRAM_END // Default enabled. Comment to disable.

// Some status report data isn't necessary for realtime, only intermittently, because the values don't
// change often. The following macros configures how many times a status report needs to be called before
// the associated data is refreshed and included in the status report. However, if one of these value
// changes, Grbl will automatically include this data in the next status report, regardless of what the
// count is at the time. This helps reduce the communication overhead involved with high frequency reporting
// and agressive streaming. There is also a busy and an idle refresh count, which sets up Grbl to send
// refreshes more often when its not doing anything important. With a good GUI, this data doesn't need
// to be refreshed very often, on the order of a several seconds.
// NOTE: WCO refresh must be 2 or greater. OVR refresh must be 1 or greater.
#define REPORT_OVR_REFRESH_BUSY_COUNT 20  // (1-255)
#define REPORT_OVR_REFRESH_IDLE_COUNT 10  // (1-255) Must be less than or equal to the busy count
#define REPORT_WCO_REFRESH_BUSY_COUNT 30  // (2-255)
#define REPORT_WCO_REFRESH_IDLE_COUNT 10  // (2-255) Must be less than or equal to the busy count

// The temporal resolution of the acceleration management subsystem. A higher number gives smoother
// acceleration, particularly noticeable on machines that run at very high feedrates, but may negatively
// impact performance. The correct value for this parameter is machine dependent, so it's advised to
// set this only as high as needed. Approximate successful values can widely range from 50 to 200 or more.
// NOTE: Changing this value also changes the execution time of a segment in the step segment buffer.
// When increasing this value, this stores less overall time in the segment buffer and vice versa. Make
// certain the step segment buffer is increased/decreased to account for these changes.
#define ACCELERATION_TICKS_PER_SECOND 100

// Adaptive Multi-Axis Step Smoothing (AMASS) is an advanced feature that does what its name implies,
// smoothing the stepping of multi-axis motions. This feature smooths motion particularly at low step
// frequencies below 10kHz, where the aliasing between axes of multi-axis motions can cause audible
// noise and shake your machine. At even lower step frequencies, AMASS adapts and provides even better
// step smoothing. See stepper.c for more details on the AMASS system works.
#define ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING  // Default enabled. Comment to disable.

// Define Adaptive Multi-Axis Step-Smoothing(AMASS) levels and cutoff frequencies. The highest level
// frequency bin starts at 0Hz and ends at its cutoff frequency. The next lower level frequency bin
// starts at the next higher cutoff frequency, and so on. The cutoff frequencies for each level must
// be considered carefully against how much it over-drives the stepper ISR, the accuracy of the 16-bit
// timer, and the CPU overhead. Level 0 (no AMASS, normal operation) frequency bin starts at the
// Level 1 cutoff frequency and up to as fast as the CPU allows (over 30kHz in limited testing).
// NOTE: AMASS cutoff frequency multiplied by ISR overdrive factor must not exceed maximum step frequency.
// NOTE: Current settings are set to overdrive the ISR to no more than 16kHz, balancing CPU overhead
// and timer accuracy.  Do not alter these settings unless you know what you are doing.
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
  #define MAX_AMASS_LEVEL 3
  #if MAX_AMASS_LEVEL <= 0
    error "AMASS must have 1 or more levels to operate correctly."
  #endif
#endif

// Sets the maximum step rate allowed to be written as a Grbl setting. This option enables an error
// check in the settings module to prevent settings values that will exceed this limitation. The maximum
// step rate is strictly limited by the CPU speed and will change if something other than an AVR running
// at 16MHz is used.
// NOTE: For now disabled, will enable if flash space permits.
// #define MAX_STEP_RATE_HZ 30000 // Hz

// Sets which axis the tool length offset is applied. Assumes the spindle is always parallel with
// the selected axis with the tool oriented toward the negative direction. In other words, a positive
// tool length offset value is subtracted from the current location.
#define TOOL_LENGTH_OFFSET_AXIS Z_AXIS // Default z-axis. Valid values are X_AXIS, Y_AXIS, or Z_AXIS.

// Disables spindle PWM output in driver (if supported) for different RPM values.
#define VARIABLE_SPINDLE // Default enabled. Comment to disable.

// Used by variable spindle output only. This forces the PWM output to a minimum duty cycle when enabled.
// The PWM pin will still read 0V when the spindle is disabled. Most users will not need this option, but
// it may be useful in certain scenarios. This minimum PWM settings coincides with the spindle rpm minimum
// setting, like rpm max to max PWM. This is handy if you need a larger voltage difference between 0V disabled
// and the voltage set by the minimum PWM for minimum rpm. This difference is 0.02V per PWM value. So, when
// minimum PWM is at 1, only 0.02 volts separate enabled and disabled. At PWM 5, this would be 0.1V. Keep
// in mind that you will begin to lose PWM resolution with increased minimum PWM values, since you have less
// and less range over the total 255 PWM levels to signal different spindle speeds.
// NOTE: Compute duty cycle at the minimum PWM by this equation: (% duty cycle)=(SPINDLE_PWM_MIN_VALUE/255)*100
// #define SPINDLE_PWM_MIN_VALUE 5 // Default disabled. Uncomment to enable. Must be greater than zero. Integer (1-255).

// Alters the behavior of the spindle enable pin. By default,
// Grbl will not disable the enable pin if spindle speed is zero and M3/4 is active, but still sets the PWM
// output to zero. This allows the users to know if the spindle is active and use it as an additional control
// input. However, in some use cases, user may want the enable pin to disable with a zero spindle speed and
// re-enable when spindle speed is greater than zero. This option does that.
// #define SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED // Default disabled. Uncomment to enable.

// With this enabled, Grbl sends back an echo of the line it has received, which has been pre-parsed (spaces
// removed, capitalized letters, no comments) and is to be immediately executed by Grbl. Echoes will not be
// sent upon a line buffer overflow, but should for all normal lines sent to Grbl. For example, if a user
// sendss the line 'g1 x1.032 y2.45 (test comment)', Grbl will echo back in the form '[echo: G1X1.032Y2.45]'.
// NOTE: Only use this for debugging purposes!! When echoing, this takes up valuable resources and can effect
// performance. If absolutely needed for normal operation, the serial write buffer should be greatly increased
// to help minimize transmission waiting within the serial write protocol.
// #define REPORT_ECHO_LINE_RECEIVED // Default disabled. Uncomment to enable.

// Minimum planner junction speed. Sets the default minimum junction speed the planner plans to at
// every buffer block junction, except for starting from rest and end of the buffer, which are always
// zero. This value controls how fast the machine moves through junctions with no regard for acceleration
// limits or angle between neighboring block line move directions. This is useful for machines that can't
// tolerate the tool dwelling for a split second, i.e. 3d printers or laser cutters. If used, this value
// should not be much greater than zero or to the minimum value necessary for the machine to work.
#define MINIMUM_JUNCTION_SPEED 0.0f // (mm/min)

// Sets the minimum feed rate the planner will allow. Any value below it will be set to this minimum
// value. This also ensures that a planned motion always completes and accounts for any floating-point
// round-off errors. Although not recommended, a lower value than 1.0 mm/min will likely work in smaller
// machines, perhaps to 0.1mm/min, but your success may vary based on multiple factors.
#define MINIMUM_FEED_RATE 1.0f // (mm/min)

// Number of arc generation iterations by small angle approximation before exact arc trajectory
// correction with expensive sin() and cos() calculations. This parameter maybe decreased if there
// are issues with the accuracy of the arc generations, or increased if arc execution is getting
// bogged down by too many trig calculations.
#define N_ARC_CORRECTION 12 // Integer (1-255)

// The arc G2/3 g-code standard is problematic by definition. Radius-based arcs have horrible numerical
// errors when arc at semi-circles(pi) or full-circles(2*pi). Offset-based arcs are much more accurate
// but still have a problem when arcs are full-circles (2*pi). This define accounts for the floating
// point issues when offset-based arcs are commanded as full circles, but get interpreted as extremely
// small arcs with around machine epsilon (1.2e-7rad) due to numerical round-off and precision issues.
// This define value sets the machine epsilon cutoff to determine if the arc is a full-circle or not.
// NOTE: Be very careful when adjusting this value. It should always be greater than 1.2e-7 but not too
// much greater than this. The default setting should capture most, if not all, full arc error situations.
#define ARC_ANGULAR_TRAVEL_EPSILON 5E-7f // Float (radians)

// Time delay increments performed during a dwell. The default value is set at 50ms, which provides
// a maximum time delay of roughly 55 minutes, more than enough for most any application. Increasing
// this delay will increase the maximum dwell time linearly, but also reduces the responsiveness of
// run-time command executions, like status reports, since these are performed between each dwell
// time step.
#define DWELL_TIME_STEP 50 // Integer (1-255) (milliseconds)

// The number of linear motions in the planner buffer to be planned at any give time. The vast
// majority of RAM that Grbl uses is based on this buffer size. Only increase if there is extra
// available RAM, like when re-compiling for MCU with ample amounts of RAM. Or decrease if the MCU begins to
// crash due to the lack of available RAM or if the CPU is having trouble keeping up with planning
// new incoming motions as they are executed.
// #define BLOCK_BUFFER_SIZE 16 // Uncomment to override default in planner.h.

// Governs the size of the intermediary step segment buffer between the step execution algorithm
// and the planner blocks. Each segment is set of steps executed at a constant velocity over a
// fixed time defined by ACCELERATION_TICKS_PER_SECOND. They are computed such that the planner
// block velocity profile is traced exactly. The size of this buffer governs how much step
// execution lead time there is for other Grbl processes have to compute and do their thing
// before having to come back and refill this buffer, currently at ~50msec of step moves.
// #define SEGMENT_BUFFER_SIZE 6 // Uncomment to override default in stepper.h.

// Serial send and receive buffer size. The receive buffer is often used as another streaming
// buffer to store incoming blocks to be processed by Grbl when its ready. Most streaming
// interfaces will character count and track each block send to each block response. So,
// increase the receive buffer if a deeper receive buffer is needed for streaming and avaiable
// memory allows. The send buffer primarily handles messages in Grbl. Only increase if large
// messages are sent and Grbl begins to stall, waiting to send the rest of the message.
// NOTE: Grbl generates an average status report in about 0.5msec, but the serial TX stream at
// 115200 baud will take 5 msec to transmit a typical 55 character report. Worst case reports are
// around 90-100 characters. As long as the serial TX buffer doesn't get continually maxed, Grbl
// will continue operating efficiently. Size the TX buffer around the size of a worst-case report.
// NOTE: buffer allocation is delgated to the HAL driver

// Configures the position after a probing cycle during Grbl's check mode. Disabled sets
// the position to the probe target, when enabled sets the position to the start position.
// #define SET_CHECK_MODE_PROBE_TO_START // Default disabled. Uncomment to enable.

// Force Grbl to check the state of the hard limit switches when the processor detects a pin
// change inside the hard limit ISR routine. By default, Grbl will trigger the hard limits
// alarm upon any pin change, since bouncing switches can cause a state check like this to
// misread the pin. When hard limits are triggered, they should be 100% reliable, which is the
// reason that this option is disabled by default. Only if your system/electronics can guarantee
// that the switches don't bounce, we recommend enabling this option. This will help prevent
// triggering a hard limit when the machine disengages from the switch.
// NOTE: This option has no effect if SOFTWARE_DEBOUNCE is enabled.
// #define HARD_LIMIT_FORCE_STATE_CHECK // Default disabled. Uncomment to enable.

// Adjusts homing cycle search and locate scalars. These are the multipliers used by Grbl's
// homing cycle to ensure the limit switches are engaged and cleared through each phase of
// the cycle. The search phase uses the axes max-travel setting times the SEARCH_SCALAR to
// determine distance to look for the limit switch. Once found, the locate phase begins and
// uses the homing pull-off distance setting times the LOCATE_SCALAR to pull-off and re-engage
// the limit switch.
// NOTE: Both of these values must be greater than 1.0 to ensure proper function.
// #define HOMING_AXIS_SEARCH_SCALAR  1.5f // Uncomment to override defaults in limits.c.
// #define HOMING_AXIS_LOCATE_SCALAR  10.0f // Uncomment to override defaults in limits.c.

// Enable the '$RST=*', '$RST=$', and '$RST=#' eeprom restore commands. There are cases where
// these commands may be undesirable. Simply comment the desired macro to disable it.
// NOTE: See SETTINGS_RESTORE_ALL macro for customizing the `$RST=*` command.
#define ENABLE_RESTORE_EEPROM_WIPE_ALL         // '$RST=*' Default enabled. Comment to disable.
#define ENABLE_RESTORE_EEPROM_DEFAULT_SETTINGS // '$RST=$' Default enabled. Comment to disable.
#define ENABLE_RESTORE_EEPROM_CLEAR_PARAMETERS // '$RST=#' Default enabled. Comment to disable.
#define ENABLE_RESTORE_DRIVER_PARAMETERS       // '$RST=!' Default enabled. Comment to disable. For drivers that implements non-generic settings.

// Defines the EEPROM data restored upon a settings version change and `$RST=*` command. Whenever the
// the settings or other EEPROM data structure changes between Grbl versions, Grbl will automatically
// wipe and restore the EEPROM. This macro controls what data is wiped and restored. This is useful
// particularily for OEMs that need to retain certain data. For example, the BUILD_INFO string can be
// written into the EEPROM via a separate .INO sketch to contain product data. Altering this
// macro to not restore the build info EEPROM will ensure this data is retained after firmware upgrades.
// NOTE: Uncomment to override defaults in settings.h
// #define SETTINGS_RESTORE_ALL (SETTINGS_RESTORE_DEFAULTS|SETTINGS_RESTORE_PARAMETERS|SETTINGS_RESTORE_STARTUP_LINES|SETTINGS_RESTORE_BUILD_INFO|SETTINGS_RESTORE_DRIVER_PARAMETERS)

// Enable the '$I=(string)' build info write command. If disabled, any existing build info data must
// be placed into EEPROM via external means with a valid checksum value. This macro option is useful
// to prevent this data from being over-written by a user, when used to store OEM product data.
// NOTE: If disabled and to ensure Grbl can never alter the build info line, you'll also need to enable
// the SETTING_RESTORE_ALL macro above and remove SETTINGS_RESTORE_BUILD_INFO from the mask.
// NOTE: See the included grblWrite_BuildInfo.ino example file to write this string seperately.
#define ENABLE_BUILD_INFO_WRITE_COMMAND // '$I=' Default enabled. Comment to disable.

// In Grbl v0.9 and prior, there is an old outstanding bug where the `WPos:` work position reported
// may not correlate to what is executing, because `WPos:` is based on the g-code parser state, which
// can be several motions behind. This option forces the planner buffer to empty, sync, and stop
// motion whenever there is a command that alters the work coordinate offsets `G10,G43.1,G92,G54-59`.
// This is the simplest way to ensure `WPos:` is always correct. Fortunately, it's exceedingly rare
// that any of these commands are used need continuous motions through them.
#define FORCE_BUFFER_SYNC_DURING_WCO_CHANGE // Default enabled. Comment to disable.

// By default, Grbl disables feed rate overrides for all G38.x probe cycle commands. Although this
// may be different than some pro-class machine control, it's arguable that it should be this way.
// Most probe sensors produce different levels of error that is dependent on rate of speed. By
// keeping probing cycles to their programmed feed rates, the probe sensor should be a lot more
// repeatable. If needed, you can disable this behavior by uncommenting the define below.
// #define ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES // Default disabled. Uncomment to enable.

// Enables and configures parking motion methods upon a safety door state. Primarily for OEMs
// that desire this feature for their integrated machines. At the moment, Grbl assumes that
// the parking motion only involves one axis, although the parking implementation was written
// to be easily refactored for any number of motions on different axes by altering the parking
// source code. At this time, Grbl only supports parking one axis (typically the Z-axis) that
// moves in the positive direction upon retracting and negative direction upon restoring position.
// The motion executes with a slow pull-out retraction motion, power-down, and a fast park.
// Restoring to the resume position follows these set motions in reverse: fast restore to
// pull-out position, power-up with a time-out, and plunge back to the original position at the
// slower pull-out rate.
// NOTE: Still a work-in-progress. Machine coordinates must be in all negative space and
// does not work with HOMING_FORCE_SET_ORIGIN enabled. Parking motion also moves only in
// positive direction.
//#define PARKING_ENABLE  // Default disabled. Uncomment to enable

// Configure options for the parking motion, if enabled.
#define PARKING_AXIS Z_AXIS // Define which axis that performs the parking motion
#define PARKING_TARGET -5.0f // Parking axis target. In mm, as machine coordinate [-max_travel,0].
#define PARKING_RATE 500.0f // Parking fast rate after pull-out in mm/min.
#define PARKING_PULLOUT_RATE 100.0f // Pull-out/plunge slow feed rate in mm/min.
#define PARKING_PULLOUT_INCREMENT 5.0f // Spindle pull-out and plunge distance in mm. Incremental distance.
                                      // Must be positive value or equal to zero.

// Enables a special set of M-code commands that enables and disables the parking motion.
// These are controlled by `M56`, `M56 P1`, or `M56 Px` to enable and `M56 P0` to disable.
// The command is modal and will be set after a planner sync. Since it is g-code, it is
// executed in sync with g-code commands. It is not a real-time command.
// NOTE: PARKING_ENABLE is required. By default, M56 is active upon initialization. Use
// DEACTIVATE_PARKING_UPON_INIT to set M56 P0 as the power-up default.
// #define ENABLE_PARKING_OVERRIDE_CONTROL   // Default disabled. Uncomment to enable
// #define DEACTIVATE_PARKING_UPON_INIT // Default disabled. Uncomment to enable.

// This option will automatically disab
// Enables and configures Grbl's sleep mode feature. If the spindle or coolant are powered and Grbl
// is not actively moving or receiving any commands, a sleep timer will start. If any data or commands
// are received, the sleep timer will reset and restart until the above condition are not satisfied.
// If the sleep timer elaspes, Grbl will immediately execute the sleep mode by shutting down the spindle
// and coolant and entering a safe sleep state. If parking is enabled, Grbl will park the machine as
// well. While in sleep mode, only a hard/soft reset will exit it and the job will be unrecoverable.
// NOTE: Sleep mode is a safety feature, primarily to address communication disconnect problems. To
// keep Grbl from sleeping, employ a stream of '?' status report commands as a connection "heartbeat".
// #define SLEEP_ENABLE  // Default disabled. Uncomment to enable.
#define SLEEP_DURATION 5.0f // Float (0.25 - 61.0) seconds before sleep mode is executed.

// This option will automatically disable the laser during a feed hold by invoking a spindle stop
// override immediately after coming to a stop. However, this also means that the laser still may
// be reenabled by disabling the spindle stop override, if needed. This is purely a safety feature
// to ensure the laser doesn't inadvertently remain powered while at a stop and cause a fire.
#define DISABLE_LASER_DURING_HOLD // Default enabled. Comment to disable.

// Max length of gcode lines (blocks) stored in EEPROM, do not set > 80 unless EEPROM space is reallocated
#define MAX_STORED_LINE_LENGTH 80

// Number of tools in ATC tool table, comment out to disable
//#define N_TOOLS 8

// Enable EEPROM emulation/buffering in RAM (allocated from heap)
// Can be used for MCUs with no EEPROM or as buffer in order to avoid writing to EEPROM when not in idle state.
// The buffer will be written to EEPROM when in idle state.
#define EMULATE_EEPROM

// Max number of entries in log for PID data reporting, to be used for tuning
#define PID_LOG 1000 // Default disabled. Uncomment to enable.

// Define some default values if not defined above

#ifndef ST_DEENERGIZE_MASK
#define ST_DEENERGIZE_MASK 0
#endif

#ifndef INVERT_ST_ENABLE_MASK
#define INVERT_ST_ENABLE_MASK 0
#endif

#ifndef INVERT_LIMIT_PIN_MASK
#define INVERT_LIMIT_PIN_MASK 0
#endif

#ifndef INVERT_CONTROL_PIN_MASK
#define INVERT_CONTROL_PIN_MASK 0
#endif

#ifndef INVERT_SPINDLE_ENABLE_PIN
#define INVERT_SPINDLE_ENABLE_PIN 0
#endif

#ifndef DISABLE_LIMIT_PINS_PULL_UP_MASK
#define DISABLE_LIMIT_PINS_PULL_UP_MASK 0
#endif

#ifndef DISABLE_PROBE_PIN_PULL_UP
#define DISABLE_PROBE_PIN_PULL_UP 0
#endif

#ifndef DISABLE_CONTROL_PINS_PULL_UP_MASK
#define DISABLE_CONTROL_PINS_PULL_UP_MASK 0
#endif

#endif
