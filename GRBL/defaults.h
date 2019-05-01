/*
  defaults.h - defaults settings configuration file
  Part of Grbl

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

#ifndef defaults_h

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

// By default, Grbl disables feed rate overrides for all G38.x probe cycle commands. Although this
// may be different than some pro-class machine control, it's arguable that it should be this way.
// Most probe sensors produce different levels of error that is dependent on rate of speed. By
// keeping probing cycles to their programmed feed rates, the probe sensor should be a lot more
// repeatable. If needed, you can disable this behavior by uncommenting the define below.
//#define ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES 1 // Default disabled. Uncomment to enable.

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

// Number of homing cycles performed after when the machine initially jogs to limit switches.
// This help in preventing overshoot and should improve repeatability. This value should be one or
// greater.
#define DEFAULT_N_HOMING_LOCATE_CYCLE 1 // Integer (1-127)

// The status report change for Grbl v1.1 and after also removed the ability to disable/enable most data
// fields from the report. This caused issues for GUI developers, who've had to manage several scenarios
// and configurations. The increased efficiency of the new reporting style allows for all data fields to
// be sent without potential performance issues.
// NOTE: The options below are here only provide a way to disable certain data fields if a unique
// situation demands it, but be aware GUIs may depend on this data. If disabled, it may not be compatible.
#define REPORT_FIELD_BUFFER_STATE       1 // Default enabled. Set to 0 disable.
#define REPORT_FIELD_PIN_STATE          1 // Default enabled. Set to 0 disable.
#define REPORT_FIELD_CURRENT_FEED_SPEED 1 // Default enabled. Set to 0 disable.
#define REPORT_FIELD_WORK_COORD_OFFSET  1 // Default enabled. Set to 0 disable.
#define REPORT_FIELD_OVERRIDES          1 // Default enabled. Set to 0 disable.
#define REPORT_FIELD_LINE_NUMBERS       1 // Default enabled. Set to 0 disable.

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

// Grbl default settings

#define DEFAULT_STREAM 0 // 0 = Serial, 1 = Bluetooth, 2 = Ethernet, 3 = WiFi
#define DEFAULT_X_STEPS_PER_MM 250.0f
#define DEFAULT_Y_STEPS_PER_MM 250.0f
#define DEFAULT_Z_STEPS_PER_MM 250.0f
#define DEFAULT_X_MAX_RATE 500.0f // mm/min
#define DEFAULT_Y_MAX_RATE 500.0f // mm/min
#define DEFAULT_Z_MAX_RATE 500.0f // mm/min
#define DEFAULT_X_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define DEFAULT_Y_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define DEFAULT_Z_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define DEFAULT_X_MAX_TRAVEL 200.0f // mm NOTE: Must be a positive value.
#define DEFAULT_Y_MAX_TRAVEL 200.0f // mm NOTE: Must be a positive value.
#define DEFAULT_Z_MAX_TRAVEL 200.0f // mm NOTE: Must be a positive value.
#define DEFAULT_X_CURRENT 0.0 // amps
#define DEFAULT_Y_CURRENT 0.0 // amps
#define DEFAULT_Z_CURRENT 0.0 // amps
#define DEFAULT_A_CURRENT 0.0 // amps
#define DEFAULT_SPINDLE_PWM_FREQ          5000	// Hz
#define DEFAULT_SPINDLE_PWM_OFF_VALUE     0.0f    // Percent
#define DEFAULT_SPINDLE_PWM_MIN_VALUE     0.0f	// Percent
#define DEFAULT_SPINDLE_PWM_MAX_VALUE     100.0f	// Percent
#define DEFAULT_SPINDLE_RPM_MAX 1000.0 // rpm
#define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
#define DEFAULT_SPINDLE_ENABLE_OFF_WITH_ZERO_SPEED 0
#define DEFAULT_STEP_PULSE_MICROSECONDS 10
#define DEFAULT_STEP_PULSE_DELAY 0
#define DEFAULT_STEPPING_INVERT_MASK 0
#define DEFAULT_DIRECTION_INVERT_MASK 0
#define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
#define DEFAULT_JUNCTION_DEVIATION 0.01f // mm
#define DEFAULT_ARC_TOLERANCE 0.002f // mm
#define DEFAULT_REPORT_INCHES 0 // false
#define DEFAULT_INVERT_LIMIT_PINS 0 // false
#define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
#define DEFAULT_HARD_LIMIT_ENABLE 0  // false
#define DEFAULT_INVERT_PROBE_PIN 0 // false
#define DEFAULT_LASER_MODE 0 // false
#define DEFAULT_LATHE_MODE 0 // false
#define DEFAULT_HOMING_ENABLE 0  // false
#define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
#define DEFAULT_HOMING_FEED_RATE 25.0f // mm/min
#define DEFAULT_HOMING_SEEK_RATE 500.0f // mm/min
#define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
#define DEFAULT_HOMING_PULLOFF 1.0f // mm

#define DEFAULT_A_STEPS_PER_MM 250.0f
#define DEFAULT_A_MAX_RATE 500.0f // mm/min
#define DEFAULT_A_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define DEFAULT_A_MAX_TRAVEL 200.0f // mm

#define DEFAULT_B_STEPS_PER_MM 250.0f
#define DEFAULT_B_MAX_RATE 500.0f // mm/min
#define DEFAULT_B_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define DEFAULT_B_MAX_TRAVEL 200.0f // mm

#define DEFAULT_C_STEPS_PER_MM 250.0f
#define DEFAULT_C_MAX_RATE 500.0f // mm/min
#define DEFAULT_C_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
#define DEFAULT_C_MAX_TRAVEL 200.0f // mm

#define DEFAULT_G73_RETRACT 0.1f // mm

// When the HAL driver supports spindle sync then this option sets the number of pulses per revolution
// for the spindle encoder. Depending on the driver this may lead to the "spindle at speed" detection
// beeing enabled. When this is enabled grbl will wait for the spindle to reach the programmed speed
// before continue processing. NOTE: Currently there is no timeout for this wait.
// Default value is 0, meaning spindle sync is disabled
#define DEFAULT_SPINDLE_PPR 0 // Pulses per revolution. Default 0.

// When spindle sync is available these are the default settings for the PID algorithm.
#define DEFAULT_SPINDLE_P_GAIN  1.0f
#define DEFAULT_SPINDLE_I_GAIN  0.01f
#define DEFAULT_SPINDLE_D_GAIN  0.0f
#define DEFAULT_SPINDLE_I_MAX   10.0f

// Enables and configures Grbl's sleep mode feature. If the spindle or coolant are powered and Grbl
// is not actively moving or receiving any commands, a sleep timer will start. If any data or commands
// are received, the sleep timer will reset and restart until the above condition are not satisfied.
// If the sleep timer elaspes, Grbl will immediately execute the sleep mode by shutting down the spindle
// and coolant and entering a safe sleep state. If parking is enabled, Grbl will park the machine as
// well. While in sleep mode, only a hard/soft reset will exit it and the job will be unrecoverable.
// NOTE: Sleep mode is a safety feature, primarily to address communication disconnect problems. To
// keep Grbl from sleeping, employ a stream of '?' status report commands as a connection "heartbeat".
#define DEFAULT_SLEEP_ENABLE 0

// This option will automatically disable the laser during a feed hold by invoking a spindle stop
// override immediately after coming to a stop. However, this also means that the laser still may
// be reenabled by disabling the spindle stop override, if needed. This is purely a safety feature
// to ensure the laser doesn't inadvertently remain powered while at a stop and cause a fire.
#define DEFAULT_DISABLE_LASER_DURING_HOLD 1 // Default enabled. Set to 0 to disable.

// When Grbl powers-cycles or is hard reset with the MCU reset button, Grbl boots up with no ALARM
// by default. This is to make it as simple as possible for new users to start using Grbl. When homing
// is enabled and a user has installed limit switches, Grbl will boot up in an ALARM state to indicate
// Grbl doesn't know its position and to force the user to home before proceeding. This option forces
// Grbl to always initialize into an ALARM state regardless of homing or not. This option is more for
// OEMs and LinuxCNC users that would like this power-cycle behavior.
#define DEFAULT_FORCE_INITIALIZATION_ALARM 0 // Default disabled. Set to 1 to enable.

// At power-up or a reset, Grbl will check the limit switch states to ensure they are not active
// before initialization. If it detects a problem and the hard limits setting is enabled, Grbl will
// simply message the user to check the limits and enter an alarm state, rather than idle. Grbl will
// not throw an alarm message.
#define DEFAULT_CHECK_LIMITS_AT_INIT 0 // Default disabled. Set to 1 to enable.

// If homing is enabled, homing init lock sets Grbl into an alarm state upon power up. This forces
// the user to perform the homing cycle (or override the locks) before doing anything else. This is
// mainly a safety feature to remind the user to home, since position is unknown to Grbl.
#define DEFAULT_HOMING_INIT_LOCK 0 // Default disabled. Set to 1 to enable.

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
#define DEFAULT_PARKING_ENABLE 0  // Default disabled. Uncomment to enable

// Configure options for the parking motion, if enabled.
#define DEFAULT_PARKING_AXIS Z_AXIS // Define which axis that performs the parking motion
#define DEFAULT_PARKING_TARGET -5.0f // Parking axis target. In mm, as machine coordinate [-max_travel,0].
#define DEFAULT_PARKING_RATE 500.0f // Parking fast rate after pull-out in mm/min.
#define DEFAULT_PARKING_PULLOUT_RATE 100.0f // Pull-out/plunge slow feed rate in mm/min.
#define DEFAULT_PARKING_PULLOUT_INCREMENT 5.0f // Spindle pull-out and plunge distance in mm. Incremental distance.
                                      // Must be positive value or equal to zero.

// Enables a special set of M-code commands that enables and disables the parking motion.
// These are controlled by `M56`, `M56 P1`, or `M56 Px` to enable and `M56 P0` to disable.
// The command is modal and will be set after a planner sync. Since it is g-code, it is
// executed in sync with g-code commands. It is not a real-time command.
// NOTE: PARKING_ENABLE is required. By default, M56 is active upon initialization. Use
// DEACTIVATE_PARKING_UPON_INIT to set M56 P0 as the power-up default.
#define DEFAULT_ENABLE_PARKING_OVERRIDE_CONTROL 0  // Default disabled. Uncomment to enable
#define DEFAULT_DEACTIVATE_PARKING_UPON_INIT 0 // Default disabled. Uncomment to enable.

// Using printable ASCII characters for realtime commands can cause issues with
// files containing such characters in comments or settings. If the GCode sender support the
// use of the top-bit set alternatives for these then they may be disabled.
// NOTE: support for the top-bit set alternatives is always enabled.
// NOTE: when disabled they are still active outside of comments and $ settings
//       allowing their use from manual input, eg. from a terminal or MDI.
#define DEFAULT_LEGACY_RTCOMMANDS 1  // Default enabled. Set to 0 to disable.

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

#ifndef ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES
#define ALLOW_FEED_OVERRIDE_DURING_PROBE_CYCLES 0
#endif

#endif
