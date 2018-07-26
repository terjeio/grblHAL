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

/* The defaults.h file serves as a central default settings selector for different machine
   types, from DIY CNC mills to CNC conversions of off-the-shelf machines. The settings
   files listed here are supplied by users, so your results may vary. However, this should
   give you a good starting point as you get to know your machine and tweak the settings for
   your nefarious needs.
   NOTE: Ensure one and only one of these DEFAULTS_XXX values is defined in config.h */

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


#ifdef DEFAULTS_GENERIC
  // Grbl generic default settings. Should work across different machines.
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
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEP_PULSE_DELAY 0
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK 0
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.01f // mm
  #define DEFAULT_ARC_TOLERANCE 0.002f // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
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
 #define DEFAULT_SPINDLE_PPR 100 // pulses per revolution

#endif

#ifdef DEFAULTS_SHERLINE_5400
  // Description: Sherline 5400 mill with three NEMA 23 Keling  KL23H256-21-8B 185 oz-in stepper motors,
  // driven by three Pololu A4988 stepper drivers with a 30V, 6A power supply at 1.5A per winding.
  #define MICROSTEPS 2
  #define STEPS_PER_REV 200.0
  #define MM_PER_REV (0.050*MM_PER_INCH) // 0.050 inch/rev leadscrew
  #define DEFAULT_X_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_Y_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_Z_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_X_MAX_RATE 635.0 // mm/min (25 ipm)
  #define DEFAULT_Y_MAX_RATE 635.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 635.0 // mm/min
  #define DEFAULT_X_ACCELERATION (50.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (50.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (50.0*60*60) // 50*60*60 mm/min^2 = 50 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 225.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 125.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 170.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 2800.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<Y_AXIS)|(1<<Z_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.01 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // true
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 50.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 635.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_SHAPEOKO
  // Description: Shapeoko CNC mill with three NEMA 17 stepper motors, driven by Synthetos
  // grblShield with a 24V, 4.2A power supply.
  #define MICROSTEPS_XY 8
  #define STEP_REVS_XY 400
  #define MM_PER_REV_XY (0.08*18*MM_PER_INCH) // 0.08 in belt pitch, 18 pulley teeth
  #define MICROSTEPS_Z 2
  #define STEP_REVS_Z 400
  #define MM_PER_REV_Z 1.250 // 1.25 mm/rev leadscrew
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_X_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_X_ACCELERATION (15.0*60*60) // 15*60*60 mm/min^2 = 15 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (15.0*60*60) // 15*60*60 mm/min^2 = 15 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (15.0*60*60) // 15*60*60 mm/min^2 = 15 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 200.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 200.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 200.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<Y_AXIS)|(1<<Z_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 250.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_SHAPEOKO_2
  // Description: Shapeoko CNC mill with three NEMA 17 stepper motors, driven by Synthetos
  // grblShield at 28V.
  #define MICROSTEPS_XY 8
  #define STEP_REVS_XY 200
  #define MM_PER_REV_XY (2.0*20) // 2mm belt pitch, 20 pulley teeth
  #define MICROSTEPS_Z 2
  #define STEP_REVS_Z 200
  #define MM_PER_REV_Z 1.250 // 1.25 mm/rev leadscrew
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_X_MAX_RATE 5000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 5000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (250.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (250.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (50.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 290.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 290.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 100.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<X_AXIS)|(1<<Z_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 250.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_SHAPEOKO_3
  // Description: Shapeoko CNC mill with three NEMA 23 stepper motors, driven by CarbideMotion
  #define MICROSTEPS_XY 8
  #define STEP_REVS_XY 200
  #define MM_PER_REV_XY (2.0*20) // 2mm belt pitch, 20 pulley teeth
  #define MICROSTEPS_Z 8
  #define STEP_REVS_Z 200
  #define MM_PER_REV_Z (2.0*20) // 2mm belt pitch, 20 pulley teeth
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_X_MAX_RATE 5000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 5000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 5000.0 // mm/min
  #define DEFAULT_X_ACCELERATION (400.0*60*60) // 400*60*60 mm/min^2 = 400 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (400.0*60*60) // 400*60*60 mm/min^2 = 400 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (400.0*60*60) // 400*60*60 mm/min^2 = 400 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 425.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 465.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 80.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<X_AXIS)|(1<<Z_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.01 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 100.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 1000.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 25 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 5.0 // mm
#endif

#ifdef DEFAULTS_X_CARVE_500MM
  // Description: X-Carve 3D Carver CNC mill with three 200 step/rev motors driven by Synthetos
  // grblShield at 24V.
  #define MICROSTEPS_XY 8
  #define STEP_REVS_XY 200
  #define MM_PER_REV_XY (2.0*20) // 2mm belt pitch, 20 pulley teeth
  #define MICROSTEPS_Z 2
  #define STEP_REVS_Z 200
  #define MM_PER_REV_Z 2.117 // ACME 3/8-12 Leadscrew
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_X_MAX_RATE 8000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 8000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (500.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (500.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (50.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 290.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 290.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 100.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<X_AXIS)|(1<<Y_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 3 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 750.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_X_CARVE_1000MM
  // Description: X-Carve 3D Carver CNC mill with three 200 step/rev motors driven by Synthetos
  // grblShield at 24V.
  #define MICROSTEPS_XY 8
  #define STEP_REVS_XY 200
  #define MM_PER_REV_XY (2.0*20) // 2mm belt pitch, 20 pulley teeth
  #define MICROSTEPS_Z 2
  #define STEP_REVS_Z 200
  #define MM_PER_REV_Z 2.117 // ACME 3/8-12 Leadscrew
  #define DEFAULT_X_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Y_STEPS_PER_MM (MICROSTEPS_XY*STEP_REVS_XY/MM_PER_REV_XY)
  #define DEFAULT_Z_STEPS_PER_MM (MICROSTEPS_Z*STEP_REVS_Z/MM_PER_REV_Z)
  #define DEFAULT_X_MAX_RATE 8000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 8000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (500.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (500.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (50.0*60*60) // 25*60*60 mm/min^2 = 25 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 740.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 790.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 100.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<X_AXIS)|(1<<Y_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 255 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 3 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 750.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_ZEN_TOOLWORKS_7x7
  // Description: Zen Toolworks 7x7 mill with three Shinano SST43D2121 65oz-in NEMA 17 stepper motors.
  // Leadscrew is different from some ZTW kits, where most are 1.25mm/rev rather than 8.0mm/rev here.
  // Driven by 30V, 6A power supply and TI DRV8811 stepper motor drivers.
  #define MICROSTEPS 8
  #define STEPS_PER_REV 200.0
  #define MM_PER_REV 8.0 // 8 mm/rev leadscrew
  #define DEFAULT_X_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_Y_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_Z_STEPS_PER_MM (STEPS_PER_REV*MICROSTEPS/MM_PER_REV)
  #define DEFAULT_X_MAX_RATE 6000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 6000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 6000.0 // mm/min
  #define DEFAULT_X_ACCELERATION (600.0*60*60) // 600*60*60 mm/min^2 = 600 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (600.0*60*60) // 600*60*60 mm/min^2 = 600 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (600.0*60*60) // 600*60*60 mm/min^2 = 600 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 190.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 180.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 150.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 10000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK ((1<<Y_AXIS))
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 250.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_OXCNC
  // Grbl settings for OpenBuilds OX CNC Machine
  // http://www.openbuilds.com/builds/openbuilds-ox-cnc-machine.341/
  #define DEFAULT_X_STEPS_PER_MM 26.670
  #define DEFAULT_Y_STEPS_PER_MM 26.670
  #define DEFAULT_Z_STEPS_PER_MM 50
  #define DEFAULT_X_MAX_RATE 500.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 500.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 500.0 // mm/min
  #define DEFAULT_X_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (10.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 500.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 750.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 80.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 1000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK 0
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.02 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 500.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#ifdef DEFAULTS_SIMULATOR
  // Settings only for Grbl Simulator (www.github.com/grbl/grbl-sim)
  // Grbl generic default settings. Should work across different machines.
  #define DEFAULT_X_STEPS_PER_MM 1000.0
  #define DEFAULT_Y_STEPS_PER_MM 1000.0
  #define DEFAULT_Z_STEPS_PER_MM 1000.0
  #define DEFAULT_X_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_Y_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_Z_MAX_RATE 1000.0 // mm/min
  #define DEFAULT_X_ACCELERATION (100.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Y_ACCELERATION (100.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_Z_ACCELERATION (100.0*60*60) // 10*60*60 mm/min^2 = 10 mm/sec^2
  #define DEFAULT_X_MAX_TRAVEL 1000.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Y_MAX_TRAVEL 1000.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_Z_MAX_TRAVEL 1000.0 // mm NOTE: Must be a positive value.
  #define DEFAULT_SPINDLE_RPM_MAX 1000.0 // rpm
  #define DEFAULT_SPINDLE_RPM_MIN 0.0 // rpm
  #define DEFAULT_STEP_PULSE_MICROSECONDS 10
  #define DEFAULT_STEPPING_INVERT_MASK 0
  #define DEFAULT_DIRECTION_INVERT_MASK 0
  #define DEFAULT_STEPPER_IDLE_LOCK_TIME 25 // msec (0-254, 255 keeps steppers enabled)
  #define DEFAULT_STATUS_REPORT_MASK 1 // MPos enabled
  #define DEFAULT_JUNCTION_DEVIATION 0.01 // mm
  #define DEFAULT_ARC_TOLERANCE 0.002 // mm
  #define DEFAULT_REPORT_INCHES 0 // false
  #define DEFAULT_INVERT_ST_ENABLE 0 // false
  #define DEFAULT_INVERT_LIMIT_PINS 0 // false
  #define DEFAULT_SOFT_LIMIT_ENABLE 0 // false
  #define DEFAULT_HARD_LIMIT_ENABLE 0  // false
  #define DEFAULT_INVERT_PROBE_PIN 0 // false
  #define DEFAULT_LASER_MODE 0 // false
  #define DEFAULT_HOMING_ENABLE 0  // false
  #define DEFAULT_HOMING_DIR_MASK 0 // move positive dir
  #define DEFAULT_HOMING_FEED_RATE 25.0 // mm/min
  #define DEFAULT_HOMING_SEEK_RATE 500.0 // mm/min
  #define DEFAULT_HOMING_DEBOUNCE_DELAY 250 // msec (0-65k)
  #define DEFAULT_HOMING_PULLOFF 1.0 // mm
#endif

#endif
