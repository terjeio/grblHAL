/*
  sourcerabbit_4axis.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2021 Terje Io

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

#define BOARD_NAME "SourceRabbit 4-axis CNC"

// timer definitions
#define STEP_TIMER_GROUP TIMER_GROUP_0
#define STEP_TIMER_INDEX TIMER_0

#if MODBUS_ENABLE
#error VFD Spindle not supported!
#endif

#if KEYPAD_ENABLE
#error Keypad not supported!
#endif

#if SDCARD_ENABLE
#error SD card not supported!
#endif

// Define step pulse output pins.
#define X_STEP_PIN  GPIO_NUM_0
#define Y_STEP_PIN  GPIO_NUM_25
#define Z_STEP_PIN  GPIO_NUM_27
#if NUM_AXES > 3
#define A_STEP_PIN  GPIO_NUM_12
#endif

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN GPIO_NUM_33
#define Y_DIRECTION_PIN GPIO_NUM_26
#define Z_DIRECTION_PIN GPIO_NUM_14
#if NUM_AXES > 3
#define A_DIRECTION_PIN GPIO_NUM_13
#endif

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_DISABLE_PIN    GPIO_NUM_15

// Define homing/hard limit switch input pins and limit interrupt vectors.
#define X_LIMIT_PIN GPIO_NUM_36
#define Y_LIMIT_PIN GPIO_NUM_39
#define Z_LIMIT_PIN GPIO_NUM_34

// Define spindle enable and spindle direction output pins.

#define SPINDLE_ENABLE_PIN  GPIO_NUM_21
#define SPINDLEPWMPIN       GPIO_NUM_2

// Define flood and mist coolant enable output pins.

#define COOLANT_FLOOD_PIN   GPIO_NUM_22
#define COOLANT_MIST_PIN    GPIO_NUM_23

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
// N/A

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PIN    GPIO_NUM_32
#endif
