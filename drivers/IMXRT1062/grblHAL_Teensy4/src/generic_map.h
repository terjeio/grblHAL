/*
  generic_map.h - driver code for IMXRT1062 processor (on Teensy 4.0 board)

  Part of grblHAL

  Copyright (c) 2020 Terje Io

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

// Define step pulse output pins.
#define X_STEP_PIN      (2u)
#define Y_STEP_PIN      (4u)
#define Z_STEP_PIN      (6u)

// Define step direction output pins.
#define X_DIRECTION_PIN (3u)
#define Y_DIRECTION_PIN (5u)
#define Z_DIRECTION_PIN (7u)

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_ENABLE_PIN (10u)

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN     (20u)
#define Y_LIMIT_PIN     (21u)
#define Z_LIMIT_PIN     (22u)

#if N_AXIS > 3
#define A_STEP_PIN      (8u)
#define A_DIRECTION_PIN (9u)
#define A_LIMIT_PIN     (23u)
#endif

#if N_AXIS > 4
#define B_STEP_PIN      (26u)
#define B_DIRECTION_PIN (27u)
#define B_LIMIT_PIN     (28u)
#endif

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PIN      (12u)
#define SPINDLE_DIRECTION_PIN   (11u)
#define SPINDLEPWMPIN           (13u) // NOTE: only pin 12 or pin 13 can be assigned!

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PIN   (19u)
#define COOLANT_MIST_PIN    (18u)

// Define user-control CONTROLs (cycle start, reset, feed hold, door) input pins.
#define RESET_PIN           (14u)
#define FEED_HOLD_PIN       (16u)
#define CYCLE_START_PIN     (17u)
#define SAFETY_DOOR_PIN     (29u)

// Define probe switch input pin.
#define PROBE_PIN           (15U)

#if EEPROM_ENABLE || KEYPAD_ENABLE
#define I2C_PORT    4
#define I2C_SCL4    (24u) // Not used, for info only
#define I2C_SDA4    (25u) // Not used, for info only
#endif

#if QEI_ENABLE
#define QEI_A_PIN      (0)
#define QEI_B_PIN      (3)
// #define QEI_INDEX_PIN  GPIO2_PIN
#define QEI_SELECT_PIN (1)
#endif

