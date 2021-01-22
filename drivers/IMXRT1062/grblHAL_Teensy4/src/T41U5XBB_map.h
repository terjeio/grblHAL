/*
  T41U5XBB_map.h - driver code for IMXRT1062 processor (on Teensy 4.1 board)

  Part of grblHAL

  Board by Phil Barrett: https://github.com/phil-barrett/grblHAL-teensy-4.x

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

#define BOARD_NAME "T41U5XBB"
#define HAS_BOARD_INIT

void board_init (void);

#if N_AXIS > 5
#error Max number of axes is 5 for T41U5XBB
#endif

// Default pin assignments allow only one axis to be ganged or auto squared.
// B axis pin numbers are used for the ganged/auto squared axis.
// If a second axis is to be ganged/auto squared pin assignments needs to be changed!
// Set to 1 to enable, 0 to disable.
#define X_GANGED        0
#define X_AUTO_SQUARE   0
#define Y_GANGED        0
#define Y_AUTO_SQUARE   0
#define Z_GANGED        0
#define Z_AUTO_SQUARE   0
//

#define X_STEP_PIN      (2u)
#define X_DIRECTION_PIN (3u)
#define X_ENABLE_PIN    (10u)
#define X_LIMIT_PIN     (20u)

#if X_GANGED || X_AUTO_SQUARE
#define X2_STEP_PIN      (26u)
#define X2_DIRECTION_PIN (27u)
#define X2_ENABLE_PIN    (37u)
#if X_AUTO_SQUARE
  #define X2_LIMIT_PIN   (28u)
#endif
#endif

#define Y_STEP_PIN      (4u)
#define Y_DIRECTION_PIN (5u)
#define Y_ENABLE_PIN    (40u)
#define Y_LIMIT_PIN     (21u)

#if Y_GANGED || Y_AUTO_SQUARE
#define Y2_STEP_PIN      (26u)
#define Y2_DIRECTION_PIN (27u)
#define Y2_ENABLE_PIN    (37u)
#if Y_AUTO_SQUARE
  #define Y2_LIMIT_PIN   (28u)
#endif
#endif

#define Z_STEP_PIN      (6u)
#define Z_DIRECTION_PIN (7u)
#define Z_ENABLE_PIN    (39u)
#define Z_LIMIT_PIN     (22u)

#if Z_GANGED || Z_AUTO_SQUARE
#define Z2_STEP_PIN      (26u)
#define Z2_DIRECTION_PIN (27u)
#define Z2_ENABLE_PIN    (37u)
#define Z2_LIMIT_PIN     (28u)
#endif

#if N_AXIS > 3
#define A_STEP_PIN      (8u)
#define A_DIRECTION_PIN (9u)
#define A_ENABLE_PIN    (38u)
#define A_LIMIT_PIN     (23u)
#endif

#if N_AXIS > 4
#define B_STEP_PIN      (26u)
#define B_DIRECTION_PIN (27u)
#define B_ENABLE_PIN    (37u)
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

// Define auxillary input pins
#define AUXINPUT0_PIN       (36u) // ST0
#if !QEI_ENABLE
#define AUXINPUT1_PIN       (30u) // ST1
#define AUXINPUT2_PIN       (34u) // ST2
#define AUXINPUT3_PIN       (35u) // ST3
#define AUX_N_IN 4
#define AUX_IN_MASK 0b1111
#else
#define AUX_N_IN 1
#define AUX_IN_MASK 0b1
#endif

#define AUXOUTPUT0_PIN      (31u) // AUX0
#define AUXOUTPUT1_PIN      (32u) // AUX1
#define AUXOUTPUT2_PIN      (33u) // AUX2
#define AUX_N_OUT 3
#define AUX_OUT_MASK 0b111

#if KEYPAD_ENABLE
#define KEYPAD_STROBE_PIN   (41u) // I2C ST
#endif

#if EEPROM_ENABLE || KEYPAD_ENABLE
#define I2C_PORT    4
#define I2C_SCL4    (24u) // Not used, for info only
#define I2C_SDA4    (25u) // Not used, for info only
#endif

#if QEI_ENABLE
#define QEI_A_PIN      (30u) // ST1
#define QEI_B_PIN      (34u) // ST2
#define QEI_SELECT_PIN (35u) // ST3
#endif
