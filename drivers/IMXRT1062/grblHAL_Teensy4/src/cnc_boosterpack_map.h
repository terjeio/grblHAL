/*
  cnc_boosterpack_map.h - driver code for IMXRT1062 processor (on Teensy 4.0 board)

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

#define BOARD_NAME "CNC BoosterPack"

#if N_AXIS > 3
#error Max number of axes is 3 for CNC BoosterPack
#endif

#ifdef EEPROM_ENABLE
#undef EEPROM_ENABLE
#endif
#define EEPROM_ENABLE   1 // CNC BoosterPack has on-board EEPROM

// Define step pulse output pins.
#define X_STEP_PIN      (32u)
#define Y_STEP_PIN      (30u)
#define Z_STEP_PIN      (26u)

// Define step direction output pins.
#define X_DIRECTION_PIN (5u)
#define Y_DIRECTION_PIN (33u)
#define Z_DIRECTION_PIN (13u)

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_ENABLE_PIN (24u)
#define Z_ENABLE_PIN        (8u)

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN     (10u)
#define Y_LIMIT_PIN     (1u)
#define Z_LIMIT_PIN     (0u)

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PIN      (12u)
#define SPINDLE_DIRECTION_PIN   (16u)
#define SPINDLEPWMPIN           (12u)

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PIN   (4u)
#define COOLANT_MIST_PIN    (31u)

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PIN           (11u)
#define FEED_HOLD_PIN       (7u)
#define CYCLE_START_PIN     (6u)
#define SAFETY_DOOR_PIN     (9u)

// Define probe switch input pin.
#define PROBE_PIN           (15U)

#if KEYPAD_ENABLE
#define KEYPAD_STROBE_PIN   (28U)
#endif

#if EEPROM_ENABLE || KEYPAD_ENABLE
#define I2C_PORT    0
#define I2C_SCL0    (19u) // Not used, for info only
#define I2C_SDA0    (18u) // Not used, for info only
#endif

#define UART_PORT   5
#define UART_RX5    (21u) // Not used, for info only
#define UART_TX5    (20u) // Not used, for info only

#define GPIO0_PIN   (3u)
#define GPIO1_PIN   (29u)
#define GPIO2_PIN   (27u)
#define GPIO3_PIN   (2u)

#if QEI_ENABLE
    #define QEI_A_PIN      GPIO0_PIN
    #define QEI_B_PIN      GPIO3_PIN
//    #define QEI_INDEX_PIN  GPIO2_PIN
    #define QEI_SELECT_PIN GPIO1_PIN
#endif

/* EOF */
