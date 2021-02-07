/*
  cnc_boosterpack_map.h - driver code for SAMD21 processor (on MKRZERO board)

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

#include "driver.h"

#define BOARD_NAME "CNC BoosterPack"

#if TRINAMIC_ENABLE
#ifdef TRINAMIC_MIXED_DRIVERS
#undef TRINAMIC_MIXED_DRIVERS
#endif
#define TRINAMIC_MIXED_DRIVERS 0
#ifdef TRINAMIC_I2C
#undef TRINAMIC_I2C
#endif
#define TRINAMIC_I2C 1
#endif

#ifdef EEPROM_ENABLE
#undef EEPROM_ENABLE
#endif

#ifdef IOEXPAND_ENABLE
#undef IOEXPAND_ENABLE
#endif

#define EEPROM_ENABLE   1 // only change if BoosterPack does not have EEPROM mounted
#define IOEXPAND_ENABLE 1

// Define step pulse output pins.
#define X_STEP_PIN      (19u)
#define Y_STEP_PIN      (20u)
#define Z_STEP_PIN      (21u)

// Define step direction output pins.
#define X_DIRECTION_PIN (3u)
#define Y_DIRECTION_PIN (15u)
#define Z_DIRECTION_PIN (2u)

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN     (7u)
#define Y_LIMIT_PIN     (1u)
#define Z_LIMIT_PIN     (0u)

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PIN       (17u)
#define FEED_HOLD_PIN   (9u)
#define CYCLE_START_PIN (8u)
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define SAFETY_DOOR_PIN (16u)
#endif

// Start of PWM & Stepper Enabled Spindle
#define SPINDLE_PWM_TIMER   TCC0
#define SPINDLE_PWM_CCREG   2
#define SPINDLEPWMPIN       (6u)

#if IOEXPAND_ENABLE

typedef union {
    uint8_t mask;
    struct {
        uint8_t stepper_enable_z  :1,
                reserved0         :1,
                flood_on          :1,
                mist_on           :1,
                reserved1         :1,
                spindle_dir       :1,
                stepper_enable_xy :1,
                spindle_on        :1;
    };
} ioexpand_t;

#else
#error Configuration requires I2C I/O expander!
#endif
