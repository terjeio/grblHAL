/*
  driver.h - driver code for IMXRT1062 processor (on Teensy 4.0 board)

  Part of GrblHAL

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

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include "imxrt.h"
#include "core_pins.h"
#include "pins_arduino.h"

#include "src/grbl/grbl.h"

// NOTE: Only one board may be enabled! If none is enabled pin mappings from defaults below will be used
//#define BOARD_SIMPLE_MACHINE // Do not use, to be added!

// Configuration
// Set value to 1 to enable, 0 to disable

#define USB_SERIAL_GRBL    0
#define ESTOP_ENABLE       0 // When enabled it only real-time report requests will be executed when the reset pin is asserted.
#define CNC_BOOSTERPACK    0 // do not change!

// NOTE: none of these extensions are available, TBC!
#if CNC_BOOSTERPACK
  #define KEYPAD_ENABLE    0 // I2C keypad for jogging etc.
  #define EEPROM_ENABLE    1 // I2C EEPROM (24LC16) support.
  #define TRINAMIC_ENABLE  0 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
  #define TRINAMIC_I2C     0 // Trinamic I2C - SPI bridge interface.
  #define TRINAMIC_DEV     0 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code.
#else
  #define KEYPAD_ENABLE    0 // I2C keypad for jogging etc.
  #define EEPROM_ENABLE    0 // I2C EEPROM (24LC16) support.
  #define TRINAMIC_ENABLE  0 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
  #define TRINAMIC_I2C     0 // Trinamic I2C - SPI bridge interface.
  #define TRINAMIC_DEV     0 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code.
#endif

// End configuration

#if TRINAMIC_ENABLE
#include "src/tmc2130/trinamic.h"
#endif

#if TRINAMIC_ENABLE || KEYPAD_ENABLE

#define DRIVER_SETTINGS

typedef struct {
#if TRINAMIC_ENABLE
    trinamic_settings_t trinamic;
#endif
#if KEYPAD_ENABLE
    jog_settings_t jog;
#endif
} driver_settings_t;

extern driver_settings_t driver_settings;

#endif

#if CNC_BOOSTERPACK
  #include "cnc_boosterpack_map.h"
#elif defined(BOARD_SIMPLE_MACHINE)
  #include "simple_machine_map.h"
#else // default board

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

#if EEPROM_ENABLE
#define I2C_PORT    4
#define I2C_SCL4    (24u) // Not used, for info only
#define I2C_SDA4    (25u) // Not used, for info only
#endif

#endif // default pin mappings

#ifndef I2C_PORT
  #if EEPROM_ENABLE
  #error "EEPROM_ENABLE requires I2C_PORT to be defined!"
  #endif
#endif

#if !(SPINDLEPWMPIN == 12 || SPINDLEPWMPIN == 13)
  #error "SPINDLEPWMPIN can only be routed to pin 12 or 13!"
#endif

// The following struct is pulled from the Teensy Library core, Copyright (c) 2019 PJRC.COM, LLC.

typedef struct {
    const uint8_t pin;              // The pin number
    const uint32_t mux_val;         // Value to set for mux;
    volatile uint32_t *select_reg;  // Which register controls the selection
    const uint32_t select_val;      // Value for that selection
} pin_info_t;

#endif // __DRIVER_H__
