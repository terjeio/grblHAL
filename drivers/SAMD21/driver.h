/*
  driver.h - driver code for Atmel SAMD21 ARM processor

  Part of GrblHAL

  Copyright (c) 2018-2019 Terje Io

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

#include "src/grbl/grbl.h"

// Configuration
// Set value to 1 to enable, 0 to disable

#define USB_SERIAL         0
#define CNC_BOOSTERPACK    0 // do not change!

#if CNC_BOOSTERPACK
  #define KEYPAD_ENABLE    0 // I2C keypad for jogging etc.
  #define IOEXPAND_ENABLE  1 // I2C IO expander for some output signals.
  #define EEPROM_ENABLE    1 // I2C EEPROM (24LC16) support.
  #define TRINAMIC_ENABLE  0 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
  #define TRINAMIC_I2C     0 // Trinamic I2C - SPI bridge interface.
  #define TRINAMIC_DEV     0 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code
#else
  #define KEYPAD_ENABLE    0 // I2C keypad for jogging etc.
  #define IOEXPAND_ENABLE  0 // I2C IO expander for some output signals.
  #define EEPROM_ENABLE    0 // I2C EEPROM (24LC16) support.
  #define TRINAMIC_ENABLE  0 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
  #define TRINAMIC_I2C     0 // Trinamic I2C - SPI bridge interface.
  #define TRINAMIC_DEV     0 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code
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

// clock definitions

#define CLKTCC_0_1 GCLK_CLKCTRL_GEN_GCLK4

// timer definitions

#define STEP_TIMER          TC3
#define STEP_TIMER_IRQn     TC3_IRQn

#define STEPPER_TIMER       TC4 // 32bit - TC4 & TC5 combined!
#define STEPPER_TIMER_IRQn  TC4_IRQn

#define DEBOUNCE_TIMER      TCC1
#define DEBOUNCE_TIMER_IRQn TCC1_IRQn

#if CNC_BOOSTERPACK == 0

// Define step pulse output pins.
#define X_STEP_PIN      (19u)
#define Y_STEP_PIN      (20u)
#define Z_STEP_PIN      (21u)

// Define step direction output pins.
#define X_DIRECTION_PIN (2u)
#define Y_DIRECTION_PIN (3u)
#define Z_DIRECTION_PIN (4u)

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_DISABLE_PIN    (10u)

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN     (0u)
#define Y_LIMIT_PIN     (1u)
#define Z_LIMIT_PIN     (8u)

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PIN      (7u)
#define SPINDLE_DIRECTION_PIN   (15u)

// Start of PWM & Stepper Enabled Spindle
#define SPINDLE_PWM_TIMER   TCC0
#define SPINDLE_PWM_CCREG   2
#define SPINDLEPWMPIN       (6u)

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PIN   (12u)
#define COOLANT_MIST_PIN    (11u)

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PIN           (9u)
#define FEED_HOLD_PIN       (17u)
#define CYCLE_START_PIN     (16u)
#define SAFETY_DOOR_PIN     (5u)

// Define probe switch input pin.
#define PROBE_PIN       (18U)

#else // CNC Boosterpack pin assignments

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
#define SAFETY_DOOR_PIN (16u)

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

#endif // CNC Boosterpack pin assignments

#if KEYPAD_ENABLE
#define KEYPAD_PIN (5u)
#endif

// Define SD card detect pin.
#define SD_CD_PIN   30

void IRQRegister(int32_t IRQnum, void (*IRQhandler)(void));
void IRQUnRegister(int32_t IRQnum);

#if KEYPAD_ENABLE || IOEXPAND_ENABLE || EEPROM_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)

// Define I2C port/pins
#define I2C_PORT SERCOM0
#define I2C_SDA_PIN 11
#define I2C_SCL_PIN 12
#define I2C_CLOCK 100000

#endif

#endif
