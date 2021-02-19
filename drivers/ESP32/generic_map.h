/*
  generic_map.h - driver code for ESP32

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

#ifdef VFD_SPINDLE
#error "Generic board does not have support for VFD spindle."
#endif

#if SDCARD_ENABLE

// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5

#endif // SDCARD_ENABLE

// timer definitions
#define STEP_TIMER_GROUP TIMER_GROUP_0
#define STEP_TIMER_INDEX TIMER_0

// Define step pulse output pins.
#define X_STEP_PIN  GPIO_NUM_12
#define Y_STEP_PIN  GPIO_NUM_14
#define Z_STEP_PIN  GPIO_NUM_27

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN GPIO_NUM_26
#define Y_DIRECTION_PIN GPIO_NUM_25
#define Z_DIRECTION_PIN GPIO_NUM_33

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_DISABLE_PIN    GPIO_NUM_13

// Define homing/hard limit switch input pins and limit interrupt vectors.
#define X_LIMIT_PIN GPIO_NUM_2
#define Y_LIMIT_PIN GPIO_NUM_4
#define Z_LIMIT_PIN GPIO_NUM_15

#ifndef VFD_SPINDLE
// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PIN      GPIO_NUM_18
#define SPINDLE_DIRECTION_PIN   GPIO_NUM_5
#define SPINDLEPWMPIN           GPIO_NUM_17
#endif

// Define flood and mist coolant enable output pins.

#define COOLANT_FLOOD_PIN   GPIO_NUM_16
#define COOLANT_MIST_PIN    GPIO_NUM_21

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PIN       GPIO_NUM_34
#define FEED_HOLD_PIN   GPIO_NUM_36
#define CYCLE_START_PIN GPIO_NUM_39
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define SAFETY_DOOR_PIN GPIO_NUM_35
#endif

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PIN   GPIO_NUM_32
#endif

#if KEYPAD_ENABLE
#error No free pins for keypad!
#endif

#if I2C_ENABLE
// Define I2C port/pins
#define I2C_PORT  I2C_NUM_1
#define I2C_SDA   GPIO_NUM_21
#define I2C_SCL   GPIO_NUM_22
#define I2C_CLOCK 100000
#endif

#if IOEXPAND_ENABLE
typedef union {
    uint8_t mask;
    struct {
        uint8_t spindle_on       :1,
                spindle_dir      :1,
                mist_on          :1,
                flood_on         :1,
                stepper_enable_z :1,
                stepper_enable_x :1,
                stepper_enable_y :1,
                reserved         :1;
    };
} ioexpand_t;
#endif
