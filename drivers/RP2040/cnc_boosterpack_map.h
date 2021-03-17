/*
  cnc_boosterpack_map.h - driver code for RP2040 ARM processors

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

#if TRINAMIC_ENABLE
#error Trinamic plugin not supported!
#endif

#define BOARD_NAME "CNC BoosterPack"

#define IOEXPAND_ENABLE 1

#undef EEPROM_ENABLE
#define EEPROM_ENABLE 1

// Define step pulse output pins.
#define STEP_PINS_BASE      2 // N_AXIS number of consecutive pins are used by PIO

// Define step direction output pins.
#define X_DIRECTION_PIN     5
#define Y_DIRECTION_PIN     6
#define Z_DIRECTION_PIN     7
#define X_DIRECTION_BIT     (1<<X_DIRECTION_PIN)
#define Y_DIRECTION_BIT     (1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_BIT     (1<<Z_DIRECTION_PIN)
#define DIRECTION_MASK      (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT)
#define DIRECTION_OUTMODE   GPIO_SHIFT5

// Define stepper driver enable/disable output pin.
#define STEPPERS_DISABLEX_PIN     6
#define STEPPERS_DISABLEZ_PIN     0
#define STEPPERS_DISABLE_OUTMODE  GPIO_IOEXPAND

// Define homing/hard limit switch input pins.
#define LIMIT_PORT       GPIO_IN
#define X_LIMIT_PIN      19
#define Y_LIMIT_PIN      20
#define Z_LIMIT_PIN      10
#define X_LIMIT_BIT      (1<<X_LIMIT_PIN)
#define Y_LIMIT_BIT      (1<<Y_LIMIT_PIN)
#define Z_LIMIT_BIT      (1<<Z_LIMIT_PIN)
#define LIMIT_MASK       (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT)
#define LIMIT_INMODE GPIO_OE

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PIN    0
#define SPINDLE_DIRECTION_PIN 5
#define SPINDLE_OUTMODE       GPIO_IOEXPAND

// Define spindle PWM output pin.
#define SPINDLE_PWM_PIN       11
#define SPINDLE_PWM_BIT       (1<<SPINDLE_PWM_PIN)

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PIN     2
#define COOLANT_MIST_PIN      3
#define COOLANT_OUTMODE       GPIO_IOEXPAND

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_RESET_PIN           12
#define CONTROL_RESET_BIT           (1<<CONTROL_RESET_PIN)
#define CONTROL_FEED_HOLD_PIN       13
#define CONTROL_FEED_HOLD_BIT       (1<<CONTROL_FEED_HOLD_PIN)
#define CONTROL_CYCLE_START_PIN     14
#define CONTROL_CYCLE_START_BIT     (1<<CONTROL_CYCLE_START_PIN)
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define CONTROL_SAFETY_DOOR_PIN     15
#define CONTROL_SAFETY_DOOR_BIT     (1<<CONTROL_SAFETY_DOOR_PIN)
#define CONTROL_MASK                (CONTROL_RESET_BIT|CONTROL_FEED_HOLD_BIT|CONTROL_CYCLE_START_BIT|CONTROL_SAFETY_DOOR_BIT)
#else
#define CONTROL_MASK                (CONTROL_RESET_BIT|CONTROL_FEED_HOLD_BIT|CONTROL_CYCLE_START_BIT)
#endif
#define CONTROL_INMODE              GPIO_OE

// Define probe switch input pin.
#define PROBE_PORT                  GPIO_IN
#define PROBE_PIN                   16
#define PROBE_BIT                   (1<<PROBE_PIN)

#if KEYPAD_ENABLE
#define KEYPAD_STROBE_PIN           17
#define KEYPAD_STROBE_BIT           (1<<KEYPAD_STROBE_PIN)
#endif

#if SDCARD_ENABLE
#define SPI_PORT spi0
#define SD_MISO_PIN    16
#define SD_CS_PIN      17
#define SD_SCK_PIN     18
#define SD_MOSI_PIN    19
#endif

#if MPG_MODE_ENABLE
#define MODE_SWITCH_PIN 18
#endif

//I2C: 26,27
//Free: 21,21,22,28
