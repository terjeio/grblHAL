/*
  espduino-32_wemos_d1_r32_uno_map.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

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

/*
  This map is for relatively common ESP32 boards replicating the form factor of Arduino UNO.
  This map allows use of such uno-compatible board with very popular
  "Protoneer Arduino CNC shield" and is based on its pinout.
  This makes perfect match for retrofiting older Arduino+GRBL based machines
  with 32b microcontroler capable of running grblHAL and providing few extra IO pins (eg. for modbus).

  These boards are sold under several names, for instance:
   + ESPDUINO-32  (USB type B)
   + Wemos D1 R32 (Micro USB)
*/

#define BOARD_NAME "ESPDUINO-32 Wemos D1 R32"

// timer definitions
#define STEP_TIMER_GROUP TIMER_GROUP_0
#define STEP_TIMER_INDEX TIMER_0

#if SDCARD_ENABLE

// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
/*
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
*/

#endif // SDCARD_ENABLE

// Define step pulse output pins.
#define X_STEP_PIN      GPIO_NUM_26
#define Y_STEP_PIN      GPIO_NUM_25
#define Z_STEP_PIN      GPIO_NUM_17

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN     GPIO_NUM_16
#define Y_DIRECTION_PIN     GPIO_NUM_27
#define Z_DIRECTION_PIN     GPIO_NUM_14

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_DISABLE_PIN    GPIO_NUM_12

// Define homing/hard limit switch input pins and limit interrupt vectors.
#define X_LIMIT_PIN     GPIO_NUM_13
#define Y_LIMIT_PIN     GPIO_NUM_5
#define Z_LIMIT_PIN     GPIO_NUM_23

// Define spindle enable and spindle direction output pins.

#ifndef VFD_SPINDLE
#define SPINDLE_ENABLE_PIN  GPIO_NUM_18
#define SPINDLEPWMPIN       GPIO_NUM_19
#endif

// Define flood enable output pin.
#if !MODBUS_ENABLE
#define COOLANT_FLOOD_PIN   GPIO_NUM_32
#endif

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PIN           GPIO_NUM_2
#define FEED_HOLD_PIN       GPIO_NUM_4
#define CYCLE_START_PIN     GPIO_NUM_35

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PIN   GPIO_NUM_39
#endif

#if MODBUS_ENABLE
#define UART2_RX_PIN            GPIO_NUM_33
#define UART2_TX_PIN            GPIO_NUM_32
#define MODBUS_DIRECTION_PIN    GPIO_NUM_15
#define MODBUS_BAUD             19200
#endif


#if KEYPAD_ENABLE
#error No free pins for keypad!
#endif
