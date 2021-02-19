/*

  driver.h - driver code for NXP LPC176x ARM processors

  Part of grblHAL

  Copyright (c) 2018-2021 Terje Io

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

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <stdbool.h>
#include <stdint.h>

#include "grbl/hal.h"
#include "grbl/nuts_bolts.h"

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "portmacros.h"

// Configuration
// Set value to 1 to enable, 0 to disable

#ifndef USB_SERIAL_CDC
#define USB_SERIAL_CDC      0 // for UART comms
#endif
#ifndef SDCARD_ENABLE
#define SDCARD_ENABLE       0
#endif
#ifndef EEPROM_ENABLE
#define EEPROM_ENABLE       0
#endif
#ifndef EEPROM_IS_FRAM
#define EEPROM_IS_FRAM      0
#endif
#ifndef LIMIT_MAX_ENABLE
#define LIMIT_MAX_ENABLE    0
#endif

// Define GPIO output mode options

#define GPIO_SHIFT0  0
#define GPIO_SHIFT1  1
#define GPIO_SHIFT2  2
#define GPIO_SHIFT3  3
#define GPIO_SHIFT4  4
#define GPIO_SHIFT5  5
#define GPIO_MAP     8
#define GPIO_BITBAND 9

// NOTE:
// P0.27, P0.28 are dedicated I2C pins without pull up/down.
// P0.29, P0.30 must have same direction as used for USB operation.

#ifdef SMOOTHIEBOARD
  #include "smoothieboard_map.h"
#elif defined(BOARD_RAMPS_16)
  #include "ramps_1.6_map.h"
#elif defined(BOARD_BTT_SKR_13)
  #include "btt_skr_1.3_map.h"
#elif defined(BOARD_BTT_SKR_14_TURBO)
  #include "btt_skr_1.4_turbo_map.h"
#elif defined(BOARD_MKS_SBASE_13)
  #include "mks_sbase_map.h"
#elif defined(BOARD_MY_MACHINE)
  #include "my_machine_map.h"
#else
  #include "generic_map.h"
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 2.2f // microseconds
#endif

// End configuration

#if EEPROM_ENABLE == 0
#define FLASH_ENABLE 1
#else
#define FLASH_ENABLE 0
#endif

#if EEPROM_ENABLE
#define I2C_ENABLE 1
#elif !defined(I2C_ENABLE)
#define I2C_ENABLE 0
#endif

#if TRINAMIC_ENABLE
#ifndef TRINAMIC_MIXED_DRIVERS
#define TRINAMIC_MIXED_DRIVERS 1
#endif
#include "motors/trinamic.h"
#endif

#ifndef X_STEP_PORT
#define X_STEP_PORT STEP_PORT
#endif
#ifndef Y_STEP_PORT
#define Y_STEP_PORT STEP_PORT
#endif
#ifndef Z_STEP_PORT
#define Z_STEP_PORT STEP_PORT
#endif
#if defined(A_AXIS) && !defined(A_STEP_PORT)
#define A_STEP_PORT STEP_PORT
#endif
#if defined(B_AXIS) && !defined(B_STEP_PORT)
#define B_STEP_PORT STEP_PORT
#endif

#ifndef X_DIRECTION_PORT
#define X_DIRECTION_PORT DIRECTION_PORT
#endif
#ifndef Y_DIRECTION_PORT
#define Y_DIRECTION_PORT DIRECTION_PORT
#endif
#ifndef Z_DIRECTION_PORT
#define Z_DIRECTION_PORT DIRECTION_PORT
#endif
#if defined(A_AXIS) && !defined(A_DIRECTION_PORT)
#define A_DIRECTION_PORT DIRECTION_PORT
#endif
#if defined(B_AXIS) && !defined(B_DIRECTION_PORT)
#define B_DIRECTION_PORT DIRECTION_PORT
#endif

#ifndef X_DISABLE_PORT
#define X_DISABLE_PORT DISABLE_PORT
#endif
#ifndef Y_DISABLE_PORT
#define Y_DISABLE_PORT DISABLE_PORT
#endif
#ifndef Z_DISABLE_PORT
#define Z_DISABLE_PORT DISABLE_PORT
#endif
#if defined(A_AXIS) && !defined(A_DISABLE_PORT)
#define A_DISABLE_PORT DISABLE_PORT
#endif
#if defined(B_AXIS) && !defined(B_DISABLE_PORT)
#define B_DISABLE_PORT DISABLE_PORT
#endif

#ifndef X_LIMIT_PORT
#define X_LIMIT_PORT   LIMIT_PORT
  #if LIMIT_PORTN != 0
    #define X_LIMIT_INTCLR LIMIT_INTCLR
    #define X_LIMIT_INTENR LIMIT_INTENR
  #endif
#endif
#ifndef Y_LIMIT_PORT
#define Y_LIMIT_PORT   LIMIT_PORT
  #if LIMIT_PORTN != 0
    #define Y_LIMIT_INTCLR LIMIT_INTCLR
    #define Y_LIMIT_INTENR LIMIT_INTENR
  #endif
#ifndef Z_LIMIT_PORT
#define Z_LIMIT_PORT   LIMIT_PORT
  #if LIMIT_PORTN != 0
    #define Z_LIMIT_INTCLR LIMIT_INTCLR
    #define Z_LIMIT_INTENR LIMIT_INTENR
  #endif
#endif
#endif

#ifndef RESET_PORT
#define RESET_PORT CONTROL_PORT
#define RESET_INTENR CONTROL_INTENR
#define RESET_INTENF CONTROL_INTENF
#define RESET_INTCLR CONTROL_INTCLR

#endif
#ifndef FEED_HOLD_PORT
#define FEED_HOLD_PORT CONTROL_PORT
#define FEED_HOLD_INTENR CONTROL_INTENR
#define FEED_HOLD_INTENF CONTROL_INTENF
#define FEED_HOLD_INTCLR CONTROL_INTCLR
#endif
#ifndef CYCLE_START_PORT
#define CYCLE_START_PORT CONTROL_PORT
#define CYCLE_START_INTENR CONTROL_INTENR
#define CYCLE_START_INTENF CONTROL_INTENF
#define CYCLE_START_INTCLR CONTROL_INTCLR
#endif

// Driver initialization entry point

bool driver_init (void);

#endif // __DRIVER_H__
