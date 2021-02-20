/*
  driver.h - driver code for Atmel SAMD21 ARM processor

  Part of grblHAL

  Copyright (c) 2018-2020 Terje Io

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

#include "grbl/hal.h"

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#ifndef USB_SERIAL_CDC
#define USB_SERIAL_CDC      0 // for UART comms
#endif
#ifndef USB_SERIAL_WAIT
#define USB_SERIAL_WAIT     0
#endif
#ifndef IOEXPAND_ENABLE
#define IOEXPAND_ENABLE     0
#endif
#ifndef SDCARD_ENABLE
#define SDCARD_ENABLE       0
#endif
#ifndef KEYPAD_ENABLE
#define KEYPAD_ENABLE       0
#endif
#ifndef EEPROM_ENABLE
#define EEPROM_ENABLE       0
#endif
#ifndef EEPROM_IS_FRAM
#define EEPROM_IS_FRAM      0
#endif
#ifndef TRINAMIC_ENABLE
#define TRINAMIC_ENABLE     0
#endif
#ifndef TRINAMIC_I2C
#define TRINAMIC_I2C        0
#endif
#ifndef TRINAMIC_DEV
#define TRINAMIC_DEV        0
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

#ifdef BOARD_CNC_BOOSTERPACK
  #include "cnc_boosterpack_map.h"
#elif defined(BOARD_MY_MACHINE)
  #include "my_machine_map.h"
#else
  #include "generic_map.h"
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 2.3f // microseconds
#endif

// End configuration

#if TRINAMIC_ENABLE
#ifndef TRINAMIC_MIXED_DRIVERS
#define TRINAMIC_MIXED_DRIVERS 1
#endif
#include "motors/trinamic.h"
#include "trinamic/common.h"
#endif

#if KEYPAD_ENABLE
#define KEYPAD_PIN (5u)
#endif

// Define SD card detect pin.
#define SD_CD_PIN   30

void IRQRegister(int32_t IRQnum, void (*IRQhandler)(void));
void IRQUnRegister(int32_t IRQnum);

#if KEYPAD_ENABLE || IOEXPAND_ENABLE || EEPROM_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)

#define I2C_ENABLE 1

// Define I2C port/pins
#define I2C_PORT SERCOM0
#define I2C_SDA_PIN 11
#define I2C_SCL_PIN 12
#define I2C_CLOCK 100000

#else
#define I2C_ENABLE 0
#endif

#endif
