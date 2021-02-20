/*
  driver.h - driver code for Atmel SAM3X8E ARM processor

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

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

#include "Arduino.h"
#include "grbl/hal.h"

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

/******************************************************************************
* Definitions for bit band access and dynamic IRQ registration                *
******************************************************************************/

/* Bit band SRAM definitions */
#define BITBAND_SRAM_REF   0x20000000
#define BITBAND_SRAM_BASE  0x22000000

#define BITBAND_SRAM(a,b) (*((__IO uint32_t *)((BITBAND_SRAM_BASE + ((((uint32_t)(uint32_t *)&a)-BITBAND_SRAM_REF)<<5) + (b<<2)))))

/* Bit band PERIPHERAL definitions */
#define BITBAND_PERI_REF   0x40000000
#define BITBAND_PERI_BASE  0x42000000

#define BITBAND_PERI(a,b) (*((__IO uint32_t *)((BITBAND_PERI_BASE + ((((uint32_t)(uint32_t *)&a)-BITBAND_PERI_REF)<<5) + (b<<2)))))

void IRQRegister(int32_t IRQnum, void (*IRQhandler)(void));
void IRQUnRegister(int32_t IRQnum);

/*****************************************************************************/

#ifndef USB_SERIAL_CDC
#define USB_SERIAL_CDC      0 // for UART comms
#endif
#ifndef USB_SERIAL_WAIT
#define USB_SERIAL_WAIT     0
#endif
#ifndef SPINDLE_HUANYANG
#define SPINDLE_HUANYANG    0
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

// timer definitions

#define STEPPER_TIMER       (TC0->TC_CHANNEL[0])
#define STEPPER_TIMER_IRQn  TC0_IRQn
#define STEP_TIMER          (TC0->TC_CHANNEL[1])
#define STEP_TIMER_IRQn     TC1_IRQn

#define DEBOUNCE_TIMER      (TC1->TC_CHANNEL[0])
#define DEBOUNCE_TIMER_IRQn TC3_IRQn

#ifdef BOARD_TINYG2_DUE
    #include "tinyg2_due_map.h"
#elif defined(BOARD_RAMPS_16)
    #include "ramps_1.6_map.h"
#elif defined(BOARD_CMCGRATH)
    #include "cmcgrath_rev3_map.h"
#elif defined(BOARD_MEGA256)
    #include "mega_2560_map.h"
#elif defined(BOARD_PROTONEER)
    #include "protoneer_3.xx_map.h"
#elif defined(BOARD_RADDS_16)
    #include "radds_1.6_map.h"
#elif defined(BOARD_MY_MACHINE)
    #include "my_machine_map.h"
#else
    #include "generic_map.h"
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 1.0f // microseconds
#endif

// End configuration

#if TRINAMIC_ENABLE == 2130
#include "tmc2130/trinamic.h"
#endif

#if SPINDLE_HUANYANG
#include "spindle/huanyang.h"
#endif

#if EEPROM_ENABLE || KEYPAD_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)

#define I2C_ENABLE 1
// Define I2C port/pins
#define I2C_PERIPH  TWI0
#define I2C_ID      ID_TWI0
#define I2C_IRQ     TWI0_IRQn
#define I2C_PORT    PIOA
#define I2C_SDA_PIN 17  // Arduino Due SDA1 pin
#define I2C_SCL_PIN 18  // Arduino Due SCL1 pin
#define I2C_SDA_BIT (1<<I2C_SDA_PIN)
#define I2C_SCL_BIT (1<<I2C_SCL_PIN)

#define I2C_CLOCK 100000
#else
#define I2C_ENABLE 0
#endif

// Simple sanity check...

#if KEYPAD_ENABLE && !defined(KEYPAD_PIN)
#error Keypad plugin is not available for this driver
#endif

#if N_AXIS > 3 && !defined(A_STEP_PIN)
#error A motor must be defined for 4 or more axes
#endif
#if N_AXIS > 4 && !defined(B_STEP_PIN)
#error B motor must be defined for 5 or more axes
#endif
#if N_AXIS > 5 && !defined(C_STEP_PIN)
#error C motor must be defined for 6 axes
#endif

#endif
