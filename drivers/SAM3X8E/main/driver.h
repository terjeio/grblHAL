/*
  driver.h - driver code for Atmel SAM3X8E ARM processor

  Part of GrblHAL

  Copyright (c) 2019 Terje Io

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

#include "Arduino.h"
#include "src/grbl/grbl.h"

// NOTE: Only one board may be enabled!
//#define BOARD_TINYG2_DUE
//#define BOARD_RAMPS_16
#define BOARD_CMCGRATH

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

void IRQRegister(uint32_t IRQnum, void (*IRQhandler)(void));
void IRQUnRegister(uint32_t IRQnum);

/*****************************************************************************/

// Configuration
// Set value to 1 to enable, 0 to disable
// NOTE: none of these options are ready yet. DO NOT ENABLE!
#define USB_SERIAL       0
#define SDCARD_ENABLE    0
#define KEYPAD_ENABLE    0 // I2C keypad for jogging etc.
#define EEPROM_ENABLE    0 // I2C EEPROM (24LC16) support.
#define TRINAMIC_ENABLE  0 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
#define TRINAMIC_I2C     0 // Trinamic I2C - SPI bridge interface.
#define TRINAMIC_DEV     0 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code

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

// timer definitions

#define STEPPER_TIMER  		(TC0->TC_CHANNEL[0])
#define STEPPER_TIMER_IRQn 	TC0_IRQn
#define STEP_TIMER			(TC0->TC_CHANNEL[1])
#define STEP_TIMER_IRQn 	TC1_IRQn

#define DEBOUNCE_TIMER    	(TC1->TC_CHANNEL[0])
#define DEBOUNCE_TIMER_IRQn	TC3_IRQn

#ifdef BOARD_TINYG2_DUE
	#include "tinyg2_due_map.h"
#elif defined(BOARD_RAMPS_16)
	#include "ramps_1.6_map.h"
#elif defined(BOARD_CMCGRATH)
	#include "cmcgrath_rev3_map.h"
#else // default board - NOTE: NOT FINAL VERSION!

// Define step pulse output pins.
#define X_STEP_PORT     	PIOA
#define X_STEP_PIN      	16
#define X_STEP_BIT      	(1<<X_STEP_PIN)
#define Y_STEP_PORT     	PIOA
#define Y_STEP_PIN      	3
#define Y_STEP_BIT      	(1<<Y_STEP_PIN)
#define Z_STEP_PORT     	PIOC
#define Z_STEP_PIN      	17
#define Z_STEP_BIT			(1<<Z_STEP_PIN)

// Define step direction output pins.
#define X_DIRECTION_PORT	PIOA
#define X_DIRECTION_PIN		24
#define X_DIRECTION_BIT		(1<<X_DIRECTION_PIN)
#define Y_DIRECTION_PORT	PIOA
#define Y_DIRECTION_PIN		2
#define Y_DIRECTION_BIT		(1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_PORT	PIOC
#define Z_DIRECTION_PIN		15
#define Z_DIRECTION_BIT  	(1<<Z_STEP_PIN)

// Define stepper driver enable/disable output pin(s).
#define X_DISABLE_PORT		PIOC
#define X_DISABLE_PIN		6
#define X_DISABLE_BIT		(1<<X_DISABLE_PIN)
#define Y_DISABLE_PORT		PIOA
#define Y_DISABLE_PIN		23
#define Y_DISABLE_BIT		(1<<Y_DISABLE_PIN)
#define Z_DISABLE_PORT		PIOB
#define Z_DISABLE_PIN		17
#define Z_DISABLE_BIT		(1<<Z_DISABLE_PIN)

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT		PIOB // C28
#define X_LIMIT_PIN			25
#define X_LIMIT_BIT			(1<<X_LIMIT_PIN)
#define Y_LIMIT_PORT		PIOD // D14
#define Y_LIMIT_PIN			25
#define Y_LIMIT_BIT			(1<<Y_LIMIT_PIN)
#define Z_LIMIT_PORT		PIOC // A11
#define Z_LIMIT_PIN			24
#define Z_LIMIT_BIT			(1<<Z_LIMIT_PIN)

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     PIOA
#define SPINDLE_ENABLE_PIN      15
#define SPINDLE_ENABLE_BIT      (1<<SPINDLE_ENABLE_PIN)
#define SPINDLE_DIRECTION_PORT	PIOD
#define SPINDLE_DIRECTION_PIN   3
#define SPINDLE_DIRECTION_BIT   (1<<SPINDLE_DIRECTION_PIN)

// Start of PWM & Stepper Enabled Spindle
#define SPINDLE_PWM_TIMER	(TC2->TC_CHANNEL[0])
#define SPINDLE_PWM_PORT	PIOC
#define SPINDLE_PWM_PIN		25  // TIOA6
#define SPINDLE_PWM_BIT		(1<<SPINDLE_PWM_PIN)

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT  PIOC
#define COOLANT_FLOOD_PIN   5
#define COOLANT_FLOOD_BIT   (1<<COOLANT_FLOOD_PIN)
#define COOLANT_MIST_PORT  	PIOC
#define COOLANT_MIST_PIN   	3
#define COOLANT_MIST_BIT   	(1<<COOLANT_MIST_PIN)

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PORT          PIOC
#define RESET_PIN           12
#define RESET_BIT           (1<<RESET_PIN)

#define FEED_HOLD_PORT      PIOC
#define FEED_HOLD_PIN       14
#define FEED_HOLD_BIT       (1<<FEED_HOLD_PIN)

#define CYCLE_START_PORT    PIOC
#define CYCLE_START_PIN     16
#define CYCLE_START_BIT     (1<<CYCLE_START_PIN)

#define SAFETY_DOOR_PORT    PIOC
#define SAFETY_DOOR_PIN     18
#define SAFETY_DOOR_BIT     (1<<SAFETY_DOOR_PIN)

// Define probe switch input pin.
#define PROBE_PORT			PIOC
#define PROBE_PIN			13
#define PROBE_BIT			(1<<PROBE_PIN)

#if KEYPAD_ENABLE
#define KEYPAD_PORT			PIOA
#define KEYPAD_PIN			5
#define KEYPAD_BIT			(1<<KEYPAD_PIN)
#endif

#if SDCARD_ENABLE
// Define SD card detect pin.
#define SD_CD_PORT			PIOA
#define SD_CD_PIN			30
#define SD_CD_BIT			(1<<SD_CD_PIN)
#endif

#if EEPROM_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)

// Define I2C port/pins
#define I2C_PORT SERCOM0
#define I2C_SDA_PIN 11
#define I2C_SCL_PIN 12
#define I2C_CLOCK 100000

#endif

#endif // default board

// Simple sanity check...

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
