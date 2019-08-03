/*

  driver.h - driver code for STM32F103C8 ARM processors

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

#include "main.h"
#include "grbl.h"

#ifndef __DRIVER_H__
#define __DRIVER_H__

//-------------------------------------------------
#include <stdbool.h>
#include <stdint.h>
//-------------------------------------------------

#define BITBAND_PERI(x, b) (*((__IO uint8_t *) (PERIPH_BB_BASE + (((uint32_t)(volatile const uint32_t *)&(x)) - PERIPH_BASE)*32 + (b)*4)))

// Configuration
// Set value to 1 to enable, 0 to disable

#define USB_ENABLE      0
#define KEYPAD_ENABLE   0 // I2C keypad for jogging etc.
#define TRINAMIC_ENABLE	0 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
#define TRINAMIC_I2C    0 // Trinamic I2C - SPI bridge interface.
#define TRINAMIC_DEV    0 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code
#define CNC_BOOSTERPACK 0

#if CNC_BOOSTERPACK
#if N_AXIS > 3
#error Max number of axes is 3!
#endif
#define SDCARD_ENABLE 0 // Run jobs from SD card.
#define EEPROM_ENABLE 0 // I2C EEPROM (24LC16) support.
#else
#define SDCARD_ENABLE 0 // Run jobs from SD card.
#define EEPROM_ENABLE 0 // I2C EEPROM (24LC16) support.
#endif

#if EEPROM_ENABLE == 0
#define FLASH_ENABLE 1
#else
#define FLASH_ENABLE 0
#endif

#if EEPROM_ENABLE|| KEYPAD_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)
#define I2C_PORT
#endif

#if TRINAMIC_ENABLE || KEYPAD_ENABLE
#define DRIVER_SETTINGS
#endif

#ifdef DRIVER_SETTINGS

#include "tmc2130/trinamic.h"

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

// End configuration

// Define GPIO output mode options

#define GPIO_SHIFT0   0
#define GPIO_SHIFT1   1
#define GPIO_SHIFT2   2
#define GPIO_SHIFT3   3
#define GPIO_SHIFT4   4
#define GPIO_SHIFT5   5
#define GPIO_SHIFT6   6
#define GPIO_SHIFT7   7
#define GPIO_SHIFT8   8
#define GPIO_SHIFT9   9
#define GPIO_SHIFT10 10
#define GPIO_SHIFT11 11
#define GPIO_SHIFT12 12
#define GPIO_SHIFT13 13
#define GPIO_MAP     14
#define GPIO_BITBAND 15

// Define timer allocations.
#define SPINDLE_PWM_TIMER TIM1
#define STEPPER_TIMER TIM2
#define PULSE_TIMER TIM3
#define DEBOUNCE_TIMER TIM4

// Define step pulse output pins.
#define STEP_PORT       GPIOA
#define X_STEP_PIN      0
#define Y_STEP_PIN      1
#define Z_STEP_PIN      2
#define X_STEP_BIT      (1<<X_STEP_PIN)
#define Y_STEP_BIT      (1<<Y_STEP_PIN)
#define Z_STEP_BIT      (1<<Z_STEP_PIN)
#if N_AXIS > 3
#define A_STEP_PIN      3
#define A_STEP_BIT      (1<<A_STEP_PIN)
#define STEP_MASK       (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|A_STEP_BIT) // All step bits
#else
#define STEP_MASK       (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT) // All step bits
#endif
#define STEP_OUTMODE GPIO_SHIFT0

// Define step direction output pins.
#define DIRECTION_PORT		GPIOA
#define X_DIRECTION_PIN   	4
#define Y_DIRECTION_PIN   	5
#define Z_DIRECTION_PIN   	6
#define X_DIRECTION_BIT   	(1<<X_DIRECTION_PIN)
#define Y_DIRECTION_BIT   	(1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_BIT   	(1<<Z_DIRECTION_PIN)
#if N_AXIS > 3
#define A_DIRECTION_PIN   	7
#define A_DIRECTION_BIT   	(1<<A_DIRECTION_PIN)
#define DIRECTION_MASK    	(X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|A_DIRECTION_BIT) // All direction bits
#else
#define DIRECTION_MASK    	(X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT) // All direction bits
#endif
#define DIRECTION_OUTMODE 	GPIO_SHIFT4

// Define stepper driver enable/disable output pin.
#define STEPPERS_DISABLE_PORT   GPIOA
#define STEPPERS_DISABLE_PIN    15
#define STEPPERS_DISABLE_BIT    (1<<STEPPERS_DISABLE_PIN)
#define STEPPERS_DISABLE_MASK   STEPPERS_DISABLE_BIT

// Define homing/hard limit switch input pins.
#define LIMIT_PORT       GPIOB
#define X_LIMIT_PIN      12
#define Y_LIMIT_PIN      13
#define Z_LIMIT_PIN      14
#define X_LIMIT_BIT      (1<<X_LIMIT_PIN)
#define Y_LIMIT_BIT      (1<<Y_LIMIT_PIN)
#define Z_LIMIT_BIT      (1<<Z_LIMIT_PIN)
#if N_AXIS > 3
#define A_LIMIT_PIN      15
#define A_LIMIT_BIT      (1<<A_LIMIT_PIN)
#define LIMIT_MASK       (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT|A_LIMIT_BIT) // All limit bits
#else
#define LIMIT_MASK       (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT) // All limit bits
#endif
#define LIMIT_INMODE GPIO_SHIFT12

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT   		GPIOB
#define SPINDLE_ENABLE_PIN    		1
#define SPINDLE_ENABLE_BIT    		(1<<SPINDLE_ENABLE_PIN)
#define SPINDLE_DIRECTION_PORT   	GPIOB
#define SPINDLE_DIRECTION_PIN 		0
#define SPINDLE_DIRECTION_BIT		(1<<SPINDLE_DIRECTION_PIN)

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT            GPIOA
#define SPINDLE_PWM_PIN	            8
#define SPINDLE_PWM_BIT	            (1<<SPINDLE_PWM_PIN)

// Define flood and mist coolant enable output pins.
#if CNC_BOOSTERPACK
#define COOLANT_FLOOD_PORT         	GPIOC
#define COOLANT_FLOOD_PIN          	15
#define COOLANT_FLOOD_BIT          	(1<<COOLANT_FLOOD_PIN)
#define COOLANT_MIST_PORT         	GPIOC
#define COOLANT_MIST_PIN           	14
#define COOLANT_MIST_BIT           	(1<<COOLANT_MIST_PIN)
#else
#define COOLANT_FLOOD_PORT         	GPIOB
#define COOLANT_FLOOD_PIN          	4
#define COOLANT_FLOOD_BIT          	(1<<COOLANT_FLOOD_PIN)
#define COOLANT_MIST_PORT         	GPIOB
#define COOLANT_MIST_PIN           	3
#define COOLANT_MIST_BIT           	(1<<COOLANT_MIST_PIN)
#endif

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT               	GPIOB
#if CNC_BOOSTERPACK
#define CONTROL_RESET_PIN        	6
#define CONTROL_FEED_HOLD_PIN     	7
#define CONTROL_CYCLE_START_PIN    	8
#define CONTROL_SAFETY_DOOR_PIN  	9
#define CONTROL_INMODE GPIO_SHIFT6
#else
#define CONTROL_RESET_PIN        	5
#define CONTROL_FEED_HOLD_PIN     	6
#define CONTROL_CYCLE_START_PIN    	7
#define CONTROL_SAFETY_DOOR_PIN  	8
#define CONTROL_INMODE GPIO_SHIFT5
#endif
#define CONTROL_RESET_BIT        	(1<<CONTROL_RESET_PIN)
#define CONTROL_FEED_HOLD_BIT     	(1<<CONTROL_FEED_HOLD_PIN)
#define CONTROL_CYCLE_START_BIT   	(1<<CONTROL_CYCLE_START_PIN)
#define CONTROL_SAFETY_DOOR_BIT  	(1<<CONTROL_SAFETY_DOOR_PIN)
#define CONTROL_MASK            	(CONTROL_RESET_BIT|CONTROL_FEED_HOLD_BIT|CONTROL_CYCLE_START_BIT|CONTROL_SAFETY_DOOR_BIT)

// Define probe switch input pin.
#define PROBE_PORT                 	GPIOA
#define PROBE_PIN                	7
#define PROBE_BIT                 	(1<<PROBE_PIN)

#if KEYPAD_ENABLE
#define KEYPAD_PORT               	GPIOB
#define KEYPAD_STROBE_PIN        	15
#define KEYPAD_STROBE_BIT        	(1<<KEYPAD_STROBE_PIN)
#endif

#if SDCARD_ENABLE
#define SD_CS_PORT	GPIOA
#define SD_CS_PIN	3
#define SD_CS_BIT	(1<<SD_CS_PIN)
// The following defines are not used but defined for reference
// Port init and remap is done by HAL_SPI_MspInit() in stm32f1xx_hal_msp.c
#define SD_IO_PORT	GPIOB
#define SD_SCK_PIN  3
#define SD_SCK_BIT  (1<<SD_SCK_PIN)
#define SD_MISO_PIN	4
#define SD_MISO_BIT	(1<<SD_MISO_PIN)
#define SD_MOSI_PIN	5
#define SD_MOSI_BIT	(1<<SD_MOSI_PIN)
#endif

bool driver_init (void);

#endif // __DRIVER_H__
