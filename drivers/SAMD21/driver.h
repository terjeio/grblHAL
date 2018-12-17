/*
  driver.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Atmel SAMD21

  Part of Grbl

  Copyright (c) 2018 Terje Io

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

#include "grbl.h"

// Configuration
// Set value to 1 to enable, 0 to disable

#define SDCARD_ENABLE 1 // Run jobs from SD card.
#define USB_SERIAL    0 // Streaming over USB direct.

// End configuration

// clock definitions

#define CLKTCC_0_1 GCLK_CLKCTRL_GEN_GCLK4

// timer definitions

#define STEP_TIMER         TC3
#define STEP_TIMER_IRQn    TC3_IRQn

#define STEPPER_TIMER         TC4 // 32bit - TC4 & TC5 combined!
#define STEPPER_TIMER_IRQn    TC4_IRQn

#define DEBOUNCE_TIMER         TCC1
#define DEBOUNCE_TIMER_IRQn    TCC1_IRQn
#
// Define step pulse output pins.
#define X_STEP_PIN      (19u)
#define Y_STEP_PIN      (20u)
#define Z_STEP_PIN      (21u)

// Define step direction output pins.
#define X_DIRECTION_PIN     (2u)
#define Y_DIRECTION_PIN     (3u)
#define Z_DIRECTION_PIN     (4u)

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_DISABLE_PIN    (25u)

// Define homing/hard limit switch input pins and limit interrupt vectors.
#define X_LIMIT_PIN     (0u)
#define Y_LIMIT_PIN     (1u)
#define Z_LIMIT_PIN     (10u)

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PIN      (16u)
#define SPINDLE_DIRECTION_PIN   (17u)

// Start of PWM & Stepper Enabled Spindle
#define SPINDLE_PWM_TIMER		TCC0
#define SPINDLE_PWM_CCREG		2
#define SPINDLEPWMPIN     		(6u)

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PIN   (15u)
#define COOLANT_MIST_PIN    (7u)

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PIN           (5u)
#define FEED_HOLD_PIN       (9u)
#define CYCLE_START_PIN     (8u)
//#define SAFETY_DOOR_PIN     (7u)

// Define probe switch input pin.
#define PROBE_PIN       (18U)

// Define SD card detect pin.
#define SD_CD_PIN   30

void IRQRegister(uint32_t IRQnum, void (*IRQhandler)(void));
void IRQUnRegister(uint32_t IRQnum);

#endif
