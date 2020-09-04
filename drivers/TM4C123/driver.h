/*
  driver.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Grbl driver code for Texas Instruments Tiva C (TM4C123GH6PM) ARM processor

  Part of Grbl

  Copyright (c) 2016-2020 Terje Io
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include "tiva.h"

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "grbl/hal.h"
#include "grbl/grbl.h"
#include "grbl/nuts_bolts.h"

#ifndef PWM_RAMPED
#define PWM_RAMPED              0
#endif
#ifndef KEYPAD_ENABLE
#define KEYPAD_ENABLE           0
#endif
#ifndef LASER_PPI
#define LASER_PPI               0
#endif
#ifndef TRINAMIC_ENABLE
#define TRINAMIC_ENABLE         0
#endif
#ifndef TRINAMIC_I2C
#define TRINAMIC_I2C            0
#endif
#ifndef TRINAMIC_DEV
#define TRINAMIC_DEV            0
#endif
#ifndef CNC_BOOSTERPACK_SHORTS
#define CNC_BOOSTERPACK_SHORTS  0
#endif
#ifndef CNC_BOOSTERPACK_A4998
#define CNC_BOOSTERPACK_A4998   0
#endif

#define CNC_BOOSTERPACK         0

#define timerBase(t) timerB(t)
#define timerB(t) t ## _BASE
#define timerPeriph(t) timerP(t)
#define timerP(t) SYSCTL_PERIPH_ ## t
#define timerINT(t, i) timerI(t, i)
#define timerI(t, i) INT_ ## t ## i

// Define GPIO output mode options

#define GPIO_SHIFT0  0
#define GPIO_SHIFT1  1
#define GPIO_SHIFT2  2
#define GPIO_SHIFT3  3
#define GPIO_SHIFT4  4
#define GPIO_SHIFT5  5
#define GPIO_MAP     8
#define GPIO_BITBAND 9

// timer definitions

#define STEPPER_TIM WTIMER0
#define STEPPER_TIMER_PERIPH timerPeriph(STEPPER_TIM)
#define STEPPER_TIMER_BASE timerBase(STEPPER_TIM)
#define STEPPER_TIMER_INT timerINT(STEPPER_TIM, A)

#define PULSE_TIM TIMER0
#define PULSE_TIMER_PERIPH timerPeriph(PULSE_TIM)
#define PULSE_TIMER_BASE timerBase(PULSE_TIM)
#define PULSE_TIMER_INT timerINT(PULSE_TIM, A)

#define DEBOUNCE_TIM TIMER4
#define DEBOUNCE_TIMER_PERIPH timerPeriph(DEBOUNCE_TIM)
#define DEBOUNCE_TIMER_BASE timerBase(DEBOUNCE_TIM)
#define DEBOUNCE_TIMER_INT timerINT(DEBOUNCE_TIM, A)

#define SPINDLE_PWM_TIM TIMER3
#define SPINDLE_PWM_TIMER_PERIPH timerPeriph(SPINDLE_PWM_TIM)
#define SPINDLE_PWM_TIMER_BASE timerBase(SPINDLE_PWM_TIM)
//#define SPINDLE_PWM_TIMER_INT timerINT(SPINDLE_PWM_TIM, A)

#ifdef BOARD_CNC_BOOSTERPACK
#include "cnc_boosterpack_map.h"
#else
#error No board!
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 1.3f // microseconds
#endif

// End configuration

#if TRINAMIC_ENABLE || KEYPAD_ENABLE
#define DRIVER_SETTINGS
#endif

#ifdef DRIVER_SETTINGS

#if TRINAMIC_ENABLE
#include "tmc2130/trinamic.h"
#endif

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


#if LASER_PPI

#define LASER_PPI_TIM TIMER2
#define LASER_PPI_TIMER_PERIPH timerPeriph(LASER_PPI_TIM)
#define LASER_PPI_TIMER_BASE timerBase(LASER_PPI_TIM)
#define LASER_PPI_TIMER_INT timerINT(LASER_PPI_TIM, A)

typedef struct {
    float ppi;
    uint_fast16_t steps_per_pulse;
    uint_fast16_t pulse_length; // uS
    uint32_t next_pulse;
} laser_ppi_t;

extern laser_ppi_t laser;

void laser_ppi_mode (bool on);

#endif

#endif
