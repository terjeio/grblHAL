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

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include "tiva.h"

#include "grbl/hal.h"
#include "grbl/grbl.h"
#include "grbl/nuts_bolts.h"

// Configuration
// Set value to 1 to enable, 0 to disable

#define PWM_RAMPED              0 // Ramped spindle PWM.
#define LASER_PPI               0 // Laser PPI (Pulses Per Inch) option.
#define KEYPAD_ENABLE           0 // I2C keypad for jogging etc.
#define ATC_ENABLE              0 // do not change!
#define TRINAMIC_ENABLE         0 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
#define TRINAMIC_I2C            0 // Trinamic I2C - SPI bridge interface.
#define TRINAMIC_DEV            1 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code
#define CNC_BOOSTERPACK         1 // Use CNC Boosterpack pin assignments.
#if CNC_BOOSTERPACK
  #define CNC_BOOSTERPACK_SHORTS  1 // Shorts added to BoosterPack for some signals (for faster and simpler driver)
  #define CNC_BOOSTERPACK_A4998   1 // Using Polulu A4998 drivers - for suppying VDD via GPIO (PE5)
#else
  #define CNC_BOOSTERPACK_SHORTS 0 // do not change!
  #define CNC_BOOSTERPACK_A4998  0 // do not change!
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#define STEP_PULSE_LATENCY 1.3f // microseconds

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

#define LASER_PPI_TIM TIMER2
#define LASER_PPI_TIMER_PERIPH timerPeriph(LASER_PPI_TIM)
#define LASER_PPI_TIMER_BASE timerBase(LASER_PPI_TIM)
#define LASER_PPI_TIMER_INT timerINT(LASER_PPI_TIM, A)

// Define step pulse output pins.
#define STEP_PORT   GPIO_PORTD_BASE
#define X_STEP_PIN  GPIO_PIN_1
#define Y_STEP_PIN  GPIO_PIN_2
#define Z_STEP_PIN  GPIO_PIN_3
#define HWSTEP_MASK (X_STEP_PIN|Y_STEP_PIN|Z_STEP_PIN) // All step bits
#define STEP_OUTMODE GPIO_SHIFT1

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIRECTION_PORT      GPIO_PORTB_BASE
#define X_DIRECTION_PIN     GPIO_PIN_7
#define Y_DIRECTION_PIN     GPIO_PIN_6
#define Z_DIRECTION_PIN     GPIO_PIN_4
#define HWDIRECTION_MASK    (X_DIRECTION_PIN|Y_DIRECTION_PIN|Z_DIRECTION_PIN) // All direction bits
#define DIRECTION_OUTMODE   GPIO_MAP

#if TRINAMIC_ENABLE
#define TRINAMIC_DIAG_IRQ_PORT      GPIO_PORTE_BASE
#define TRINAMIC_DIAG_IRQ_PIN       GPIO_PIN_1
#define TRINAMIC_WARN_IRQ_PORT      GPIO_PORTF_BASE
#define TRINAMIC_WARN_IRQ_PIN       GPIO_PIN_0
// Define stepper driver enable/disable output pin(s).
#elif CNC_BOOSTERPACK
#define STEPPERS_DISABLE_XY_PORT    GPIO_PORTE_BASE
#define STEPPERS_DISABLE_XY_PIN     GPIO_PIN_1
#define STEPPERS_DISABLE_Z_PORT     GPIO_PORTF_BASE
#define STEPPERS_DISABLE_Z_PIN      GPIO_PIN_0
#else
#define STEPPERS_DISABLE_PORT   GPIO_PORTE_BASE
#define STEPPERS_DISABLE_PIN    GPIO_PIN_1
#endif

#if CNC_BOOSTERPACK_A4998
// Stepper driver VDD supply
#define STEPPERS_VDD_PORT GPIO_PORTE_BASE
#define STEPPERS_VDD_PIN  GPIO_PIN_5
#endif

// Define homing/hard limit switch input pins and limit interrupt vectors.
#if CNC_BOOSTERPACK
  #if CNC_BOOSTERPACK_SHORTS
    #define LIMIT_PORT      GPIO_PORTF_BASE
    #define X_LIMIT_PIN     GPIO_PIN_1
    #define Y_LIMIT_PIN     GPIO_PIN_3
    #define Z_LIMIT_PIN     GPIO_PIN_2
    #define HWLIMIT_MASK    (X_LIMIT_PIN|Y_LIMIT_PIN|Z_LIMIT_PIN) // All limit bits
  #endif
#else
    #define LIMIT_PORT   GPIO_PORTA_BASE
    #define X_LIMIT_PIN  GPIO_PIN_1
    #define Y_LIMIT_PIN  GPIO_PIN_3
    #define Z_LIMIT_PIN  GPIO_PIN_2
    #define HWLIMIT_MASK   (X_LIMIT_PIN|Y_LIMIT_PIN|Z_LIMIT_PIN) // All limit bits
#endif

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     GPIO_PORTE_BASE
#define SPINDLE_ENABLE_PIN      GPIO_PIN_2

#define SPINDLE_DIRECTION_PORT      GPIO_PORTE_BASE
#define SPINDLE_DIRECTION_PIN       GPIO_PIN_3

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT      GPIO_PORTD_BASE
#define COOLANT_FLOOD_PIN       GPIO_PIN_6

#define COOLANT_MIST_PORT   GPIO_PORTD_BASE
#define COOLANT_MIST_PIN    GPIO_PIN_7

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#if CNC_BOOSTERPACK
  #if CNC_BOOSTERPACK_SHORTS
    #define CONTROL_PORT        GPIO_PORTC_BASE
    #define RESET_PIN           GPIO_PIN_7
    #define FEED_HOLD_PIN       GPIO_PIN_5
    #define CYCLE_START_PIN     GPIO_PIN_6
    #define SAFETY_DOOR_PIN     GPIO_PIN_4
    #define HWCONTROL_MASK      (RESET_PIN|FEED_HOLD_PIN|CYCLE_START_PIN|SAFETY_DOOR_PIN)
  #endif
#else
#define CONTROL_PORT        GPIO_PORTC_BASE
#define RESET_PIN           GPIO_PIN_7
#define FEED_HOLD_PIN       GPIO_PIN_6
#define CYCLE_START_PIN     GPIO_PIN_5
#define SAFETY_DOOR_PIN     GPIO_PIN_4
#define HWCONTROL_MASK      (RESET_PIN|FEED_HOLD_PIN|CYCLE_START_PIN|SAFETY_DOOR_PIN)
#endif

// Define probe switch input pin.
#define PROBE_PORT      GPIO_PORTA_BASE
#define PROBE_PIN       GPIO_PIN_5

// Start of PWM & Stepper Enabled Spindle
#define SPINDLEPPERIPH      SYSCTL_PERIPH_GPIOB
#define SPINDLEPPORT        GPIO_PORTB_BASE
#define SPINDLEPPIN         GPIO_PIN_2
#define SPINDLEPWM_MAP      GPIO_PB2_T3CCP0

#if KEYPAD_ENABLE
#define KEYINTR_PIN   GPIO_PIN_4
#define KEYINTR_PORT  GPIO_PORTE_BASE
#endif

#if LASER_PPI

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
