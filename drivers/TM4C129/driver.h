/*
  driver.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Texas Instruments Tiva C (TM4C1294MCPDT) ARM processor/LaunchPad

  Part of Grbl

  Copyright (c) 2018 Terje Io
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

// Configuration

#define CNC_BOOSTERPACK         // comment in if for CNC Boosterpack
#ifdef CNC_BOOSTERPACK
#define CNC_BOOSTERPACK_SHORTS  // comment in if for CNC Boosterpack with shorts
#define CNC_BOOSTERPACK_A4998   // comment in if used with Polulu A4998 drivers - for suppying VDD via GPIO (PE5)
#if N_AXIS > 3
#define CNC_BOOSTERPACK2
#endif
#endif

//#define HAS_KEYPAD //uncomment to enable I2C keypad for jogging etc.
//#define PWM_RAMPED
//#define LASER_PPI

// End configuration

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

#define STEPPER_TIM TIMER1
#define STEPPER_TIMER_PERIPH    timerPeriph(STEPPER_TIM)
#define STEPPER_TIMER_BASE      timerBase(STEPPER_TIM)
#define STEPPER_TIMER_INT       timerINT(STEPPER_TIM, A)

#define PULSE_TIM TIMER0
#define PULSE_TIMER_PERIPH      timerPeriph(PULSE_TIM)
#define PULSE_TIMER_BASE        timerBase(PULSE_TIM)
#define PULSE_TIMER_INT         timerINT(PULSE_TIM, A)

#define DEBOUNCE_TIM TIMER4
#define DEBOUNCE_TIMER_PERIPH   timerPeriph(DEBOUNCE_TIM)
#define DEBOUNCE_TIMER_BASE     timerBase(DEBOUNCE_TIM)
#define DEBOUNCE_TIMER_INT      timerINT(DEBOUNCE_TIM, A)

#define SPINDLE_PWM_TIM TIMER3
#define SPINDLE_PWM_TIMER_PERIPH    timerPeriph(SPINDLE_PWM_TIM)
#define SPINDLE_PWM_TIMER_BASE      timerBase(SPINDLE_PWM_TIM)
//#define SPINDLE_PWM_TIMER_INT   timerINT(SPINDLE_PWM_TIM, A)

#define LASER_PPI_TIM TIMER2
#define LASER_PPI_TIMER_PERIPH  timerPeriph(LASER_PPI_TIM)
#define LASER_PPI_TIMER_BASE    timerBase(LASER_PPI_TIM)
#define LASER_PPI_TIMER_INT     timerINT(LASER_PPI_TIM, A)

// Define step pulse output pins.
#define STEP_PORT       GPIO_PORTE_BASE
#define X_STEP_PIN      GPIO_PIN_1
#define Y_STEP_PIN      GPIO_PIN_2
#define Z_STEP_PIN      GPIO_PIN_3
#define HWSTEP_MASK     (X_STEP_PIN|Y_STEP_PIN|Z_STEP_PIN) // All step bits
#define STEP_OUTMODE    GPIO_SHIFT1
#ifdef CNC_BOOSTERPACK2
#define STEP_PORT_AB    GPIO_PORTK_BASE
#define STEP_PORT_C     GPIO_PORTB_BASE
#define A_STEP_PIN      GPIO_PIN_0
#define B_STEP_PIN      GPIO_PIN_1
#define C_STEP_PIN      GPIO_PIN_5
#define HWSTEP_MASK_AB  (A_STEP_PIN|B_STEP_PIN)
#define STEP_OUTMODE    GPIO_SHIFT1
#endif

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIRECTION_PORT		GPIO_PORTD_BASE
#define X_DIRECTION_PIN		GPIO_PIN_1
#define Y_DIRECTION_PIN		GPIO_PIN_0
#define Z_DIRECTION_PIN		GPIO_PIN_3
#define HWDIRECTION_MASK    (X_DIRECTION_PIN|Y_DIRECTION_PIN|Z_DIRECTION_PIN) // All direction bits
#define DIRECTION_OUTMODE   GPIO_MAP
#ifdef CNC_BOOSTERPACK2
#define DIRECTION_PORT2     GPIO_PORTQ_BASE
#define A_DIRECTION_PIN     GPIO_PIN_3
#define B_DIRECTION_PIN     GPIO_PIN_0
#define C_DIRECTION_PIN     GPIO_PIN_2
#define HWDIRECTION_MASK2   (A_DIRECTION_PIN|B_DIRECTION_PIN|C_DIRECTION_PIN) // All direction bits
#define DIRECTION_OUTMODE2  GPIO_MAP
#endif

// Define stepper driver enable/disable output pin(s).
#ifdef CNC_BOOSTERPACK
#define STEPPERS_DISABLE_XY_PORT    GPIO_PORTD_BASE
#define STEPPERS_DISABLE_XY_PIN     GPIO_PIN_7
#define STEPPERS_DISABLE_Z_PORT     GPIO_PORTH_BASE
#define STEPPERS_DISABLE_Z_PIN      GPIO_PIN_3
 #ifdef CNC_BOOSTERPACK2
 #define STEPPERS_DISABLE_AC_PORT   GPIO_PORTK_BASE
 #define STEPPERS_DISABLE_AC_PIN    GPIO_PIN_2
 #define STEPPERS_DISABLE_B_PORT    GPIO_PORTA_BASE
 #define STEPPERS_DISABLE_B_PIN GPIO_PIN_7
 #endif
#else
#define STEPPERS_DISABLE_PORT   GPIO_PORTE_BASE
#define STEPPERS_DISABLE_PIN    GPIO_PIN_1
#endif

#ifdef CNC_BOOSTERPACK_A4998
// Stepper driver VDD supply
#define STEPPERS_VDD_PORT GPIO_PORTE_BASE
#define STEPPERS_VDD_PIN  GPIO_PIN_5
 #ifdef CNC_BOOSTERPACK2
  #define STEPPERS_VDD_PORT2 GPIO_PORTD_BASE
  #define STEPPERS_VDD_PIN2  GPIO_PIN_5
 #endif
#endif

// Define homing/hard limit switch input pins and limit interrupt vectors.
#ifdef CNC_BOOSTERPACK
    #define LIMIT_PORT_X    GPIO_PORTH_BASE
    #define LIMIT_PORT_YZ   GPIO_PORTF_BASE
    #define X_LIMIT_PIN     GPIO_PIN_2
    #define Y_LIMIT_PIN     GPIO_PIN_2
    #define Z_LIMIT_PIN     GPIO_PIN_1
    #define HWLIMIT_MASK_YZ (Y_LIMIT_PIN|Z_LIMIT_PIN) // All limit bits
  #ifdef CNC_BOOSTERPACK2
    #define LIMIT_PORT_A    GPIO_PORTK_BASE
    #define LIMIT_PORT_B    GPIO_PORTG_BASE
    #define LIMIT_PORT_C    GPIO_PORTP_BASE
    #define A_LIMIT_PIN     GPIO_PIN_4
    #define B_LIMIT_PIN     GPIO_PIN_1
    #define C_LIMIT_PIN     GPIO_PIN_5
  #endif
#else
    #define LIMIT_PORT      GPIO_PORTA_BASE
    #define X_LIMIT_PIN     GPIO_PIN_1
    #define Y_LIMIT_PIN     GPIO_PIN_3
    #define Z_LIMIT_PIN     GPIO_PIN_2
    #define HWLIMIT_MASK    (X_LIMIT_PIN|Y_LIMIT_PIN|Z_LIMIT_PIN) // All limit bits
#endif

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     GPIO_PORTA_BASE
#define SPINDLE_ENABLE_PIN      GPIO_PIN_6

#define SPINDLE_DIRECTION_PORT  GPIO_PORTM_BASE
#define SPINDLE_DIRECTION_PIN   GPIO_PIN_4

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT  GPIO_PORTL_BASE
#define COOLANT_FLOOD_PIN   GPIO_PIN_1

#define COOLANT_MIST_PORT	GPIO_PORTL_BASE
#define COOLANT_MIST_PIN	GPIO_PIN_2

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#ifdef CNC_BOOSTERPACK
  #ifdef CNC_BOOSTERPACK_SHORTS
    #define CONTROL_PORT    GPIO_PORTL_BASE
    #define CONTROL_PORT_SD GPIO_PORTG_BASE
    #define RESET_PIN       GPIO_PIN_0
    #define FEED_HOLD_PIN   GPIO_PIN_4
    #define CYCLE_START_PIN GPIO_PIN_5
    #define SAFETY_DOOR_PIN GPIO_PIN_0
    #define HWCONTROL_MASK  (RESET_PIN|FEED_HOLD_PIN|CYCLE_START_PIN|SAFETY_DOOR_PIN)
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
#define PROBE_PORT		GPIO_PORTC_BASE
#define PROBE_PIN		GPIO_PIN_7

// Start of PWM & Stepper Enabled Spindle
#define SPINDLEPPORT    GPIO_PORTM_BASE
#define SPINDLEPPIN     GPIO_PIN_3
#define SPINDLEPWM_MAP  GPIO_PM3_T3CCP1

#ifdef LASER_PPI

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
