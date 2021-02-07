/*
  cnc_boosterpack_map.h - driver code for Texas Instruments Tiva C (TM4C123GH6PM) ARM processor

  - on Texas Instruments MSP432P401R LaunchPad

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

#define BOARD_NAME "CNC BoosterPack"

#if TRINAMIC_ENABLE
#ifdef TRINAMIC_MIXED_DRIVERS
#undef TRINAMIC_MIXED_DRIVERS
#endif
#define TRINAMIC_MIXED_DRIVERS 0
#ifdef TRINAMIC_I2C
#undef TRINAMIC_I2C
#endif
#define TRINAMIC_I2C 1
#endif

#ifdef CNC_BOOSTERPACK
#undef CNC_BOOSTERPACK
#endif
#define CNC_BOOSTERPACK 1

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
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    #define SAFETY_DOOR_PIN     GPIO_PIN_4
#endif
  #endif
#else
#define CONTROL_PORT        GPIO_PORTC_BASE
#define RESET_PIN           GPIO_PIN_7
#define FEED_HOLD_PIN       GPIO_PIN_6
#define CYCLE_START_PIN     GPIO_PIN_5
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define SAFETY_DOOR_PIN     GPIO_PIN_4
#endif
#endif
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define HWCONTROL_MASK      (RESET_PIN|FEED_HOLD_PIN|CYCLE_START_PIN|SAFETY_DOOR_PIN)
#else
#define HWCONTROL_MASK      (RESET_PIN|FEED_HOLD_PIN|CYCLE_START_PIN)
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

/* EOF */
