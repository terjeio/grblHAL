/*

  driver.h - pin mapping configuration file for Texas Instruments MSP432 ARM processor

  Part of Grbl

  Copyright (c) 2017-2018 Terje Io

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

/*****************************************************************************************
 *                                                                                       *
 * NOTE: any assingment changes related to interrupt generating peripherals              *
 *       must be synchronized with entries in the msp432_startup_ccs.c IRQ vector table  *
 *                                                                                       *
 *****************************************************************************************/

#ifndef __DRIVER_H__
#define __DRIVER_H__

#define NO_MSP_CLASSIC_DEFINES

#define CNC_BOOSTERPACK_SHORTS
#define CNC_BOOSTERPACK_A4998 // comment out if used with Polulu A4998 drivers - for suppying VDD via GPIO (P4.3)

#include <stdbool.h>
#include <stdint.h>

#include "msp.h"

#include "GRBL\grbl.h"

#define port(p) portI(p)
#define portI(p) P ## p
#define portGpio(p) portG(p)
#define portG(p) GPIO_PORT_P ## p
#define portINT(p) portQ(p)
#define portQ(p) PORT ## p ## _IRQn

#define timer(p) timerN(p)
#define timerN(p) TIMER_ ## p
#define timerINT0(p) timerI0(p)
#define timerI0(p) T ## p ## _0_IRQn
#define timerINTN(p) timerIN(p)
#define timerIN(p) T ## p ## _N_IRQn

#define timer32(p) timer32N(p)
#define timer32N(p) TIMER32_ ## p
#define timer32INT0(p) timer32I0(p)
#define timer32I0(p) T32_INT ## p ## _IRQn

// Define GPIO I/O mode options

#define GPIO_SHIFT0   0
#define GPIO_SHIFT1   1
#define GPIO_SHIFT2   2
#define GPIO_SHIFT3   3
#define GPIO_SHIFT4   4
#define GPIO_SHIFT5   5
#define GPIO_MAP      8
#define GPIO_BITBAND  9
#define GPIO_MASKED  10

// Define timer registers

#define STEPPER_TIM 1
#define STEPPER_TIMER timer32(STEPPER_TIM)
#define STEPPER_TIMER_INT0 timer32INT0(STEPPER_TIM)

#define PULSE_TIM A2
#define PULSE_TIMER timer(PULSE_TIM)
#define PULSE_TIMER_INT0 timerINT0(PULSE_TIM)

#define DEBOUNCE_TIM A3
#define DEBOUNCE_TIMER timer(DEBOUNCE_TIM)
#define DEBOUNCE_TIMER_INT0 timerINT0(DEBOUNCE_TIM)

#define RPM_CNT A1
#define RPM_COUNTER timer(RPM_CNT)
#define RPM_COUNTER_INT0 timerINT0(RPM_CNT)
#define RPM_COUNTER_PORT P7
#define RPM_COUNTER_PIN 2
#define RPM_COUNTER_BIT (1<<RPM_COUNTER_PIN)

#define RPM_TIM 2
#define RPM_TIMER timer32(RPM_TIM)
#define RPM_TIMER_INT0 timer32INT0(RPM_TIM)

#define RPM_INDEX_PN 6
#define RPM_INDEX_PORT port(RPM_INDEX_PN)
#define RPM_INDEX_PIN 3
#define RPM_INDEX_BIT (1<<RPM_INDEX_PIN)
#define RPM_INDEX_INT portINT(RPM_INDEX_PN)

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.

#define STEP_PORT       P4
#define X_STEP_PIN      0
#define Y_STEP_PIN      2
#define Z_STEP_PIN      4
#define X_STEP_BIT      (1<<X_STEP_PIN)
#define Y_STEP_BIT      (1<<Y_STEP_PIN)
#define Z_STEP_BIT      (1<<Z_STEP_PIN)
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT) // All step bits
//#define STEP_OUTMODE GPIO_SHIFT3
//#define STEP_OUTMODE GPIO_BITBAND
#define STEP_OUTMODE GPIO_MAP

// Define step direction output pins. NOTE: All direction pins must be on the same port.

#define DIRECTION_PORT    P1
#define X_DIRECTION_PIN   6
#define Y_DIRECTION_PIN   7
#define Z_DIRECTION_PIN   5
#define X_DIRECTION_BIT   (1<<X_DIRECTION_PIN)
#define Y_DIRECTION_BIT   (1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_BIT   (1<<Z_DIRECTION_PIN)
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT) // All direction bits
//#define DIRECTION_OUTMODE GPIO_MAP
#define DIRECTION_OUTMODE GPIO_BITBAND

// Define stepper driver enable/disable output pin.

#define STEPPERS_DISABLE_Z_PORT  P5
#define STEPPERS_DISABLE_Z_PIN   7
#define STEPPERS_DISABLE_Z_BIT   (1<<STEPPERS_DISABLE_Z_PIN)

#define STEPPERS_DISABLE_XY_PORT P4
#define STEPPERS_DISABLE_X_PIN   5
#define STEPPERS_DISABLE_Y_PIN   5
#define STEPPERS_DISABLE_X_BIT   (1<<STEPPERS_DISABLE_X_PIN)
#define STEPPERS_DISABLE_Y_BIT   (1<<STEPPERS_DISABLE_Y_PIN)

#ifdef CNC_BOOSTERPACK_A4998

// Stepper driver VDD supply

#define STEPPERS_VDD_PORT P4
#define STEPPERS_VDD_PIN  3
#define STEPPERS_VDD_BIT  (1<<STEPPERS_VDD_PIN)

#endif

// Define homing/hard limit switch input pins
// NOTE: All limit bit pins must be on the same port

#ifdef CNC_BOOSTERPACK_SHORTS

#define LIMIT_PN      2
#define LIMIT_PORT    port(LIMIT_PN)
#define LIMIT_GPIO    portGpio(LIMIT_PN)
#define LIMIT_INT     portINT(LIMIT_PN)

#define X_LIMIT_PIN 3
#define Y_LIMIT_PIN 6
#define Z_LIMIT_PIN 7
#define X_LIMIT_BIT (1<<X_LIMIT_PIN)
#define Y_LIMIT_BIT (1<<Y_LIMIT_PIN)
#define Z_LIMIT_BIT (1<<Z_LIMIT_PIN)
//#define LIMIT_INMODE GPIO_BITBAND
#define LIMIT_MASK  (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT) // All limit bits

#else

#define LIMIT_PN_X      3
#define LIMIT_PORT_X    port(LIMIT_PN_X)
#define LIMIT_GPIO_X    portGpio(LIMIT_PN_X)
#define LIMIT_INT_X     portINT(LIMIT_PN_X)

#define LIMIT_PN_YZ     2
#define LIMIT_PORT_Y    port(LIMIT_PN_YZ)
#define LIMIT_PORT_Z    port(LIMIT_PN_YZ)
#define LIMIT_GPIO_YZ   portGpio(LIMIT_PN_YZ)
#define LIMIT_INT_YZ    portINT(LIMIT_PN_YZ)

#define X_LIMIT_PIN 0
#define Y_LIMIT_PIN 6
#define Z_LIMIT_PIN 7
#define X_LIMIT_BIT (1<<X_LIMIT_PIN)
#define Y_LIMIT_BIT (1<<Y_LIMIT_PIN)
#define Z_LIMIT_BIT (1<<Z_LIMIT_PIN)
#define LIMIT_INMODE GPIO_BITBAND
//#define LIMIT_SHIFT GPIO_SHIFT4 // Uncomment and set shift value if pins are consecutive and ordered
#define LIMIT_MASK_X  (X_LIMIT_BIT) // All limit bits
#define LIMIT_MASK_YZ  (Y_LIMIT_BIT|Z_LIMIT_BIT) // All limit bits

#endif

// Define flood and mist coolant output pins.

#define COOLANT_FLOOD_PORT  P5
#define COOLANT_FLOOD_PIN   1
#define COOLANT_FLOOD_BIT   (1<<COOLANT_FLOOD_PIN)
#define COOLANT_MIST_PORT   P3
#define COOLANT_MIST_PIN    5
#define COOLANT_MIST_BIT    (1<<COOLANT_MIST_PIN)

// Define user-control controls (cycle start, reset, feed hold) input pins.

#ifdef CNC_BOOSTERPACK_SHORTS

#define CONTROL_PN          6
#define CONTROL_PORT        port(CONTROL_PN)
#define CONTROL_GPIO        portGpio(CONTROL_PN)
#define CONTROL_INT         portINT(CONTROL_PN)

#define RESET_PIN           0
#define FEED_HOLD_PIN       6
#define CYCLE_START_PIN     7
#define SAFETY_DOOR_PIN     1
#define RESET_BIT           (1<<RESET_PIN)
#define FEED_HOLD_BIT       (1<<FEED_HOLD_PIN)
#define CYCLE_START_BIT     (1<<CYCLE_START_PIN)
#define SAFETY_DOOR_BIT     (1<<SAFETY_DOOR_PIN)
#define CONTROL_MASK        (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT|SAFETY_DOOR_BIT)
//#define CONTROL_SHIFT       GPIO_SHIFT0 // Uncomment and set shift value if pins are consecutive and ordered
//#define CONTROL_INMODE      GPIO_BITBAND

#else

#define CONTROL_PN_CS       2
#define CONTROL_PN_FH       5
#define CONTROL_PN_SD       6
#define CONTROL_PN_RST      6
#define CONTROL_PORT_CS     port(CONTROL_PN_CS)
#define CONTROL_PORT_FH     port(CONTROL_PN_FH)
#define CONTROL_PORT_SD     port(CONTROL_PN_SD)
#define CONTROL_PORT_RST    port(CONTROL_PN_RST)
#define CONTROL_GPIO_CS     portGpio(CONTROL_PN_CS)
#define CONTROL_GPIO_FH     portGpio(CONTROL_PN_FH)
#define CONTROL_GPIO_SD_RST portGpio(CONTROL_PN_SD)
#define CONTROL_INT_SD_RST  portINT(CONTROL_PN_SD)
#define CONTROL_INT_FH      portINT(CONTROL_PN_FH)

#define RESET_PIN           7
#define FEED_HOLD_PIN       6
#define CYCLE_START_PIN     4
#define SAFETY_DOOR_PIN     6
#define RESET_BIT           (1<<RESET_PIN)
#define FEED_HOLD_BIT       (1<<FEED_HOLD_PIN)
#define CYCLE_START_BIT     (1<<CYCLE_START_PIN)
#define SAFETY_DOOR_BIT     (1<<SAFETY_DOOR_PIN)
#define CONTROL_MASK        (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT|SAFETY_DOOR_BIT)
//#define CONTROL_SHIFT       GPIO_SHIFT0 // Uncomment and set shift value if pins are consecutive and ordered
//#define CONTROL_INMODE GPIO_BITBAND

#endif

// Define probe switch input pin.

#define PROBE_PN            4
#define PROBE_PORT          port(PROBE_PN)
#define PROBE_GPIO          portGpio(PROBE_PN)
#define PROBE_PIN           6
#define PROBE_BIT           (1<<PROBE_PIN)

// Define spindle enable, spindle direction and PWM output pins.

#define SPINDLE_ENABLE_PORT   P4
#define SPINDLE_ENABLE_PIN    7
#define SPINDLE_ENABLE_BIT    (1<<SPINDLE_ENABLE_PIN)

#define SPINDLE_DIRECTION_PORT  P5
#define SPINDLE_DIRECTION_PIN   4
#define SPINDLE_DIRECTION_BIT   (1<<SPINDLE_DIRECTION_PIN)

#define SPINDLE_PWM_PORT  P2
#define SPINDLE_PWM_PIN   5
#define SPINDLE_PWM_BIT   (1<<SPINDLE_PWM_PIN)

// Define spindle PWM timer parameters

#define SPINDLE_PWM_TIMER TIMER_A0

/*
 * CNC Boosterpack GPIO assignments
 */

#define GPIO0_PN             3
#define GPIO0_PORT           port(GPIO0_PN)
#define GPIO0_GPIO           portGpio(GPIO0_PN)
#define GPIO0_INT            portINT(GPIO0_PN)
#define GPIO0_PIN            7
#define GPIO0_BIT            (1<<GPIO0_PIN)

#define GPIO1_PN             5
#define GPIO1_PORT           port(GPIO1_PN)
#define GPIO1_GPIO           portGpio(GPIO1_PN)
#define GPIO1_INT            portINT(GPIO1_PN)
#define GPIO1_PIN            0
#define GPIO1_BIT            (1<<GPIO1_PIN)

// Normally used as MPG mode input
#define GPIO2_PN             5
#define GPIO2_PORT           port(GPIO2_PN)
#define GPIO2_GPIO           portGpio(GPIO2_PN)
#define GPIO2_INT            portINT(GPIO2_PN)
#define GPIO2_PIN            2
#define GPIO2_BIT            (1<<GPIO2_PIN)

#define GPIO3_PN             3
#define GPIO3_PORT           port(GPIO3_PN)
#define GPIO3_GPIO           portGpio(GPIO3_PN)
#define GPIO3_INT            portINT(GPIO3_PN)
#define GPIO3_PIN            6
#define GPIO3_BIT            (1<<GPIO3_PIN)

// GPIO4 is shared with UART2 RX
#define GPIO4_PN             3
#define GPIO4_PORT           port(GPIO4_PN)
#define GPIO4_GPIO           portGpio(GPIO4_PN)
#define GPIO4_INT            portINT(GPIO4_PN)
#define GPIO4_PIN            2
#define GPIO4_BIT            (1<<GPIO4_PIN)

// GPIO5 is shared with UART2 TX
#define GPIO5_PN             3
#define GPIO5_PORT           port(GPIO5_PN)
#define GPIO5_GPIO           portGpio(GPIO5_PN)
#define GPIO5_INT            portINT(GPIO5_PN)
#define GPIO5_PIN            3
#define GPIO5_BIT            (1<<GPIO5_PIN)

// Normally used as keypad strobe input
#define GPIO6_PN             4
#define GPIO6_PORT           port(GPIO6_PN)
#define GPIO6_GPIO           portGpio(GPIO6_PN)
#define GPIO6_INT            portINT(GPIO6_PN)
#define GPIO6_PIN            1
#define GPIO6_BIT            (1<<GPIO6_PIN)

// Define MPG mode input (for selecting secondary UART input)

#define MODE_PORT           GPIO2_PORT
#define MODE_GPIO           GPIO2_GPIO
#define MODE_INT            GPIO2_INT
#define MODE_SWITCH_PIN     GPIO2_PIN
#define MODE_SWITCH_BIT     GPIO2_BIT

// to be removed?
#define MODE_LED_PIN        0
#define MODE_LED_BIT        (1<<MODE_LED_PIN)

#define KEYPAD_PORT         GPIO6_PORT
#define KEYPAD_GPIO         GPIO6_GPIO
#define KEYPAD_INT          GPIO6_INT
#define KEYPAD_IRQ_PIN      GPIO6_PIN
#define KEYPAD_IRQ_BIT      GPIO6_BIT

// Driver initialization entry point

bool driver_init (void);

#endif // __DRIVER_H__

