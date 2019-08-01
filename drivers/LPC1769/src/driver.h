/*

  driver.h - driver code for NXP LPC176x ARM processors

  Part of GrblHAL

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

/*****************************************************************************************
 *                                                                                       *
 * NOTE: any assingment changes related to interrupt generating peripherals              *
 *       must be synchronized with entries in the msp432_startup_ccs.c IRQ vector table  *
 *                                                                                       *
 *****************************************************************************************/

#include "LPC17xx.h"

#ifndef __DRIVER_H__
#define __DRIVER_H__

//-------------------------------------------------
#include <stdbool.h>
#include <stdint.h>
//-------------------------------------------------

// Configuration
// Set value to 1 to enable, 0 to disable

#define SDCARD_ENABLE 0 // Run jobs from SD card.

// End configuration

#define port(p) portI(p)
#define portI(p) LPC_GPIO ## p
#define portGpio(p) portG(p)
#define portG(p) GPIO_PORT_P ## p
#define portINT(p) portQ(p)
#define portQ(p) PORT ## p ## _IRQn
#define portINTER(p) portQR(p)
#define portQR(p) LPC_GPIOINT->IO ## p ## IntEnR
#define portINTEF(p) portQF(p)
#define portQF(p) LPC_GPIOINT->IO ## p ## IntEnF
#define portINTSR(p) portQSR(p)
#define portQSR(p) LPC_GPIOINT->IO ## p ## IntStatR
#define portINTSF(p) portQSF(p)
#define portQSF(p) LPC_GPIOINT->IO ## p ## IntStatF
#define portINTCLR(p) portC(p)
#define portC(p) LPC_GPIOINT->IO ## p ## IntClr
#define portRDI(p) portR(p * 2)
#define portR(p) PINMODE ## p

#define P0Int (1<<0)
#define P2Int (1<<2)

#define PINMODE_PULLUP   0x0
#define PINMODE_REPEATER 0x1
#define PINMODE_FLOATING 0x2
#define PINMODE_PULLDOWN 0x3

#define timer(p) timerN(p)
#define timerN(p) LPC_TIM ## p
#define timerINT0(p) timerI0(p)
#define timerI0(p) TIMER ## p ## _IRQn
#define timerINTN(p) timerIN(p)
#define timerIN(p) T ## p ## _N_IRQn

#define MR0I (1<<0)
#define MR0R (1<<1)
#define MR0S (1<<2)
#define MR1I (1<<3)
#define MR1R (1<<4)
#define MR1S (1<<5)

#define MR0IFG (1<<0)
#define MR1IFG (1<<1)

#define eusci(p) eusciM(p)
#define eusciM(p) EUSCI_ ## p
#define eusciINT(p) eusciI(p)
#define eusciI(p) EUSCI ## p ## _IRQn

// Define GPIO output mode options

#define GPIO_SHIFT0  0
#define GPIO_SHIFT1  1
#define GPIO_SHIFT2  2
#define GPIO_SHIFT3  3
#define GPIO_SHIFT4  4
#define GPIO_SHIFT5  5
#define GPIO_MAP     8
#define GPIO_BITBAND 9

// Define timer registers

#define STEPPER_TIM 1
#define STEPPER_TIMER timer(STEPPER_TIM)
#define STEPPER_TIMER_INT0 timerINT0(STEPPER_TIM)

#define PULSE_TIM 0
#define PULSE_TIMER timer(PULSE_TIM)
#define PULSE_TIMER_INT0 timerINT0(PULSE_TIM)

#define DEBOUNCE_TIM 2
#define DEBOUNCE_TIMER timer(DEBOUNCE_TIM)
#define DEBOUNCE_TIMER_INT0 timerINT0(DEBOUNCE_TIM)

// Define serial port pins and module

#define SERIAL_MOD A0
#define SERIAL_MODULE eusci(SERIAL_MOD)
#define SERIAL_MODULE_INT eusciINT(SERIAL_MOD)
#define SERIAL_PORT P1
#define SERIAL_RX BIT2
#define SERIAL_TX BIT3
#define SERIAL_RTS_PORT P1
#define SERIAL_RTS_PIN 4
#define SERIAL_RTS_BIT (1<<SERIAL_RTS_PIN)

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.

#define STEP_PN         2
#define STEP_PORT       port(STEP_PN)
#define X_STEP_PIN      0
#define Y_STEP_PIN      1
#define Z_STEP_PIN      2
#define X_STEP_BIT      (1<<X_STEP_PIN)
#define Y_STEP_BIT      (1<<Y_STEP_PIN)
#define Z_STEP_BIT      (1<<Z_STEP_PIN)
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT) // All step bits
//#define STEP_OUTMODE GPIO_SHIFT3
//#define STEP_OUTMODE GPIO_BITBAND
#define STEP_OUTMODE GPIO_MAP

// Define step direction output pins. NOTE: All direction pins must be on the same port.

#define DIRECTION_PN      0
#define DIRECTION_PORT    port(DIRECTION_PN)
#define X_DIRECTION_PIN   5
#define Y_DIRECTION_PIN   11
#define Z_DIRECTION_PIN   20
#define X_DIRECTION_BIT   (1<<X_DIRECTION_PIN)
#define Y_DIRECTION_BIT   (1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_BIT   (1<<Z_DIRECTION_PIN)
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT) // All direction bits
//#define DIRECTION_OUTMODE GPIO_MAP
#define DIRECTION_OUTMODE GPIO_BITBAND

// Define stepper driver enable/disable output pin.

#define STEPPERS_DISABLE_PN		0
#define STEPPERS_DISABLE_PORT  port(STEPPERS_DISABLE_PN)
#define STEPPERS_DISABLE_PIN   7
#define STEPPERS_DISABLE_BIT   (1<<STEPPERS_DISABLE_PIN)
#define STEPPERS_DISABLE_MASK  (STEPPERS_DISABLE_BIT)

// Define homing/hard limit switch input pins
// NOTE: All limit bit pins must be on the same port

#define LIMIT_PN      2
#define LIMIT_PORT    port(LIMIT_PN)
#define LIMIT_GPIO    portGpio(LIMIT_PN)
#define LIMIT_INTENR  portINTER(LIMIT_PN)
#define LIMIT_INTENF  portINTER(LIMIT_PN)
#define LIMIT_INTSTR  portINTSR(LIMIT_PN)
#define LIMIT_INTSTF  portINTSR(LIMIT_PN)
#define LIMIT_INTCLR  portINTCLR(LIMIT_PN)

#define X_LIMIT_PIN 24
#define Y_LIMIT_PIN 26
#define Z_LIMIT_PIN 28
#define X_LIMIT_BIT (1<<X_LIMIT_PIN)
#define Y_LIMIT_BIT (1<<Y_LIMIT_PIN)
#define Z_LIMIT_BIT (1<<Z_LIMIT_PIN)
#define LIMIT_INMODE GPIO_BITBAND
#define LIMIT_MASK  (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT) // All limit bits
//#define LIMIT_SHIFT GPIO_SHIFT4 // Uncomment and set shift value if pins are consecutive and ordered

// Define flood and mist coolant output pins.

#define COOLANT_FLOOD_PN    2
#define COOLANT_FLOOD_PORT	port(COOLANT_FLOOD_PN)
#define COOLANT_FLOOD_GPIO  portGpio(COOLANT_FLOOD_PN)
#define COOLANT_FLOOD_INT   portINT(COOLANT_FLOOD_PN)
#define COOLANT_FLOOD_PIN   4
#define COOLANT_FLOOD_BIT   (1<<COOLANT_FLOOD_PIN)

#define COOLANT_MIST_PN     2
#define COOLANT_MIST_PORT   port(COOLANT_MIST_PN)
#define COOLANT_MIST_GPIO	portGpio(COOLANT_MIST_PN)
#define COOLANT_MIST_INT    portINT(COOLANT_MIST_PN)
#define COOLANT_MIST_PIN    6
#define COOLANT_MIST_BIT    (1<<COOLANT_MIST_PIN)

// Define user-control controls (cycle start, reset, feed hold) input pins.
// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).

#define CONTROL_PN       0
#define CONTROL_PORT     port(CONTROL_PN)
#define CONTROL_GPIO     portGpio(CONTROL_PN)
#define CONTROL_INTENR   portINTER(CONTROL_PN)
#define CONTROL_INTENF   portINTER(CONTROL_PN)
#define CONTROL_INTSTR   portINTSR(CONTROL_PN)
#define CONTROL_INTSTF   portINTSR(CONTROL_PN)
#define CONTROL_INTCLR   portINTCLR(CONTROL_PN)
#define CONTROL_RDI      portRDI(CONTROL_PN)

#define RESET_PIN           22
#define FEED_HOLD_PIN       23
#define CYCLE_START_PIN     31
#define SAFETY_DOOR_PIN     30
#define RESET_BIT           (1<<RESET_PIN)
#define FEED_HOLD_BIT       (1<<FEED_HOLD_PIN)
#define CYCLE_START_BIT     (1<<CYCLE_START_PIN)
#define SAFETY_DOOR_BIT     (1<<SAFETY_DOOR_PIN)
#define CONTROL_MASK        (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT|SAFETY_DOOR_BIT)
//#define CONTROL_SHIFT       GPIO_SHIFT0 // Uncomment and set shift value if pins are consecutive and ordered
#define CONTROL_INMODE GPIO_BITBAND

// Define probe switch input pin.
#define PROBE_PN    4
#define PROBE_PORT  port(PROBE_PN)
#define PROBE_PIN   6
#define PROBE_BIT   (1<<PROBE_PIN)

// Define spindle enable, spindle direction and PWM output pins.

#define SPINDLE_ENABLE_PN     4
#define SPINDLE_ENABLE_PORT   port(SPINDLE_ENABLE_PN)
#define SPINDLE_ENABLE_PIN    0
#define SPINDLE_ENABLE_BIT    (1<<SPINDLE_ENABLE_PIN)

#define SPINDLE_DIRECTION_PN  4
#define SPINDLE_DIRECTION_PORT  port(SPINDLE_DIRECTION_PN)
#define SPINDLE_DIRECTION_PIN   5
#define SPINDLE_DIRECTION_BIT   (1<<SPINDLE_DIRECTION_PIN)

#ifdef SPINDLE_PWM_PIN_2_4
#define SPINDLE_PWM_CHANNEL         PWM1_CH5    // MOSFET3 (P2.4)
#else
#define SPINDLE_PWM_CHANNEL         PWM1_CH6    // BED MOSFET (P2.5)
#endif
#define SPINDLE_PWM_USE_PRIMARY_PIN   false
#define SPINDLE_PWM_USE_SECONDARY_PIN true

// Driver initialization entry point

bool driver_init (void);

#endif // __DRIVER_H__
