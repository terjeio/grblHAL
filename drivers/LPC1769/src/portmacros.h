/*

  portmacros.h - driver code for NXP LPC176x ARM processors

  Part of grblHAL

  Copyright (c) 2018-2021 Terje Io

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

#include "chip.h"

#define DIGITAL_IN(gpio, bit) (!!(gpio->PIN & bit))
#define DIGITAL_OUT(gpio, bit, on) { if(on) gpio->SET = bit; else gpio->CLR = bit; }

// Added missing definition for GPIO0
#define LPC_GPIO0 ((LPC_GPIO_T *) LPC_GPIO0_BASE)

#define port(p) portI(p)
#define portI(p) LPC_GPIO ## p
#define portINTER(p) portQR(p)
#define portQR(p) LPC_GPIOINT->IO ## p.ENR
#define portINTEF(p) portQF(p)
#define portQF(p) LPC_GPIOINT->IO ## p.ENF
#define portINTSR(p) portQSR(p)
#define portQSR(p) LPC_GPIOINT->IO ## p.STATR
#define portINTSF(p) portQSF(p)
#define portQSF(p) LPC_GPIOINT->IO ## p.STATF
#define portINTCLR(p) portC(p)
#define portC(p) LPC_GPIOINT->IO ## p.CLR
#define portRDI(p) portR(p * 2)
#define portR(p) PINMODE ## p

#define P0Int (1<<0)
#define P2Int (1<<2)

#define GPIO_IRQHandler EINT3_IRQHandler

#define PINMODE_PULLUP   0x0
#define PINMODE_REPEATER 0x1
#define PINMODE_FLOATING 0x2
#define PINMODE_PULLDOWN 0x3

#define timer(p) timerN(p)
#define timerN(p) LPC_TIMER ## p
#define timerINT0(p) timerI0(p)
#define timerI0(p) TIMER ## p ## _IRQn
#define timerINTN(p) timerIN(p)
#define timerIN(p) T ## p ## _N_IRQn
#define timerISR(p) timerS(p)
#define timerS(p) TIMER ## p ## _IRQHandler
#define timerPCLK(p) timerCK(p)
#define timerCK(p) SYSCTL_PCLK_TIMER ## p

#define MR0I (1<<0)
#define MR0R (1<<1)
#define MR0S (1<<2)
#define MR1I (1<<3)
#define MR1R (1<<4)
#define MR1S (1<<5)

#define MR0IFG (1<<0)
#define MR1IFG (1<<1)

// Define timer registers

#define STEPPER_TIM 1
#define STEPPER_TIMER timer(STEPPER_TIM)
#define STEPPER_TIMER_INT0 timerINT0(STEPPER_TIM)
#define STEPPER_TIMER_PCLK timerPCLK(STEPPER_TIM)
#define STEPPER_IRQHandler timerISR(STEPPER_TIM)

#define PULSE_TIM 0
#define PULSE_TIMER timer(PULSE_TIM)
#define PULSE_TIMER_INT0 timerINT0(PULSE_TIM)
#define PULSE_TIMER_PCLK timerPCLK(PULSE_TIM)
#define STEPPULSE_IRQHandler timerISR(PULSE_TIM)

#define DEBOUNCE_TIM 2
#define DEBOUNCE_TIMER timer(DEBOUNCE_TIM)
#define DEBOUNCE_TIMER_INT0 timerINT0(DEBOUNCE_TIM)
#define DEBOUNCE_TIMER_PCLK timerPCLK(DEBOUNCE_TIM)
#define DEBOUNCE_IRQHandler timerISR(DEBOUNCE_TIM)

// Define serial port pins and module

#define SERIAL_MOD A0
//#define SERIAL_MODULE eusci(SERIAL_MOD)
//#define SERIAL_MODULE_INT eusciINT(SERIAL_MOD)
#define SERIAL_PORT P1
#define SERIAL_RX BIT2
#define SERIAL_TX BIT3
#define SERIAL_RTS_PORT P1
#define SERIAL_RTS_PIN 4
#define SERIAL_RTS_BIT (1<<SERIAL_RTS_PIN)

// EOF
