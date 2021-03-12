/*
  serial.h - serial stream driver

  For Texas Instruments SimpleLink ARM processors/LaunchPads

  Part of grblHAL

  Copyright (c) 2017-2019 Terje Io

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

#include <stdint.h>
#include <stdbool.h>

#include "grbl/stream.h"

#define BACKCHANNEL // comment out to use UART1 instead of UART0 (Tiva C Backchannel)
#define SERIAL2_MOD

#define uartSysctl(n) uartS(n)
#define uartS(n) SYSCTL_PERIPH_UART ## n
#define uartBase(n) uartB(n)
#define uartB(n) UART ## n ## _BASE
#define uartINT(n) uartI(n)
#define uartI(n) INT_UART ## n
#define uartIOPeriph(p) uartPh(p)
#define uartPh(p) SYSCTL_PERIPH_GPIO ## p
#define uartIOPort(p) uartPo(p)
#define uartPo(p) GPIO_PORT ## p ## _BASE
#define uartRXpin(n, p, i) uartR(n, p, i)
#define uartR(n, p, i) GPIO_P ## p ## i ## _U ## n ## RX
#define uartTXpin(n, p, i) uartT(n, p, i)
#define uartT(n, p, i) GPIO_P ## p ## i ## _U ## n ## TX
#define uartIOpins(r, t) uartP(r, t)
#define uartP(r, t) (GPIO_PIN_ ## r | GPIO_PIN_ ## t)

#define XONOK (ASCII_XON|0x80)
#define XOFFOK (ASCII_XOFF|0x80)
#define TX_BUFFER_SIZE 512      // must be a power of 2
#define RX_BUFFER_SIZE 1024     // must be a power of 2
#define RX_BUFFER_HWM 900
#define RX_BUFFER_LWM 300
#ifdef BACKCHANNEL
#define SERIAL1 0
#else
#define SERIAL1 1
#endif
#define SERIAL1_SYSCTL uartSysctl(SERIAL1)
#define SERIAL1_BASE uartBase(SERIAL1)
#define SERIAL1_INT uartINT(SERIAL1)

#if SERIAL1 == 0
 #define SERIAL1_IOPORT A
 #define SERIAL1_RX_PIN 0
 #define SERIAL1_TX_PIN 1
#elif SERIAL1 == 1
 #define SERIAL1_IOPORT B
 #define SERIAL1_RX_PIN 0
 #define SERIAL1_TX_PIN 1
#elif SERIAL1 == 2
 #define SERIAL1_IOPORT A
 #define SERIAL1_RX_PIN 6
 #define SERIAL1_TX_PIN 7
#elif SERIAL1 == 3
 #define SERIAL1_IOPORT A
 #define SERIAL1_RX_PIN 4
 #define SERIAL1_TX_PIN 5
#elif SERIAL1 == 4
 #define SERIAL1_IOPORT A
 #define SERIAL1_RX_PIN 2
 #define SERIAL1_TX_PIN 3
#elif SERIAL1 == 5
 #define SERIAL1_IOPORT B
 #define SERIAL1_RX_PIN 0
 #define SERIAL1_TX_PIN 1
#elif SERIAL1 == 6
 #define SERIAL1_IOPORT P
 #define SERIAL1_RX_PIN 0
 #define SERIAL1_TX_PIN 1
#elif SERIAL1 == 7
 #define SERIAL1_IOPORT C
 #define SERIAL1_RX_PIN 4
 #define SERIAL1_TX_PIN 5
#else
#endif

#define SERIAL1_PERIPH uartIOPeriph(SERIAL1_IOPORT)
#define SERIAL1_PORT uartIOPort(SERIAL1_IOPORT)
#define SERIAL1_RX uartRXpin(SERIAL1, SERIAL1_IOPORT, SERIAL1_RX_PIN)
#define SERIAL1_TX uartTXpin(SERIAL1, SERIAL1_IOPORT, SERIAL1_TX_PIN)
#define SERIAL1_PINS uartIOpins(SERIAL1_RX_PIN, SERIAL1_TX_PIN)
/*
#define RTS_PERIPH SYSCTL_PERIPH_GPIOL
#define RTS_PORT GPIO_PORTL_BASE
#define RTS_PIN  GPIO_PIN_3
*/

#define DEBUG_PRINT(string) serialWriteS(string)

void serialInit (void);
int16_t serialGetC (void);
bool serialPutC (const char data);
void serialWriteS (const char *data);
bool serialSuspendInput (bool suspend);

uint16_t serialTxCount(void);
uint16_t serialRxCount(void);
uint16_t serialRxFree(void);
void serialRxFlush(void);
void serialRxCancel(void);

#ifdef SERIAL2_MOD

#define SERIAL2 7
#define SERIAL2_SYSCTL uartSysctl(SERIAL2)
#define SERIAL2_BASE uartBase(SERIAL2)
#define SERIAL2_INT uartINT(SERIAL2)

#if SERIAL2 == 0
 #define SERIAL2_IOPORT A
 #define SERIAL2_RX_PIN 0
 #define SERIAL2_TX_PIN 1
#elif SERIAL2 == 1
 #define SERIAL2_IOPORT B
 #define SERIAL2_RX_PIN 0
 #define SERIAL2_TX_PIN 1
#elif SERIAL2 == 7
 #define SERIAL2_IOPORT C
 #define SERIAL2_RX_PIN 4
 #define SERIAL2_TX_PIN 5
#else
#endif

#define SERIAL2_PERIPH uartIOPeriph(SERIAL2_IOPORT)
#define SERIAL2_PORT uartIOPort(SERIAL2_IOPORT)
#define SERIAL2_RX uartRXpin(SERIAL2, SERIAL2_IOPORT, SERIAL2_RX_PIN)
#define SERIAL2_TX uartTXpin(SERIAL2, SERIAL2_IOPORT, SERIAL2_TX_PIN)
#define SERIAL2_PINS uartIOpins(SERIAL2_RX_PIN, SERIAL2_TX_PIN)

uint16_t serial2RxFree (void);
void serial2RxFlush (void);
void serial2RxCancel (void);
int16_t serial2GetC (void);
void serialSelect (bool mpg);
#endif
