/*

  serial.h - low level functions for transmitting bytes via the serial port

  Part of grblHAL

  Copyright (c) 2017-2020 Terje Io

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

#include "driver.h"
#include "grbl/grbl.h"

#define BACKCHANNEL // comment out to use Launchpad Backchannel UART

#define eusci(p) eusciM(p)
#define eusciM(p) EUSCI_ ## p
#define eusciINT(p) eusciI(p)
#define eusciI(p) EUSCI ## p ## _IRQn
#define eusciHANDLER(p) eusciH(p)
#define eusciH(p) EUSCI ## p ## _IRQHandler

#define XONOK (ASCII_XON|0x80)
#define XOFFOK (ASCII_XON|0x80)
#define RX_BUFFER_HWM 900
#define RX_BUFFER_LWM 300
//#define RTS_PORT P1
#define RTS_PIN  4
#define RTS_BIT (1<<RTS_PIN)
//#define LINE_BUFFER_SIZE 20

// Define serial port pins and module

#define SERIAL_MOD A0
#define SERIAL_MODULE eusci(SERIAL_MOD)
#define SERIAL_MODULE_INT eusciINT(SERIAL_MOD)
#define SERIAL_IRQHandler eusciHANDLER(SERIAL_MOD)
#define SERIAL_PORT P1
#define SERIAL_RX BIT2
#define SERIAL_TX BIT3
#define SERIAL_RTS_PORT P1
#define SERIAL_RTS_PIN 4
#define SERIAL_RTS_BIT (1<<SERIAL_RTS_PIN)

#if MPG_MODE_ENABLE || MODBUS_ENABLE
#define SERIAL2_MOD A2
#define SERIAL2_MODULE eusci(SERIAL2_MOD)
#define SERIAL2_MODULE_INT eusciINT(SERIAL2_MOD)
#define SERIAL2_IRQHandler eusciHANDLER(SERIAL2_MOD)
#define SERIAL2_PORT P3
#define SERIAL2_RX BIT2
#define SERIAL2_TX BIT3
#endif

//

void serialInit(void);
int16_t serialGetC(void);
bool serialPutC(const char c);
void serialWriteS(const char *s);
void serialWriteLn(const char *s);
void serialWrite(const char *s, uint16_t length);
bool serialSuspendInput (bool suspend);

uint16_t serialTxCount(void);
uint16_t serialRxCount(void);
uint16_t serialRxFree(void);
void serialRxFlush(void);
void serialRxCancel(void);

#ifdef SERIAL2_MOD
void serial2Init(uint32_t baud_rate);
bool serial2SetBaudRate (uint32_t baud_rate);
uint16_t serial2RxFree (void);
void serial2RxFlush (void);
void serial2RxCancel (void);
uint16_t serial2RxCount (void);
uint16_t serial2TxCount (void);
void serial2TxFlush (void);
void serial2Write(const char *s, uint16_t length);
int16_t serial2GetC (void);
void serialSelect (bool mpg);
#endif
