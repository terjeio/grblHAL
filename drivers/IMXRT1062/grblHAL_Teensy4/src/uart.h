/*

  uart.h - driver code for IMXRT1062 processor (on Teensy 4.0 board)

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

#ifndef _HAL_SERIAL_H_
#define _HAL_SERIAL_H_

#include <stdbool.h>
#include <stdint.h>

void serialInit (uint32_t baud_rate);
bool serialSetBaudRate (uint32_t baud_rate);
int16_t serialGetC (void);
bool serialPutC (const char c);
void serialWrite(const char *s, uint16_t length);
void serialWriteS (const char *data);
bool serialSuspendInput (bool suspend);
uint16_t serialRxFree (void);
uint16_t serialRxCount (void);
uint16_t serialTxCount (void);
void serialRxFlush (void);
void serialTxFlush (void);
void serialRxCancel (void);

#endif
