/*
  serial.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Serial driver for Cypress PSoC 5 (CY8CKIT-059)

  Part of GrblHAL

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

#define RX_BUFFER_HWM 900
#define RX_BUFFER_LWM 300

void serialInit(void);
int16_t serialGetC(void);
bool serialPutC(const char data);
void serialWriteS(const char *data);
void serialWriteLn(const char *data);
void serialWrite(const char *data, unsigned int length);

uint16_t serialTxCount(void);
uint16_t serialRxCount(void);
uint16_t serialRxFree(void);
void serialRxFlush(void);
void serialRxCancel(void);
bool serialSuspendInput (bool suspend);
