/*
  serial.h - low level functions for transmitting bytes via the serial port

  Driver code for simulator MCU

  Part of GrblHAL

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

#ifndef _HAL_SERIAL_H_
#define _HAL_SERIAL_H_

#include "grbl/grbl.h"

void serialInit (void);
int16_t serialGetC (void);
void serialWriteS (const char *data);
bool serialSuspendInput (bool suspend);
uint16_t serialRxFree (void);
void serialRxFlush (void);
void serialRxCancel (void);

#endif
