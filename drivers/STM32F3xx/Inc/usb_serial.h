/*

  usb_serial.h - low level functions for transmitting bytes via the USB virtual serial port

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

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

void usbInit (void);
int16_t usbGetC (void);
bool usbPutC (const char c);
void usbWriteS (const char *s);
uint16_t usbRxFree (void);
void usbRxFlush (void);
void usbRxCancel (void);
void usbBufferInput (uint8_t *data, uint32_t length);
bool usbSuspendInput (bool suspend);
