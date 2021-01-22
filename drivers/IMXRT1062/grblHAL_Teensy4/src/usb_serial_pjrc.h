/*

  usb_serial.h - driver code for IMXRT1062 processor (on Teensy 4.0 board)

  Part of grblHAL

  Copyright (c) 2018-2020 Terje Io

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

#ifndef _USB_SERIAL_H_
#define _USB_SERIAL_H_

#include <stdbool.h>
#include <stdint.h>

extern void usb_execute_realtime (uint_fast16_t state);

#define usb_serial_poll() usb_execute_realtime(0)

void usb_serialInit(void);
int16_t usb_serialGetC(void);
bool usb_serialPutC(const char c);
void usb_serialWriteS(const char *s);
void usb_serialWriteLn(const char *s);
void usb_serialWrite(const char *s, uint16_t length);
bool usb_serialSuspendInput (bool suspend);

uint16_t usb_serialTxCount(void);
uint16_t usb_serialRxCount(void);
uint16_t usb_serialRxFree(void);
void usb_serialRxFlush(void);
void usb_serialRxCancel(void);

#endif
