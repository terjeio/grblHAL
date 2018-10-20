/*
  bluetooth.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Bluetooth comms

  Part of Grbl

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

#ifndef _grbl_bluetooth_h_
#define _grbl_bluetooth_h_

bool bluetooth_init (char *sn, char *sv);

uint32_t BTStreamAvailable (void);
uint16_t BTStreamRXFree (void);
int16_t BTStreamGetC (void);
bool BTStreamSuspendInput (bool suspend);

bool BTStreamPutC (const char c);
void BTStreamWriteS (const char *data);

void BTStreamFlush (void);
void BTStreamCancel (void);

//void setBTBlockingCallback (bool (*fn)(void));
void setBTReceiveCallback (bool (*fn)(char));

#endif
