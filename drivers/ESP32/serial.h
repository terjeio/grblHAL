/*
  serial.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Serial comms defaults and struct

  Part of GrblHAL

  Copyright (c) 2018-2019 Terje Io

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

#ifndef _serialdefs_h_
#define _serialdefs_h_

#include <stdint.h>
#include <stdbool.h>


#define RX_BUFFER_HWM 900
#define RX_BUFFER_LWM 300

#endif
