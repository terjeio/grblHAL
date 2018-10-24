/*

  ioexpand.h - driver code for Espressif ESP32 processor

  I2C I/O expander

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

#ifndef _IOEXPAND_H_
#define _IOEXPAND_H_

#include <stdint.h>

typedef union {
	uint8_t mask;
	struct {
		uint8_t spindle_on       :1,
				spindle_dir      :1,
				mist_on          :1,
				flood_on         :1,
				stepper_enable_z :1,
				stepper_enable_x :1,
				stepper_enable_y :1,
				reserved		 :1;
	};
} ioexpand_t;

void ioexpand_init (void);
void ioexpand_out (ioexpand_t pins);
ioexpand_t ioexpand_in (void);

#endif
