/*

  ioexpand.h -  driver code for Atmel SAMD21 ARM processor

  I2C I/O expander

  Part of grblHAL

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

#ifndef _IOEXPAND_H_
#define _IOEXPAND_H_

#include "driver.h"

void ioexpand_init (void);
void ioexpand_out (ioexpand_t pins);
ioexpand_t ioexpand_in (void);

#endif
