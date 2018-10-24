/*

  ioexpand.c - driver code for Espressif ESP32 processor

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

#include "ioexpand.h"
#include "driver.h"

void ioexpand_init (void)
{
	i2c_init();
}

void ioexpand_out (ioexpand_t pins)
{
//mutex!! I2C is used by keypad and eeprom too
	/*
   i2c_cmd_handle_t cmd = i2c_cmd_link_create();
   i2c_master_start(cmd);
   i2c_master_write_byte(cmd, (0x20 << 1) | 0x01, I2C_MASTER_ACK);
   */
}

ioexpand_t ioexpand_in (void)
{
	ioexpand_t pins = {0};

	return pins;
}
