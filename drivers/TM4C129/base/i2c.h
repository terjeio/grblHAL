/*
  i2c.h - I2C bridge interface for Trinamic TMC2130 stepper drivers

  For Texas Instruments SimpleLink ARM processors/LaunchPads

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

#ifndef __I2C_DRIVER_H__
#define __I2C_DRIVER_H__

#include "trinamic\trinamic2130.h"

#define I2C_ADR_I2CBRIDGE 0x47

void I2C_DriverInit (SPI_driver_t *drv);

#endif
