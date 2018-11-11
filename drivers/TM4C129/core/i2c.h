/*
  i2c.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Texas Instruments Tiva C (TM4C1294MCPDT) ARM processor/LaunchPad
   for interfacing Trinamic TMC2130 stepper drivers via I2C bridge

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
TMC2130_status_t I2C_ReadRegister (TMC2130_t *driver, TMC2130_datagram_t *reg);
TMC2130_status_t I2C_WriteRegister (TMC2130_t *driver, TMC2130_datagram_t *reg);

#endif
