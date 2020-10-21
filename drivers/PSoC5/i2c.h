/*
  i2c.h - An embedded CNC Controller with rs274/ngc (g-code) support

  I2C driver for Cypress PSoC 5 (CY8CKIT-059)

  Part of GrblHAL

  Copyright (c) 2020 Terje Io

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

#include "driver.h"
#include "keypad/keypad.h"
#include "grbl/plugins.h"

void I2C_Init (void);
void I2C_GetKeycode (uint32_t i2cAddr, keycode_callback_ptr callback);
void I2C_ISR_ExitCallback(void);
