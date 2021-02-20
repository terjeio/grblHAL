/*
  i2c.h - I2C interface

  Driver code for Atmel SAMD21 ARM processor

  Part of grblHAL

  Copyright (c) 2018-2021 Terje Io

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

#include "driver.h"
#include "grbl/plugins.h"

#if KEYPAD_ENABLE

#include "keypad/keypad.h"

void I2C_GetKeycode (uint32_t i2cAddr, keycode_callback_ptr callback);

#endif

void i2c_init (void);
uint8_t *I2C_Receive (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes, bool block);
void I2C_Send (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes, bool block);
uint8_t *I2C_ReadRegister (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes, bool block);

#endif
