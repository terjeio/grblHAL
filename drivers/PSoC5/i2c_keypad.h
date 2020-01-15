/*
  I2CKeypad.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Cypress PSoC 5 (CY8CKIT-059)

  Part of GrblHAL

  Copyright (c) 2017-2018 Terje Io

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

#ifndef _KEYPAD_H_
#define _KEYPAD_H_

#include <stdbool.h>
#include <stdint.h>

#include "grbl.h"

#define KEYBUF_SIZE 16
#define KEYPAD_I2CADDR 0x49

typedef enum {
    JogMode_Fast = 0,
    JogMode_Slow,
    JogMode_Step
} jogmode_t;

void I2C_keypad_setup (void);
void process_keypress (uint_fast16_t state);
void I2C_ISR_ExitCallback(void);

status_code_t driver_setting (setting_type_t setting, float value, char *svalue);
void driver_settings_restore (void);
void driver_settings_report (setting_type_t setting);

#endif
