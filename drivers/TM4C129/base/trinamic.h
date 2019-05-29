/*
  trinamic.h - Trinamic TMC2130 integration

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

#ifndef _TRINAMIC_H_
#define _TRINAMIC_H_

#include "driver.h"
#include "trinamic\trinamic2130.h"

typedef struct {
    uint16_t current; // mA
    uint16_t r_sense; // mOhm
    tmc2130_microsteps_t microsteps;
    uint8_t homing_sensitivity;
} motor_settings_t;

typedef struct {
    axes_signals_t driver_enable;
    axes_signals_t homing_enable;
    motor_settings_t driver[N_AXIS];
} trinamic_settings_t;

// Init wrapper for physical interface
void SPI_DriverInit (SPI_driver_t *driver);

void trinamic_init (void);
void trinamic_configure (void);
void trinamic_homing (bool enable);
bool trinamic_setting (uint_fast16_t setting, float value, char *svalue);
void trinamic_settings_restore (uint8_t restore_flag);
void trinamic_settings_report (bool axis_settings, axis_setting_type_t setting_type, uint8_t axis_idx);

uint_fast16_t trinamic_MCodeCheck (uint_fast16_t mcode);
status_code_t trinamic_MCodeValidate (parser_block_t *gc_block, uint_fast16_t *value_words);
void trinamic_MCodeExecute (uint_fast16_t state, parser_block_t *gc_block);
#endif
