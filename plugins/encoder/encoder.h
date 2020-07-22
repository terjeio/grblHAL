/*
  encoder.h - quadrature encoder plugin

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

#ifndef _ENCODER_H_
#define _ENCODER_H_

#ifdef ARDUINO
#include "../../driver.h"
#else
#include "driver.h"
#endif

#if QEI_ENABLE

#ifdef ARDUINO
#include "../grbl/plugins.h"
#else
#include "grbl/plugins.h"
#endif

void encoder_init (encoder_t *encoder);
void encoder_event (encoder_t *encoder, int32_t position);
void encoder_execute_realtime (uint_fast16_t state);
void encoder_rt_report (stream_write_ptr stream_write, report_tracking_flags_t report);

status_code_t encoder_setting (setting_type_t setting, float value, char *svalue);
void encoder_settings_restore (void);
void encoder_settings_report (setting_type_t setting_type);

#endif
#endif
