/*

  thc.h - plasma cutter tool height control plugin

  Part of grblHAL

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

#ifndef _THC_H_
#define _THC_H_

typedef enum {
    Plasma_mode0 = 0,
    Plasma_mode1,
    Plasma_mode2
} plasma_mode_t;

typedef struct {
    float thc_delay;
    float thc_threshold;
    uint32_t vad_threshold;
    uint32_t thc_override;
    float pierce_height;
    float pierce_delay;
    float pause_at_end;
    float arc_retry_delay;
    float arc_fail_timeout;
    float arc_voltage_scale;
    float arc_voltage_offset;
    float arc_height_per_volt;
    float arc_ok_low_voltage;
    float arc_high_low_voltage;
    uint_fast8_t arc_retries;
    plasma_mode_t mode;
    pid_values_t pid;
} plasma_settings_t;

bool plasma_init (void);

#endif
