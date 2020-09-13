/*
  report.h - reporting and messaging methods

  Part of GrblHAL

  Copyright (c) 2018-2019 Terje Io
  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC

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

#ifndef _REPORT_H_
#define _REPORT_H_

#include "system.h"

// Initialize reporting subsystem
void report_init (void);
void report_init_fns (void);

// Prints system status messages.
status_code_t report_status_message (status_code_t status_code);

// Prints system alarm messages.
alarm_code_t report_alarm_message (alarm_code_t alarm_code);

// Prints miscellaneous feedback messages.
message_code_t report_feedback_message (message_code_t message_code);

// Prints welcome message.
void report_init_message (void);

// Prints Grbl help.
void report_grbl_help();

// Prints Grbl setting(s)
void report_grbl_settings (bool all);
void report_uint_setting (setting_type_t n, uint32_t val);
void report_float_setting (setting_type_t n, float val, uint8_t n_decimal);
void report_string_setting (setting_type_t n, char *val);

// Prints an echo of the pre-parsed line received right before execution.
void report_echo_line_received (char *line);

// Prints realtime status report.
void report_realtime_status (void);

// Prints recorded probe position.
void report_probe_parameters (void);

// Prints current tool offsets.
void report_tool_offsets (void);

// Prints Grbl NGC parameters (coordinate offsets, probe).
void report_ngc_parameters (void);

// Prints current g-code parser mode state.
void report_gcode_modes (void);

// Prints startup line when requested and executed.
void report_startup_line (uint8_t n, char *line);
void report_execute_startup_message (char *line, status_code_t status_code);

// Prints build info and user info.
void report_build_info (char *line);

// Prints current PID log.
void report_pid_log (void);

#endif
