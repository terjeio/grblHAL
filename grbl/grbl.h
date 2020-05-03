/*
  grbl.h - main Grbl include file
  Part of Grbl

  Copyright (c) 2017-2020 Terje Io
  Copyright (c) 2015-2016 Sungeun K. Jeon for Gnea Research LLC

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

#ifndef grbl_h
#define grbl_h

// Grbl versioning system
#define GRBL_VERSION "1.1f"
#define GRBL_VERSION_BUILD "20200503"

// Define standard libraries used by Grbl.
#include <math.h>
#include <inttypes.h>
#include <string.h>
#include <stddef.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>

#include <assert.h>

// Define the Grbl system include files. NOTE: Do not alter organization.
#include "config.h"
#include "nuts_bolts.h"
#include "settings.h"
#include "hal.h"
#include "system.h"
#include "defaults.h"
#include "coolant_control.h"
#include "eeprom.h"
#include "eeprom_emulate.h"
#include "gcode.h"
#include "limits.h"
#include "planner.h"
#include "motion_control.h"
#include "protocol.h"
#include "state_machine.h"
#include "report.h"
#include "spindle_control.h"
#include "stepper.h"
#include "system.h"
#include "override.h"
#include "sleep.h"
#include "stream.h"
#ifdef KINEMATICS_API
#include "kinematics.h"
#endif

// ---------------------------------------------------------------------------------------
// COMPILE-TIME ERROR CHECKING OF DEFINE VALUES:

#if DEFAULT_PARKING_ENABLE > 0
  #if DEFAULT_HOMING_FORCE_SET_ORIGIN > 0
    #error "HOMING_FORCE_SET_ORIGIN is not supported with PARKING_ENABLE at this time."
  #endif
#endif

#if DEFAULT_ENABLE_PARKING_OVERRIDE_CONTROL > 0
  #if DEFAULT_PARKING_ENABLE < 1
    #error "ENABLE_PARKING_OVERRIDE_CONTROL must be enabled with PARKING_ENABLE."
  #endif
#endif

#if (REPORT_WCO_REFRESH_BUSY_COUNT < REPORT_WCO_REFRESH_IDLE_COUNT)
  #error "WCO busy refresh is less than idle refresh."
#endif
#if (REPORT_OVERRIDE_REFRESH_BUSY_COUNT < REPORT_OVERRIDE_REFRESH_IDLE_COUNT)
  #error "Override busy refresh is less than idle refresh."
#endif
#if (REPORT_WCO_REFRESH_IDLE_COUNT < 2)
  #error "WCO refresh must be greater than one."
#endif
#if (REPORT_OVERRIDE_REFRESH_IDLE_COUNT < 1)
  #error "Override refresh must be greater than zero."
#endif

// ---------------------------------------------------------------------------------------

#endif
