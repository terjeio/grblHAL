/*
  driver.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Texas Instruments MSP430F5529 16-bit processor

  Part of grblHAL

  Copyright (c) 2016-2021 Terje Io

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

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#include "portmacros.h"

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#ifndef EEPROM_ENABLE
#define EEPROM_ENABLE   0
#endif
#ifndef EEPROM_IS_FRAM
#define EEPROM_IS_FRAM  0
#endif

#ifdef BOARD_CNC_BOOSTERPACK
#include "cnc_boosterpack_map.h"
#elif defined(BOARD_MY_MACHINE)
#include "my_machine_map.h"
#else
#error No board!
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 2.2f // microseconds
#endif

