/*

  ioports.c - user defined IO ports template code

  Part of GrblHAL

  Copyright (c) 2019-2020 Terje Io

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

#include <stdint.h>
#include <stdbool.h>

#ifdef ARDUINO
#include "../grbl/grbl.h"
#else
#include "grbl/grbl.h"
#endif

static driver_reset_ptr driver_reset = NULL;

static void digital_out (uint8_t port, bool on)
{
    // Add digital port io handling code here
    // NOTE: Port number starts from zero
}


static bool analog_out (uint8_t port, float value)
{
    // Add analog port io handling code here
    // NOTE: Port number starts from zero

    return true;
}

// Wait on input handler
// wait_mode_t (enum) is defined in grbl/gcode.h
static int32_t wait_on_input (bool digital, uint8_t port, wait_mode_t wait_mode, float timeout)
{
    // Add wait on input handling code here

    return -1;
}

// Optional function to be called on soft reset (ctrl-X)
static void reset (void)
{
    // Add code to handle soft reset here

    driver_reset(); // If other plugins needs to be told about the reset call the next function in the chain here.
}

// Initialize additional IO outputs and set up HAL pointers.
// Call this function on driver init.
// NOTE: port numbers start at 0 and must be sequential.
//       E.g if adding 3 digital outputs they are assigned (logical) port numbers 0 - 2.
//       Mapping to physical port/pin must thus be done in code (possibly via a lookup table).
// NOTE: all HAL fields are pre-initialized to 0 (or NULL).
void ioports_init (void)
{
    // Add port initialization here

    // Tell HAL about our ports
    hal.port.digital_out = digital_out;     // Leave unset (NULL) if no digital ports handled.
    hal.port.analog_out = analog_out;       // Leave unset (NULL) if no analog ports handled.
    hal.port.wait_on_input = wait_on_input; // Leave unset (NULL) if no wait on input is required.
    hal.port.num_digital_out = 1;           // Set to number of digital outputs handled.
    hal.port.num_analog_out = 1;            // Set to number of analog outputs handled.
    hal.port.num_digital_in = 1;            // Set to number of digital inputs handled.
    hal.port.num_analog_in = 1;             // Set to number of analog inputs handled.

    // If any action on soft reset is required uncomment the following lines:
//    if(driver_reset == NULL) {
//        driver_reset = hal.driver_reset;        // Save pointer to previous reset handler and
//        hal.driver_reset = reset;               // add our to top of the chain.
//    }
}
