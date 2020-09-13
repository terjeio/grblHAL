/*

  odometer.c - axis odometers including run time

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

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#if ODOMETER_ENABLE

#include <string.h>
#include <stdio.h>

#ifdef ARDUINO
#include "../grbl/nvs_buffer.h"
#else
#include "grbl/nvs_buffer.h"
#endif

#include "odometer.h"

static nvs_io_t *nvs = NULL;
static bool odometer_changed = false;
static uint32_t odometers_address;
static odometer_data_t odometers;
static void (*stepper_pulse_start)(stepper_t *stepper);
static status_code_t (*on_unknown_sys_command)(uint_fast16_t state, char *line, char *lcline);
void (*on_state_change)(uint_fast16_t state);

static void stepperPulseStart (stepper_t *stepper)
{
    odometer_changed = true;

    if(stepper->step_outbits.x)
        odometers.distance[X_AXIS]++;

    if(stepper->step_outbits.y)
        odometers.distance[Y_AXIS]++;

    if(stepper->step_outbits.z)
        odometers.distance[Z_AXIS]++;

#ifdef A_AXIS
    if(stepper->step_outbits.a)
        odometers.distance[A_AXIS]++;
#endif

#ifdef B_AXIS
    if(stepper->step_outbits.b)
        odometers.distance[B_AXIS]++;
#endif

#ifdef C_AXIS
    if(stepper->step_outbits.b)
        odometers.distance[C_AXIS]++;
#endif

    stepper_pulse_start(stepper);
}

void onStateChanged (uint_fast16_t state)
{
    static uint32_t ms = 0;

    if(state & (STATE_CYCLE|STATE_JOG|STATE_HOMING))
        ms = hal.get_elapsed_ticks();

    else if(odometer_changed) {
        odometer_changed = false;
        odometers.time += (hal.get_elapsed_ticks() - ms);
        nvs->memcpy_to_with_checksum(odometers_address, (uint8_t *)&odometers, sizeof(odometer_data_t));
    }

    if(on_state_change)
        on_state_change(state);
}

static void odometer_data_reset (void)
{
    // TODO: Write backup to second copy before reset
    memset(&odometers, 0, sizeof(odometer_data_t));
    nvs->memcpy_to_with_checksum(odometers_address, (uint8_t *)&odometers, sizeof(odometer_data_t));
}

static status_code_t commandExecute (uint_fast16_t state, char *line, char *lcline)
{
    status_code_t retval = Status_Unhandled;

    if(line[1] == 'O') {

        if(!strcmp(&line[1], "ODOMETERS")) {

            char buf[40];
            uint_fast8_t idx;
            uint32_t hr = odometers.time / 3600000, min = (odometers.time / 60000) % 60;

            sprintf(buf, "[MSG: ODOMETERHRS %ld:%.2ld]" ASCII_EOL, hr, min);
            hal.stream.write(buf);

            for(idx = 0 ; idx < N_AXIS ; idx++) {
                sprintf(buf, "[MSG: ODOMETER%s %s]" ASCII_EOL, axis_letter[idx], ftoa(odometers.distance[idx] / settings.axis[idx].steps_per_mm / 1000.0f, 1)); // meters
                hal.stream.write(buf);
            }

            retval = Status_OK;
        }

        if(!strcmp(&line[1], "ODOMETERS=RST")) {
            odometer_data_reset();
            retval = Status_OK;
        }
    }

    return retval == Status_Unhandled && on_unknown_sys_command ? on_unknown_sys_command(state, line, lcline) : retval;
}

void odometer_init()
{
    static bool init_ok = false;

    if(!nvs)
        nvs = nvs_buffer_get_physical();

    if(!init_ok && !(nvs->type == NVS_EEPROM || nvs->type == NVS_FRAM))
        hal.stream.write("[MSG:EEPROM or FRAM is required for odometers]" ASCII_EOL);

    else {

        stepper_pulse_start = hal.stepper_pulse_start;
        hal.stepper_pulse_start = stepperPulseStart;

        if(!init_ok) {

            init_ok = true;
            odometers_address = nvs->driver_area.address + sizeof(driver_settings_t) + 1;

            if(!nvs->memcpy_from_with_checksum((uint8_t *)&odometers, odometers_address, sizeof(odometer_data_t)))
                odometer_data_reset();

            on_unknown_sys_command = grbl.on_unknown_sys_command;
            grbl.on_unknown_sys_command = commandExecute;

            on_state_change = grbl.on_state_change;
            grbl.on_state_change = onStateChanged;
        }
    }
}

#endif
