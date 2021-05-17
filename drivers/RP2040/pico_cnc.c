/*
  pico_cnc.c - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2021 Terje Io
  
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

#include "driver.h"

#if defined(BOARD_PICO_CNC)

#include "hardware/pio.h"

#include "driverPIO.pio.h"
#include "grbl/protocol.h"

static output_sr_t *sr;
static bool state[AUX_N_OUT];

static void aux_settings_load (void);
static status_code_t aux_set_invert_out (setting_id_t id, uint_fast16_t int_value);
static uint32_t aux_get_invert_out (setting_id_t setting);
static void digital_out (uint8_t port, bool on);

static const setting_group_detail_t aux_groups[] = {
    { Group_Root, Group_AuxPorts, "Aux ports"}
};

static const setting_detail_t aux_settings[] = {
    { Settings_IoPort_InvertOut, Group_AuxPorts, "Invert I/O Port outputs", NULL, Format_Bitfield, "Port 0,Port 1,Port 2,Port 3,Port 4,Port 5,Port 6,Port 7", NULL, NULL, Setting_NonCoreFn, aux_set_invert_out, aux_get_invert_out },
};

static setting_details_t details = {
    .groups = aux_groups,
    .n_groups = sizeof(aux_groups) / sizeof(setting_group_detail_t),
    .settings = aux_settings,
    .n_settings = sizeof(aux_settings) / sizeof(setting_detail_t),
    .load = aux_settings_load,
    .save = settings_write_global
};

static setting_details_t *on_get_settings (void)
{
    return &details;
}

static void aux_settings_load (void)
{
    uint_fast8_t idx = AUX_N_OUT;

    do {
        idx--;
		digital_out(idx, false);
    } while(idx);
}

static status_code_t aux_set_invert_out (setting_id_t id, uint_fast16_t value)
{
    ioport_bus_t invert;
    invert.mask = (uint8_t)value & AUX_OUT_MASK;

    if(invert.mask != settings.ioport.invert_out.mask) {
        uint_fast8_t idx = AUX_N_OUT;
        do {
            idx--;
            if(((settings.ioport.invert_out.mask >> idx) & 0x01) != ((invert.mask >> idx) & 0x01))
                digital_out(idx, !state[idx]);
        } while(idx);

        settings.ioport.invert_out.mask = invert.mask;
    }

    return Status_OK;
}

static uint32_t aux_get_invert_out (setting_id_t setting)
{
    return settings.ioport.invert_out.mask;
}

static void digital_out (uint8_t port, bool on)
{
    if(port < AUX_N_OUT) {

        on = ((settings.ioport.invert_out.mask >> port) & 0x01) ? !on : on;
        
        state[port] = on;

        switch(port)
        {
            case 0:
                sr->aux0_out = on;
                break;
            
            case 1:
                sr->aux1_out = on;
                break;	

            case 2:
                sr->aux2_out = on;
                break;	

            case 3:
                sr->aux3_out = on;
                break;	

            case 4:
                sr->aux4_out = on;
                break;	

            case 5:
                sr->aux5_out = on;
                break;	

            case 6:
                sr->aux6_out = on;
                break;

            case 7:
                sr->aux7_out = on;
                break;
                
            default:
                break;
        }

        out_sr16_write(pio1, 1, sr->value);
    }
}

void board_init (output_sr_t *reg)
{
    sr = reg;

    hal.port.digital_out = digital_out;
    hal.port.num_digital_out = AUX_N_OUT;

    details.on_get_settings = grbl.on_get_settings;
    grbl.on_get_settings = on_get_settings;
}

#endif
