/*
  st_morpho.c - driver code for STM32F4xx ARM processors

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

#include "driver.h"

#if defined(BOARD_MORPHO_CNC)

#include <math.h>
#include <string.h>

#include "main.h"
#include "grbl/protocol.h"
#include "grbl/settings.h"

static driver_setting_ptrs_t driver_settings;

typedef struct {
    GPIO_TypeDef *gpio;
    uint32_t pin;
} aux_port_t;

static const aux_port_t aux_in[] = {
    { .gpio = AUXINPUT0_PORT, .pin = AUXINPUT0_PIN },
    { .gpio = AUXINPUT1_PORT, .pin = AUXINPUT1_PIN }
};

static const aux_port_t aux_out[] = {
    { .gpio = AUXOUTPUT0_PORT, .pin = AUXOUTPUT0_PIN },
    { .gpio = AUXOUTPUT1_PORT, .pin = AUXOUTPUT1_PIN }
};

static const setting_group_detail_t aux_groups[] = {
    { Group_Root, Group_AuxPorts, "Aux ports"}
};

static const setting_detail_t aux_settings[] = {
    { Settings_IoPort_InvertIn, Group_AuxPorts, "Invert I/O Port inputs", NULL, Format_Bitfield, "Port 0,Port 1", NULL, NULL },
//    { Settings_IoPort_Pullup_Disable, Group_AuxPorts, "I/O Port inputs pullup disable", NULL, Format_Bitfield, "Port 0,Port 1,Port 2,Port 3,Port 4,Port 5,Port 6,Port 7", NULL, NULL },
    { Settings_IoPort_InvertOut, Group_AuxPorts, "Invert I/O Port outputs", NULL, Format_Bitfield, "Port 0,Port 1", NULL, NULL },
//    { Settings_IoPort_OD_Enable, Group_AuxPorts, "I/O Port outputs as open drain", NULL, Format_Bitfield, "Port 0,Port 1,Port 2,Port 3,Port 4,Port 5,Port 6,Port 7", NULL, NULL }
};

static setting_details_t details = {
    .groups = aux_groups,
    .n_groups = sizeof(aux_groups) / sizeof(setting_group_detail_t),
    .settings = aux_settings,
    .n_settings = sizeof(aux_settings) / sizeof(setting_detail_t)
};

static setting_details_t *onReportSettings (void)
{
    return &details;
}
/*
static void aux_set_pullup (void)
{
    GPIO_InitTypeDef GPIO_Init = {0};

    GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_Init.Mode = GPIO_MODE_INPUT;

    GPIO_Init.Pin = AUXINPUT0_PIN;
    GPIO_Init.Pull = settings.ioport.pullup_disable_in.bit0 ? GPIO_PULLDOWN : GPIO_PULLUP;
    HAL_GPIO_Init(AUXINPUT0_PORT, &GPIO_Init);

    GPIO_Init.Pin = AUXINPUT1_PIN;
    GPIO_Init.Pull = settings.ioport.pullup_disable_in.bit1 ? GPIO_PULLDOWN : GPIO_PULLUP;
    HAL_GPIO_Init(AUXINPUT1_PORT, &GPIO_Init);
}
*/
static status_code_t aux_settings_set (setting_type_t setting, float value, char *svalue)
{
    status_code_t status = Status_OK;

    switch(setting) {

        case Settings_IoPort_InvertIn:
            settings.ioport.invert_in.mask = (uint8_t)value & AUX_IN_MASK;
            break;
/*
        case Settings_IoPort_Pullup_Disable:
            settings.ioport.pullup_disable_in.mask = (uint8_t)value & AUX_IN_MASK;
            aux_set_pullup();
            break;
*/
        case Settings_IoPort_InvertOut:
            {
                ioport_bus_t invert;
                invert.mask = (uint8_t)value & AUX_OUT_MASK;
                if(invert.mask != settings.ioport.invert_out.mask) {
                    uint_fast8_t idx = AUX_N_OUT;
                    do {
                        idx--;
                        if(((settings.ioport.invert_out.mask >> idx) & 0x01) != ((invert.mask >> idx) & 0x01))
                            BITBAND_PERI(aux_out[idx].gpio->ODR, aux_out[idx].pin) = !BITBAND_PERI(aux_out[idx].gpio->IDR, aux_out[idx].pin);
                    } while(idx);

                    settings.ioport.invert_out.mask = invert.mask;
                }
            }
            break;
/*
        case Settings_IoPort_OD_Enable:
            settings.ioport.od_enable_out.mask = (uint8_t)(int_value & AUX_IN_MASK);
            break;
*/
        default:
            status = Status_Unhandled;
            break;
    }


    if(status == Status_OK)
        settings_write_global();

    return status == Status_Unhandled && driver_settings.set ? driver_settings.set(setting, value, svalue) : status;
}

static void aux_settings_report (setting_type_t setting)
{
    bool reported = true;

    switch(setting) {

        case Settings_IoPort_InvertIn:
            if(hal.port.num_digital_in)
                report_uint_setting(Settings_IoPort_InvertIn, settings.ioport.invert_in.mask);
            break;
/*
        case Settings_IoPort_Pullup_Disable:
            if(hal.port.num_digital_in)
                report_uint_setting(Settings_IoPort_Pullup_Disable, settings.ioport.pullup_disable_in.mask);
            break;
*/
        case Settings_IoPort_InvertOut:
            if(hal.port.num_digital_out)
                report_uint_setting(Settings_IoPort_InvertOut, settings.ioport.invert_out.mask);
            break;
/*
        case Settings_IoPort_OD_Enable:
            if(hal.port.num_digital_out)
                report_uint_setting(Settings_IoPort_OD_Enable, settings.ioport.od_enable_out.mask);
            break;
*/
        default:
            reported = false;
            break;
    }

    if(!reported && driver_settings.report)
        driver_settings.report(setting);
}

static void aux_settings_load (void)
{
//    aux_set_pullup();

    uint_fast8_t idx = hal.port.num_digital_out;
    do {
        idx--;
        BITBAND_PERI(aux_out[idx].gpio->ODR, aux_out[idx].pin) = (settings.ioport.invert_out.mask >> idx) & 0x01;
    } while(idx);

    if(driver_settings.load)
        driver_settings.load();
}

static void digital_out (uint8_t port, bool on)
{
    if(port < AUX_N_OUT)
        BITBAND_PERI(aux_out[port].gpio->ODR, aux_out[port].pin) = ((settings.ioport.invert_out.mask >> port) & 0x01) ? !on : on;
}

inline static __attribute__((always_inline)) int32_t get_input (const aux_port_t *port, bool invert, wait_mode_t wait_mode, float timeout)
{
    if(wait_mode == WaitMode_Immediate)
        return BITBAND_PERI(port->gpio->IDR, port->pin) ^ invert;

    uint_fast16_t delay = (uint_fast16_t)ceilf((1000.0f / 50.0f) * timeout) + 1;

    bool wait_for = wait_mode != WaitMode_Low;

    do {
        if((BITBAND_PERI(port->gpio->IDR, port->pin) ^ invert) == wait_for)
            return BITBAND_PERI(port->gpio->IDR, port->pin) ^ invert;

        if(delay) {
            protocol_execute_realtime();
            hal.delay_ms(50, NULL);
        } else
            break;
    } while(--delay && !sys.abort);

    return -1;
}

static int32_t wait_on_input (bool digital, uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;

    if(digital && port < AUX_N_IN)
        value = get_input(&aux_in[port], (settings.ioport.invert_in.mask << port) & 0x01, wait_mode, timeout);

    return value;
}

void board_init (void)
{
    hal.port.wait_on_input = wait_on_input;
    hal.port.digital_out = digital_out;
    hal.port.num_digital_in = AUX_N_IN;
    hal.port.num_digital_out = AUX_N_OUT;

    memcpy(&driver_settings, &hal.driver_settings, sizeof(driver_setting_ptrs_t));

    hal.driver_settings.set = aux_settings_set;
    hal.driver_settings.report = aux_settings_report;
    hal.driver_settings.load = aux_settings_load;

    details.on_report_settings = grbl.on_report_settings;
    grbl.on_report_settings = onReportSettings;

    GPIO_InitTypeDef GPIO_Init = {0};

    GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;

    GPIO_Init.Pin = AUXOUTPUT0_BIT;
    HAL_GPIO_Init(AUXOUTPUT0_PORT, &GPIO_Init);

    GPIO_Init.Pin = AUXOUTPUT1_BIT;
    HAL_GPIO_Init(AUXOUTPUT1_PORT, &GPIO_Init);
}

#endif
