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

#include <string.h>

#include "main.h"
#include "grbl/protocol.h"
#include "grbl/settings.h"

static driver_setting_ptrs_t driver_settings;

static const setting_group_detail_t aux_groups [] = {
    { Group_Root, Group_AuxPorts, "Aux ports"}
};

static const setting_detail_t aux_settings[] = {
    { Settings_IoPort_InvertIn, Group_AuxPorts, "Invert I/O Port inputs", NULL, Format_Bitfield, "Port 0,Port 1", NULL, NULL },
//    { Settings_IoPort_Pullup_Disable, Group_AuxPorts, "I/O Port inputs pullup disable", NULL, Format_Bitfield, "Port 0,Port 1,Port 2,Port 3,Port 4,Port 5,Port 6,Port 7", NULL, NULL },
    { Settings_IoPort_InvertOut, Group_AuxPorts, "Invert I/O Port outputs", NULL, Format_Bitfield, "Port 0", NULL, NULL },
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

static status_code_t aux_settings_set (setting_type_t setting, float value, char *svalue)
{
    status_code_t status = Status_OK;

    switch(setting) {

        case Settings_IoPort_InvertIn:
            settings.ioport.invert_in.mask = (uint8_t)value & 0xFF;
            break;
/*
        case Settings_IoPort_Pullup_Disable:
            settings.ioport.pullup_disable_in.mask = (uint8_t)(int_value & 0xFF);
            break;
*/
        case Settings_IoPort_InvertOut:
            settings.ioport.invert_out.mask = (uint8_t)value & 0xFF;
            break;
/*
        case Settings_IoPort_OD_Enable:
            settings.ioport.od_enable_out.mask = (uint8_t)(int_value & 0xFF);
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
/*
static void aux_settings_restore (void)
{

    if(driver_settings.restore)
        driver_settings.restore();
}

static void aux_settings_load (void)
{
    if(driver_settings.load)
        driver_settings.load();
}
*/
static void digital_out (uint8_t port, bool on)
{
    switch(port) {
        case 0:
            BITBAND_PERI(AUXOUTPUT0_PORT->ODR, AUXOUTPUT0_PIN) = settings.ioport.invert_out.bit0 ? !on : on;
            break;

        case 1:
            BITBAND_PERI(AUXOUTPUT1_PORT->ODR, AUXOUTPUT1_PIN) = settings.ioport.invert_out.bit1 ? !on : on;
            break;
    }
}
/*
inline static __attribute__((always_inline)) int32_t get_input(gpio_t *gpio, wait_mode_t wait_mode, float timeout)
{
    uint_fast16_t delay = wait_mode == WaitMode_Immediate || timeout == 0.0f ? 0 : (uint_fast16_t)ceilf((1000.0f / 50.0f) * timeout) + 1;

    bool wait_for = wait_mode != WaitMode_Low;

    do {
        if(!!(gpio->reg->DR & gpio->bit) == wait_for)
            return !!(gpio->reg->DR & gpio->bit);

        if(delay) {
            protocol_execute_realtime();
            hal.delay_ms(50, NULL);
        } else
            break;
    } while(--delay && !sys.abort);

    return -1;
}
*/
static int32_t wait_on_input (bool digital, uint8_t port, wait_mode_t wait_mode, float timeout)
{
    int32_t value = -1;
/*
    if(digital) {
        switch(port) {
            case 0:
                value = get_input(&st0, wait_mode, timeout);
                break;

            case 1:
                value = get_input(&st1, wait_mode, timeout);
                break;
        }
    }
*/
    return value;
}

void board_init (void)
{

    hal.port.wait_on_input = wait_on_input;
    hal.port.digital_out = digital_out;
    hal.port.num_digital_in = 2;
    hal.port.num_digital_out = 2;

    memcpy(&driver_settings, &hal.driver_settings, sizeof(driver_setting_ptrs_t));

    hal.driver_settings.set = aux_settings_set;
    hal.driver_settings.report = aux_settings_report;
//    hal.driver_settings.load = aux_settings_load;
//    hal.driver_settings.restore = aux_settings_restore;

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
