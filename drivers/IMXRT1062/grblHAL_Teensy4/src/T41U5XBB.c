/*
  T41U5XBB.c - driver code for IMXRT1062 processor (on Teensy 4.1 board)

  Part of GrblHAL

  Board by Phil Barrett: https://github.com/phil-barrett/grblHAL-teensy-4.x

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

#ifdef BOARD_T41U5XBB

//#include "Arduino.h"
#include <math.h>
#include <string.h>

#include "grbl/protocol.h"

static gpio_t stx[AUX_N_IN];
static gpio_t aux_out[AUX_N_OUT];
static driver_setting_ptrs_t driver_settings;

static input_signal_t aux_in[] = {
    { .id = Input_Aux0, .port = &stx[0], .pin = AUXINPUT0_PIN, .group = 0 }
#if AUX_N_IN == 4
  , { .id = Input_Aux1, .port = &stx[1], .pin = AUXINPUT1_PIN, .group = 0 },
    { .id = Input_Aux2, .port = &stx[2], .pin = AUXINPUT2_PIN, .group = 0 },
    { .id = Input_Aux3, .port = &stx[3], .pin = AUXINPUT3_PIN, .group = 0 }
#endif
};

static const setting_group_detail_t aux_groups[] = {
    { Group_Root, Group_AuxPorts, "Aux ports"}
};

static const setting_detail_t aux_settings[] = {
#if AUX_N_IN == 4
    { Settings_IoPort_InvertIn, Group_AuxPorts, "Invert I/O Port inputs", NULL, Format_Bitfield, "Port 0,Port 1,Port 2,Port 3", NULL, NULL },
#else
    { Settings_IoPort_InvertIn, Group_AuxPorts, "Invert I/O Port inputs", NULL, Format_Bitfield, "Port 0", NULL, NULL },
#endif
    { Settings_IoPort_InvertOut, Group_AuxPorts, "Invert I/O Port outputs", NULL, Format_Bitfield, "Port 0,Port 1,Port 2", NULL, NULL },
};

static setting_details_t details = {
    .groups = aux_groups,
    .n_groups = sizeof(aux_groups) / sizeof(setting_group_detail_t),
    .settings = aux_settings,
    .n_settings = sizeof(aux_settings) / sizeof(setting_detail_t)
};

static setting_details_t *on_report_settings (void)
{
    return &details;
}

static void digital_out (uint8_t port, bool on)
{
    if(port < AUX_N_OUT)
        DIGITAL_OUT(aux_out[port], ((settings.ioport.invert_out.mask >> port) & 0x01) ? !on : on);
}

inline static __attribute__((always_inline)) int32_t get_input (gpio_t *gpio, bool invert, wait_mode_t wait_mode, float timeout)
{
    if(wait_mode == WaitMode_Immediate)
        return !!(gpio->reg->DR & gpio->bit) ^ invert;

    uint_fast16_t delay = (uint_fast16_t)ceilf((1000.0f / 50.0f) * timeout) + 1;

    bool wait_for = wait_mode != WaitMode_Low;

    do {
        if((!!(gpio->reg->DR & gpio->bit) ^ invert) == wait_for)
            return !!(gpio->reg->DR & gpio->bit);

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

    if(digital) {
        if(port < AUX_N_IN)
            value = get_input(aux_in[port].port, (settings.ioport.invert_in.mask << port) & 0x01, wait_mode, timeout);
    }
//    else if(port == 0)
//        value = analogRead(41);
/*
    hal.stream.write("[MSG:AUX");
    hal.stream.write(uitoa(port));
    hal.stream.write("=");
    hal.stream.write(value == -1 ? "fail" : uitoa(value));
    hal.stream.write("]" ASCII_EOL);
*/
    return value;
}

static status_code_t aux_settings_set (setting_type_t setting, float value, char *svalue)
{
    status_code_t status = Status_OK;

    switch(setting) {

        case Settings_IoPort_InvertIn:
            settings.ioport.invert_in.mask = (uint8_t)value & AUX_IN_MASK;
            break;

        case Settings_IoPort_InvertOut:
            {
                ioport_bus_t invert;
                invert.mask = (uint8_t)value & AUX_OUT_MASK;
                if(invert.mask != settings.ioport.invert_out.mask) {
                    uint_fast8_t idx = AUX_N_OUT;
                    do {
                        idx--;
                        if(((settings.ioport.invert_out.mask >> idx) & 0x01) != ((invert.mask >> idx) & 0x01))
                            DIGITAL_OUT(aux_out[idx], !DIGITAL_IN(aux_out[idx]));
                    } while(idx);

                    settings.ioport.invert_out.mask = invert.mask;
                }
            }
            break;

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

        case Settings_IoPort_InvertOut:
            if(hal.port.num_digital_out)
                report_uint_setting(Settings_IoPort_InvertOut, settings.ioport.invert_out.mask);
            break;

        default:
            reported = false;
            break;
    }

    if(!reported && driver_settings.report)
        driver_settings.report(setting);
}

static void aux_settings_load (void)
{
    uint_fast8_t idx = AUX_N_OUT;

    do {
        idx--;
        DIGITAL_OUT(aux_out[idx], (settings.ioport.invert_out.mask >> idx) & 0x01);
    } while(idx);

    if(driver_settings.load)
        driver_settings.load();
}

void board_init (void)
{
    hal.port.digital_out = digital_out;
    hal.port.wait_on_input = wait_on_input;
//    hal.port.num_analog_in  = 1;
    hal.port.num_digital_in = AUX_N_IN;
    hal.port.num_digital_out = AUX_N_OUT;

    memcpy(&driver_settings, &hal.driver_settings, sizeof(driver_setting_ptrs_t));

    hal.driver_settings.set = aux_settings_set;
    hal.driver_settings.report = aux_settings_report;
    hal.driver_settings.load = aux_settings_load;

    details.on_report_settings = grbl.on_report_settings;
    grbl.on_report_settings = on_report_settings;

    bool pullup;
    uint32_t i = AUX_N_IN;
    input_signal_t *signal;
    do {
        pullup = true; //??
        signal = &aux_in[--i];
        signal->irq_mode = IRQ_Mode_None;

        pinMode(signal->pin, pullup ? INPUT_PULLUP : INPUT_PULLDOWN);
        signal->gpio.reg = (gpio_reg_t *)digital_pin_to_info_PGM[signal->pin].reg;
        signal->gpio.bit = digital_pin_to_info_PGM[signal->pin].mask;

        if(signal->port != NULL)
            memcpy(signal->port, &signal->gpio, sizeof(gpio_t));
    } while(i);

    pinModeOutput(&aux_out[0], AUXOUTPUT0_PIN);
    pinModeOutput(&aux_out[1], AUXOUTPUT1_PIN);
    pinModeOutput(&aux_out[2], AUXOUTPUT2_PIN);

//    analog_init();
}

#endif
