/*
  st_morpho.c - driver code for STM32F4xx ARM processors

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

#include "driver.h"

#if defined(BOARD_MORPHO_CNC)

#include <math.h>
#include <string.h>

#include "main.h"
#include "spi.h"
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

#if TRINAMIC_ENABLE == 2130

static axes_signals_t tmc;
static uint32_t n_axis;
static TMC2130_datagram_t datagram[N_AXIS];
static uint8_t br[N_AXIS];

static TMC2130_status_t TMC_SPI_ReadRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    static TMC2130_status_t status = {0};

    uint8_t res;
    uint_fast8_t idx = N_AXIS, ridx = 0;
    uint32_t f_spi = spi_set_speed(SPI_BAUDRATEPRESCALER_32);
    volatile uint32_t dly = 100;

    datagram[driver->axis].addr.value = reg->addr.value;
    datagram[driver->axis].addr.write = 0;

    BITBAND_PERI(TRINAMIC_CS_PORT->ODR, TRINAMIC_CS_PIN) = 0;

    do {
        if(bit_istrue(tmc.mask, bit(--idx))) {
            spi_put_byte(datagram[idx].addr.value);
            spi_put_byte(0);
            spi_put_byte(0);
            spi_put_byte(0);
            spi_put_byte(0);
        }
    } while(idx);

    while(--dly);

    BITBAND_PERI(TRINAMIC_CS_PORT->ODR, TRINAMIC_CS_PIN) = 1;

    dly = 50;
    while(--dly);

    BITBAND_PERI(TRINAMIC_CS_PORT->ODR, TRINAMIC_CS_PIN) = 0;

    idx = N_AXIS;
    do {
        ridx++;
        if(bit_istrue(tmc.mask, bit(--idx))) {

            res = spi_put_byte(datagram[idx].addr.value);

            if(N_AXIS - ridx == driver->axis) {
                status.value = res;
                reg->payload.data[3] = spi_get_byte();
                reg->payload.data[2] = spi_get_byte();
                reg->payload.data[1] = spi_get_byte();
                reg->payload.data[0] = spi_get_byte();
            } else {
                spi_get_byte();
                spi_get_byte();
                spi_get_byte();
                spi_get_byte();
            }
        }
    } while(idx);

    dly = 100;
    while(--dly);

    BITBAND_PERI(TRINAMIC_CS_PORT->ODR, TRINAMIC_CS_PIN) = 1;

    dly = 50;
    while(--dly);

    spi_set_speed(f_spi);

    return status;
}

static TMC2130_status_t TMC_SPI_WriteRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    static TMC2130_status_t status = {0};

    uint8_t res;
    uint_fast8_t idx = N_AXIS, ridx = 0;
    uint32_t f_spi = spi_set_speed(SPI_BAUDRATEPRESCALER_32);
    volatile uint32_t dly = 100;

    memcpy(&datagram[driver->axis], reg, sizeof(TMC2130_datagram_t));
    datagram[driver->axis].addr.write = 1;

    BITBAND_PERI(TRINAMIC_CS_PORT->ODR, TRINAMIC_CS_PIN) = 0;

    do {
        ridx++;
        if(bit_istrue(tmc.mask, bit(--idx))) {

            res = spi_put_byte(datagram[idx].addr.value);
            spi_put_byte(datagram[idx].payload.data[3]);
            spi_put_byte(datagram[idx].payload.data[2]);
            spi_put_byte(datagram[idx].payload.data[1]);
            spi_put_byte(datagram[idx].payload.data[0]);

            if(N_AXIS - ridx == driver->axis)
                status.value = res;

            if(idx == driver->axis) {
                datagram[idx].addr.reg = TMC2130Reg_DRV_STATUS;
                datagram[idx].addr.write = 0;
            }
        }
    } while(idx);

    while(--dly);

    BITBAND_PERI(TRINAMIC_CS_PORT->ODR, TRINAMIC_CS_PIN) = 1;

    dly = 50;
    while(--dly);

    spi_set_speed(f_spi);

    return status;
}

void SPI_DriverInit (TMC_io_driver_t *driver, axes_signals_t axisflags)
{
    tmc = axisflags;
    n_axis = 0;
    while(axisflags.mask) {
        n_axis += (axisflags.mask & 0x01);
        axisflags.mask >>= 1;
    }

    driver->WriteRegister = TMC_SPI_WriteRegister;
    driver->ReadRegister = TMC_SPI_ReadRegister;
}

#endif

#if TRINAMIC_ENABLE == 2209

#include "serial.h"

static TMC2209_write_datagram_t *TMC_UART_ReadRegister (TMC2209_t *driver, TMC2209_read_datagram_t *dgr)
{
    static TMC2209_write_datagram_t wdgr = {0};
    volatile uint32_t dly = 50, ms = hal.get_elapsed_ticks();

    serial2Write((char *)dgr->data, sizeof(TMC2209_read_datagram_t));

    while(serial2TxCount());

    while(--dly);

    serial2RxFlush();

    // Wait for response with 2ms timeout
    while(serial2RxCount() < 8) {
        if(hal.get_elapsed_ticks() - ms >= 2)
            break;
    }

    if(serial2RxCount() >= 8) {
        wdgr.data[0] = serial2GetC();
        wdgr.data[1] = serial2GetC();
        wdgr.data[2] = serial2GetC();
        wdgr.data[3] = serial2GetC();
        wdgr.data[4] = serial2GetC();
        wdgr.data[5] = serial2GetC();
        wdgr.data[6] = serial2GetC();
        wdgr.data[7] = serial2GetC();
    } else
        wdgr.msg.addr.value = 0xFF;

    dly = 150;
    while(--dly);

    return &wdgr;
}

static void TMC_UART_WriteRegister (TMC2209_t *driver, TMC2209_write_datagram_t *dgr)
{
    serial2Write((char *)dgr->data, sizeof(TMC2209_write_datagram_t));
}

void UART_DriverInit (TMC_io_driver_t *driver)
{
    serial2Init(230400);

    driver->WriteRegister = TMC_UART_WriteRegister;
    driver->ReadRegister = TMC_UART_ReadRegister;
}

#endif

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

#if TRINAMIC_ENABLE == 2130

    spi_init();
    GPIO_Init.Pin = TRINAMIC_CS_BIT;
    HAL_GPIO_Init(TRINAMIC_CS_PORT, &GPIO_Init);
    BITBAND_PERI(TRINAMIC_CS_PORT->ODR, TRINAMIC_CS_PIN) = 1;

    uint_fast8_t idx = N_AXIS;
    do {
        datagram[--idx].addr.reg = TMC2130Reg_DRV_STATUS;
    } while(idx);

#endif

}

#endif
