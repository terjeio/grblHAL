/*
  i2c.c - I2C support for EEPROM, keypad and Trinamic plugins

  Part of grblHAL driver for RP2040

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

#include <stdio.h>
#include <string.h>

#include "hardware/gpio.h"
#include "hardware/i2c.h"

#include "i2c.h"
#include "grbl/hal.h"

#if KEYPAD_ENABLE
#include "keypad/keypad.h"
#endif

#define QI2C_PORT i2c1
#define MAX_PAGE_SIZE 64

#ifdef I2C_PORT

#define I2CPORT I2Cport(I2C_PORT)

void I2C_Init (void)
{

    gpio_set_function(27, GPIO_FUNC_I2C);
    gpio_set_function(26, GPIO_FUNC_I2C);

    i2c_init(QI2C_PORT, 100000UL);
}

void I2C_Send (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes, bool block)
{

    i2c_write_blocking(QI2C_PORT, i2cAddr, buf, bytes, false);
}

uint8_t *I2C_ReadRegister (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes, bool block)
{

    i2c_write_blocking(QI2C_PORT, i2cAddr, buf, 1, true);
    i2c_read_blocking(QI2C_PORT, i2cAddr, buf, 1, true);
    return buf;
}

#endif

#if EEPROM_ENABLE

nvs_transfer_result_t i2c_nvs_transfer (nvs_transfer_t *i2c, bool read)
{

    static uint8_t txbuf[MAX_PAGE_SIZE + 2];

    int retval = 0;

    if(i2c->word_addr_bytes == 2) {
        txbuf[0] = i2c->word_addr >> 8;
        txbuf[1] = i2c->word_addr & 0xFF;
    } else
        txbuf[0] = i2c->word_addr;

    if(!read)
        memcpy(&txbuf[i2c->word_addr_bytes], i2c->data, i2c->count);

    if(read) {
        i2c_write_blocking(QI2C_PORT, i2c->address, txbuf, i2c->word_addr_bytes, true);
        retval = i2c_read_blocking(QI2C_PORT, i2c->address, i2c->data, i2c->count, false);
    } else {
        retval = i2c_write_blocking(QI2C_PORT, i2c->address, txbuf, i2c->count + i2c->word_addr_bytes, false);
#if !EEPROM_IS_FRAM
        hal.delay_ms(5, NULL);
#endif
    }

    i2c->data += i2c->count;

    return retval == PICO_ERROR_GENERIC ? NVS_TransferResult_Failed : NVS_TransferResult_OK;
}

#endif

#if KEYPAD_ENABLE

static uint8_t keycode = 0;
static keycode_callback_ptr keypad_callback = NULL;

void I2C_GetKeycode (uint32_t i2cAddr, keycode_callback_ptr callback)
{
    uint8_t c;
    keycode = 0;
    keypad_callback = callback;

    if(i2c_read_blocking(QI2C_PORT, KEYPAD_I2CADDR, &c, 1, false) == 1)
        keypad_callback(c);
}
/*
void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(keypad_callback && keycode != 0) {
        keypad_callback(keycode);
        keypad_callback = NULL;
    }
}
*/
#endif

#if TRINAMIC_ENABLE && TRINAMIC_I2C

static const uint8_t tmc_addr = I2C_ADR_I2CBRIDGE << 1;

static TMC2130_status_t TMC_I2C_ReadRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    uint8_t tmc_reg, buffer[5] = {0};
    TMC2130_status_t status = {0};

    if((tmc_reg = TMCI2C_GetMapAddress((uint8_t)(driver ? (uint32_t)driver->cs_pin : 0), reg->addr).value) == 0xFF) {
        return status; // unsupported register
    }

    HAL_I2C_Mem_Read(&i2c_port, tmc_addr, tmc_reg, I2C_MEMADD_SIZE_8BIT, buffer, 5, 100);

    status.value = buffer[0];
    reg->payload.value = buffer[4];
    reg->payload.value |= buffer[3] << 8;
    reg->payload.value |= buffer[2] << 16;
    reg->payload.value |= buffer[1] << 24;

    return status;
}

static TMC2130_status_t TMC_I2C_WriteRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    uint8_t tmc_reg, buffer[4];
    TMC2130_status_t status = {0};

    reg->addr.write = 1;
    tmc_reg = TMCI2C_GetMapAddress((uint8_t)(driver ? (uint32_t)driver->cs_pin : 0), reg->addr).value;
    reg->addr.write = 0;

    if(tmc_reg != 0xFF) {

        buffer[0] = (reg->payload.value >> 24) & 0xFF;
        buffer[1] = (reg->payload.value >> 16) & 0xFF;
        buffer[2] = (reg->payload.value >> 8) & 0xFF;
        buffer[3] = reg->payload.value & 0xFF;

        HAL_I2C_Mem_Write(&i2c_port, tmc_addr, tmc_reg, I2C_MEMADD_SIZE_8BIT, buffer, 4, 100);
    }

    return status;
}

#endif
