/*
  i2c.c - I2C support for EEPROM, keypad and Trinamic plugins

  Part of GrblHAL driver for STM32F103C8

  Copyright (c) 2018-2020 Terje Io

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

#include <main.h>

#include "i2c.h"
#include "grbl.h"

#if KEYPAD_ENABLE
#include "keypad/keypad.h"
#endif

#ifdef I2C_PORT

extern I2C_HandleTypeDef hi2c2;

void i2c_init (void)
{
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed = 100000;
    hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1 = 0;
    hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2 = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    HAL_I2C_Init(&hi2c2);
    //  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
            //Error_Handler();
    HAL_NVIC_EnableIRQ(I2C2_EV_IRQn);
    HAL_NVIC_EnableIRQ(I2C2_ER_IRQn);
}

#endif

#if EEPROM_ENABLE

void i2c_eeprom_transfer (i2c_eeprom_trans_t *i2c, bool read)
{
    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);

//    while (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(0xA0), 3, 100) != HAL_OK);

    if(read)
        HAL_I2C_Mem_Read(&hi2c2, i2c->address << 1, i2c->word_addr, I2C_MEMADD_SIZE_8BIT, i2c->data, i2c->count, 100);
    else {
        HAL_I2C_Mem_Write(&hi2c2, i2c->address << 1, i2c->word_addr, I2C_MEMADD_SIZE_8BIT, i2c->data, i2c->count, 100);
        hal.delay_ms(5, NULL);
    }
    i2c->data += i2c->count;
}

#endif

#if KEYPAD_ENABLE

static uint8_t keycode = 0;
static keycode_callback_ptr keypad_callback = NULL;

void I2C_GetKeycode (uint32_t i2cAddr, keycode_callback_ptr callback)
{
    keycode = 0;
    keypad_callback = callback;

    HAL_StatusTypeDef ret = HAL_I2C_Master_Receive_IT(&hi2c2, KEYPAD_I2CADDR << 1, &keycode, 1);

    if(!ret)
        ret = HAL_I2C_Master_Receive_IT(&hi2c2, KEYPAD_I2CADDR << 1, &keycode, 1);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    if(keypad_callback && keycode != 0) {
        keypad_callback(keycode);
        keypad_callback = NULL;
    }
}

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

    HAL_I2C_Mem_Read(&hi2c2, tmc_addr, tmc_reg, I2C_MEMADD_SIZE_8BIT, buffer, 5, 100);

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

        HAL_I2C_Mem_Write(&hi2c2, tmc_addr, tmc_reg, I2C_MEMADD_SIZE_8BIT, buffer, 4, 100);
    }

    return status;
}

void I2C_DriverInit (TMC_io_driver_t *driver)
{
    driver->WriteRegister = TMC_I2C_WriteRegister;
    driver->ReadRegister = TMC_I2C_ReadRegister;
}

#endif
