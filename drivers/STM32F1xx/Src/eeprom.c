/*

  eeprom.c - driver code for STM32F103C8 ARM processors

  for 2K EEPROM on CNC Boosterpack (Microchip 24LC16B)

  Part of Grbl

  Copyright (c) 2017-2018 Terje Io

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
#include <stdint.h>
#include <stdbool.h>

#include "../GRBL/grbl.h"

extern I2C_HandleTypeDef hi2c2;

#define EEPROM_I2C_ADDRESS (0xA0 >> 1)
#define EEPROM_ADDR_BITS_LO 8
#define EEPROM_BLOCK_SIZE (2 ^ EEPROM_LO_ADDR_BITS)
#define EEPROM_PAGE_SIZE 16

typedef struct {
    uint8_t addr;
    volatile int16_t count;
    uint8_t *data;
    uint8_t word_addr;
} i2c_trans_t;

static i2c_trans_t i2c;

void eepromInit (void)
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
	if (HAL_I2C_Init(&hi2c2) != HAL_OK)
		Error_Handler();
}

static void StartI2C (bool read)
{

    while (HAL_I2C_GetState(&hi2c2) != HAL_I2C_STATE_READY);

//    while (HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(0xA0), 3, 100) != HAL_OK);

    if(read)
    	HAL_I2C_Mem_Read(&hi2c2, i2c.addr, i2c.word_addr, I2C_MEMADD_SIZE_8BIT, i2c.data, i2c.count, 100);
    else {
    	HAL_I2C_Mem_Write(&hi2c2, i2c.addr, i2c.word_addr, I2C_MEMADD_SIZE_8BIT, i2c.data, i2c.count, 100);
        hal.delay_ms(5, NULL);
    }
}

uint8_t eepromGetByte (uint32_t addr)
{
    static uint8_t value = 0x55;

    i2c.addr = EEPROM_I2C_ADDRESS | (addr >> 8);
    i2c.word_addr = addr & 0xFF;
    i2c.data = &value;
    i2c.count = 1;
    StartI2C(true);

    return value;
}

void eepromPutByte (uint32_t addr, uint8_t new_value)
{
    i2c.addr = EEPROM_I2C_ADDRESS | (addr >> 8);
    i2c.word_addr = addr & 0xFF;
    i2c.data = &new_value;
    i2c.count = 1;

    StartI2C(false);
}

void eepromWriteBlockWithChecksum (uint32_t destination, uint8_t *source, uint32_t size)
{
    uint32_t bytes = size;

    i2c.word_addr = destination & 0xFF;
    i2c.data = source;

    while(bytes > 0) {
        i2c.count = EEPROM_PAGE_SIZE - (destination & (EEPROM_PAGE_SIZE - 1));
        i2c.count = bytes < i2c.count ? bytes : i2c.count;
        i2c.addr = EEPROM_I2C_ADDRESS | (destination >> EEPROM_ADDR_BITS_LO);
        bytes -= i2c.count;
        destination += i2c.count;

        StartI2C(false);

        i2c.word_addr = destination & 0xFF;
    }

    if(size > 0)
        eepromPutByte(destination, calc_checksum(source, size));
}

bool eepromReadBlockWithChecksum (uint8_t *destination, uint32_t source, uint32_t size)
{
    i2c.addr = EEPROM_I2C_ADDRESS | (source >> 8);
    i2c.word_addr = source & 0xFF;
    i2c.count = size;
    i2c.data = destination;

    StartI2C(true);

    return calc_checksum(destination, size) == eepromGetByte(source + size);
}
