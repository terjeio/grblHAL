/*

  eeprom_24LC16B.c - plugin for for I2C EEPROM (Microchip 24LC16B)

  Part of grblHAL

  Copyright (c) 2017-2020 Terje Io

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

#if EEPROM_ENABLE == 1

#include "grbl/hal.h"
#include "grbl/nuts_bolts.h"

#define EEPROM_I2C_ADDRESS (0xA0 >> 1)
#define EEPROM_ADDR_BITS_LO 8
#define EEPROM_BLOCK_SIZE (2 ^ EEPROM_LO_ADDR_BITS)
#define EEPROM_PAGE_SIZE 16

static nvs_transfer_t i2c = { .word_addr_bytes = 1 };

static uint8_t getByte (uint32_t addr)
{
    uint8_t value = 0;

    i2c.address = EEPROM_I2C_ADDRESS | (addr >> 8);
    i2c.word_addr = addr & 0xFF;
    i2c.data = &value;
    i2c.count = 1;

    i2c_nvs_transfer(&i2c, true);

    return value;
}

static void putByte (uint32_t addr, uint8_t new_value)
{
    i2c.address = EEPROM_I2C_ADDRESS | (addr >> 8);
    i2c.word_addr = addr & 0xFF;
    i2c.data = &new_value;
    i2c.count = 1;

    i2c_nvs_transfer(&i2c, false);
}


static nvs_transfer_result_t writeBlock (uint32_t destination, uint8_t *source, uint32_t size, bool with_checksum)
{
    uint32_t remaining = size;
    uint8_t *target = source;

    while(remaining > 0) {
        i2c.address = EEPROM_I2C_ADDRESS | (destination >> EEPROM_ADDR_BITS_LO);
        i2c.word_addr = destination & 0xFF;
        i2c.count = EEPROM_PAGE_SIZE - (destination & (EEPROM_PAGE_SIZE - 1));
        i2c.count = remaining < i2c.count ? remaining : i2c.count;
        i2c.data = target;
        remaining -= i2c.count;
        target += i2c.count;
        destination += i2c.count;

        i2c_nvs_transfer(&i2c, false);
    }

    if(size > 0 && with_checksum)
        putByte(destination, calc_checksum(source, size));

    return NVS_TransferResult_OK;
}

static nvs_transfer_result_t readBlock (uint8_t *destination, uint32_t source, uint32_t size, bool with_checksum)
{
    uint32_t remaining = size;
    uint8_t *target = destination;

    while(remaining) {
        i2c.address = EEPROM_I2C_ADDRESS | (source >> 8);
        i2c.word_addr = source & 0xFF;
        i2c.count = remaining > 255 ? 255 : (uint8_t)remaining;
        i2c.data = target;
        remaining -= i2c.count;
        target += i2c.count;
        source += i2c.count;

        i2c_nvs_transfer(&i2c, true);
    }

    return with_checksum ? (calc_checksum(destination, size) == getByte(source) ? NVS_TransferResult_OK : NVS_TransferResult_Failed) : NVS_TransferResult_OK;
}

void i2c_eeprom_init (void)
{
#if EEPROM_IS_FRAM
    hal.nvs.type = NVS_FRAM;
#else
    hal.nvs.type = NVS_EEPROM;
#endif
    hal.nvs.get_byte = getByte;
    hal.nvs.put_byte = putByte;
    hal.nvs.memcpy_to_nvs = writeBlock;
    hal.nvs.memcpy_from_nvs = readBlock;
}

#endif
