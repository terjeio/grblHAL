/*

  eeprom_24AAxxx.c - plugin for for I2C EEPROM (Microchip 24AAxxx > 16kbit, 2 byte address)

  NOTE: only tested with 24AA256

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
#include "../../driver.h"
#else
#include "driver.h"
#endif

#if EEPROM_ENABLE == 2
xx
#ifdef ARDUINO
#include "../grbl/grbl.h"
#include "../grbl/plugins.h"
#else
#include "grbl/grbl.h"
#include "grbl/plugins.h"
#endif

#define EEPROM_I2C_ADDRESS (0xA0 >> 1)
#define EEPROM_PAGE_SIZE 64

static i2c_eeprom_trans_t i2c = { .word_addr_bytes = 2 };

void eepromInit (void)
{
    i2c_init();
}

uint8_t eepromGetByte (uint32_t addr)
{
    uint8_t value = 0;

    i2c.address = EEPROM_I2C_ADDRESS;
    i2c.word_addr = addr;
    i2c.data = &value;
    i2c.count = 1;

    i2c_eeprom_transfer(&i2c, true);

    return value;
}

void eepromPutByte (uint32_t addr, uint8_t new_value)
{
    i2c.address = EEPROM_I2C_ADDRESS;
    i2c.word_addr = addr;
    i2c.data = &new_value;
    i2c.count = 1;

    i2c_eeprom_transfer(&i2c, false);
}

void eepromWriteBlockWithChecksum (uint32_t destination, uint8_t *source, uint32_t size)
{
    uint32_t remaining = size;
    uint8_t *target = source;

    while(remaining > 0) {
        i2c.address = EEPROM_I2C_ADDRESS;
        i2c.word_addr = destination;
        i2c.count = EEPROM_PAGE_SIZE - (destination & (EEPROM_PAGE_SIZE - 1));
        i2c.count = remaining < i2c.count ? remaining : i2c.count;
        i2c.data = target;
        remaining -= i2c.count;
        target += i2c.count;
        destination += i2c.count;

        i2c_eeprom_transfer(&i2c, false);
    }

    if(size > 0)
        eepromPutByte(destination, calc_checksum(source, size));
}

bool eepromReadBlockWithChecksum (uint8_t *destination, uint32_t source, uint32_t size)
{
    uint32_t remaining = size;
    uint8_t *target = destination;

    while(remaining) {
        i2c.address = EEPROM_I2C_ADDRESS;
        i2c.word_addr = source;
        i2c.count = remaining > 255 ? 255 : (uint8_t)remaining;
        i2c.data = target;
        remaining -= i2c.count;
        target += i2c.count;
        source += i2c.count;

        i2c_eeprom_transfer(&i2c, true);
    }

    return calc_checksum(destination, size) == eepromGetByte(source);
}

#endif
