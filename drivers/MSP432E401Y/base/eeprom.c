/*
  eeprom.c - EEPROM driver

  For Texas Instruments SimpleLink ARM processors/LaunchPads

  Part of GrblHAL

  Copyright (c) 2017 Terje Io

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

#define EEPROMOFFSET 0

static inline uint32_t putByte (uint32_t target, uint32_t source, uint32_t byte) {
    uint32_t mask = 0xFF;
    byte <<= 3;
    mask = ~(mask << byte);
    return (target & mask) | (source << byte);
}

static inline uint8_t getByte (uint32_t data, uint32_t byte) {
    return (data >> (byte << 3)) & 0xFF;
}

// Read/write eeprom configuration - do not set HAL pointers if not supported by MCU
// Used by code in settingcs.c
// NOTE: This implementation need some heap memory to work correctly, at least as much as the largest EEPROM block used + 4 bytes.
//       It is due to Tiva C having word-aligned EEPROM access, the heap is used for temporary storage when word aligning the structs read/written.
uint8_t eepromGetByte (uint32_t addr) {

    uint32_t data;

    EEPROMRead(&data, (addr + EEPROMOFFSET) & 0xFFFFFFFC, 4);

    return getByte(data, addr & 0x00000003);

}

void eepromPutByte (uint32_t addr, uint8_t new_value) {

    uint32_t data;

    EEPROMRead(&data, (addr + EEPROMOFFSET) & 0xFFFFFFFC, 4);

    data = putByte(data, (uint32_t)new_value, addr & 0x03);

    EEPROMProgram(&data, (addr + EEPROMOFFSET) & 0xFFFFFFFC, 4);
}

void eepromWriteBlockWithChecksum (uint32_t destination, uint8_t *source, uint32_t size) {

    uint8_t *data;
    uint32_t alignstart = (destination & 0x03), alignend = 4 - ((alignstart + size) & 0x03), alignsize = size + alignstart + alignend;

    if(alignstart || alignend) {

        if((data = malloc(alignsize)) != 0) {

            EEPROMRead((uint32_t *)data, (destination - alignstart) + EEPROMOFFSET, alignsize);

            memcpy(data + alignstart, source, size);

            EEPROMProgram((uint32_t *)data, (destination - alignstart) + EEPROMOFFSET, alignsize);

            free(data);

        }

    } else
        EEPROMProgram((uint32_t *)source, destination + EEPROMOFFSET, size);

    eepromPutByte(destination + size, calc_checksum(source, size));
}

bool eepromReadBlockWithChecksum (uint8_t *destination, uint32_t source, uint32_t size) {

    uint8_t *data;
    uint32_t alignstart = (source & 0x03), alignend = 4 - ((alignstart + size) & 0x03), alignsize = size + alignstart + alignend;

    if(alignstart || alignend) {

        if((data = malloc(alignsize)) != 0) {

            EEPROMRead((uint32_t *)data, (source - alignstart) + EEPROMOFFSET, alignsize);

            memcpy(destination, data + alignstart, size);

            free(data);

        }

    } else
        EEPROMRead((uint32_t *)destination, source + EEPROMOFFSET, size);

    return calc_checksum(destination, size) == eepromGetByte(source + size);
}
