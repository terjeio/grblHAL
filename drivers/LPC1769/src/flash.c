/*

  flash.h - driver code for NXP LPC176x ARM processors

  Part of grblHAL

  Copyright (c) 2018-2020 Terje Io

  This code reads/writes the whole RAM-based emulated EPROM contents from/to flash

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

#include <string.h>

#include "chip.h"
#include "grbl/hal.h"

#define IAP_CMD_SUCCESS 0

static const uint32_t flash_sector = 29;        // Last 32k sector number
static const uint32_t flash_target = 0x1000UL * 16 + 0x8000UL * (flash_sector - 16);

//static const uint32_t flash_sector = 15;      // Last 4k sector number
//static const uint32_t flash_target = 0xF000UL;    // Last 4k sector start adress

bool memcpy_from_flash (uint8_t *dest)
{
    memcpy(dest, (const void *)flash_target, hal.nvs.size);
    return true;
}

bool memcpy_to_flash (uint8_t *source)
{
    if (!memcmp(source, (const void *)flash_target, hal.nvs.size))
        return true;

    uint8_t ret;

    __disable_irq();

    ret = Chip_IAP_PreSectorForReadWrite(flash_sector, flash_sector);
    ret = Chip_IAP_EraseSector(flash_sector, flash_sector);
    ret = Chip_IAP_PreSectorForReadWrite(flash_sector, flash_sector);
    ret = Chip_IAP_CopyRamToFlash(flash_target, (uint32_t *)source, hal.nvs.size != 1024 ? 4096 : hal.nvs.size);

    __enable_irq();

    return ret == IAP_CMD_SUCCESS;
}
