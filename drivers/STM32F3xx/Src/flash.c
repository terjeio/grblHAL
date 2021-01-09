/*

  flash.c - driver code for STM32F3xx ARM processors

  Part of grblHAL

  Copyright (c) 2021 Terje Io

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

#include "main.h"
#include "grbl/hal.h"

#define FLASH_TARGET (FLASH_BASE + ((*((uint16_t *)FLASH_SIZE_DATA_REGISTER) * FLASH_PAGE_SIZE) / 2) - (FLASH_PAGE_SIZE * 2))

bool memcpy_from_flash (uint8_t *dest)
{
    memcpy(dest, (uint8_t *)FLASH_TARGET, hal.nvs.size);
    return true;
}

bool memcpy_to_flash (uint8_t *source)
{
    if (!memcmp(source, (uint8_t *)FLASH_TARGET, hal.nvs.size))
        return true;

    HAL_FLASH_Unlock();

    FLASH_EraseInitTypeDef erase = {
        .TypeErase = FLASH_TYPEERASE_PAGES,
        .PageAddress = FLASH_TARGET,
        .NbPages = 1
    };

    uint32_t error;

    HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase, &error);

    uint16_t *data = (uint16_t *)source;
    uint32_t address = (uint32_t)FLASH_TARGET, remaining = (uint32_t)hal.nvs.size;

    while(remaining && status == HAL_OK) {
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address, *data++);
        status = HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD, address + 2, *data++);
        address += 4;
        remaining -= 4;
    }

    HAL_FLASH_Lock();

    return status == HAL_OK;
}
