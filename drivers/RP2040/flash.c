/*

  flash.c - driver code for RP2040 ARM processor

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

/*
	IMPORTANT! If both cores are in use synchronization has to be added since flash cannot be read during programming
	
	See 4.1.8. hardware_flash in the SDK documentation.
*/

#include <string.h>

#include "hardware/flash.h"
#include "hardware/regs/addressmap.h"

#include "driver.h"

#define FLASH_TARGET_OFFSET (1024 * 512)

static const uint8_t *flash_target = (const uint8_t *)(XIP_BASE + FLASH_TARGET_OFFSET);    // Last page start adress

bool memcpy_from_flash (uint8_t *dest)
{
    memcpy(dest, flash_target, hal.nvs.size);

    return true;
}

bool memcpy_to_flash (uint8_t *source)
{
    if (!memcmp(source, flash_target, hal.nvs.size))
        return true;
 
    __disable_irq();
    flash_range_erase(FLASH_TARGET_OFFSET, FLASH_SECTOR_SIZE);
    flash_range_program(FLASH_TARGET_OFFSET, source, FLASH_PAGE_SIZE * (hal.nvs.size / FLASH_PAGE_SIZE + (hal.nvs.size % FLASH_PAGE_SIZE ? 1 :0)));
    __enable_irq();
 
    return !memcmp(source, flash_target, hal.nvs.size);
}
