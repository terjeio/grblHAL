/*

  flash.h - driver code for NXP LPC176x ARM processors

  Part of Grbl

  Copyright (c) 2018 Terje Io

  Based on code by Todd Fleming and info from UM10360 - LPC176x/5x User manual

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

#include "LPC17xx.h"
#include "grbl/grbl.h"

#define IAP_PREP_WRITE   50
#define IAP_WRITE_SECTOR 51
#define IAP_ERASE_SECTOR 52

#define IAP_CMD_SUCCESS 0

typedef struct {
	uint32_t command;
	uint32_t param0;
	uint32_t param1;
	uint32_t param2;
	uint32_t param3;
} iap_command_t;

typedef struct {
	uint32_t return_code;
	uint32_t result0;
	uint32_t result1;
	uint32_t result2;
	uint32_t result3;
} iap_output_t;

typedef void (*Iap)(iap_command_t *, iap_output_t *);

static const uint32_t flash_sector = 15;				// Last 4k sector number
static const uint8_t *flash_target = (uint8_t *)0xF000;	// Last 4k sector start adress
static const Iap iap = (Iap)0x1FFF1FF1;					// IAP entry point in ROM

bool memcpy_from_flash (uint8_t *dest)
{
    memcpy(dest, flash_target, GRBL_EEPROM_SIZE);
    return true;
}

bool memcpy_to_flash (uint8_t *source)
{
    if (!memcmp(source, flash_target, GRBL_EEPROM_SIZE))
        return true;

    iap_command_t prepare = {
    	IAP_PREP_WRITE,
        flash_sector,
        flash_sector,
		0,
		0
    };

    iap_command_t action = {
    	IAP_ERASE_SECTOR,
        flash_sector,
        flash_sector,
		SystemCoreClock / 1000,
		0
    };

    iap_output_t output;

    iap(&prepare, &output);
    iap(&action, &output);
    iap(&prepare, &output);

    action.command = IAP_WRITE_SECTOR;
    action.param0  = (uint32_t)flash_target;
    action.param1  = (uint32_t)source;
    action.param2  = GRBL_EEPROM_SIZE;
    action.param2  = SystemCoreClock / 1000;

    iap(&action, &output);

    return output.return_code == IAP_CMD_SUCCESS;
}
