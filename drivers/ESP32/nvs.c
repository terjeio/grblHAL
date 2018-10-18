/*
  nvs.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Peristent storage of settings in flash

  Part of Grbl

  Copyright (c) 2018 Terje Io

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

// NOTE: use with eeprom emulation enabled in order to avoid excessive write cycles

#include "esp_partition.h"

#include "GRBL/grbl.h"

static const esp_partition_t *grblNVS;
static uint8_t grbl_settings[GRBL_EEPROM_SIZE];
static bool dirty = false;

static bool commit (void)
{
	if(dirty &&
	    esp_partition_erase_range(grblNVS, 0, SPI_FLASH_SEC_SIZE) == ESP_OK &&
		 esp_partition_write(grblNVS, 0, &grbl_settings, GRBL_EEPROM_SIZE) == ESP_OK)
		dirty = false;

	return !dirty;
}

uint8_t nvsGetByte (uint32_t addr)
{
    return grbl_settings[addr];
}

void nvsPutByte (uint32_t addr, uint8_t new_value)
{
	dirty = dirty || grbl_settings[addr] != new_value;
	grbl_settings[addr] = new_value;
	commit();
}

void nvsWriteBlockWithChecksum (uint32_t destination, uint8_t *source, uint32_t size)
{
	dirty = dirty || memcmp(grbl_settings + destination, source, size);

	memcpy(grbl_settings + destination, source, size);

	grbl_settings[destination + size] = calc_checksum(source, size);

	commit();
}

bool nvsReadBlockWithChecksum (uint8_t *destination, uint32_t source, uint32_t size)
{
	memcpy(destination, grbl_settings + source, size);

    return calc_checksum(destination, size) == nvsGetByte(source + size);
}

bool nvsInit (void)
{
	grblNVS = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "grbl");

	if(grblNVS && !esp_partition_read(grblNVS, 0, (void *)grbl_settings, GRBL_EEPROM_SIZE) == ESP_OK)
		grblNVS = NULL;

	return grblNVS != NULL;
}
