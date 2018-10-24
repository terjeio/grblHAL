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

#include "esp_partition.h"
#include "esp_log.h"

#include "nvs.h"
#include "GRBL/grbl.h"

#ifndef EMULATE_EEPROM
#error EMULATE_EPROM must be enabled to use flash for settings storage
#endif

static const esp_partition_t *grblNVS = NULL;

bool nvsRead (uint8_t *dest)
{
	bool ok;

	if(!(ok = grblNVS && esp_partition_read(grblNVS, 0, (void *)dest, GRBL_NVS_SIZE) == ESP_OK))
		grblNVS = NULL;

	return ok;
}

bool nvsWrite (uint8_t *source)
{
	return grblNVS &&
			esp_partition_erase_range(grblNVS, 0, SPI_FLASH_SEC_SIZE) == ESP_OK &&
			 esp_partition_write(grblNVS, 0, (void *)source, GRBL_NVS_SIZE) == ESP_OK;
}

bool nvsInit (void)
{
	grblNVS = esp_partition_find_first(ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "grbl");

	return grblNVS != NULL;
}
