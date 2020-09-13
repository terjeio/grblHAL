/*
  nvs.h - non-volative storage data structures

  Part of GrblHAL

  Copyright (c) 2017-2020 Terje Io
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#ifndef _NVS_H_
#define _NVS_H_

#define GRBL_NVS_SIZE 1024

typedef enum {
    NVS_None = 0,
    NVS_EEPROM,
    NVS_FRAM,
    NVS_Flash,
    NVS_Emulated
} nvs_type;

typedef struct {
    uint16_t address;
    uint16_t size;
} nvs_driver_area_t;

typedef struct {
    nvs_type type;
    uint16_t size;
    nvs_driver_area_t driver_area;
    uint8_t (*get_byte)(uint32_t addr);
    void (*put_byte)(uint32_t addr, uint8_t new_value);
    void (*memcpy_to_with_checksum)(uint32_t destination, uint8_t *source, uint32_t size);
    bool (*memcpy_from_with_checksum)(uint8_t *destination, uint32_t source, uint32_t size);
    bool (*memcpy_from_flash)(uint8_t *dest);
    bool (*memcpy_to_flash)(uint8_t *source);
} nvs_io_t;

#endif
