/*
  nvs_buffer.c - RAM based non-volatile storage buffer/emulation

  Part of GrblHAL

  Copyright (c) 2017-2020 Terje Io
  Copyright (c) 2012-2016 Sungeun K. Jeon for Gnea Research LLC
  Copyright (c) 2009-2011 Simen Svale Skogsrud

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License.
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

//
// Can be used by MCUs with no nonvolatile storage options, be sure to allocate enough heap memory before use
//

#include <string.h>
#include <stdlib.h>

#include "hal.h"
#include "nvs_buffer.h"

static uint8_t *nvsbuffer = NULL;
static nvs_io_t physical_nvs;
static bool dirty;

settings_dirty_t settings_dirty;

typedef struct {
    uint16_t addr;
    uint8_t type;
    uint8_t offset;
} emap_t;

#define NVS_GROUP_GLOBAL 0
#define NVS_GROUP_TOOLS 1
#define NVS_GROUP_PARAMETERS 2
#define NVS_GROUP_STARTUP 3
#define NVS_GROUP_BUILD 4

#define PARAMETER_ADDR(n) (NVS_ADDR_PARAMETERS + n * (sizeof(coord_data_t) + 1))
#define STARTLINE_ADDR(n) (NVS_ADDR_STARTUP_BLOCK + n * (MAX_STORED_LINE_LENGTH + 1))
#ifdef N_TOOLS
#define TOOL_ADDR(n) (NVS_ADDR_TOOL_TABLE + n * (sizeof(tool_data_t) + 1))
#endif

static const emap_t target[] = {
    {NVS_ADDR_GLOBAL, NVS_GROUP_GLOBAL, 0},
#ifdef N_TOOLS
    {TOOL_ADDR(0), NVS_GROUP_TOOLS, 0},
    {TOOL_ADDR(1), NVS_GROUP_TOOLS, 1},
    {TOOL_ADDR(2), NVS_GROUP_TOOLS, 2},
    {TOOL_ADDR(3), NVS_GROUP_TOOLS, 3},
    {TOOL_ADDR(4), NVS_GROUP_TOOLS, 4},
    {TOOL_ADDR(5), NVS_GROUP_TOOLS, 5},
    {TOOL_ADDR(6), NVS_GROUP_TOOLS, 6},
    {TOOL_ADDR(7), NVS_GROUP_TOOLS, 7},
#if N_TOOL > 8
#error Increase number of tool entries!
#endif
#endif
    {PARAMETER_ADDR(0), NVS_GROUP_PARAMETERS, 0},
    {PARAMETER_ADDR(1), NVS_GROUP_PARAMETERS, 1},
    {PARAMETER_ADDR(2), NVS_GROUP_PARAMETERS, 2},
    {PARAMETER_ADDR(3), NVS_GROUP_PARAMETERS, 3},
    {PARAMETER_ADDR(4), NVS_GROUP_PARAMETERS, 4},
    {PARAMETER_ADDR(5), NVS_GROUP_PARAMETERS, 5},
    {PARAMETER_ADDR(6), NVS_GROUP_PARAMETERS, 6},
    {PARAMETER_ADDR(7), NVS_GROUP_PARAMETERS, 7},
    {PARAMETER_ADDR(8), NVS_GROUP_PARAMETERS, 8},
    {PARAMETER_ADDR(9), NVS_GROUP_PARAMETERS, 9},
    {PARAMETER_ADDR(10), NVS_GROUP_PARAMETERS, 10},
    {PARAMETER_ADDR(11), NVS_GROUP_PARAMETERS, 11},
    {STARTLINE_ADDR(0), NVS_GROUP_STARTUP, 0},
    {STARTLINE_ADDR(1), NVS_GROUP_STARTUP, 1},
#if N_STARTUP_LINE > 2
#error Increase number of startup line entries!
#endif
    {NVS_ADDR_BUILD_INFO, NVS_GROUP_BUILD, 0},
    {0, 0, 0} // list termination - do not remove
};

inline static uint8_t ram_get_byte (uint32_t addr)
{
    return nvsbuffer[addr];
}

inline static void ram_put_byte (uint32_t addr, uint8_t new_value)
{
    dirty = dirty || nvsbuffer[addr] != new_value;
    nvsbuffer[addr] = new_value;
}

// Extensions added as part of Grbl

static void memcpy_to_ram_with_checksum (uint32_t destination, uint8_t *source, uint32_t size)
{
    if(hal.nvs.driver_area.address && destination > hal.nvs.driver_area.address + hal.nvs.driver_area.size) {
        physical_nvs.memcpy_to_with_checksum(destination, source, size);
        return;
    }

    uint32_t dest = destination;
    uint8_t checksum = calc_checksum(source, size);

    dirty = false;

    for(; size > 0; size--)
        ram_put_byte(dest++, *(source++));

    ram_put_byte(dest, checksum);

    if(dirty && physical_nvs.type != NVS_None) {

        uint8_t idx = 0;

        settings_dirty.is_dirty = true;

        if(hal.nvs.driver_area.address && destination == hal.nvs.driver_area.address)
            settings_dirty.driver_settings = true;

        else {

            do {
                if(target[idx].addr == destination)
                    break;
            } while(target[++idx].addr);

            if(target[idx].addr) switch(target[idx].type) {

                case NVS_GROUP_GLOBAL:
                    settings_dirty.global_settings = true;
                    break;
#ifdef N_TOOLS
                case NVS_GROUP_TOOLS:
                    settings_dirty.tool_data |= (1 << target[idx].offset);
                    break;
#endif
                case NVS_GROUP_PARAMETERS:
                    settings_dirty.coord_data |= (1 << target[idx].offset);
                    break;

                case NVS_GROUP_STARTUP:
                    settings_dirty.startup_lines |= (1 << target[idx].offset);
                    break;

                case NVS_GROUP_BUILD:
                    settings_dirty.build_info = true;
                    break;
            }
        }
    }
}

static bool memcpy_from_ram_with_checksum (uint8_t *destination, uint32_t source, uint32_t size)
{
    if(hal.nvs.driver_area.address && source > hal.nvs.driver_area.address + hal.nvs.driver_area.size)
        return physical_nvs.memcpy_from_with_checksum(destination, source, size);

    uint8_t checksum = calc_checksum(&nvsbuffer[source], size);

    for(; size > 0; size--)
        *(destination++) = ram_get_byte(source++);

    return checksum == ram_get_byte(source);
}

//
// Try to allocate RAM for buffer/emulation and switch over to RAM based copy.
// Changes to RAM based copy will be written to physical storage when Grbl is in IDLE state.
// NOTE: enough free heap memory is required.
bool nvs_buffer_init (void)
{
    if(hal.nvs.size == 0)
        hal.nvs.size = GRBL_NVS_SIZE;

    uint_fast16_t idx = hal.nvs.size;

    memcpy(&physical_nvs, &hal.nvs, sizeof(nvs_io_t)); // save pointers to physical storage handler functions

    if((nvsbuffer = malloc(hal.nvs.size)) != 0) {

        if(physical_nvs.type == NVS_Flash)
            physical_nvs.memcpy_from_flash(nvsbuffer);
        else if(physical_nvs.type != NVS_None) {
            // Initialize physical storage on settings version mismatch
            if(physical_nvs.get_byte(0) != SETTINGS_VERSION)
                settings_init();

            // Copy physical storage content to RAM
            do {
                idx--;
                ram_put_byte(idx, physical_nvs.get_byte(idx));
            } while(idx);
        }

        // Switch hal to use RAM version of non-volatile storage data
        hal.nvs.type = NVS_Emulated;
        hal.nvs.get_byte = &ram_get_byte;
        hal.nvs.put_byte = &ram_put_byte;
        hal.nvs.memcpy_to_with_checksum = &memcpy_to_ram_with_checksum;
        hal.nvs.memcpy_from_with_checksum = &memcpy_from_ram_with_checksum;
        hal.nvs.memcpy_from_flash = NULL;
        hal.nvs.memcpy_to_flash = NULL;

        // If no physical storage available or if flash load fails import default settings to RAM
        if(physical_nvs.type == NVS_None || ram_get_byte(0) != SETTINGS_VERSION) {
            settings_restore((settings_restore_t){0xFF});
            if(physical_nvs.type == NVS_Flash) {
                physical_nvs.memcpy_to_flash(nvsbuffer);
                grbl.report.status_message(Status_SettingReadFail);
            }
        }
    }

    // Clear settings dirty flags
    memset(&settings_dirty, 0, sizeof(settings_dirty_t));

    return nvsbuffer != NULL;
}

// Write RAM changes to physical storage
void nvs_buffer_sync_physical (void)
{
    if(!settings_dirty.is_dirty)
        return;

    if(physical_nvs.memcpy_to_with_checksum) {

        if(settings_dirty.build_info) {
            settings_dirty.build_info = false;
            physical_nvs.memcpy_to_with_checksum(NVS_ADDR_BUILD_INFO, (uint8_t *)(nvsbuffer + NVS_ADDR_BUILD_INFO), MAX_STORED_LINE_LENGTH);
        }

        if(settings_dirty.global_settings) {
            settings_dirty.global_settings = false;
            physical_nvs.memcpy_to_with_checksum(NVS_ADDR_GLOBAL, (uint8_t *)(nvsbuffer + NVS_ADDR_GLOBAL), sizeof(settings_t));
        }

        uint_fast8_t idx = N_STARTUP_LINE, offset;
        if(settings_dirty.startup_lines) do {
            idx--;
            if(bit_istrue(settings_dirty.startup_lines, bit(idx))) {
                bit_false(settings_dirty.startup_lines, bit(idx));
                offset = NVS_ADDR_STARTUP_BLOCK + idx * (MAX_STORED_LINE_LENGTH + 1);
                physical_nvs.memcpy_to_with_checksum(offset, (uint8_t *)(nvsbuffer + offset), MAX_STORED_LINE_LENGTH);
            }
        } while(idx);

        idx = N_CoordinateSystems;
        if(settings_dirty.coord_data) do {
            if(bit_istrue(settings_dirty.coord_data, bit(idx))) {
                bit_false(settings_dirty.coord_data, bit(idx));
                offset = NVS_ADDR_PARAMETERS + idx * (sizeof(coord_data_t) + 1);
                physical_nvs.memcpy_to_with_checksum(offset, (uint8_t *)(nvsbuffer + offset), sizeof(coord_data_t));
            }
        } while(idx--);

        if(settings_dirty.driver_settings) {
            settings_dirty.driver_settings = false;
            if(hal.nvs.driver_area.size > 0)
                physical_nvs.memcpy_to_with_checksum(hal.nvs.driver_area.address, (uint8_t *)(nvsbuffer + hal.nvs.driver_area.address), hal.nvs.driver_area.size);
        }

#ifdef N_TOOLS
        idx = N_TOOLS;
        if(settings_dirty.tool_data) do {
            idx--;
            if(bit_istrue(settings_dirty.tool_data, bit(idx))) {
                bit_false(settings_dirty.tool_data, bit(idx));
                offset = NVS_ADDR_TOOL_TABLE + idx * (sizeof(tool_data_t) + 1);
                physical_nvs.memcpy_to_with_checksum(offset, (uint8_t *)(nvsbuffer + offset), sizeof(tool_data_t));
            }
        } while(idx);
#endif

    } else if(physical_nvs.memcpy_to_flash)
        physical_nvs.memcpy_to_flash(nvsbuffer);

    settings_dirty.is_dirty = false;
}

nvs_io_t *nvs_buffer_get_physical (void)
{
    return hal.nvs.type == NVS_Emulated ? &physical_nvs : &hal.nvs;
}
