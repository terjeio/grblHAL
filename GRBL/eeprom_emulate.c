/*
  eeprom_emulate.h - RAM based EEPROM emulation
  Part of Grbl

  Copyright (c) 2017-2018 Terje Io
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
// Call noeeprom_init(), if returns true then set up EEPROM HAL pointers to these functions
//

#include "grbl.h"

static uint8_t *noepromdata = 0;
static eeprom_io_t physical_eeprom;
static bool dirty;

settings_dirty_t settings_dirty;

typedef struct {
    uint16_t addr;
    uint8_t type;
    uint8_t offset;
} emap_t;

#define EEPROM_GRP_GLOBAL 0
#define EEPROM_GRP_TOOLS 1
#define EEPROM_GRP_PARAMETERS 2
#define EEPROM_GRP_STARTUP 3
#define EEPROM_GRP_BUILD 4

#define TOOL_ADDR(n) (EEPROM_ADDR_TOOL_TABLE + n * (sizeof(tool_data_t) + 1))
#define PARAMETER_ADDR(n) (EEPROM_ADDR_PARAMETERS + n * (sizeof(coord_data_t) + 1))
#define STARTLINE_ADDR(n) (EEPROM_ADDR_STARTUP_BLOCK + n * (MAX_STORED_LINE_LENGTH + 1))

static const emap_t target[] = {
    {EEPROM_ADDR_GLOBAL, EEPROM_GRP_GLOBAL, 0},
#ifdef N_TOOLS
    {TOOL_ADDR(0), EEPROM_GRP_TOOLS, 0},
    {TOOL_ADDR(1), EEPROM_GRP_TOOLS, 1},
    {TOOL_ADDR(2), EEPROM_GRP_TOOLS, 2},
    {TOOL_ADDR(3), EEPROM_GRP_TOOLS, 3},
    {TOOL_ADDR(4), EEPROM_GRP_TOOLS, 4},
    {TOOL_ADDR(5), EEPROM_GRP_TOOLS, 5},
    {TOOL_ADDR(6), EEPROM_GRP_TOOLS, 6},
    {TOOL_ADDR(7), EEPROM_GRP_TOOLS, 7},
#if N_TOOL > 8
#error Increase number of tool entries!
#endif
#endif
    {PARAMETER_ADDR(0), EEPROM_GRP_PARAMETERS, 0},
    {PARAMETER_ADDR(1), EEPROM_GRP_PARAMETERS, 1},
    {PARAMETER_ADDR(2), EEPROM_GRP_PARAMETERS, 2},
    {PARAMETER_ADDR(3), EEPROM_GRP_PARAMETERS, 3},
    {PARAMETER_ADDR(4), EEPROM_GRP_PARAMETERS, 4},
    {PARAMETER_ADDR(5), EEPROM_GRP_PARAMETERS, 5},
    {PARAMETER_ADDR(6), EEPROM_GRP_PARAMETERS, 6},
    {PARAMETER_ADDR(7), EEPROM_GRP_PARAMETERS, 7},
    {PARAMETER_ADDR(8), EEPROM_GRP_PARAMETERS, 8},
    {PARAMETER_ADDR(9), EEPROM_GRP_PARAMETERS, 9},
    {PARAMETER_ADDR(10), EEPROM_GRP_PARAMETERS, 10},
    {PARAMETER_ADDR(11), EEPROM_GRP_PARAMETERS, 11},
#if N_COORDINATE_SYSTEM > 9
#error Increase number of parameter entries!
#endif
    {STARTLINE_ADDR(0), EEPROM_GRP_STARTUP, 0},
    {STARTLINE_ADDR(1), EEPROM_GRP_STARTUP, 1},
#if N_STARTUP_LINE > 2
#error Increase number of startup line entries!
#endif
    {EEPROM_ADDR_BUILD_INFO, EEPROM_GRP_BUILD, 0},
    {0, 0, 0} // list termination - do not remove
};

inline static uint8_t ram_get_byte (uint32_t addr)
{
    return noepromdata[addr];
}

inline static void ram_put_byte (uint32_t addr, uint8_t new_value)
{
    dirty = dirty || noepromdata[addr] != new_value;
    noepromdata[addr] = new_value;
}

// Extensions added as part of Grbl

static void memcpy_to_ram_with_checksum (uint32_t destination, uint8_t *source, uint32_t size)
{
    uint32_t dest = destination;
    uint8_t checksum = calc_checksum(source, size);

    dirty = false;

    for(; size > 0; size--)
        ram_put_byte(dest++, *(source++));

    ram_put_byte(dest, checksum);

    if(dirty) {

        uint8_t idx = 0;

        settings_dirty.is_dirty = dirty;

        if(hal.eeprom.driver_area.address && destination == hal.eeprom.driver_area.address)
            settings_dirty.driver_settings = true;

        else {

            do {
                if(target[idx].addr == destination)
                    break;
            } while(target[++idx].addr);

            if(target[idx].addr) switch(target[idx].type) {

                case EEPROM_GRP_GLOBAL:
                    settings_dirty.global_settings = true;
                    break;
#ifdef N_TOOLS
                case EEPROM_GRP_TOOLS:
                    settings_dirty.tool_data |= (1 << target[idx].offset);
                    break;
#endif
                case EEPROM_GRP_PARAMETERS:
                    settings_dirty.coord_data |= (1 << target[idx].offset);
                    break;

                case EEPROM_GRP_STARTUP:
                    settings_dirty.startup_lines |= (1 << target[idx].offset);
                    break;

                case EEPROM_GRP_BUILD:
                    settings_dirty.build_info = true;
                    break;
            }
        }
    }
}

static bool memcpy_from_ram_with_checksum (uint8_t *destination, uint32_t source, uint32_t size)
{
    uint8_t checksum = calc_checksum(&noepromdata[source], size);

    for(; size > 0; size--)
        *(destination++) = ram_get_byte(source++);

    return checksum == ram_get_byte(source);
}

//
// Try to allocate RAM for EEPROM emulation and switch over to RAM based copy
// Changes to RAM based copy will be written to EEPROM when Grbl is in IDLE state
// NOTE: enough free heap memory is required
bool eeprom_emu_init()
{
    if(hal.eeprom.size == 0)
        hal.eeprom.size = GRBL_EEPROM_SIZE;

    uint_fast16_t idx = hal.eeprom.size;

    memcpy(&physical_eeprom, &hal.eeprom, sizeof(eeprom_io_t)); // save pointers to physical EEPROM handler functions

    if((noepromdata = malloc(hal.eeprom.size)) != 0) {

        if(physical_eeprom.type == EEPROM_Physical) {

            // Initialize physical EEPROM on settings version mismatch
            if(physical_eeprom.get_byte(0) != SETTINGS_VERSION)
                settings_init();

            // Copy physical EEPROM content to RAM
            do {
                idx--;
                ram_put_byte(idx, physical_eeprom.get_byte(idx));
            } while(idx);
        } else if(hal.eeprom.memcpy_from_flash)
            hal.eeprom.memcpy_from_flash(noepromdata);

        // Switch hal to use RAM version of EEPROM data
        hal.eeprom.type = EEPROM_Emulated;
        hal.eeprom.get_byte = &ram_get_byte;
        hal.eeprom.put_byte = &ram_put_byte;
        hal.eeprom.memcpy_to_with_checksum = &memcpy_to_ram_with_checksum;
        hal.eeprom.memcpy_from_with_checksum = &memcpy_from_ram_with_checksum;

        // If no physical EEPROM available then import default settings to RAM
        if(physical_eeprom.type == EEPROM_None)
            settings_restore((settings_restore_t){0xFF});
    }

    // Clear settings dirty flags
    memset(&settings_dirty, 0, sizeof(settings_dirty_t));

    return noepromdata != 0;
}

// Write RAM changes to EEPROM
void eeprom_emu_sync_physical ()
{
    if(!settings_dirty.is_dirty)
        return;

    if(physical_eeprom.type == EEPROM_Physical) {

        if(settings_dirty.build_info) {
            settings_dirty.build_info = false;
            physical_eeprom.memcpy_to_with_checksum(EEPROM_ADDR_BUILD_INFO, (uint8_t *)(noepromdata + EEPROM_ADDR_BUILD_INFO), MAX_STORED_LINE_LENGTH);
        }

        if(settings_dirty.global_settings) {
            settings_dirty.global_settings = false;
            physical_eeprom.memcpy_to_with_checksum(EEPROM_ADDR_GLOBAL, (uint8_t *)(noepromdata + EEPROM_ADDR_GLOBAL), sizeof(settings_t));
        }

        uint_fast8_t idx = N_STARTUP_LINE, offset;
        if(settings_dirty.startup_lines) do {
            idx--;
            if(bit_istrue(settings_dirty.startup_lines, bit(idx))) {
                bit_false(settings_dirty.startup_lines, bit(idx));
                offset = EEPROM_ADDR_STARTUP_BLOCK + idx * (MAX_STORED_LINE_LENGTH + 1);
                physical_eeprom.memcpy_to_with_checksum(offset, (uint8_t *)(noepromdata + offset), MAX_STORED_LINE_LENGTH);
            }
        } while(idx);

        idx = SETTING_INDEX_NCOORD;
        if(settings_dirty.coord_data) do {
            if(bit_istrue(settings_dirty.coord_data, bit(idx))) {
                bit_false(settings_dirty.coord_data, bit(idx));
                offset = EEPROM_ADDR_PARAMETERS + idx * (sizeof(coord_data_t) + 1);
                physical_eeprom.memcpy_to_with_checksum(offset, (uint8_t *)(noepromdata + offset), sizeof(coord_data_t));
            }
        } while(idx--);

        if(settings_dirty.driver_settings) {
            settings_dirty.driver_settings = false;
            if(hal.eeprom.driver_area.size > 0)
                physical_eeprom.memcpy_to_with_checksum(hal.eeprom.driver_area.address, (uint8_t *)(noepromdata + hal.eeprom.driver_area.address), hal.eeprom.driver_area.size);
        }

#ifdef N_TOOLS
        idx = N_TOOLS;
        if(settings_dirty.tool_data) do {
            idx--;
            if(bit_istrue(settings_dirty.tool_data, bit(idx))) {
                bit_false(settings_dirty.tool_data, bit(idx));
                offset = EEPROM_ADDR_TOOL_TABLE + idx * (sizeof(tool_data_t) + 1);
                physical_eeprom.memcpy_to_with_checksum(offset, (uint8_t *)(noepromdata + offset), sizeof(tool_data_t));
            }
        } while(idx);
#endif

    } else if(hal.eeprom.memcpy_to_flash)
        hal.eeprom.memcpy_to_flash(noepromdata);

    settings_dirty.is_dirty = false;
}
