/*

  my_plugin.c - user defined plugin template with settings handling

  Part of grblHAL

  Copyright (c) 2020 Terje Io

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
 * NOTE: this plugin does not add any other fuctionality than settings handling, attach to other HAL entry points to provide that.
 *       See the mcode.c template or standard plugins for how to do this.
 */

#ifdef ARDUINO
#include "../grbl/hal.h"
#include "../grbl/report.h"
#include "../grbl/report.h"
#include "../grbl/nvs_buffer.h"
#else
#include "grbl/hal.h"
#include "grbl/report.h"
#include "grbl/nvs_buffer.h"
#include "grbl/nuts_bolts.h"
#endif

typedef struct {
    float fvalue;
    uint32_t ivalue;
} plugin_settings_t;

static driver_setting_ptrs_t driver_settings;
static on_report_options_ptr on_report_options;
static plugin_settings_t my_settings;

// Add info about our settings for $help and enumerations potentially used by senders for settings UI.
// This is optional and can be removed.
static const setting_group_detail_t user_groups [] = {
    { Group_Root, Group_UserSettings, "My settings"}
};

static const setting_detail_t user_settings[] = {
    { Setting_UserDefined_0, Group_UserSettings, "My setting 1", NULL, Format_Decimal, "#0.0", "0", "15" },
    { Setting_UserDefined_1, Group_UserSettings, "My setting 2", "milliseconds", Format_Integer, "####0", "50", "250" }
};

static setting_details_t details = {
    .groups = user_groups,
    .n_groups = sizeof(user_groups) / sizeof(setting_group_detail_t),
    .settings = user_settings,
    .n_settings = sizeof(user_settings) / sizeof(setting_detail_t)
};

static setting_details_t *on_report_settings (void)
{
    return &details;
}

// Set a settings value.
// Call set() function of next plugin in chain if not our setting.
static status_code_t plugin_settings_set (setting_type_t setting, float value, char *svalue)
{
    status_code_t status = Status_OK;

    switch(setting) {

        case Setting_UserDefined_0:
            my_settings.fvalue = value;
            break;

        case Setting_UserDefined_1:
            // Return error if not integer or not in range 0 - 15
            if(isintf(value)) {
                if(value >= 0.0f && value <= 15.0f)
                    my_settings.ivalue = (uint32_t)value;
                else
                    status = Status_GcodeValueOutOfRange;
            } else
                status = Status_BadNumberFormat;
            break;

        default:
            status = Status_Unhandled;
            break;
    }

    if(status == Status_OK)
        hal.nvs.memcpy_to_nvs(driver_settings.nvs_address, (uint8_t *)&my_settings, sizeof(plugin_settings_t), true);

    return status == Status_Unhandled && driver_settings.set ? driver_settings.set(setting, value, svalue) : status;
}

// Report settings value when called for.
// Call report() function of next plugin in chain if not our setting.
static void plugin_settings_report (setting_type_t setting)
{
    bool reported = true;

    switch(setting) {

        case Setting_UserDefined_0:
            report_float_setting(setting, my_settings.fvalue, 1);
            break;

        case Setting_UserDefined_1:
            report_uint_setting(setting, my_settings.ivalue);
            break;

        default:
            reported = false;
            break;
    }

    if(!reported && driver_settings.report)
        driver_settings.report(setting);
}

// Restore default settings and write to non volatile storage (NVS).
// Call restore() function of next plugin in chain.
static void plugin_settings_restore (void)
{
    my_settings.fvalue = 3.1f;
    my_settings.ivalue = 2;

    hal.nvs.memcpy_to_nvs(driver_settings.nvs_address, (uint8_t *)&my_settings, sizeof(plugin_settings_t), true);

    if(driver_settings.restore)
        driver_settings.restore();
}

// Load our settings from non volatile storage (NVS).
// If load fails restore to default values.
// Call load() function of next plugin in chain.
static void plugin_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&my_settings, driver_settings.nvs_address, sizeof(plugin_settings_t), true) != NVS_TransferResult_OK)
        plugin_settings_restore();

    if(driver_settings.load)
        driver_settings.load();
}

// Add info about our plugin to the $I report.
static void on_report_options (void)
{
    on_report_options();
    hal.stream.write("[PLUGIN:My plugin v1.01]"  ASCII_EOL);
}

// A call my_plugin_init will be issued automatically at startup.
// There is no need to change any source code elsewhere.
void my_plugin_init (void)
{
    // Try to allocate space for our settings in non volatile storage (NVS).
    // If successful make a copy of current settings handlers and add ours.
    if((hal.driver_settings.nvs_address = nvs_alloc(sizeof(plugin_settings_t)))) {
        memcpy(&driver_settings, &hal.driver_settings, sizeof(driver_setting_ptrs_t));
        hal.driver_settings.set = plugin_settings_set;
        hal.driver_settings.report = plugin_settings_report;
        hal.driver_settings.load = plugin_settings_load;
        hal.driver_settings.restore = plugin_settings_restore;

        // Add info about our plugin to the $I report.
        on_report_options = grbl.on_report_options;
        grbl.on_report_options = on_report_options;

        // Add info about our settings for $help and enumerations potentially used by senders for settings UI.
        // This is optional and can be removed.
        details.on_report_settings = grbl.on_report_settings;
        grbl.on_report_settings = on_report_settings;

        // "Hook" into other HAL pointers here to provide functionality.
    }
}
