/*
  mks_sbase.c - driver code for LPC176x processor, MKS SBASE V1.3 board

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

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

#include "driver.h"

#ifdef BOARD_MKS_SBASE_13

#include <math.h>
#include <string.h>

#include "i2c.h"
#include "grbl/hal.h"
#include "grbl/nvs_buffer.h"
#include "grbl/settings.h"

typedef struct {
    float current; // mA
} motor_settings_t;

typedef struct {
    motor_settings_t driver[N_AXIS];
} mks_settings_t;

static mks_settings_t mks;
static driver_setting_ptrs_t driver_settings;

static const setting_detail_t mks_settings[] = {
    { Setting_AxisStepperCurrentBase, Group_Axis0, "?-axis motor current", "A", Format_Decimal, "0.0#", "0", "2.2" }
};

static setting_details_t details = {
    .settings = mks_settings,
    .n_settings = sizeof(mks_settings) / sizeof(setting_detail_t)
};

static setting_details_t *on_report_settings (void)
{
    return &details;
}

static void mks_set_current (uint_fast8_t axis, float current)
{
    static const uint8_t wiper_registers[] = {0x09, 0x01, 0x06, 0x07};

    //TODO: add support for B axis?

    uint8_t cmd[2];

    cmd[0] = wiper_registers[axis] << 4;
    cmd[1] = (uint8_t)min(255.0f, roundf(current * 113.33f));

    i2c_write(MCP44XX_I2C_ADDR, cmd, sizeof(cmd));
}

// Parse and set driver specific parameters
static status_code_t mks_setting (setting_id_t setting, float value, char *svalue)
{
    status_code_t status = Status_OK;

    if((setting_id_t)setting >= Setting_AxisSettingsBase && (setting_id_t)setting <= Setting_AxisSettingsMax) {

        uint_fast16_t base_idx = (uint_fast16_t)setting - (uint_fast16_t)Setting_AxisSettingsBase;
        uint_fast8_t idx = base_idx % AXIS_SETTINGS_INCREMENT;

        if(idx < N_AXIS) switch((base_idx - idx) / AXIS_SETTINGS_INCREMENT) {

            case AxisSetting_StepperCurrent:
                mks.driver[idx].current = value;
                mks_set_current(idx, value);
                break;

            default:
                status = Status_Unhandled;
                break;
        }
    } else
        status = Status_Unhandled;

    if(status == Status_OK)
        hal.nvs.memcpy_to_nvs(driver_settings.nvs_address, (uint8_t *)&mks, sizeof(mks_settings_t), true);

    return status == Status_Unhandled && driver_settings.set ? driver_settings.set(setting, value, svalue) : status;
}

// Initialize default EEPROM settings
static void mks_settings_restore (void)
{
    uint_fast8_t idx = N_AXIS;

    do {
        mks.driver[--idx].current = 0.6f;
    } while(idx);

    hal.nvs.memcpy_to_nvs(driver_settings.nvs_address, (uint8_t *)&mks, sizeof(mks_settings_t), true);

    if(driver_settings.restore)
        driver_settings.restore();
}

static void mks_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&mks, driver_settings.nvs_address, sizeof(mks_settings_t), true) != NVS_TransferResult_OK)
        mks_settings_restore();

    uint_fast8_t idx = N_AXIS;

    do {
        idx--;
        mks_set_current(idx, mks.driver[idx].current);
    } while(idx);

    if(driver_settings.load)
        driver_settings.load();
}

static void mks_axis_settings_report (axis_setting_id_t setting, uint8_t axis_idx)
{
    bool reported = true;
    setting_id_t basetype = (setting_id_t)(Setting_AxisSettingsBase + setting * AXIS_SETTINGS_INCREMENT);

    switch(setting) {

        case AxisSetting_StepperCurrent:
            report_float_setting((setting_id_t)(basetype + axis_idx), mks.driver[axis_idx].current, 1);
            break;

        default:
            reported = false;
            break;
    }

    if(!reported && driver_settings.axis_report)
        driver_settings.axis_report(setting, axis_idx);
}

void board_init (void)
{
    if((hal.driver_settings.nvs_address = nvs_alloc(sizeof(mks_settings_t)))) {

        memcpy(&driver_settings, &hal.driver_settings, sizeof(driver_setting_ptrs_t));

        hal.driver_settings.set = mks_setting;
        hal.driver_settings.axis_report = mks_axis_settings_report;
        hal.driver_settings.load = mks_settings_load;
        hal.driver_settings.restore = mks_settings_restore;

        details.on_report_settings = grbl.on_report_settings;
        grbl.on_report_settings = on_report_settings;
    }
}

#endif
