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
static nvs_address_t nvs_address;

static void mks_settings_restore (void);
static void mks_settings_load (void);
static status_code_t set_axis_setting (setting_id_t setting, float value);
static float get_axis_setting (setting_id_t setting);

static const setting_detail_t mks_settings[] = {
    { Setting_AxisStepperCurrent, Group_Axis0, "?-axis motor current", "A", Format_Decimal, "0.0#", "0", "2.2", Setting_NonCoreFn, set_axis_setting, get_axis_setting }
};

static void mks_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&mks, sizeof(mks_settings_t), true);
}

static setting_details_t details = {
    .settings = mks_settings,
    .n_settings = sizeof(mks_settings) / sizeof(setting_detail_t),
    .load = mks_settings_load,
    .save = mks_settings_save,
    .restore = mks_settings_restore
};

static setting_details_t *on_get_settings (void)
{
    return &details;
}

static void mks_set_current (uint_fast8_t axis, float current)
{
    static const uint8_t wiper_registers[] = {0x00, 0x01, 0x06, 0x07};

    //TODO: add support for B axis?

    uint8_t cmd[2];

    cmd[0] = wiper_registers[axis] << 4;
    cmd[1] = (uint8_t)min(255.0f, roundf(current * 100.85f));

    i2c_write(MCP44XX_I2C_ADDR, cmd, sizeof(cmd));
}

// Parse and set driver specific parameters
static status_code_t set_axis_setting (setting_id_t setting, float value)
{
    uint_fast8_t idx;
    status_code_t status = Status_OK;

    switch(settings_get_axis_base(setting, &idx)) {

        case Setting_AxisStepperCurrent:
            mks.driver[idx].current = value;
            mks_set_current(idx, value);
            break;

        default:
            status = Status_Unhandled;
            break;
    }


    return status;
}

static float get_axis_setting (setting_id_t setting)
{
    float value = 0.0f;
    uint_fast8_t idx;

    switch(settings_get_axis_base(setting, &idx)) {

        case Setting_AxisStepperCurrent:
            value = mks.driver[idx].current;
            break;

        default:
            break;
    }

    return value;
}


// Initialize default EEPROM settings
static void mks_settings_restore (void)
{
    uint_fast8_t idx = N_AXIS;

    do {
        mks.driver[--idx].current = 0.6f;
    } while(idx);

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&mks, sizeof(mks_settings_t), true);
}

static void mks_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&mks, nvs_address, sizeof(mks_settings_t), true) != NVS_TransferResult_OK)
        mks_settings_restore();

    uint_fast8_t idx = N_AXIS;

    do {
        idx--;
        mks_set_current(idx, mks.driver[idx].current);
    } while(idx);
}

void board_init (void)
{
    if((nvs_address = nvs_alloc(sizeof(mks_settings_t)))) {

        details.on_get_settings = grbl.on_get_settings;
        grbl.on_get_settings = on_get_settings;
    }
}

#endif
