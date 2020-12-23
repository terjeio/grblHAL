/*
  keypad.c - I2C keypad plugin

  Part of GrblHAL

  Copyright (c) 2017-2020 Terje Io

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


#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#if KEYPAD_ENABLE

#include <string.h>

#include "keypad.h"

#ifdef ARDUINO
#include "../i2c.h"
#include "../grbl/report.h"
#include "../grbl/override.h"
#include "../grbl/protocol.h"
#include "../grbl/nvs_buffer.h"
#else
#include "i2c.h"
#include "grbl/report.h"
#include "grbl/override.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#endif

typedef struct {
    char buf[KEYBUF_SIZE];
    volatile uint_fast8_t head;
    volatile uint_fast8_t tail;
} keybuffer_t;

static bool jogging = false, keyreleased = true;
static jogmode_t jogMode = JogMode_Fast;
static jog_settings_t jog;
static keybuffer_t keybuf = {0};
static driver_setting_ptrs_t driver_settings;
static on_report_options_ptr on_report_options;

keypad_t keypad = {0};
/*
static const setting_group_detail_t keypad_groups [] = {
    { Group_Root, Group_Jogging, "Jogging"}
};
*/
static const setting_detail_t keypad_settings[] = {
    { Setting_JogStepSpeed, Group_Jogging, "Step jog speed", "mm/min", Format_Decimal, "###0.0", NULL, NULL },
    { Setting_JogSlowSpeed, Group_Jogging, "Slow jog speed", "mm/min", Format_Decimal, "###0.0", NULL, NULL },
    { Setting_JogFastSpeed, Group_Jogging, "Fast jog speed", "mm/min", Format_Decimal, "###0.0", NULL, NULL },
    { Setting_JogStepDistance, Group_Jogging, "Step jog distance", "mm", Format_Decimal, "#0.000", NULL, NULL },
    { Setting_JogSlowDistance, Group_Jogging, "Slow jog distance", "mm", Format_Decimal, "###0.0", NULL, NULL },
    { Setting_JogFastDistance, Group_Jogging, "Fast jog distance", "mm", Format_Decimal, "###0.0", NULL, NULL }
};

static setting_details_t details = {
//    .groups = keypad_groups,
//    .n_groups = sizeof(keypad_groups) / sizeof(setting_group_detail_t),
    .settings = keypad_settings,
    .n_settings = sizeof(keypad_settings) / sizeof(setting_detail_t)
};

static setting_details_t *onReportSettings (void)
{
    return &details;
}

static status_code_t keypad_settings_set (setting_type_t setting, float value, char *svalue)
{
    status_code_t status = Status_OK;

    switch(setting) {

        case Setting_JogStepSpeed:
            jog.step_speed = value;
            break;

        case Setting_JogSlowSpeed:
            jog.slow_speed = value;
            break;

        case Setting_JogFastSpeed:
            jog.fast_speed = value;
            break;

        case Setting_JogStepDistance:
            jog.step_distance = value;
            break;

        case Setting_JogSlowDistance:
            jog.slow_distance = value;
           break;

        case Setting_JogFastDistance:
            jog.fast_distance = value;
            break;

        default:
            status = Status_Unhandled;
            break;
    }


    if(status == Status_OK)
        hal.nvs.memcpy_to_nvs(driver_settings.nvs_address, (uint8_t *)&jog, sizeof(jog_settings_t), true);

    return status == Status_Unhandled && driver_settings.set ? driver_settings.set(setting, value, svalue) : status;
}

static void keypad_settings_report (setting_type_t setting)
{
    bool reported = true;

    switch(setting) {

        case Setting_JogStepSpeed:
            report_float_setting(setting, jog.step_speed, 0);
            break;

        case Setting_JogSlowSpeed:
            report_float_setting(setting, jog.slow_speed, 0);
            break;

        case Setting_JogFastSpeed:
            report_float_setting(setting, jog.fast_speed, 0);
            break;

        case Setting_JogStepDistance:
            report_float_setting(setting, jog.step_distance, N_DECIMAL_SETTINGVALUE);
            break;

        case Setting_JogSlowDistance:
            report_float_setting(setting, jog.slow_distance, 0);
            break;

        case Setting_JogFastDistance:
            report_float_setting(setting, jog.fast_distance, 0);
            break;

        default:
            reported = false;
            break;
    }

    if(!reported && driver_settings.report)
        driver_settings.report(setting);
}

static void keypad_settings_restore (void)
{
    jog.step_speed    = 100.0f;
    jog.slow_speed    = 600.0f;
    jog.fast_speed    = 3000.0f;
    jog.step_distance = 0.25f;
    jog.slow_distance = 500.0f;
    jog.fast_distance = 3000.0f;

    hal.nvs.memcpy_to_nvs(driver_settings.nvs_address, (uint8_t *)&jog, sizeof(jog_settings_t), true);

    if(driver_settings.restore)
        driver_settings.restore();
}

static void keypad_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&jog, driver_settings.nvs_address, sizeof(jog_settings_t), true) != NVS_TransferResult_OK)
        keypad_settings_restore();

    if(driver_settings.load)
        driver_settings.load();
}

// Returns 0 if no keycode enqueued
static char keypad_get_keycode (void)
{
    uint32_t data = 0, bptr = keybuf.tail;

    if(bptr != keybuf.head) {
        data = keybuf.buf[bptr++];               // Get next character, increment tmp pointer
        keybuf.tail = bptr & (KEYBUF_SIZE - 1);  // and update pointer
    }

    return data;
}

// BE WARNED: this function may be dangerous to use...
static char *strrepl (char *str, int c, char *str3)
{
    char tmp[30];
    char *s = strrchr(str, c);

    while(s) {
        strcpy(tmp, str3);
        strcat(tmp, s + 1);
        strcpy(s, tmp);
        s = strrchr(str, c);
    }

    return str;
}

static void keypad_process_keypress (uint_fast16_t state)
{
    bool addedGcode, jogCommand = false;
    char command[30] = "", keycode = keypad_get_keycode();

    if(state == STATE_ESTOP)
        return;

    if(keycode) {

        if(keypad.on_keypress_preview && keypad.on_keypress_preview(keycode, state))
            return;

        switch(keycode) {

            case 'M':                                   // Mist override
                enqueue_accessory_override(CMD_OVERRIDE_COOLANT_MIST_TOGGLE);
                break;

            case 'C':                                   // Coolant override
                enqueue_accessory_override(CMD_OVERRIDE_COOLANT_FLOOD_TOGGLE);
                break;

            case CMD_FEED_HOLD_LEGACY:                  // Feed hold
                hal.stream.enqueue_realtime_command(CMD_FEED_HOLD);
                break;

            case CMD_CYCLE_START_LEGACY:                // Cycle start
                hal.stream.enqueue_realtime_command(CMD_CYCLE_START);
                break;

            case '0':
            case '1':
            case '2':                                   // Set jog mode
                jogMode = (jogmode_t)(keycode - '0');
                break;

            case 'h':                                   // "toggle" jog mode
                jogMode = jogMode == JogMode_Step ? JogMode_Fast : (jogMode == JogMode_Fast ? JogMode_Slow : JogMode_Step);
                if(keypad.on_jogmode_changed)
                    keypad.on_jogmode_changed(jogMode);
                break;

            case 'H':                                   // Home axes
                strcpy(command, "$H");
                break;

            case JOG_XR:                                // Jog X
                strcpy(command, "$J=G91X?F");
                break;

            case JOG_XL:                                // Jog -X
                strcpy(command, "$J=G91X-?F");
                break;

            case JOG_YF:                                // Jog Y
                strcpy(command, "$J=G91Y?F");
                break;

            case JOG_YB:                                // Jog -Y
                strcpy(command, "$J=G91Y-?F");
                break;

            case JOG_ZU:                                // Jog Z
                strcpy(command, "$J=G91Z?F");
                break;

            case JOG_ZD:                                // Jog -Z
                strcpy(command, "$J=G91Z-?F");
                break;

            case JOG_XRYF:                              // Jog XY
                strcpy(command, "$J=G91X?Y?F");
                break;

            case JOG_XRYB:                              // Jog X-Y
                strcpy(command, "$J=G91X?Y-?F");
                break;

            case JOG_XLYF:                              // Jog -XY
                strcpy(command, "$J=G91X-?Y?F");
                break;

            case JOG_XLYB:                              // Jog -X-Y
                strcpy(command, "$J=G91X-?Y-?F");
                break;

            case JOG_XRZU:                              // Jog XZ
                strcpy(command, "$J=G91X?Z?F");
                break;

            case JOG_XRZD:                              // Jog X-Z
                strcpy(command, "$J=G91X?Z-?F");
                break;

            case JOG_XLZU:                              // Jog -XZ
                strcpy(command, "$J=G91X-?Z?F");
                break;

            case JOG_XLZD:                              // Jog -X-Z
                strcpy(command, "$J=G91X-?Z-?F");
                break;
        }

        if(command[0] != '\0') {

            // add distance and speed to jog commands
            if((jogCommand = (command[0] == '$' && command[1] == 'J'))) switch(jogMode) {

                case JogMode_Slow:
                    strrepl(command, '?', ftoa(jog.slow_distance, 0));
                    strcat(command, ftoa(jog.slow_speed, 0));
                    break;

                case JogMode_Step:
                    strrepl(command, '?', ftoa(jog.step_distance, 3));
                    strcat(command, ftoa(jog.step_speed, 0));
                    break;

                default:
                    strrepl(command, '?', ftoa(jog.fast_distance, 0));
                    strcat(command, ftoa(jog.fast_speed, 0));
                    break;

            }

            if(!(jogCommand && keyreleased)) { // key still pressed? - do not execute jog command if released!
                addedGcode = grbl.protocol_enqueue_gcode((char *)command);
                jogging = jogging || (jogCommand && addedGcode);
            }
        }
    }
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:KEYPAD v1.00]"  ASCII_EOL);
}

bool keypad_init (void)
{
    if((hal.driver_settings.nvs_address = nvs_alloc(sizeof(jog_settings_t)))) {
        memcpy(&driver_settings, &hal.driver_settings, sizeof(driver_setting_ptrs_t));
        hal.driver_settings.set = keypad_settings_set;
        hal.driver_settings.report = keypad_settings_report;
        hal.driver_settings.load = keypad_settings_load;
        hal.driver_settings.restore = keypad_settings_restore;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        details.on_report_settings = grbl.on_report_settings;
        grbl.on_report_settings = onReportSettings;

        if(keypad.on_jogmode_changed)
            keypad.on_jogmode_changed(jogMode);
    }

    return driver_settings.nvs_address != 0;
}

ISR_CODE void keypad_enqueue_keycode (char c)
{
    uint32_t bptr = (keybuf.head + 1) & (KEYBUF_SIZE - 1);    // Get next head pointer

    if(bptr != keybuf.tail) {           // If not buffer full
        keybuf.buf[keybuf.head] = c;    // add data to buffer
        keybuf.head = bptr;             // and update pointer
        // Tell foreground process to process keycode
        if(hal.driver_settings.nvs_address != 0)
            protocol_enqueue_rt_command(keypad_process_keypress);
    }
}

ISR_CODE void keypad_keyclick_handler (bool keydown)
{
    keyreleased = !keydown;

    if(keydown)
        I2C_GetKeycode(KEYPAD_I2CADDR, keypad_enqueue_keycode);

    else if(jogging) {
        jogging = false;
        hal.stream.enqueue_realtime_command(CMD_JOG_CANCEL);
        keybuf.tail = keybuf.head; // flush keycode buffer
    }
}

#endif
