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
#include "../../driver.h"
#else
#include "driver.h"
#endif

#if KEYPAD_ENABLE

#include <string.h>

#include "keypad.h"

#ifdef ARDUINO
#include "../../i2c.h"
#include "../grbl/report.h"
#include "../grbl/override.h"
#else
#include "i2c.h"
#include "grbl/report.h"
#include "grbl/override.h"
#endif

#define KEYBUF_SIZE 16

static bool jogging = false, keyreleased = true;
static char keybuf_buf[KEYBUF_SIZE];
static jogmode_t jogMode = JogMode_Fast;
static volatile uint32_t keybuf_head = 0, keybuf_tail = 0;

status_code_t keypad_setting (setting_type_t setting, float value, char *svalue)
{
    status_code_t status = Status_Unhandled;

    switch(setting) {

        case Setting_JogStepSpeed:
            driver_settings.jog.step_speed = value;
            status = Status_OK;
            break;

        case Setting_JogSlowSpeed:
            driver_settings.jog.slow_speed = value;
            status = Status_OK;
            break;

        case Setting_JogFastSpeed:
            driver_settings.jog.fast_speed = value;
            status = Status_OK;
            break;

        case Setting_JogStepDistance:
            driver_settings.jog.step_distance = value;
            status = Status_OK;
            break;

        case Setting_JogSlowDistance:
            driver_settings.jog.slow_distance = value;
            status = Status_OK;
           break;

        case Setting_JogFastDistance:
            driver_settings.jog.fast_distance = value;
            status = Status_OK;
            break;

        default:
            break;
    }

    return status;
}

void keypad_settings_restore (void)
{
    driver_settings.jog.step_speed    = 100.0f;
    driver_settings.jog.slow_speed    = 600.0f;
    driver_settings.jog.fast_speed    = 3000.0f;
    driver_settings.jog.step_distance = 0.25f;
    driver_settings.jog.slow_distance = 500.0f;
    driver_settings.jog.fast_distance = 3000.0f;
}

void keypad_settings_report (setting_type_t setting)
{
    switch(setting) {

        case Setting_JogStepSpeed:
            report_float_setting(setting, driver_settings.jog.step_speed, 0);
            break;

        case Setting_JogSlowSpeed:
            report_float_setting(setting, driver_settings.jog.slow_speed, 0);
            break;

        case Setting_JogFastSpeed:
            report_float_setting(setting, driver_settings.jog.fast_speed, 0);
            break;

        case Setting_JogStepDistance:
            report_float_setting(setting, driver_settings.jog.step_distance, N_DECIMAL_SETTINGVALUE);
            break;

        case Setting_JogSlowDistance:
            report_float_setting(setting, driver_settings.jog.slow_distance, 0);
           break;

        case Setting_JogFastDistance:
            report_float_setting(setting, driver_settings.jog.fast_distance, 0);
            break;

        default:
            break;
    }
}

void keypad_enqueue_keycode (char c)
{
    uint32_t bptr = (keybuf_head + 1) & (KEYBUF_SIZE - 1);    // Get next head pointer

    if(bptr != keybuf_tail) {           // If not buffer full
        keybuf_buf[keybuf_head] = c;    // add data to buffer
        keybuf_head = bptr;             // and update pointer
    }
}

// Returns 0 if no keycode enqueued
static char keypad_get_keycode (void)
{
    uint32_t data = 0, bptr = keybuf_tail;

    if(bptr != keybuf_head) {
        data = keybuf_buf[bptr++];               // Get next character, increment tmp pointer
        keybuf_tail = bptr & (KEYBUF_SIZE - 1);  // and update pointer
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

void keypad_process_keypress (uint_fast16_t state)
{
    bool addedGcode, jogCommand = false;
    char command[30] = "", keycode = keypad_get_keycode();

    if(state == STATE_ESTOP)
        return;

    if(keycode)
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
                strrepl(command, '?', ftoa(driver_settings.jog.slow_distance, 0));
                strcat(command, ftoa(driver_settings.jog.slow_speed, 0));
                break;

            case JogMode_Step:
                strrepl(command, '?', ftoa(driver_settings.jog.step_distance, 3));
                strcat(command, ftoa(driver_settings.jog.step_speed, 0));
                break;

            default:
                strrepl(command, '?', ftoa(driver_settings.jog.fast_distance, 0));
                strcat(command, ftoa(driver_settings.jog.fast_speed, 0));
                break;

        }

        if(!(jogCommand && keyreleased)) { // key still pressed? - do not execute jog command if released!
            addedGcode = hal.protocol_enqueue_gcode((char *)command);
            jogging = jogging || (jogCommand && addedGcode);
        }
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
        keybuf_tail = keybuf_head = 0; // flush keycode buffer
    }
}

#endif
