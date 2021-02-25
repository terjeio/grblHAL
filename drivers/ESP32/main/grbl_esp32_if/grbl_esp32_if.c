/*
  grbl_esp32_if.c - adds Grbl_Esp32 commands to grblHAL

  Part of grblHAL

  Copyright (c) 2021 Terje Io

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
 NOTE: work in progress, code has experimental status and is currently unused
 */

#include <string.h>
#include <cJSON.h>
#include <stdlib.h>

#include "grbl/hal.h"
#include "networking/strutils.h"

typedef status_code_t (*set_setting_ptr)(setting_id_t id, char* value);

typedef enum {
    ESP32_GRBL = 1,  // Classic GRBL settings like $100
    ESP32_EXTENDED,  // Settings added by early versions of Grbl_Esp32
    ESP32_WEBSET,    // Settings for ESP3D_WebUI, stored in NVS
    ESP32_GRBLCMD,   // Non-persistent GRBL commands like $H
    ESP32_WEBCMD,    // ESP3D_WebUI commands that are not directly settings
} grbl_esp32_type_t;

typedef enum {
    ESP32_WG,  // Readable and writable as guest
    ESP32_WU,  // Readable and writable as user and admin
    ESP32_WA,  // Readable as user and admin, writable as admin
} grbl_esp32_permissions_t;

typedef enum {
    ESPCmd_GetSetSTA_SSID = 100,
    ESPCmd_GetSetSTA_Password = 101,
    ESPCmd_GetSetSTA_IPMode = 102,
    ESPCmd_GetSetSTA_IP = 103,
    ESPCmd_GetSetAP_SSID = 105,
    ESPCmd_GetSetAP_Password = 106,
    ESPCmd_GetSetAP_IP = 107,
    ESPCmd_GetSetAP_Channel = 108,
    ESPCmd_GetSetRadioMode = 110,
    ESPCmd_GetCurrentIP = 111,
    ESPCmd_GetSetHostname = 112,
    //ESPCmd_GetSetRadioOnOff = 115,
    ESPCmd_GetSetHTTPOnOff = 120,
    ESPCmd_GetSetHttpPort = 121,
    ESPCmd_GetSetTelnetOnOff = 130,
    ESPCmd_GetSetTelnetPort = 131,
    ESPCmd_GetSetBluetoothName = 140,
    ESPCmd_GetSDCardStatus = 200,
    ESPCmd_GetSDCardContent = 210,
    //ESPCmd_DeleteSDCardFile = 215,
    ESPCmd_PrintSD = 220,
    ESPCmd_GetSettings = 400,
    ESPCmd_SetEEPROMSetting = 401,
    ESPCmd_GetAPList = 410,
    ESPCmd_GetStatus = 420,
    ESPCmd_Reboot = 444,
    //ESPCmd_SetUserPassword = 555,
    //ESPCmd_SendMessage = 600,
    //ESPCmd_GetSetNotifications = 600,
    ESPCmd_ReadLocalFile = 700,
    ESPCmd_FormatFlashFS = 710,
    ESPCmd_GetFlashFSCapacity = 720,
    ESPCmd_GetFirmwareSpec = 800
} esp_cmd_t;

typedef struct grbl_esp32_setting {
    setting_id_t id;
    grbl_esp32_type_t type;
    grbl_esp32_permissions_t permissions;
    const char *name;
    esp_cmd_t esp_cmd;
    status_code_t (*execute)(const struct grbl_esp32_setting *setting, char *value);
} grbl_esp32_command_t;

typedef struct
{
    cJSON *parent;
    const grbl_esp32_command_t *esp_setting;
} grbl_esp32_json_t;

status_code_t set_setting (const grbl_esp32_command_t *setting, char* value);
status_code_t list_settings (const grbl_esp32_command_t *setting, char *value);
status_code_t list_settings_all (const grbl_esp32_command_t *setting, char *value);
status_code_t list_settings_webui (const grbl_esp32_command_t *setting, char *value);

static char cmdbuf[40];
static on_unknown_sys_command_ptr on_unknown_sys_command;
static const char *tmap[] = { "B", "B", "B", "B", "I", "I", "B", "S", "B", "A", "I", "I" };
static const grbl_esp32_command_t grbl_esp32_commands[] = {
    { Setting_PWMMaxValue, ESP32_GRBL, ESP32_WA, "Spindle/PWM/Max",0 , set_setting },
    { Setting_PWMMinValue, ESP32_GRBL, ESP32_WA, "Spindle/PWM/Min", 0, set_setting },
    { Setting_PWMFreq, ESP32_GRBL, ESP32_WA, "Spindle/PWM/Frequency", 0, set_setting },
    { Setting_SpindleInvertMask, ESP32_GRBL, ESP32_WA, "Spindle/PWM/Invert", 0, set_setting },
/*    {, "Spindle/Delay/SpinUp" },
    {, "Spindle/Delay/SpinDown" }, */
    { Setting_SpindlePWMBehaviour, ESP32_GRBL, ESP32_WA, "Spindle/Enable/OffWithSpeed", 0, set_setting },
    { Setting_SpindleInvertMask, ESP32_GRBL, ESP32_WA, "Spindle/Enable/Invert", 0, set_setting },
    { Setting_SettingsMax, ESP32_GRBL, ESP32_WA, "GCode/Line0", 0, set_setting },
    { Setting_SettingsMax, ESP32_GRBL, ESP32_WA, "GCode/Line1", 0, set_setting },
    { Setting_Mode, ESP32_GRBL, ESP32_WA, "GCode/LaserMode", 0, set_setting },
/*    {, "Laser/FullPower" }, */
    { Setting_RpmMin, ESP32_GRBL, ESP32_WA, "GCode/MinS", 0, set_setting },
    { Setting_RpmMax, ESP32_GRBL, ESP32_WA, "GCode/MaxS", 0, set_setting },
    { Setting_HomingPulloff, ESP32_GRBL, ESP32_WA, "Homing/Pulloff", 0, set_setting },
    { Setting_HomingDebounceDelay, ESP32_GRBL, ESP32_WA, "Homing/Debounce", 0, set_setting },
    { Setting_HomingSeekRate, ESP32_GRBL, ESP32_WA, "Homing/Seek", 0, set_setting },
    { Setting_HomingFeedRate, ESP32_GRBL, ESP32_WA, "Homing/Feed", 0, set_setting },
/*    {, "Homing/Squared" }, */
    { Setting_HomingDirMask, ESP32_GRBL, ESP32_WA, "Homing/DirInvert", 0, set_setting },
    { Setting_HomingEnable, ESP32_GRBL, ESP32_WA, "Homing/Enable", 0, set_setting },
    { Setting_HardLimitsEnable, ESP32_GRBL, ESP32_WA, "Limits/Hard", 0, set_setting },
    { Setting_SoftLimitsEnable, ESP32_GRBL, ESP32_WA, "Limits/Soft", 0, set_setting },
/*    {"Firmware/Build" }, */
    { Setting_ReportInches, ESP32_GRBL, ESP32_WA, "Report/Inches", 0, set_setting },
    { Setting_ArcTolerance, ESP32_GRBL, ESP32_WA, "GCode/ArcTolerance", 0, set_setting },
    { Setting_JunctionDeviation, ESP32_GRBL, ESP32_WA, "GCode/JunctionDeviation", 0, set_setting },
    { Setting_StatusReportMask, ESP32_GRBL, ESP32_WA, "Report/Status", 0, set_setting },
    { Setting_InvertProbePin, ESP32_GRBL, ESP32_WA, "Probe/Invert", 0, set_setting },
    { Setting_LimitPinsInvertMask, ESP32_GRBL, ESP32_WA, "Limits/Invert", 0, set_setting },
    { Setting_InvertStepperEnable, ESP32_GRBL, ESP32_WA, "Stepper/EnableInvert", 0, set_setting },
    { Setting_DirInvertMask, ESP32_GRBL, ESP32_WA, "Stepper/DirInvert", 0, set_setting },
    { Setting_StepInvertMask, ESP32_GRBL, ESP32_WA, "Stepper/StepInvert", 0, set_setting },
    { Setting_StepperIdleLockTime, ESP32_GRBL, ESP32_WA, "Stepper/IdleTime", 0, set_setting },
    { Setting_PulseMicroseconds, ESP32_GRBL, ESP32_WA, "Stepper/Pulse", 0, set_setting },
/*    {"Report/StallGuard" }, */
    { Setting_HomingCycle_6, ESP32_GRBL, ESP32_WA, "Homing/Cycle5", 0, set_setting },
    { Setting_HomingCycle_5, ESP32_GRBL, ESP32_WA, "Homing/Cycle4", 0, set_setting },
    { Setting_HomingCycle_4, ESP32_GRBL, ESP32_WA, "Homing/Cycle3", 0, set_setting },
    { Setting_HomingCycle_3, ESP32_GRBL, ESP32_WA, "Homing/Cycle2", 0, set_setting },
    { Setting_HomingCycle_2, ESP32_GRBL, ESP32_WA, "Homing/Cycle1", 0, set_setting },
    { Setting_HomingCycle_1, ESP32_GRBL, ESP32_WA, "Homing/Cycle0", 0, set_setting },

#if AUTH_ENABLE
    { Setting_AdminPassword, ESP32_WEBSET, ESP32_WA, "WebUI/AdminPassword", 0, set_setting },
    { Setting_UserPassword, ESP32_WEBSET, ESP32_WA, "WebUI/UserPassword", 0, set_setting },
#endif
    { Setting_WiFi_STA_SSID, ESP32_WEBSET, ESP32_WA, "Sta/SSID", ESPCmd_GetSetSTA_SSID, set_setting },
    { Setting_WiFi_STA_Password, ESP32_WEBSET, ESP32_WA, "Sta/Password", ESPCmd_GetSetSTA_Password, set_setting },
    { Setting_Hostname, ESP32_WEBSET, ESP32_WA, "System/Hostname", 0, set_setting },
/*    { Setting_IpMode, ESP32_GRBL, ESP32_WA, "Sta/IPMode", 0, set_setting }, */
    { Setting_IpAddress, ESP32_WEBSET, ESP32_WA, "Sta/IP", ESPCmd_GetSetSTA_IP, set_setting },
    { Setting_Gateway, ESP32_WEBSET, ESP32_WA, "Sta/Gateway", 0, set_setting },
    { Setting_NetMask, ESP32_WEBSET, ESP32_WA, "Sta/Netmask", 0, set_setting },
#if WIFI_SOFTAP
    { Setting_WifiMode, ESP32_WEBSET, ESP32_WA, "Radio/Mode", ESPCmd_GetSetRadioMode, set_setting },
    { Setting_WiFi_AP_SSID, ESP32_WEBSET, ESP32_WA, "AP/SSID", ESPCmd_GetSetAP_SSID, set_setting },
    { Setting_WiFi_AP_Password, ESP32_WEBSET, ESP32_WA, "AP/Password", ESPCmd_GetSetAP_Password, set_setting },
//    { Setting_Hostname2, ESP32_WEBSET, ESP32_WA, "Homing/Cycle0", 0, set_setting },
    { Setting_IpAddress2, ESP32_WEBSET, ESP32_WA, "AP/IP", ESPCmd_GetSetAP_IP, set_setting },
//    { Setting_Gateway2, ESP32_WEBSET, ESP32_WA, "Homing/Cycle0", 0, set_setting },
//    { Setting_NetMask2, ESP32_WEBSET, ESP32_WA, "Homing/Cycle0", 0, set_setting },
#else
    { Setting_WifiMode, ESP32_WEBSET, ESP32_WA, "Homing/Cycle0", 0, set_setting },
#endif
#if TELNET_ENABLE
    { Setting_NetworkServices, ESP32_WEBSET, ESP32_WA, "Telnet/Enable", ESPCmd_GetSetTelnetOnOff, set_setting },
    { Setting_TelnetPort, ESP32_WEBSET, ESP32_WA, "Telnet/Port", ESPCmd_GetSetTelnetPort, set_setting },
#endif
#if HTTP_ENABLE
    { Setting_NetworkServices, ESP32_WEBSET, ESP32_WA, "Http/Enable", ESPCmd_GetSetHTTPOnOff, set_setting },
    { Setting_HttpPort, ESP32_WEBSET, ESP32_WA, "Http/Port", ESPCmd_GetSetHttpPort, set_setting },
#endif
#if WEBSOCKET_ENABLE
//    { Setting_WebSocketPort, ESP32_WEBSET, ESP32_WA, "Homing/Cycle0", 0, set_setting },
#endif
#if BLUETOOTH_ENABLE
    { Setting_BlueToothDeviceName, ESP32_WEBSET, ESP32_WA, "Bluetooth/Name", ESPCmd_GetSetBluetoothName, set_setting },
#endif
    { Setting_SettingsMax, ESP32_WEBCMD, ESP32_WA, "WebUI/List", ESPCmd_GetSettings, list_settings_webui },
    { Setting_SettingsMax, ESP32_GRBLCMD, ESP32_WG, "GrblSettings/List", 0, list_settings },
    { Setting_SettingsMax, ESP32_GRBLCMD, ESP32_WG, "ExtendedSettings/List", 0, list_settings_all }
};

status_code_t set_setting (const grbl_esp32_command_t *setting, char *value)
{
    return value ? settings_store_setting(setting->id, value) : Status_BadNumberFormat;
}

// Add setting to the JSON response array
static bool setting_to_json (const setting_detail_t *setting, uint_fast16_t offset, void *data)
{
    bool ok;

    cJSON *settingobj;

    if((ok = (setting->is_available == NULL ||setting->is_available(setting)) && (settingobj = cJSON_CreateObject()) != NULL))
    {
        ok  = !!cJSON_AddStringToObject(settingobj, "F", "network");
        ok &= !!cJSON_AddStringToObject(settingobj, "P", ((grbl_esp32_json_t *)data)->esp_setting->name);
        ok &= !!cJSON_AddStringToObject(settingobj, "T", tmap[setting->datatype]);
        ok &= !!cJSON_AddStringToObject(settingobj, "V", setting_get_value(setting, offset));
        ok &= !!cJSON_AddStringToObject(settingobj, "H", setting->name);

        switch(setting->datatype) {

            case Format_Bitfield:
            case Format_XBitfield:
            case Format_RadioButtons:
                {
                    uint32_t i, j = strnumentries(setting->format, ',');
                    char opt[50]; //, val[20];
                    cJSON *option, *options = cJSON_AddArrayToObject(settingobj, "O");

                    for(i = 0; i < j; i++) {
                        option = cJSON_CreateObject();
                        cJSON_AddStringToObject(option, strgetentry(opt, setting->format, i, ','), uitoa(i) /*strgetentry(val, m, i, ',')*/);
                        cJSON_AddItemToArray(options, option);
                    }
                }
                break;

            case Format_IPv4:
                break;

            default:
                if(setting->min_value && !setting_is_list(setting))
                    ok &= !!cJSON_AddStringToObject(settingobj, "S", setting->min_value);
                if(setting->max_value)
                    ok &= !!cJSON_AddStringToObject(settingobj, "M", setting->max_value);
                break;

        }

        if(ok)
            cJSON_AddItemToArray(((grbl_esp32_json_t *)data)->parent, settingobj);
    }

    return ok;
}

status_code_t list_settings (const grbl_esp32_command_t *setting, char *value)
{
    strcpy(cmdbuf, "$$");

    return system_execute_line(cmdbuf);
}

status_code_t list_settings_all (const grbl_esp32_command_t *setting, char *value)
{
    strcpy(cmdbuf, "$+");

    return system_execute_line(cmdbuf);
}

status_code_t list_settings_webui (const grbl_esp32_command_t *setting, char *value)
{
    grbl_esp32_json_t json;
    cJSON *root = cJSON_CreateObject();

    if(root && (json.parent = cJSON_AddArrayToObject(root, "EEPROM"))) {

        setting_output_ptr setting_org = grbl.report.setting;
        uint_fast16_t idx, n_settings = sizeof(grbl_esp32_commands) / sizeof(grbl_esp32_command_t);

        grbl.report.setting = setting_to_json;

        for(idx = 0; idx < n_settings; idx++) {
            json.esp_setting = &grbl_esp32_commands[idx];
            if(json.esp_setting->type == ESP32_WEBSET)
                report_grbl_setting(json.esp_setting->id, &json);
        }

        grbl.report.setting = setting_org;

        char *resp = cJSON_PrintUnformatted(root);

        hal.stream.write(resp);

//        webui_print(resp);
        free(resp);

    }

    if(root)
        cJSON_Delete(root);

    return Status_OK;
}

static status_code_t on_unknown_command (sys_state_t state, char *line)
{
    status_code_t retval = Status_Unhandled;
    char *value = strchr(line++, '=');
    uint_fast16_t n_settings = sizeof(grbl_esp32_commands) / sizeof(grbl_esp32_command_t), idx;

    if(value)
        *value++ = '\0';

    for(idx = 0; idx < n_settings; idx++) {
        if(!strcasecmp(line, grbl_esp32_commands[idx].name)) {
            retval = grbl_esp32_commands[idx].execute(&grbl_esp32_commands[idx], value);
            break;
        }
    }

    return retval == Status_Unhandled && on_unknown_sys_command ? on_unknown_sys_command(state, line) : retval;
}

void grbl_esp32_if_init (void)
{
    *cmdbuf = '\0';
    on_unknown_sys_command = grbl.on_unknown_sys_command;
    grbl.on_unknown_sys_command = on_unknown_command;
}
