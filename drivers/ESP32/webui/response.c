/*
  webui/response.c - An embedded CNC Controller with rs274/ngc (g-code) support

  WebUI backend for https://github.com/luc-github/ESP3D-webui

  Part of grblHAL

  Copyright (c) 2019-2020 Terje Io

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

#if WEBUI_ENABLE

#include <stdio.h>
#include <string.h>
#include <sys/unistd.h>
#include <sys/stat.h>

#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"

#include "webui.h"
#include "grbl/grbl.h"

static const char *TAG = "webui";

static bool chunked = false;
static httpd_req_t *http_request = NULL;

static status_code_t webui_parse_command (char *cmd)
{
    status_code_t status = Status_GcodeUnsupportedCommand;

    http_request = NULL;

    if(!strncmp(cmd, "[ESP", 4)) {

        char *args = NULL;

        if((args = strchr(&cmd[4], ']'))) {
            *args++ = '\0';
            status = webui_command_handler(atol(&cmd[4]), args);
        }
    }

    if(status != Status_OK) {
        char msg[100];
        sprintf(msg, "[MSG: Unknow Command...%s]\r\n", cmd); // sic
        hal.stream.write(msg);
    }

    return status;
}

void webui_init (void)
{
    grbl.on_user_command = webui_parse_command;

    esp_vfs_spiffs_conf_t conf = {
      .base_path = "/spiffs",
      .partition_label = NULL,
      .max_files = 50,
      .format_if_mount_failed = true
    };

    // Use settings defined above to initialize and mount SPIFFS filesystem.
    // Note: esp_vfs_spiffs_register is an all-in-one convenience function.
    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
    }
}

void webui_set_http_request (httpd_req_t *req)
{
    chunked = false;
    http_request = req;
    httpd_resp_set_hdr(http_request, "Cache-Control", "no-cache");
}

void webui_print (const char *s)
{
    if(http_request)
        httpd_resp_sendstr(http_request, s);
    else {
        size_t len = strlen(s);
        if(len) {
            if(s[--len] == '\n') {
                char *t = malloc(len + 3);
                if(t) {
                    memcpy(t, s, len);
                    t[len++] = '\r';
                    t[len++] = '\n';
                    t[len] = '\0';
                    hal.stream.write(t);
                    free(t);
                }
            } else {
                hal.stream.write(s);
                hal.stream.write(ASCII_EOL);
            }
        } else
            hal.stream.write(ASCII_EOL);
    }
}

void webui_print_chunk (const char *s)
{
    chunked = true;
    if(http_request)
        httpd_resp_sendstr_chunk(http_request, s);
    else
        webui_print(s);
}

void webui_print_is_json (void)
{
    if(http_request)
        httpd_resp_set_type(http_request, HTTPD_TYPE_JSON);
}

void webui_print_flush (void)
{
    if(http_request) {
        if(chunked)
            httpd_resp_sendstr_chunk(http_request, NULL);
    }
}

#endif
