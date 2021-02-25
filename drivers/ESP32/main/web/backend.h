/*
  backend.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Webserver backend

  Part of grblHAL

  Copyright (c) 2019 Terje Io

  Some parts of the code is based on example code by Espressif, in the public domain

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

#ifndef __WEB_BACKEND_H__
#define __WEB_BACKEND_H__

#include <esp_vfs.h>
#include <esp_http_server.h>

#include "driver.h"

#define SCRATCH_BUFSIZE  8192

typedef char fs_scratch_t[SCRATCH_BUFSIZE];
typedef char fs_path_t[ESP_VFS_PATH_MAX + 1];
typedef char fs_filename_t[CONFIG_SPIFFS_OBJ_NAME_LEN + 1];
typedef char fs_filepath_t[ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN + 1];
typedef char fs_queryarg_t[100];

typedef struct {
    fs_path_t base_path;
    fs_scratch_t scratch;
} file_server_data_t;

bool httpdaemon_start(network_settings_t *network);
void httpdaemon_stop();
esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename);
char *http_get_key_value (char *qstring, char *key, char *s, size_t val_size);

#endif

