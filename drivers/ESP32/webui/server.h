/*
  webui/server.h - An embedded CNC Controller with rs274/ngc (g-code) support

  WebUI backend for https://github.com/luc-github/ESP3D-webui

  Part of GrblHAL

  Copyright (c) 2019 Terje Io

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

#ifndef __WEBUI_SERVER_H__
#define __WEBUI_SERVER_H__

#include <esp_http_server.h>

#include "web/backend.h"

esp_err_t webui_http_command_handler (httpd_req_t *req);
esp_err_t webui_sdcard_handler (httpd_req_t *req);
esp_err_t webui_sdcard_upload_handler (httpd_req_t *req);
esp_err_t webui_spiffs_handler (httpd_req_t *req);
esp_err_t webui_spiffs_upload_handler (httpd_req_t *req);
esp_err_t webui_index_html_get_handler (httpd_req_t *req);

#endif

