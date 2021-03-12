/*
  webui/response.h - An embedded CNC Controller with rs274/ngc (g-code) support

  WebUI backend for https://github.com/luc-github/ESP3D-webui

  Part of grblHAL

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

#ifndef __WEBUI_RESPONSE_H__
#define __WEBUI_RESPONSE_H__

#include <esp_http_server.h>

void webui_init (void);
void webui_print (const char *s);
void webui_print_chunk (const char *s);
void webui_print_flush (void);
void webui_print_is_json (void);
void webui_set_http_request (httpd_req_t *req);

#endif

