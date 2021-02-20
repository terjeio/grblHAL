/*
  web/upload.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Webserver backend - file upload

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

#ifndef __WEB_UPLOAD_H__
#define __WEB_UPLOAD_H__

#include <stdlib.h>

#include <esp_http_server.h>
#include <esp_vfs_fat.h>

typedef enum
{
    Upload_Parsing = 0,
    Upload_GetPath,
    Upload_GetSize,
    Upload_Write,
    Upload_Failed,
    Upload_Complete
} upload_state_t;

typedef union {
    FILE *handle;
    FIL *fatfs_handle;
} file_handle_t;

typedef struct {
    upload_state_t state;
    bool to_fatfs;
    char header_name[100];
    char header_value[100];
    char filename[100];
    char path[100];
    char size_str[15];
    file_handle_t file;
    httpd_req_t *req;
    FIL fatfs_fd;
    size_t size;
    size_t uploaded;
} file_upload_t;

bool upload_start (httpd_req_t *req, const char* boundary, bool to_fatfs);
size_t upload_chunk (httpd_req_t *req, const char* data, size_t size);

#endif

