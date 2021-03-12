/*
  web/upload.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Webserver backend - file upload

  Part of grblHAL

  Copyright (c) 2019 Terje Io

  Some parts of the code is based on test code by francoiscolas
  https://github.com/francoiscolas/multipart-parser/blob/master/tests.cpp

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

#include "upload.h"

#include "networking/multipartparser.h"

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#endif

static struct multipartparser parser;
static struct multipartparser_callbacks *sd_callbacks = NULL;

static void do_cleanup (file_upload_t *upload)
{
    // close and unlink open file
    if(upload->file.handle) {
        if(upload->to_fatfs) {
            f_close(upload->file.fatfs_handle);
            f_unlink(upload->filename);
        } else {
            fclose(upload->file.handle);
            unlink(upload->filename);
        }
        upload->file.handle = NULL;
    }
}

static void cleanup (void *upload)
{
    do_cleanup((file_upload_t *)upload);

    free(upload);
}

static int on_body_begin(struct multipartparser *parser)
{
    return 0;
}

static int on_part_begin(struct multipartparser *parser)
{
    return 0;
}

static void on_header_done(struct multipartparser *parser)
{
    file_upload_t *upload = (file_upload_t *)parser->data;

    if(*upload->header_value) {

        if(!strcmp(upload->header_name, "Content-Disposition")) {

            char *name;

            if(strstr(upload->header_value, "name=\"path\"")) {
                upload->state = Upload_GetPath;
                *upload->path = '\0';
            } else if((name = strstr(upload->header_value, "filename=\""))) {
                if(!upload->to_fatfs)
                    strcpy(upload->filename, "/spiffs");
                else
                    *upload->filename = '\0';
                strcat(upload->filename, name + 10);
                upload->filename[strlen(upload->filename) - 1] = '\0';
                if(*upload->size_str)
                    upload->size = atoi(upload->size_str);
            } else if(strstr(upload->header_value, "name=\"")) {
                upload->state = Upload_GetSize;
                *upload->size_str = '\0';
            }
        }

        if(*upload->filename && strstr(upload->header_name, "Content-Type")) {

            if(upload->to_fatfs) {
                upload->file.fatfs_handle = &upload->fatfs_fd;
                if(f_open(upload->file.fatfs_handle, upload->filename, FA_WRITE|FA_CREATE_ALWAYS) == FR_OK)
                    upload->state = Upload_Write;
                else
                    upload->file.fatfs_handle = NULL;
            }  else if((upload->file.handle = fopen(upload->filename, "w")))
                upload->state = Upload_Write;

            upload->uploaded = 0;
        }
    }

    *upload->header_name = '\0';
    *upload->header_value = '\0';
}

static int on_header_field(struct multipartparser *parser, const char* data, size_t size)
{
    if (*((file_upload_t *)parser->data)->header_value)
        on_header_done(parser);
    strncat(((file_upload_t *)parser->data)->header_name, data, size);

    return 0;
}

static int on_header_value(struct multipartparser *parser, const char* data, size_t size)
{
    strncat(((file_upload_t *)parser->data)->header_value, data, size);
    return 0;
}

static int on_headers_complete(struct multipartparser *parser)
{
    if (*((file_upload_t *)parser->data)->header_value)
        on_header_done(parser);

    return 0;
}

static int on_data(struct multipartparser *parser, const char* data, size_t size)
{
    file_upload_t *upload = (file_upload_t *)parser->data;

    switch(upload->state) {

        case Upload_Write:
            {
                size_t count;
                if(upload->to_fatfs) {
                    f_write(upload->file.fatfs_handle, data, size, &count);
                } else
                    count = fwrite(data, sizeof(char), size, upload->file.handle);
                if(count != size)
                    upload->state = Upload_Failed;
                upload->uploaded += count;
            }
            break;

        case Upload_GetPath:
            strncat(upload->path, data, size);
            break;

        case Upload_GetSize:
            strncat(upload->size_str, data, size);
            break;

        default:
            break;
    }

    return 0;
}

static int on_part_end(struct multipartparser *parser)
{
    file_upload_t *upload = (file_upload_t *)parser->data;

    switch(upload->state) {

        case Upload_Write:
            if(upload->to_fatfs) {
                f_close(upload->file.fatfs_handle);
                upload->file.fatfs_handle = NULL;
            } else {
                fclose(upload->file.handle);
                upload->file.handle = NULL;
            }
            break;

        case Upload_Failed:
            do_cleanup(upload);
            break;

        default:
            break;
    }

    upload->state = Upload_Parsing;

    return 0;
}

static int on_body_end(struct multipartparser *parser)
{
    ((file_upload_t *)parser->data)->state = Upload_Complete;

    return 0;
}

bool upload_start (httpd_req_t *req, const char* boundary, bool to_fatfs)
{
    if(sd_callbacks == NULL && (sd_callbacks = malloc(sizeof(struct multipartparser_callbacks)))) {
        multipartparser_callbacks_init(sd_callbacks);

        sd_callbacks->on_body_begin = on_body_begin;
        sd_callbacks->on_part_begin = on_part_begin;
        sd_callbacks->on_header_field = on_header_field;
        sd_callbacks->on_header_value = on_header_value;
        sd_callbacks->on_headers_complete = on_headers_complete;
        sd_callbacks->on_data = on_data;
        sd_callbacks->on_part_end = on_part_end;
        sd_callbacks->on_body_end = on_body_end;
    }

    if(sd_callbacks) {

        multipartparser_init(&parser, boundary);

        if((parser.data = req->sess_ctx = malloc(sizeof(file_upload_t)))) {
            req->free_ctx = cleanup;
            memset(parser.data, 0, sizeof(file_upload_t));
            ((file_upload_t *)parser.data)->to_fatfs = to_fatfs;
        }
    }

    return parser.data != NULL;
}

size_t upload_chunk (httpd_req_t *req, const char* data, size_t size)
{
    return multipartparser_execute(&parser, sd_callbacks, data, size);
}

#endif

