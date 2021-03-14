/*
  backend.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Webserver backend

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

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

#include "driver.h"

#if WEBUI_ENABLE

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <dirent.h>

#include <esp_log.h>
#include "esp_wifi.h"
#include <esp_system.h>
#include <esp_http_server.h>
#include "esp_spiffs.h"
#include <cJSON.h>

#include "backend.h"
#include "wifi.h"
#include "upload.h"
#include "grbl/report.h"
#include "networking/urldecode.h"

#if WEBUI_ENABLE
#include "webui/server.h"
#endif

//#define CORS_ENABLE 1 // Enable only when debugging

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "esp_vfs_fat.h"
#endif

static httpd_handle_t httpdaemon = NULL;

#define MAX_APs 20

static const char *TAG = "httpd_server";

/* Max size of an individual file. Make sure this
 * value is same as that set in upload_script.html */
#define MAX_FILE_SIZE   (200*1024) // 200 KB
#define MAX_FILE_SIZE_STR "200KB"

static file_server_data_t file_server_data, spiff_fs_data;
#if WEBUI_ENABLE
static file_server_data_t sd_fs_data;
#endif

/* Handler to redirect incoming GET request for /index.html to /
 * This can be overridden by uploading file with same name */
static esp_err_t redirect_html_get_handler(httpd_req_t *req, char *location)
{
    httpd_resp_set_status(req, "307 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", location);
    httpd_resp_send(req, NULL, 0);  // Response body can be empty
    return ESP_OK;
}

/* Handler to respond with an icon file embedded in flash.
 * Browsers expect to GET website icon at URI /favicon.ico.
 * This can be overridden by uploading file with same name */

static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    extern const unsigned char favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const unsigned char favicon_ico_end[]   asm("_binary_favicon_ico_end");
    const size_t favicon_ico_size = (favicon_ico_end - favicon_ico_start);
    httpd_resp_set_type(req, "image/x-icon");
    return httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_size);
}

static esp_err_t index_html_get_handler(httpd_req_t *req)
{
#if WEBUI_ENABLE
    return webui_index_html_get_handler(req);
#else
    extern const unsigned char index_html_start[] asm("_binary_index_html_start");
    extern const unsigned char index_html_end[]   asm("_binary_index_html_end");
    const size_t index_html_size = (index_html_end - index_html_start);
    return httpd_resp_send(req, (const char *)index_html_start, index_html_size);
#endif
}

static esp_err_t ap_login_html_get_handler(httpd_req_t *req)
{
    extern const unsigned char ap_login_html_start[] asm("_binary_ap_login_html_start");
    extern const unsigned char ap_login_html_end[]   asm("_binary_ap_login_html_end");
    return httpd_resp_send(req, (const char *)ap_login_html_start, ap_login_html_end - ap_login_html_start);
}

#define IS_FILE_EXT(filename, ext) \
    (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0)

/* Set HTTP response content type according to file extension */
esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename)
{
    if(filename && strlen(filename) > 3) {
        if (IS_FILE_EXT(filename, ".pdf"))
            return httpd_resp_set_type(req, "application/pdf");
        else if (IS_FILE_EXT(filename, ".html"))
            return httpd_resp_set_type(req, "text/html");
        else if (IS_FILE_EXT(filename, ".jpeg") || IS_FILE_EXT(filename, ".jpg"))
            return httpd_resp_set_type(req, "image/jpeg");
        else if (IS_FILE_EXT(filename, ".ico"))
            return httpd_resp_set_type(req, "image/x-icon");
        else if (IS_FILE_EXT(filename, ".gz"))
            return httpd_resp_set_type(req, "application/x-gzip");
    }
    /* This is a limited set only */
    /* For any other type always set as plain text */
    return httpd_resp_set_type(req, "text/plain");
}

// Fetch and decode value for query key
char *http_get_key_value (char *qstring, char *key, char *val, size_t val_size)
{
    if(httpd_query_key_value(qstring, key, val, val_size) == ESP_OK)
        urldecode(val, val);
    else {
        *val = '\0';
        val = NULL;
    }

    return val;
}

/* Copies the full path into destination buffer and returns
 * pointer to path (skipping the preceding base path) */
static const char *get_path_from_uri (char *dest, httpd_req_t *req, size_t destsize)
{
    *dest = '\0';
    if(req->user_ctx)
        strcpy(dest, ((file_server_data_t *)req->user_ctx)->base_path);

    char *uri = (char *)req->uri, *p;
    const size_t base_pathlen = strlen(dest);
    size_t pathlen = strlen(uri);
    if(pathlen >= base_pathlen && !strncmp(dest, uri, base_pathlen)) {
        uri += base_pathlen;
        pathlen -= base_pathlen;
    }

    if ((p = strchr(uri, '?')))
        pathlen = MIN(pathlen, p - uri);

    if ((p = strchr(uri, '#')))
        pathlen = MIN(pathlen, p - uri);

    if (base_pathlen + pathlen + 1 > destsize)
        /* Full path string won't fit into destination buffer */
        return NULL;

    urldecode(dest + base_pathlen, uri);

    dest[base_pathlen + pathlen] = '\0';

    /* Return pointer to path, skipping the base */
    return dest + base_pathlen;
}

static bool getPayload (httpd_req_t *req, char *buf)
{
    int ret;
    size_t off = 0;

    if (buf == NULL) {
        // Failed to allocate memory for payload
        httpd_resp_send_500(req);
        return false;
    }

     // Process received data
    while (off < req->content_len) {

        if ((ret = httpd_req_recv(req, buf + off, req->content_len - off)) <= 0) {
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
                httpd_resp_send_408(req);
            return false;
        }

        off += ret;
    }

    buf[off] = '\0';

    return true;
}

esp_err_t spiffs_get_handler (httpd_req_t *req)
{
    fs_filepath_t filepath;
    const char *filename = get_path_from_uri(filepath, req, sizeof(filepath));

    if (!filename) {
        ESP_LOGE(TAG, "Filename is too long");
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    if (!strcmp(filename, "/favicon.ico"))
        return favicon_get_handler(req);

    /* If name has trailing '/', respond with directory contents */
    if (filename[strlen(filename) - 1] == '/') {
   //     return http_resp_dir_html(req, filepath);
    }

    FILE *file;
    struct stat st;

    if (stat(filepath, &st) != 0) {
        /* If file not present on SPIFFS check if URI
         * corresponds to one of the hardcoded paths */
        if (strcmp(filename, "/index.html") == 0) {
            return index_html_get_handler(req);
        } else if (strcmp(filename, "/favicon.ico") == 0) {
      //      return favicon_get_handler(req);
        }
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");
        return ESP_FAIL;
    }

    if ((file = fopen(filepath, "r")) == NULL) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    set_content_type_from_file(req, filename);

    size_t chunksize;
    char *chunk = ((file_server_data_t *)req->user_ctx)->scratch;

    do {
        chunksize = fread(chunk, sizeof(char), sizeof(fs_scratch_t), file);

        if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) {
            fclose(file);
            httpd_resp_sendstr_chunk(req, NULL);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
            return ESP_FAIL;
        }

    } while (chunksize != 0);

    fclose(file);
    httpd_resp_send_chunk(req, NULL, 0);

    return ESP_OK;
}

#if SDCARD_ENABLE

static esp_err_t sdcard_get_handler (httpd_req_t *req)
{
    fs_filepath_t filepath;
    const char *filename = get_path_from_uri(filepath, req, sizeof(filepath));

 //   sdcard_getfs(); // ensure card is mounted

    if (!filename) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    if (!strcmp(filename, "/favicon.ico"))
        return favicon_get_handler(req);

    /* If name has trailing '/', respond with directory contents */
    if (filename[strlen(filename) - 1] == '/') {
   //     return http_resp_dir_html(req, filepath);
    }

    FIL file;
    FILINFO st;

    if(f_stat(filename, &st) != FR_OK) {
        /* If file not present on SPIFFS check if URI
         * corresponds to one of the hardcoded paths */
        if (strcmp(filename, "/index.html") == 0) {
            return index_html_get_handler(req);
        } else if (strcmp(filename, "/favicon.ico") == 0) {
      //      return favicon_get_handler(req);
        }
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");
        return ESP_FAIL;
    }

    if (f_open(&file, filename, FA_READ) != FR_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

    set_content_type_from_file(req, filename);

    size_t chunksize;
    char *chunk = ((file_server_data_t *)req->user_ctx)->scratch;

    do {
        f_read(&file, chunk, sizeof(fs_scratch_t), &chunksize);
        if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) {
            f_close(&file);
            httpd_resp_sendstr_chunk(req, NULL);
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
            return ESP_FAIL;
        }
    } while (chunksize != 0);

    f_close(&file);
    httpd_resp_send_chunk(req, NULL, 0);

    return ESP_OK;
}

#if !WEBUI_ENABLE

static esp_err_t sdcard_form_upload_handler (httpd_req_t *req)
{
    bool ok = false;
    int ret;

    char *rqhdr = NULL, *boundary;
    size_t len = httpd_req_get_hdr_value_len(req, "Content-Type");

    if(len) {
        rqhdr = malloc(len + 1);
        httpd_req_get_hdr_value_str(req, "Content-Type", rqhdr, len + 1);

        if((ok = (boundary = strstr(rqhdr, "boundary=")))) {
            boundary += strlen("boundary=");
            ok = upload_start(req, boundary, true);
        }
    }

    char *scratch = ((file_server_data_t *)req->user_ctx)->scratch;
    file_upload_t *upload = (file_upload_t *)req->sess_ctx;

    if (ok) do { // Process received data

        if ((ret = httpd_req_recv(req, scratch, sizeof(fs_scratch_t))) <= 0) {
            ok = false;
            if (ret == HTTPD_SOCK_ERR_TIMEOUT)
                httpd_resp_send_408(req);
            break;
        }

        upload_chunk(req, scratch, (size_t)ret);

    } while(upload->state != Upload_Complete);

    if(ok) {
        httpd_resp_set_status(req, "202 Accepted");
        httpd_resp_send(req, NULL, 0);
    } else
        httpd_resp_send_err(req, 400, "Upload failed");

    if(req->sess_ctx && req->free_ctx) {
        req->free_ctx(req->sess_ctx);
        req->sess_ctx = NULL;
    }

    if(rqhdr)
        free(rqhdr);

    return ok ? ESP_OK : ESP_FAIL;
}

#endif // WEBUI_ENABLE

#endif //SDCARD_ENABLE

static esp_err_t get_handler(httpd_req_t *req)
{
    if(wifi_dns_running()) { // captive portal, redirect requests to ourself...

        if (!strcmp(req->uri, "/ap_login.html"))
            return ap_login_html_get_handler(req);

        bool internal = false;
        struct sockaddr_in6 addr;   // esp_http_server uses IPv6 addressing
        socklen_t addr_size = sizeof(struct sockaddr_in6);
        char ipstr[INET6_ADDRSTRLEN];

        if (getpeername(httpd_req_to_sockfd(req), (struct sockaddr *)&addr, &addr_size) == 0) {

            ap_list_t *ap_list = wifi_get_aplist();

            if(ap_list) { // Request is from local STA?
                internal = ap_list->ap_selected && memcmp(&ap_list->ip_addr, &addr.sin6_addr.un.u32_addr[3], sizeof(ip4_addr_t)) == 0;
                wifi_release_aplist();
            }

            inet_ntop(AF_INET, &addr.sin6_addr.un.u32_addr[3], ipstr, sizeof(ipstr));

            wifi_settings_t *settings = get_wifi_settings();

            // From local AP?
            if(!internal && memcmp(&settings->ap.network.ip, &addr.sin6_addr.un.u32_addr[3], sizeof(ip4_addr_t))) {

                char loc[75];

                inet_ntop(AF_INET, &settings->ap.network.ip, ipstr, sizeof(ipstr));

                sprintf(loc, "http://%s/ap_login.html", ipstr);

                return redirect_html_get_handler(req, loc);
            }
        }
    }

    fs_filepath_t filepath;
    const char *filename = get_path_from_uri(filepath, req, sizeof(filepath));

#if SDCARD_ENABLE
    // If file exists on SD card get it from there
    FILINFO file;
    if(f_stat(filename, &file) == FR_OK) {
#if WEBUI_ENABLE
        req->user_ctx = &sd_fs_data;
#endif
        return sdcard_get_handler(req);
    }
#endif

    // If file exists in spiffs get it from there
    struct stat st;
    strcat(strcpy(spiff_fs_data.scratch, spiff_fs_data.base_path), filename);

    if (stat(spiff_fs_data.scratch, &st) == 0) {
        req->user_ctx = &spiff_fs_data;
        return spiffs_get_handler(req);
    }

    if (!strcmp(req->uri, "/index.html") || !strcmp(req->uri, "/"))
        return index_html_get_handler(req);

    if (!strcmp(req->uri, "/favicon.ico"))
        return favicon_get_handler(req);

    httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");

    return ESP_FAIL;
}

// add setting to the JSON response array
static bool report_setting (const setting_detail_t *setting, uint_fast16_t offset, void *data)
{
    bool ok = true;

    cJSON *settingobj;

    if((ok = (settingobj = cJSON_CreateObject()) != NULL))
    {
        ok = !!cJSON_AddNumberToObject(settingobj, "id", (double)(setting->id + offset));
        ok &= !!cJSON_AddStringToObject(settingobj, "value", setting_get_value(setting, offset));
        if(ok)
            cJSON_AddItemToArray((cJSON *)data, settingobj);
    }

    return ok;
}

static esp_err_t settings_get_handler(httpd_req_t *req)
{
    bool ok;

//  size_t ql = httpd_req_get_url_query_len(req);

    int setting = strlen(req->uri) > 9 ? atoi(&req->uri[10]) : -1;

    cJSON *json_settings, *root = cJSON_CreateObject();

    if((ok = (root && (json_settings = cJSON_AddArrayToObject(root, "settings"))))) {

        setting_output_ptr org_ptr = grbl.report.setting;
        grbl.report.setting = report_setting;

        if(setting == -1)
            report_grbl_settings(false, &json_settings);
        else
            report_grbl_setting((setting_id_t)setting, &json_settings);

        grbl.report.setting = org_ptr;

        char *resp = cJSON_PrintUnformatted(root);

        httpd_resp_set_type(req, HTTPD_TYPE_JSON);
        httpd_resp_send(req, resp, strlen(resp));

        free(resp);

    } else
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to generate response");

    if(root)
        cJSON_Delete(root);

    return ok ? ESP_OK : ESP_FAIL;
}

static esp_err_t settings_set_handler(httpd_req_t *req)
{
//  heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);

    bool ok;
    char *buf = malloc(req->content_len + 1);

    //httpd_req_get_hdr_value_str(httpd_req_t *r, const char *field, char *val, size_t val_size)

    // check content type?

    if((ok = getPayload(req, buf))) {

        cJSON *root, *settings, *setting;
        status_code_t status = Status_InvalidStatement;

        if((ok = (root = cJSON_Parse(buf)))) {
            settings = cJSON_GetObjectItemCaseSensitive(root, "settings");
            cJSON_ArrayForEach(setting, settings)
            {
                cJSON *id = cJSON_GetObjectItemCaseSensitive(setting, "id");
                cJSON *value = cJSON_GetObjectItemCaseSensitive(setting, "value");

                if((status = settings_store_setting ((setting_id_t)((int)id->valuedouble), value->valuestring)) != Status_OK)
                    break;
            }
            cJSON_Delete(root);
        }

        if(!ok)
            ESP_LOGE(TAG, "Failed to parse %s!", buf);

        if(ok) {
            httpd_resp_set_status(req, "202 Accepted");
            httpd_resp_send(req, NULL, 0);
        } else
            httpd_resp_send_err(req, 400, "Invalid JSON data");
    }

    if(buf)
        free(buf);

    return ok ? ESP_OK : ESP_FAIL;
}

static char *getAuthModeName(wifi_auth_mode_t authmode)
{
    static char mode[15];

    sprintf(mode, "%s",
            authmode == WIFI_AUTH_OPEN ? "open" :
            authmode == WIFI_AUTH_WEP ? "wep" :
            authmode == WIFI_AUTH_WPA_PSK ? "wpa-psk" :
            authmode == WIFI_AUTH_WPA2_PSK ? "wpa-psk" :
            authmode == WIFI_AUTH_WPA_WPA2_PSK ? "wpa-wpa2-psk" :
            authmode == WIFI_AUTH_WPA2_ENTERPRISE ? "wpa-eap" :
            "unknown");

    return mode;
}

static esp_err_t wifi_scan_handler (httpd_req_t *req)
{
    bool ok = false;
    ap_list_t *ap_list = wifi_get_aplist();

    if(ap_list && ap_list->ap_records) {

        cJSON *root;

        if((root = cJSON_CreateObject())) {

            cJSON *ap, *aps;

            ok = cJSON_AddStringToObject(root, "ap", ap_list->ap_selected ? (char *)ap_list->ap_selected : "") != NULL;
            ok &= cJSON_AddStringToObject(root, "status", ap_list->ap_status) != NULL;

            if(ap_list->ap_selected)
                cJSON_AddStringToObject(root, "ip", ip4addr_ntoa(&ap_list->ip_addr));

            if((aps = cJSON_AddArrayToObject(root, "aplist"))) {

                for(int i = 0; i < ap_list->ap_num; i++) {
                    if((ok = (ap = cJSON_CreateObject()) != NULL))
                    {
                        ok = cJSON_AddStringToObject(ap, "ssid", (char *)ap_list->ap_records[i].ssid) != NULL;
                        ok &= cJSON_AddStringToObject(ap, "security", getAuthModeName(ap_list->ap_records[i].authmode)) != NULL;
                        ok &= cJSON_AddNumberToObject(ap, "primary", (double)ap_list->ap_records[i].primary) != NULL;
                        ok &= cJSON_AddNumberToObject(ap, "rssi",  (double)ap_list->ap_records[i].rssi) != NULL;
                        if(ok)
                            cJSON_AddItemToArray(aps, ap);
                    }
                }
            }

            if(ok) {
                char *resp = cJSON_PrintUnformatted(root);
#if xCORS_ENABLE
                httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
                httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "POST,GET,OPTIONS");
#endif
                httpd_resp_set_type(req, HTTPD_TYPE_JSON);
                httpd_resp_send(req, resp, strlen(resp));

                free(resp);
            }

            if(root)
                cJSON_Delete(root);
        }

        if(!ok)
            httpd_resp_send_500(req);
    }

    if(ap_list)
        wifi_release_aplist();

    return ok ? ESP_OK : ESP_FAIL;
}

static esp_err_t wifi_connect_handler (httpd_req_t *req)
{
    bool ok;
    char *buf = malloc(req->content_len + 1);

    if((ok = getPayload(req, buf))) {

        cJSON *cred;

        if((ok = (cred = cJSON_Parse(buf)))) {
            cJSON *ssid = cJSON_GetObjectItemCaseSensitive(cred, "ssid");
            cJSON *password = cJSON_GetObjectItemCaseSensitive(cred, "password");

            ok = ssid && password && wifi_ap_connect(ssid->valuestring, password->valuestring);
            cJSON_Delete(cred);
        }

        if(ok) {
            char resp[] = "Connecting...";
            httpd_resp_set_status(req, "202 Accepted");
            httpd_resp_send(req, resp, sizeof(resp));
        } else
            httpd_resp_send_err(req, 400, "Invalid connect information");
    }

    if(buf)
        free(buf);

    return ok ? ESP_OK : ESP_FAIL;
}

static esp_err_t wifi_disconnect_handler (httpd_req_t *req)
{
    bool disconnect = false;

    ap_list_t *ap_list = wifi_get_aplist();

    if(ap_list) {
        disconnect = ap_list->ap_selected;
        wifi_release_aplist();
    }

    if(disconnect)
        wifi_ap_connect(NULL, NULL);

    httpd_resp_send(req, NULL, 0);

    return ESP_OK;
}

#if CORS_ENABLE
static esp_err_t wifi_options_handler (httpd_req_t *req)
{
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "Access-Control-Request-Headers", "*");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Methods", "GET,POST,DELETE,OPTIONS");

    return wifi_scan_handler(req);
}
#endif

static const httpd_uri_t basic_handlers[] = {
    { .uri       = "/*",
      .method    = HTTP_GET,
      .handler   = get_handler,
      .user_ctx  = &file_server_data,
    },
    { .uri      = "/spiffs/*",
      .method   = HTTP_GET,
      .handler  = spiffs_get_handler,
      .user_ctx = &spiff_fs_data
    },
#if SDCARD_ENABLE
  #if WEBUI_ENABLE
    { .uri      = "/upload",
      .method   = HTTP_GET,
      .handler  = webui_sdcard_handler,
      .user_ctx = &file_server_data
    },
    { .uri      = "/upload",
      .method   = HTTP_POST,
      .handler  = webui_sdcard_upload_handler,
      .user_ctx = &file_server_data
    },
    { .uri      = "/files",
      .method   = HTTP_GET,
      .handler  = webui_spiffs_handler,
      .user_ctx = &spiff_fs_data
    },
    { .uri      = "/files/*",
      .method   = HTTP_GET,
      .handler  = webui_spiffs_handler,
      .user_ctx = &spiff_fs_data
    },
    { .uri      = "/files",
      .method   = HTTP_POST,
      .handler  = webui_spiffs_upload_handler,
      .user_ctx = &spiff_fs_data
    },
    { .uri      = "/SD/*",
      .method   = HTTP_GET,
      .handler  = sdcard_get_handler,
      .user_ctx = &sd_fs_data
    },
  #else
    { .uri      = "/sdcard/*",
      .method   = HTTP_GET,
      .handler  = sdcard_get_handler,
      .user_ctx = &file_server_data
    },
    { .uri      = "/sdcard",
      .method   = HTTP_POST,
      .handler  = sdcard_form_upload_handler,
      .user_ctx = &file_server_data
    },
  #endif
#endif
    { .uri      = "/settings",
      .method   = HTTP_GET,
      .handler  = settings_get_handler,
      .user_ctx = NULL
    },
    { .uri      = "/settings/*",
      .method   = HTTP_GET,
      .handler  = settings_get_handler,
      .user_ctx = NULL
    },
    { .uri      = "/settings",
      .method   = HTTP_POST,
      .handler  = settings_set_handler,
      .user_ctx = NULL
    },
    { .uri      = "/wifi",
      .method   = HTTP_GET,
      .handler  = wifi_scan_handler,
      .user_ctx = NULL
    },
#if WEBUI_ENABLE
    { .uri      = "/command",
      .method   = HTTP_GET,
      .handler  = webui_http_command_handler,
      .user_ctx = NULL
    },
    { .uri      = "/login",
      .method   = HTTP_GET,
      .handler  = webui_login_handler,
      .user_ctx = NULL
    },
#endif
#if CORS_ENABLE
    { .uri      = "/wifi",
      .method   = HTTP_OPTIONS,
      .handler  = wifi_options_handler,
      .user_ctx = NULL
    },
#endif
    { .uri      = "/wifi",
      .method   = HTTP_POST,
      .handler  = wifi_connect_handler,
      .user_ctx = NULL
    },
    { .uri      = "/wifi",
      .method   = HTTP_DELETE,
      .handler  = wifi_disconnect_handler,
      .user_ctx = NULL
    }
};

void register_basic_handlers(httpd_handle_t hd)
{
    uint32_t i = sizeof(basic_handlers) / sizeof(httpd_uri_t);

    wifi_mode_t currentMode;

    esp_wifi_get_mode(&currentMode);

    if(currentMode != WIFI_MODE_APSTA)
        i -= 2;

    do {
        if (httpd_register_uri_handler(hd, &basic_handlers[--i]) != ESP_OK) {
            ESP_LOGW(TAG, "register uri failed for %d", i);
            return;
        }
    } while(i);
}

void httpdaemon_stop (void)
{
    if(httpdaemon)
        httpd_stop(httpdaemon);

    httpdaemon = NULL;
}

bool httpdaemon_start (network_settings_t *network)
{
    //pre_start_mem = esp_get_free_heap_size();

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.max_uri_handlers = max(config.max_uri_handlers, sizeof(basic_handlers) / sizeof(httpd_uri_t));
    config.server_port = network->http_port;
    config.uri_match_fn = httpd_uri_match_wildcard;
    config.stack_size = 10240;

    httpdaemon_stop();

    /* This check should be a part of http_server */
    config.max_open_sockets = (CONFIG_LWIP_MAX_SOCKETS - 3);

    *file_server_data.base_path = '\0';
    strcpy(spiff_fs_data.base_path, "/spiffs");
#if WEBUI_ENABLE
    strcpy(sd_fs_data.base_path, "/SD");
#endif

    if (httpd_start(&httpdaemon, &config) == ESP_OK)
        register_basic_handlers(httpdaemon);

    return httpdaemon != NULL;
}

#endif
