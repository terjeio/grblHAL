/*
  backend.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Webserver backend

  Part of Grbl

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

#include <stdlib.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <dirent.h>

#include <esp_log.h>
#include "esp_wifi.h"
#include <esp_system.h>
#include <esp_http_server.h>
#include "esp_vfs.h"
#include "esp_spiffs.h"
#include "esp_http_server.h"
#include <cJSON.h>

#include "driver.h"
#include "wifi.h"

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "esp_vfs_fat.h"
#endif

static int setting = -1;
static char sbuf[100];
static httpd_handle_t httpdaemon = NULL;
static cJSON *json_settings;

static stream_write_ptr org_stream;

#define MAX_APs 20

static const char *TAG = "httpd_server";

/* Max length a file path can have on storage */
#define FILE_PATH_MAX (ESP_VFS_PATH_MAX + CONFIG_SPIFFS_OBJ_NAME_LEN)

/* Max size of an individual file. Make sure this
 * value is same as that set in upload_script.html */
#define MAX_FILE_SIZE   (200*1024) // 200 KB
#define MAX_FILE_SIZE_STR "200KB"

/* Scratch buffer size */
#define SCRATCH_BUFSIZE  8192

typedef struct {
    /* Base path of file storage */
    char base_path[ESP_VFS_PATH_MAX + 1];

    /* Scratch buffer for temporary storage during file transfer */
    char scratch[SCRATCH_BUFSIZE];
} file_server_data_t;

static file_server_data_t file_server_data;

/* Handler to redirect incoming GET request for /index.html to /
 * This can be overridden by uploading file with same name */
static esp_err_t index_html_get_handler(httpd_req_t *req)
{
    httpd_resp_set_status(req, "307 Temporary Redirect");
    httpd_resp_set_hdr(req, "Location", "/");
    httpd_resp_send(req, NULL, 0);  // Response body can be empty
    return ESP_OK;
}

/* Handler to respond with an icon file embedded in flash.
 * Browsers expect to GET website icon at URI /favicon.ico.
 * This can be overridden by uploading file with same name */
/*
static esp_err_t favicon_get_handler(httpd_req_t *req)
{
    extern const unsigned char favicon_ico_start[] asm("_binary_favicon_ico_start");
    extern const unsigned char favicon_ico_end[]   asm("_binary_favicon_ico_end");
    const size_t favicon_ico_size = (favicon_ico_end - favicon_ico_start);
    httpd_resp_set_type(req, "image/x-icon");
    httpd_resp_send(req, (const char *)favicon_ico_start, favicon_ico_size);
    return ESP_OK;
}
*/
esp_err_t spiffs_get_handler(httpd_req_t *req)
{
    return ESP_OK;
}

#define IS_FILE_EXT(filename, ext) \
    (strcasecmp(&filename[strlen(filename) - sizeof(ext) + 1], ext) == 0)

/* Set HTTP response content type according to file extension */
static esp_err_t set_content_type_from_file(httpd_req_t *req, const char *filename)
{
    if (IS_FILE_EXT(filename, ".pdf")) {
        return httpd_resp_set_type(req, "application/pdf");
    } else if (IS_FILE_EXT(filename, ".html")) {
        return httpd_resp_set_type(req, "text/html");
    } else if (IS_FILE_EXT(filename, ".jpeg")) {
        return httpd_resp_set_type(req, "image/jpeg");
    } else if (IS_FILE_EXT(filename, ".ico")) {
        return httpd_resp_set_type(req, "image/x-icon");
    }
    /* This is a limited set only */
    /* For any other type always set as plain text */
    return httpd_resp_set_type(req, "text/plain");
}

/* Copies the full path into destination buffer and returns
 * pointer to path (skipping the preceding base path) */
static const char* get_path_from_uri(char *dest, const char *base_path, const char *uri, size_t destsize)
{
    const size_t base_pathlen = strlen(base_path);
    size_t pathlen = strlen(uri);

    const char *quest = strchr(uri, '?');
    if (quest)
        pathlen = MIN(pathlen, quest - uri);

    const char *hash = strchr(uri, '#');
    if (hash)
        pathlen = MIN(pathlen, hash - uri);


    if (base_pathlen + pathlen + 1 > destsize)
        /* Full path string won't fit into destination buffer */
        return NULL;

    /* Construct full path (base + path) */
    strcpy(dest, base_path);
    strlcpy(dest + base_pathlen, uri, pathlen + 1);

    /* Return pointer to path, skipping the base */
    return dest + base_pathlen;
}

static bool getPayload(httpd_req_t *req, char *buf)
{
	size_t off = 0;
	int ret;

	if (buf == NULL) {
		ESP_LOGE(TAG, "Failed to allocate memory of %d bytes!", req->content_len + 1);
		httpd_resp_send_500(req);
		return false;
	}

	while (off < req->content_len) {
		/* Read data received in the request */
		ret = httpd_req_recv(req, buf + off, req->content_len - off);
		if (ret <= 0) {
			if (ret == HTTPD_SOCK_ERR_TIMEOUT)
				httpd_resp_send_408(req);
			return false;
		}
		off += ret;
		ESP_LOGI(TAG, "/echo handler recv length %d", ret);
	}
	buf[off] = '\0';

	return true;
}

#if SDCARD_ENABLE

/* Handler to download a file kept on the server */
static esp_err_t sdcard_get_handler(httpd_req_t *req)
{
    char filepath[FILE_PATH_MAX];

 //   sdcard_getfs(); // ensure card is mounted

    const char *filename = get_path_from_uri(filepath, ((file_server_data_t *)req->user_ctx)->base_path, &req->uri[7], sizeof(filepath));

//const char *filename = &req->uri[7];

//strcpy(filepath, filename);

	ESP_LOGE(TAG, "%s - %s", req->uri, filename);

	if (!filename) {
        ESP_LOGE(TAG, "Filename is too long");
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Filename too long");
        return ESP_FAIL;
    }

    /* If name has trailing '/', respond with directory contents */
    if (filename[strlen(filename) - 1] == '/') {
   //     return http_resp_dir_html(req, filepath);
    }

    FIL sdfile, *fh;
//    size_t fsize;

    if (f_open(&sdfile, filename, FA_READ) != FR_OK) {
        /* If file not present on SPIFFS check if URI
         * corresponds to one of the hardcoded paths */
        if (strcmp(filename, "/index.html") == 0) {
            return index_html_get_handler(req);
        } else if (strcmp(filename, "/favicon.ico") == 0) {
      //      return favicon_get_handler(req);
        }
        ESP_LOGE(TAG, "Failed to stat file : %s", filepath);
        /* Respond with 404 Not Found */
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File does not exist");
        return ESP_FAIL;
    }

    fh = &sdfile;

    if (fh == NULL) {
        ESP_LOGE(TAG, "Failed to read existing file : %s", filepath);
        /* Respond with 500 Internal Server Error */
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to read existing file");
        return ESP_FAIL;
    }

//    fsize = f_size(fh);

//    ESP_LOGI(TAG, "Sending file : %s (%ul bytes)...", filename, fsize);
    set_content_type_from_file(req, filename);

    /* Retrieve the pointer to scratch buffer for temporary storage */
    char *chunk = ((file_server_data_t *)req->user_ctx)->scratch;
    size_t chunksize;
    do {
        /* Read file in chunks into the scratch buffer */
        f_read(fh, chunk, SCRATCH_BUFSIZE, &chunksize);

        /* Send the buffer contents as HTTP response chunk */
        if (httpd_resp_send_chunk(req, chunk, chunksize) != ESP_OK) {
        	f_close(fh);
            ESP_LOGE(TAG, "File sending failed!");
            /* Abort sending file */
            httpd_resp_sendstr_chunk(req, NULL);
            /* Respond with 500 Internal Server Error */
            httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to send file");
            return ESP_FAIL;
        }

        /* Keep looping till the whole file is sent */
    } while (chunksize != 0);

    /* Close file after sending complete */
    f_close(fh);
    ESP_LOGI(TAG, "File sending complete");

    /* Respond with an empty chunk to signal HTTP response completion */
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

#endif

// add setting to the JSON response array
static bool add_setting (int id, char *value)
{
	bool ok = true;

	cJSON *setting;

	if((ok = (setting = cJSON_CreateObject()) != NULL))
	{
		ok = cJSON_AddNumberToObject(setting, "id", (double)id) != NULL;
		ok &= cJSON_AddStringToObject(setting, "value", value) != NULL;
		if(ok)
			cJSON_AddItemToArray(json_settings, setting);
	}

	return ok;
}

// Used for trapping settings report and generating a JSON array
static void backendWriteS (const char *data)
{
	strcat(sbuf, data);

	uint32_t len = strlen(sbuf);

	if(sbuf[len - 2] == '\r')
	{
		sbuf[len - 2] = '\0';
		char *eq = strchr(sbuf, '=');
		if(eq) {
			*eq = '\0';
			uint_fast8_t counter = 1;
			float parameter;
			read_float(sbuf, &counter, &parameter);
			if(setting == -1 || (int)parameter == setting)
				add_setting((int)parameter, eq + 1);
		}
		sbuf[0] = '\0';
	}
}

static esp_err_t settings_get_handler(httpd_req_t *req)
{
	sbuf[0] = '\0';

//	size_t ql = httpd_req_get_url_query_len(req);
	if(strlen(req->uri) > 9) {
		setting = atoi(&req->uri[10]);
	} else
		setting = -1;

	cJSON *root = cJSON_CreateObject();

	if((root && (json_settings = cJSON_AddArrayToObject(root, "settings")))) {

		org_stream = hal.stream.write;
		hal.stream.write = backendWriteS;

		report_grbl_settings();

		hal.stream.write = org_stream;

		char *resp = cJSON_PrintUnformatted(root);

		httpd_resp_set_type(req, HTTPD_TYPE_JSON);
		httpd_resp_send(req, resp, strlen(resp));

		free(resp);

	} else {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to generate response");
        return ESP_FAIL;
	}

	if(root)
		cJSON_Delete(root);

    return ESP_OK;
}

static esp_err_t settings_set_handler(httpd_req_t *req)
{
//	heap_caps_print_heap_info(MALLOC_CAP_DEFAULT);

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

				if((status = settings_store_global_setting ((setting_type_t)((int)id->valuedouble), value->valuestring)) != Status_OK)
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

				httpd_resp_set_type(req, HTTPD_TYPE_JSON);
				httpd_resp_send(req, resp, strlen(resp));

				ESP_LOGI(TAG, "%s\n", resp);

				free(resp);
			}

			if(root)
				cJSON_Delete(root);
		}

		wifi_release_aplist();

		if(!ok)
			httpd_resp_send_500(req);
	}

	return ok ? ESP_OK : ESP_FAIL;
}

static esp_err_t wifi_login_handler (httpd_req_t *req)
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

static const httpd_uri_t basic_handlers[] = {
    { .uri      = "/spiffs",
      .method   = HTTP_GET,
      .handler  = spiffs_get_handler,
      .user_ctx = NULL,
    },
#if SDCARD_ENABLE
    { .uri      = "/sdcard/*",
      .method   = HTTP_GET,
      .handler  = sdcard_get_handler,
      .user_ctx = &file_server_data,
    },
#endif
    { .uri      = "/settings",
      .method   = HTTP_GET,
      .handler  = settings_get_handler,
      .user_ctx = NULL,
    },
    { .uri      = "/settings/*",
      .method   = HTTP_GET,
      .handler  = settings_get_handler,
      .user_ctx = NULL,
    },
    { .uri      = "/settings",
      .method   = HTTP_POST,
      .handler  = settings_set_handler,
      .user_ctx = NULL,
    },
    { .uri      = "/wifi",
      .method   = HTTP_GET,
      .handler  = wifi_scan_handler,
      .user_ctx = NULL,
    },
    { .uri      = "/wifi",
      .method   = HTTP_POST,
      .handler  = wifi_login_handler,
      .user_ctx = NULL,
    }
};

void register_basic_handlers(httpd_handle_t hd)
{
    uint32_t i = sizeof(basic_handlers) / sizeof(httpd_uri_t);

    ESP_LOGI(TAG, "Registering basic handlers");
    ESP_LOGI(TAG, "No of handlers = %d", i);

	wifi_mode_t currentMode;

	if(esp_wifi_get_mode(&currentMode) != WIFI_MODE_APSTA)
		i -= 2;

    do {
        if (httpd_register_uri_handler(hd, &basic_handlers[--i]) != ESP_OK) {
            ESP_LOGW(TAG, "register uri failed for %d", i);
            return;
        }
    } while(i);

    ESP_LOGI(TAG, "Success");
}

void httpdaemon_stop (void)
{
    if(httpdaemon) {
		httpd_stop(httpdaemon);
//		ESP_LOGI(TAG, "HTTPD Stop: Current free memory: %d", esp_get_free_heap_size());
    }

    httpdaemon = NULL;
}

bool httpdaemon_start (network_settings_t *network)
{
    //pre_start_mem = esp_get_free_heap_size();

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();

    config.server_port = network->http_port;
    config.uri_match_fn = httpd_uri_match_wildcard;
	config.stack_size = 8192;

    httpdaemon_stop();

    /* This check should be a part of http_server */
    config.max_open_sockets = (CONFIG_LWIP_MAX_SOCKETS - 3);

    file_server_data.base_path[0] = '\0';

    if (httpd_start(&httpdaemon, &config) == ESP_OK) {
        ESP_LOGI(TAG, "Started HTTP server on port: '%d'", config.server_port);
        ESP_LOGI(TAG, "Max URI handlers: '%d'", config.max_uri_handlers);
        ESP_LOGI(TAG, "Max Open Sessions: '%d'", config.max_open_sockets);
        ESP_LOGI(TAG, "Max Header Length: '%d'", HTTPD_MAX_REQ_HDR_LEN);
        ESP_LOGI(TAG, "Max URI Length: '%d'", HTTPD_MAX_URI_LEN);
        ESP_LOGI(TAG, "Max Stack Size: '%d'", config.stack_size);
		register_basic_handlers(httpdaemon);
	}

    return httpdaemon != NULL;
}

