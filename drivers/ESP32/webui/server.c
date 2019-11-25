/*
  webui/server.c - An embedded CNC Controller with rs274/ngc (g-code) support

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

#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

#include <cJSON.h>

#include "esp_spiffs.h"

#include "driver.h"
#include "wifi.h"
#include "webui.h"
#include "networking/WsStream.h"
#include "networking/urldecode.h"
#include "networking/utils.h"
#include "networking/strutils.h"
#include "web/upload.h"

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "esp_vfs_fat.h"
#endif

char *get_key_value (char *qstring, char *key, char *s)
{
	char buf[100];

	if(httpd_query_key_value(qstring, key, buf, sizeof(buf)) == ESP_OK)
		urldecode(s, buf);
	else
		*s = '\0';

	return key;
}

esp_err_t webui_http_command_handler (httpd_req_t *req)
{
	bool ok;
	char *data = NULL, *cmd;

	char buf[200];

	buf[100] = '\0';

	httpd_req_get_url_query_str(req, buf, sizeof(buf));

	if(httpd_query_key_value(buf, "commandText", &buf[100], sizeof(buf) - 100) == ESP_OK) {
		buf[0] = '\0';
		data = urldecode(buf, &buf[100]);
	} else if(httpd_query_key_value(buf, "plain", &buf[100], sizeof(buf) - 100) == ESP_OK) {
		buf[0] = '\0';
		data = urldecode(buf, &buf[100]);
	}

//	hal.delay_ms(100, NULL);

	if((ok = (data != NULL))) {

		if((cmd = strstr(data, "[ESP"))) {

			cmd += 4;

			char *args = NULL;

			if((ok = (args = strchr(cmd, ']')))) {
				*args++ = '\0';
				webui_set_http_request(req);
				ok = webui_command_handler(atol(cmd), args) == Status_OK;
			}

		} else { // GCode - add to websocket input buffer

			if(strlen(data) == 1)
				WsStreamRxInsert(*data);

			else {

				size_t len;
				char c, *block = strtok(data, "\n");

				while(block) {

					if((len = strlen(block)) == 2 && *block == 0xC2) {
						block++;
						len--;
					}

					while((c = *block++))
						WsStreamRxInsert(c);

					if(len > 1)
						WsStreamRxInsert(ASCII_LF);

					block = strtok(NULL, "\n");
				}
			}

			httpd_resp_send(req, NULL, 0);
		}
	}

    return ok ? ESP_OK : ESP_FAIL;
}

#if SDCARD_ENABLE

// add file to the JSON response array
static bool add_file (cJSON *files, char *path, FILINFO *file)
{
	bool ok;

/*
	char buf[200];

	if(strlen(path) > 1) {
		strcpy(buf, &path[1]);
		strcat(buf, "/");
	} else
		*buf = '\0';

	strcat(buf, file->fname);
*/
	cJSON *fileinfo;

	if((ok = (fileinfo = cJSON_CreateObject()) != NULL))
	{
		ok = cJSON_AddStringToObject(fileinfo, "name", file->fname) != NULL;
		ok &= cJSON_AddStringToObject(fileinfo, "shortname", file->fname) != NULL;
		ok &= cJSON_AddStringToObject(fileinfo, "datetime", "") != NULL;
		if(file->fattrib & AM_DIR)
			ok &= cJSON_AddNumberToObject(fileinfo, "size", -1.0f) != NULL;
		else
			ok &= cJSON_AddStringToObject(fileinfo, "size", btoa(file->fsize)) != NULL;
		if(ok)
			cJSON_AddItemToArray(files, fileinfo);
	}

	return ok;
}

static FRESULT sd_scan_dir (cJSON *files, char *path, uint_fast8_t depth)
{
#if defined(ESP_PLATFORM)
    FF_DIR dir;
#else
    DIR dir;
#endif
    FILINFO fno;
    FRESULT res;
    bool subdirs = false;
#if _USE_LFN
    static TCHAR lfn[_MAX_LFN + 1];   /* Buffer to store the LFN */
    fno.lfname = lfn;
    fno.lfsize = sizeof(lfn);
#endif

   if((res = f_opendir(&dir, path)) != FR_OK)
        return res;

   // Pass 1: Scan files
    while(true) {

        if((res = f_readdir(&dir, &fno)) != FR_OK || fno.fname[0] == '\0')
            break;

		subdirs |= fno.fattrib & AM_DIR;

        if(!(fno.fattrib & AM_DIR))
        	add_file(files, path, &fno);
    }

    if((subdirs = (subdirs && depth)))
    	f_readdir(&dir, NULL); // Rewind

	// Pass 2: Scan directories
    while(subdirs) {

        if((res = f_readdir(&dir, &fno)) != FR_OK || *fno.fname == '\0')
            break;

        if((fno.fattrib & AM_DIR) && strcmp(fno.fname, "System Volume Information")) {

            size_t pathlen = strlen(path);
//			if(pathlen + strlen(get_name(&fno)) > (MAX_PATHLEN - 1))
				//break;
        	add_file(files, path, &fno);
        	if(depth > 1) {
				sprintf(&path[pathlen], "/%s", fno.fname);
				if((res = sd_scan_dir(files, path, depth - 1)) != FR_OK)
					break;
				path[pathlen] = '\0';
        	}
        }
    }

#if defined(__MSP432E401Y__) || defined(ESP_PLATFORM)
    f_closedir(&dir);
#endif

    return res;
}

static bool sd_ls (httpd_req_t *req, char *path, char *status)
{
	bool ok;
	cJSON *root = cJSON_CreateObject(), *files = NULL;

	if((ok = (root && (files = cJSON_AddArrayToObject(root, "files"))))) {

		if(strlen(path) > 1)
			path[strlen(path) - 1] = '\0';

		sd_scan_dir(files, path, 1);

		cJSON_AddStringToObject(root, "path", path);

	    FATFS *fs;
	    DWORD fre_clust, used_sect, tot_sect;

	    if(f_getfree("", &fre_clust, &fs) == FR_OK) {
	        tot_sect = (fs->n_fatent - 2) * fs->csize;
	        used_sect = tot_sect - fre_clust * fs->csize;
	        uint32_t pct_used = (used_sect * 100) / tot_sect;
	        cJSON_AddStringToObject(root, "total", btoa(tot_sect << 9)); // assuming 512 byte sector size
	        cJSON_AddStringToObject(root, "used", btoa(used_sect << 9));
	        cJSON_AddStringToObject(root, "occupation", uitoa(pct_used == 0 ? 1 : pct_used));
	    }
		cJSON_AddStringToObject(root, "mode", "direct");
		cJSON_AddStringToObject(root, "status", status);

		char *resp = cJSON_PrintUnformatted(root);

	    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
		httpd_resp_set_type(req, HTTPD_TYPE_JSON);
		httpd_resp_send(req, resp, strlen(resp));

		free(resp);
	}

	if(root)
		cJSON_Delete(root);

	return ok;
}

static bool sd_rmdir (char *path)
{
	bool ok = true;

#if defined(ESP_PLATFORM)
    FF_DIR dir;
#else
    DIR dir;
#endif
    FILINFO fno;

#if _USE_LFN
    static TCHAR lfn[_MAX_LFN + 1];   /* Buffer to store the LFN */
    fno.lfname = lfn;
    fno.lfsize = sizeof(lfn);
#endif

   if(f_opendir(&dir, path) != FR_OK)
        return false;

	size_t pathlen = strlen(path);

    while(ok) {

        if(f_readdir(&dir, &fno) != FR_OK || *fno.fname == '\0')
            break;

    	strcat(strcat(path, "/"), fno.fname);

        ok = (fno.fattrib & AM_DIR) ? sd_rmdir(path) : f_unlink(path) == FR_OK;

        path[pathlen] = '\0';
    }

#if defined(__MSP432E401Y__) || defined(ESP_PLATFORM)
    f_closedir(&dir);
#endif

	return ok && f_unlink(path) == FR_OK;
}

esp_err_t webui_sdcard_handler (httpd_req_t *req)
{
	bool ok;
	char *query = NULL;
	char path[100], filename[100], action[20], status[100];

	size_t qlen = httpd_req_get_url_query_len(req);

	*status = '\0';

	if(qlen && (query = malloc(qlen))) {

		httpd_req_get_url_query_str(req, query, qlen);

		get_key_value(query, "path", path);
		get_key_value(query, "filename", filename);
		get_key_value(query, "action", action);

		if(*action && *filename) {

			FILINFO file;

			char *fullname = ((file_server_data_t *)req->user_ctx)->scratch;

			strcat(strcpy(fullname, path), filename);

			switch(strlookup(action, "delete,createdir,deletedir", ',')) {

				case 0: // delete
					if(f_stat(fullname, &file) == FR_OK) {
						if(!(file.fattrib & AM_DIR) && f_unlink(fullname) == FR_OK)
							sprintf(status, "%s deleted", filename);
						else
							sprintf(status, "Cannot delete %s!", filename);
					} else
						sprintf(status, "%s does not exist!", filename);
					break;

				case 1: // createdir
					if(f_stat(fullname, &file) != FR_OK) {
						if(f_mkdir(fullname) == FR_OK)
							sprintf(status, "%s created", filename);
						else
							sprintf(status, "Cannot create %s!", filename);
					} else
						sprintf(status, "%s already exists!", filename);
					break;

				case 2: // deletedir
					if(strlen(fullname) == 1)
						strcpy(status, "Cannot delete root directory!");
					else if(f_stat(fullname, &file) == FR_OK) {
						if(sd_rmdir(fullname))
							sprintf(status, "%s deleted", filename);
						else
							sprintf(status, "Error deleting %s!", filename);
					} else
						sprintf(status, "%s does not exist!", filename);
					break;

				default:
					sprintf(status, "Invalid action \"%s\" for %s!", action, filename);
					break;
			}
		}
	}

	if(*path == '\0')
		strcpy(path, "/");

	if(!(ok = sd_ls(req, path, status)))
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to generate response");

	if(query)
		free(query);

    return ok ? ESP_OK : ESP_FAIL;
}

esp_err_t webui_sdcard_upload_handler (httpd_req_t *req)
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

	char path[100];
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

		if(*path == '\0' && *upload->path)
			strcpy(path, upload->path);

	} while(upload->state != Upload_Complete);

	if(*path == '\0') // in case something failed...
		strcpy(path, "/");

	if(!(ok && sd_ls(req, path, "ok")))
		httpd_resp_send_err(req, 400, "Upload failed"); // or did it?

	if(req->sess_ctx && req->free_ctx) {
		req->free_ctx(req->sess_ctx);
		req->sess_ctx = NULL;
	}

	hal.stream.write(ok ? "[MSG:Upload ok]\r\n" : "[MSG:Upload failed]\r\n");

	if(rqhdr)
		free(rqhdr);

	return ok ? ESP_OK : ESP_FAIL;
}

static bool spiffs_isdirectory(char *filename)
{
	return strrchr(filename, '/') && filename[strlen(filename) - 1] == '.';
}

static bool spiffs_add_file (cJSON *files, char *filename, struct stat *file)
{
	bool ok;
	cJSON *fileinfo;

	if((ok = (fileinfo = cJSON_CreateObject()) != NULL))
	{
		ok = cJSON_AddStringToObject(fileinfo, "name", filename) != NULL;
		if(S_ISDIR(file->st_mode))
			ok &= cJSON_AddNumberToObject(fileinfo, "size", -1.0f) != NULL;
		else
			ok &= cJSON_AddStringToObject(fileinfo, "size", btoa(file->st_size)) != NULL;
		if(ok)
			cJSON_AddItemToArray(files, fileinfo);
	}

	return ok;
}

static bool spiffs_scan_dir (cJSON *files, char *path, uint_fast8_t depth)
{
    DIR *dir;
    struct dirent *entry;
    struct stat file;
    char *fname;

    if(!(dir = opendir(path)))
        return false;

    size_t pathlen = strlen(path);

    if(path[pathlen - 1] != '/')
    	path[pathlen++] = '/';

    while((entry = readdir(dir))) {

    	strcat(path, entry->d_name);

        if(stat(path, &file) == 0) {
	    	fname = strrchr(path, '/');
        	if(spiffs_isdirectory(fname)) {
    	    	file.st_mode = S_IFDIR;
    	    	*fname = '\0';
    	    	fname = strrchr(path, '/') + 1;
        	} else
        		fname++;

	        if(fname - path == pathlen)
        		spiffs_add_file(files, fname, &file);
            if(path[pathlen - 1] == '\0')
				path[pathlen - 1] = '/';
        }

        path[pathlen] = '\0';
    }

    closedir(dir);

    return true;
}

static bool spiffs_ls (httpd_req_t *req, char *path, char *status)
{
	bool ok;
	cJSON *root = cJSON_CreateObject(), *files = NULL;

	if((ok = (root && (files = cJSON_AddArrayToObject(root, "files"))))) {

		if(strlen(path) > 1)
			path[strlen(path) - 1] = '\0';

		spiffs_scan_dir(files, path, 1);

		cJSON_AddStringToObject(root, "path", path);

	    size_t total = 0, used = 0;

	    if(esp_spiffs_info(NULL, &total, &used) == ESP_OK) {

	        uint32_t pct_used = (used * 100) / total;
	        cJSON_AddStringToObject(root, "total", btoa(total));
	        cJSON_AddStringToObject(root, "used", btoa(used));
	        cJSON_AddStringToObject(root, "occupation", uitoa(pct_used == 0 ? 1 : pct_used));
	    }
		cJSON_AddStringToObject(root, "mode", "direct");
		cJSON_AddStringToObject(root, "status", status);

		char *resp = cJSON_PrintUnformatted(root);

	    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
		httpd_resp_set_type(req, HTTPD_TYPE_JSON);
		httpd_resp_send(req, resp, strlen(resp));

		free(resp);
	}

	if(root)
		cJSON_Delete(root);

	return ok;
}

static bool spiffs_rmdir (char *path)
{
	bool ok = true;
    DIR *dir;
    struct dirent *entry;
    char filepath[200];

    if(!(dir = opendir("/spiffs")))
        return false;

    path += 8; // skip "/spiffs/" part

    size_t pathlen = strlen(path);

    while((entry = readdir(dir))) {
    	if(strlen(entry->d_name) >= pathlen && memcmp(entry->d_name, path, pathlen) == 0) {
    		strcat(strcpy(filepath, "/spiffs/"), entry->d_name);
    		ok = ok & (unlink(filepath) == 0);
        }
    }

    closedir(dir);

	return ok;
}

esp_err_t webui_spiffs_handler (httpd_req_t *req)
{
	bool ok;
	char *query = NULL;
	char path[100], filename[100], action[20], status[100];
	size_t qlen = httpd_req_get_url_query_len(req);

	*path = '\0';
	*status = '\0';

	if(qlen && (query = malloc(qlen))) {

		httpd_req_get_url_query_str(req, query, qlen);

		get_key_value(query, "path", path);
		get_key_value(query, "filename", filename);
		get_key_value(query, "action", action);

		if(*action && *filename) {

		    struct stat file;
			char *fullname = ((file_server_data_t *)req->user_ctx)->scratch;

			strcat(strcpy(fullname, "/spiffs"), path);
			strcpy(path, fullname);
			strcat(fullname, filename);

			switch(strlookup(action, "delete,createdir,deletedir,list", ',')) {

				case 0: // delete
					if(stat(fullname, &file) == 0) {
						if(!spiffs_isdirectory(filename) && unlink(fullname) == 0)
							sprintf(status, "%s deleted", filename);
						else
							sprintf(status, "Cannot delete %s!", filename);
					} else
						sprintf(status, "%s does not exist!", filename);
					break;

				case 1: // createdir
					strcat(fullname, "/.");
					if(stat(fullname, &file) != 0) {
						FILE *dir;
						if((dir = fopen(fullname, "w"))) {
							sprintf(status, "%s created", filename);
							fclose(dir);
						} else
							sprintf(status, "Cannot create %s!", filename);
					} else
						sprintf(status, "%s already exists!", filename);
					break;

				case 2: // deletedir
					strcat(fullname, "/.");
					if(strlen(fullname) == 10)
						strcpy(status, "Cannot delete root directory!");
					else if(stat(fullname, &file) == 0) {
						if(spiffs_rmdir(fullname))
							sprintf(status, "%s deleted", filename);
						else
							sprintf(status, "Error deleting %s!", filename);
					} else
						sprintf(status, "%s does not exist!", filename);
					break;

				case 3: // list
					{
						strcpy(path, fullname);
						char *file = strrchr(path, '/');
						*++file = '\0'; // strip filename... WebUI sends "all" for some reason - which is not handled in original backend
					}
					break;

				default:
					sprintf(status, "Invalid action \"%s\" for %s!", action, filename);
					break;
			}
		}
	}

	if(*path == '\0')
		strcpy(path, "/spiffs/");

	if(!(ok = spiffs_ls(req, path, status)))
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to generate response");

	if(query)
		free(query);

    return ok ? ESP_OK : ESP_FAIL;
}

esp_err_t webui_spiffs_upload_handler (httpd_req_t *req)
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
			ok = upload_start(req, boundary, false);
		}
	}

	char path[100];
	char *scratch = ((file_server_data_t *)req->user_ctx)->scratch;
	file_upload_t *upload = (file_upload_t *)req->sess_ctx;

	*path = '\0';

	if (ok) do { // Process received data

		if ((ret = httpd_req_recv(req, scratch, sizeof(fs_scratch_t))) <= 0) {
			ok = false;
			if (ret == HTTPD_SOCK_ERR_TIMEOUT)
				httpd_resp_send_408(req);
			break;
		}

		upload_chunk(req, scratch, (size_t)ret);

		if(*path == '\0' && *upload->path)
			strcat(strcpy(path, "/spiffs"), upload->path);

	} while(upload->state != Upload_Complete);

	if(*path == '\0') // in case something failed...
		strcpy(path, "/");

	if(!(ok = spiffs_ls(req, path, "ok")))
		httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "Failed to generate response");

	if(req->sess_ctx && req->free_ctx) {
		req->free_ctx(req->sess_ctx);
		req->sess_ctx = NULL;
	}

	if(rqhdr)
		free(rqhdr);

	return ok ? ESP_OK : ESP_FAIL;
}

#endif

esp_err_t webui_index_html_get_handler (httpd_req_t *req)
{
    extern const unsigned char index_html_gz_start[] asm("_binary_index_html_gz_start");
    extern const unsigned char index_html_gz_end[]   asm("_binary_index_html_gz_end");
    set_content_type_from_file(req, "index.html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    httpd_resp_send(req, (const char *)index_html_gz_start, index_html_gz_end - index_html_gz_start);
    return ESP_OK;
}
