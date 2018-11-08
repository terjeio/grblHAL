/*
  wifi.c - An embedded CNC Controller with rs274/ngc (g-code) support

  WiFi comms

  Part of Grbl

  Copyright (c) 2018 Terje Io

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

#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"

#include "nvs_flash.h"

#include "lwip/timeouts.h"

#include "wifi.h"
#include "driver.h"
#include "TCPStream.h"

static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */

static uint16_t port;
const static int CONNECTED_BIT = BIT0;

const static char *TAG = "grbl wifi";

void lwIPHostTimerHandler (void *arg)
{
    TCPStreamHandler();
    sys_timeout(STREAM_POLL_INTERVAL, lwIPHostTimerHandler, NULL);
}

static esp_err_t wifi_event_handler (void *ctx, system_event_t *event)
{
    switch(event->event_id) {

    	case SYSTEM_EVENT_STA_START:
			esp_wifi_connect();
			break;

    	case SYSTEM_EVENT_STA_GOT_IP:
    		hal.stream_write_all("[MSG:WIFI ACTIVE]\r\n");
			xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
		    TCPStreamInit();
		    TCPStreamListen(port);
//			ESP_LOGI(TAG, "got ip:%s\n",	ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
    	    sys_timeout(STREAM_POLL_INTERVAL, lwIPHostTimerHandler, NULL);
			break;

    	case SYSTEM_EVENT_STA_DISCONNECTED:
		    selectStream(StreamSetting_Serial); // Fall back to previous?
    		hal.stream_write_all("[MSG:WIFI DISCONNECTED]\r\n");
			/* This is a workaround as ESP32 WiFi libs don't currently
			   auto-reassociate. */
			esp_wifi_connect();
			xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
			//TCPStreamNotifyLinkStatus(false);
			break;

    	default:
    		break;
    }

    return ESP_OK;
}

bool wifi_init (wifi_settings_t *settings)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if((ret = nvs_flash_erase()) == ESP_OK)
        	ret = nvs_flash_init();
    }

	if(ret != ESP_OK)
    	return false;

    tcpip_adapter_init();

    //tcpip_adapter_set_hostname()
    wifi_event_group = xEventGroupCreate();

    // TODO: add error messages on fail?

    if((ret = esp_event_loop_init(wifi_event_handler, NULL)) != ESP_OK)
    	return false;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

    if((ret = esp_wifi_init(&cfg)) != ESP_OK)
    	return false;

    if((ret = esp_wifi_set_storage(WIFI_STORAGE_RAM)) != ESP_OK)
    	return false;

    wifi_config_t wifi_config;
    memset(&wifi_config, 0, sizeof(wifi_config));

    if(strlcpy((char *)wifi_config.sta.ssid, settings->ssid, sizeof(wifi_config.sta.ssid)) >= sizeof(wifi_config.sta.ssid))
    	return false;

    if(strlcpy((char *)wifi_config.sta.password, settings->password, sizeof(wifi_config.sta.password)) >= sizeof(wifi_config.sta.password))
    	return false;

    if((ret = esp_wifi_set_mode(WIFI_MODE_STA)) != ESP_OK)
    	return false;

    if((ret = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config)) != ESP_OK)
    	return false;

    ESP_LOGI(TAG, "start the WIFI SSID:[%s] password:[%s]\n", wifi_config.sta.ssid, wifi_config.sta.password);

    if((ret = esp_wifi_start()) != ESP_OK)
    	return false;

    port = settings->port == 0 ? 23 : settings->port;

    return true;
}
