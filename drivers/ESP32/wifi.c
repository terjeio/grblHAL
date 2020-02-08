/*
  wifi.c - An embedded CNC Controller with rs274/ngc (g-code) support

  WiFi comms

  Part of GrblHAL

  Copyright (c) 2018-2020 Terje Io

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

#include "networking/networking.h"
#include "networking/utils.h"
//#include "lwip/timeouts.h"

#include "wifi.h"
#include "driver.h"
#include "dns_server.h"
#include "web/backend.h"

static EventGroupHandle_t wifi_event_group;

/* The event group allows multiple bits for each event,
   but we only care about one event - are we connected
   to the AP with an IP? */

static network_settings_t *network;
const static int CONNECTED_BIT = BIT0;
const static int SCANNING_BIT = BIT1;
const static int APSTA_BIT = BIT2;

static network_services_t services = {0};
static wifi_config_t wifi_sta_config;
static SemaphoreHandle_t aplist_mutex = NULL;
static ap_list_t ap_list = {0};

ap_list_t *wifi_get_aplist (void)
{
    if(ap_list.ap_records && xSemaphoreTake(aplist_mutex, pdMS_TO_TICKS(10)) == pdTRUE)
        return &ap_list;
    else
        return NULL;
}

void wifi_release_aplist (void)
{
    xSemaphoreGive(aplist_mutex);
}

char *iptoa(void *ip) {
    static char aip[INET6_ADDRSTRLEN];
    inet_ntop(AF_INET, ip, aip, INET6_ADDRSTRLEN);
    return aip;
}

char *wifi_get_ip (void)
{
    ip4_addr_t *ip;

#if NETWORK_IPMODE_STATIC
    ip = (ip4_addr_t *)&driver_settings.wifi.sta.network.ip;
#else
    ip = ap_list.ap_selected ? &ap_list.ip_addr : (ip4_addr_t *)&driver_settings.wifi.ap.network.ip;
#endif

    return iptoa(ip);
}

bool wifi_dns_running (void)
{
    return services.dns == On;
}

static void lwIPHostTimerHandler (void *arg)
{
#if TELNET_ENABLE
    if(services.telnet)
        TCPStreamPoll();
#endif
#if WEBSOCKET_ENABLE
    if(services.telnet)
        WsStreamPoll();
#endif
    if(services.mask)
        sys_timeout(STREAM_POLL_INTERVAL, lwIPHostTimerHandler, NULL);
}

static void start_services (void)
{
#if TELNET_ENABLE
    if(network->services.telnet && !services.telnet) {
        TCPStreamInit();
        TCPStreamListen(network->telnet_port == 0 ? 23 : network->telnet_port);
        services.telnet = On;
        sys_timeout(STREAM_POLL_INTERVAL, lwIPHostTimerHandler, NULL);
    }
#endif
#if WEBSOCKET_ENABLE
    if(network->services.websocket && !services.websocket) {
        WsStreamInit();
        WsStreamListen(network->websocket_port == 0 ? 80 : network->websocket_port);
        services.websocket = On;
        sys_timeout(STREAM_POLL_INTERVAL, lwIPHostTimerHandler, NULL);
    }
#endif
#if HTTP_ENABLE
    if(network->services.http && !services.http)
        services.http = httpdaemon_start(network);
#endif
}

static void stop_services (void)
{
#if HTTP_ENABLE
    if(services.http)
        httpdaemon_stop();
#endif
#if TELNET_ENABLE
    if(services.telnet)
        TCPStreamClose();
#endif
#if WEBSOCKET_ENABLE
    if(services.dns)
        dns_server_stop();
#endif
    if(services.dns)
        dns_server_stop();

    services.mask = 0;
}

static void wifi_ap_scan (void)
{
    // https://esp32.com/viewtopic.php?t=5536
    // https://esp32.com/viewtopic.php?t=7305

    static const wifi_scan_config_t scan_config = {
        .ssid = 0,
        .bssid = 0,
        .channel = 0,
        .scan_time.active = {
          .min = 500,
          .max = 1500
        },
        .scan_type = WIFI_SCAN_TYPE_ACTIVE,
        .show_hidden = false
    };

    if(esp_wifi_scan_start(&scan_config, false) == ESP_OK)
        xEventGroupSetBits(wifi_event_group, SCANNING_BIT);
}

static esp_err_t wifi_event_handler (void *ctx, system_event_t *event)
{
    switch(event->event_id) {

        case SYSTEM_EVENT_AP_START:
            hal.stream.write_all("[MSG:WIFI AP READY]\r\n");
            start_services();
            if(xEventGroupGetBits(wifi_event_group) & APSTA_BIT) {
                dns_server_start();
                services.dns = On;
            }
            break;

        case SYSTEM_EVENT_AP_STACONNECTED:
            hal.stream.write_all("[MSG:WIFI AP CONNECTED]\r\n");
//          start_services();
//          ESP_LOGI(TAG, "station:"MACSTR" join, AID=%d", MAC2STR(event->event_info.sta_connected.mac), event->event_info.sta_connected.aid);
            break;

        case SYSTEM_EVENT_SCAN_DONE:

            xEventGroupClearBits(wifi_event_group, SCANNING_BIT);

            if(xSemaphoreTake(aplist_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

                if(ap_list.ap_records)
                    free(ap_list.ap_records);

                ap_list.ap_num = 0;

                esp_wifi_scan_get_ap_num(&ap_list.ap_num);

                if((ap_list.ap_records = (wifi_ap_record_t *)malloc(sizeof(wifi_ap_record_t) * ap_list.ap_num)) != NULL)
                    esp_wifi_scan_get_ap_records(&ap_list.ap_num, ap_list.ap_records);

                hal.stream.write_all("[MSG:WIFI AP SCAN COMPLETED]" ASCII_EOL);

                xSemaphoreGive(aplist_mutex);
            }
            // Start a new scan in 10 secs if no station connected...
//          if(!(xEventGroupGetBits(wifi_event_group) & CONNECTED_BIT))
//              wifi_ap_scan();
            break;

        case SYSTEM_EVENT_AP_STADISCONNECTED:
            selectStream(StreamType_Serial); // Fall back to previous?
            hal.stream.write_all("[MSG:WIFI AP DISCONNECTED]\r\n");
//          ESP_LOGI(TAG, "station:"MACSTR"leave, AID=%d", MAC2STR(event->event_info.sta_disconnected.mac), event->event_info.sta_disconnected.aid);
            break;
                
        case SYSTEM_EVENT_STA_START:
            if(wifi_sta_config.sta.ssid[0] != '\0')
                esp_wifi_connect();
            break;

        case SYSTEM_EVENT_STA_GOT_IP:
            // handle IP change (ip_change)
            hal.stream.write_all("[MSG:WIFI STA ACTIVE]\r\n");
            xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
            ap_list.ap_selected = wifi_sta_config.sta.ssid;
            memcpy(&ap_list.ip_addr, &event->event_info.got_ip.ip_info.ip, sizeof(ip4_addr_t));
            strcpy(ap_list.ap_status, "Connected");
            start_services();
            if(services.dns) {
                services.dns = Off;
                dns_server_stop();
            }
            if(xEventGroupGetBits(wifi_event_group) & APSTA_BIT) {
                strcpy(driver_settings.wifi.sta.ssid, (char *)wifi_sta_config.sta.ssid);
                strcpy(driver_settings.wifi.sta.password, (char *)wifi_sta_config.sta.password);
                // commit to EEPROM
            }
            break;

        case SYSTEM_EVENT_STA_DISCONNECTED:
            //stop_services();
            selectStream(StreamType_Serial); // Fall back to previous?
            hal.stream.write_all("[MSG:WIFI STA DISCONNECTED]\r\n");

            switch(event->event_info.disconnected.reason) {

                case WIFI_REASON_4WAY_HANDSHAKE_TIMEOUT:
                case WIFI_REASON_NO_AP_FOUND:
                    strcpy(ap_list.ap_status, "Connection failed");
                case WIFI_REASON_ASSOC_LEAVE:
                    memset(&wifi_sta_config, 0, sizeof(wifi_config_t));
                    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_sta_config);
                    ap_list.ap_selected = NULL;
                    if(true)
                        wifi_ap_scan();
                    break;

                default:
                    if(xEventGroupGetBits(wifi_event_group) & CONNECTED_BIT)
                        esp_wifi_connect(); // This is a workaround as ESP32 WiFi libs don't currently auto-reassociate.
                    break;
            }

            xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
            //TCPStreamNotifyLinkStatus(false);
            break;

        default:
            break;
    }

    return ESP_OK;
}

static bool init_adapter (tcpip_adapter_if_t tcpip_if, network_settings_t *settings)
{
    network = settings;

    tcpip_adapter_ip_info_t ipInfo;

    if(network->ip_mode == IpMode_Static) {
        ipInfo.ip = *((ip4_addr_t *)network->ip);
        ipInfo.gw = *((ip4_addr_t *)network->gateway);
        ipInfo.netmask = *((ip4_addr_t *)network->mask);
        tcpip_adapter_set_ip_info(tcpip_if, &ipInfo);
    }

    return network->ip_mode == IpMode_DHCP;
}

static wifi_mode_t settingToMode(grbl_wifi_mode_t mode)
{
    return mode == WiFiMode_AP ? WIFI_MODE_AP :
           mode == WiFiMode_STA ? WIFI_MODE_STA :
           mode == WiFiMode_APSTA ? WIFI_MODE_APSTA :
           WIFI_MODE_NULL;
}

bool wifi_init (wifi_settings_t *settings)
{
    esp_err_t ret;
    wifi_mode_t currentMode;

#if !WIFI_SOFTAP
    if(settings->mode == WiFiMode_APSTA) // Reset to default
        settings->mode = WIFI_MODE;
#endif

    if(esp_wifi_get_mode(&currentMode) == ESP_ERR_WIFI_NOT_INIT) {

        tcpip_adapter_init();

        wifi_event_group = xEventGroupCreate();
        aplist_mutex = xSemaphoreCreateMutex();

        // TODO: add error messages on fail?

        if(esp_event_loop_init(wifi_event_handler, NULL) != ESP_OK)
            return false;

        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

        if(esp_wifi_init(&cfg) != ESP_OK)
            return false;

        if(esp_wifi_set_storage(WIFI_STORAGE_RAM) != ESP_OK)
            return false;

        if(esp_wifi_set_mode(WIFI_MODE_NULL) != ESP_OK)
            return false;
    }

    wifi_config_t wifi_config;

    if(currentMode != settingToMode(settings->mode) && (settings->mode == WiFiMode_AP || settings->mode == WiFiMode_APSTA)) {

        tcpip_adapter_dhcps_stop(TCPIP_ADAPTER_IF_AP);

        settings->ap.network.ip_mode = IpMode_Static; // Only mode supported

        init_adapter(TCPIP_ADAPTER_IF_AP, &settings->ap.network);

        tcpip_adapter_dhcps_start(TCPIP_ADAPTER_IF_AP);

        memset(&wifi_config, 0, sizeof(wifi_config));

        if(settings->ap.ssid[0] == '\0')
            return false;

        if(strlcpy((char *)wifi_config.ap.ssid, settings->ap.ssid, sizeof(wifi_config.ap.ssid)) >= sizeof(wifi_config.ap.ssid))
            return false;

        if (strlen(settings->ap.password) == 0)
            wifi_config.ap.authmode = WIFI_AUTH_OPEN;
        else if(strlcpy((char *)wifi_config.ap.password, settings->ap.password, sizeof(wifi_config.ap.password)) < sizeof(wifi_config.ap.password))
            wifi_config.ap.authmode = WIFI_AUTH_WPA_WPA2_PSK;
        else
            return false;

        wifi_config.ap.max_connection = 1;

        if(esp_wifi_set_mode(settingToMode(settings->mode)) != ESP_OK)
            return false;

        if(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config) != ESP_OK)
            return false;

        if(settings->mode == WiFiMode_APSTA)
            xEventGroupSetBits(wifi_event_group, APSTA_BIT);

//      ESP_LOGI(TAG, "WIFI AP SSID:[%s] password:[%s]\n", wifi_config.ap.ssid, wifi_config.ap.password);
    }

    if(currentMode != settingToMode(settings->mode) && (settings->mode == WiFiMode_STA || settings->mode == WiFiMode_APSTA)) {

        tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA);
    
        settings->sta.network.ip_mode = IpMode_DHCP; // For now...

        if(init_adapter(TCPIP_ADAPTER_IF_STA, &settings->sta.network))
            tcpip_adapter_dhcpc_start(TCPIP_ADAPTER_IF_STA);

        memset(&wifi_sta_config, 0, sizeof(wifi_config));

        if(settings->sta.ssid[0] != '\0') {

            if(strlcpy((char *)wifi_sta_config.sta.ssid, settings->sta.ssid, sizeof(wifi_sta_config.sta.ssid)) >= sizeof(wifi_sta_config.sta.ssid))
                return false;

            if(strlcpy((char *)wifi_sta_config.sta.password, settings->sta.password, sizeof(wifi_sta_config.sta.password)) >= sizeof(wifi_sta_config.sta.password))
                return false;
        }

        if(esp_wifi_set_mode(settingToMode(settings->mode)) != ESP_OK)
            return false;

        if(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_sta_config) != ESP_OK)
            return false;

//      ESP_LOGI(TAG, "WIFI STA SSID:[%s] password:[%s]\n", wifi_sta_config.sta.ssid, wifi_sta_config.sta.password);
    }

    if(esp_wifi_start() != ESP_OK)
        return false;

    if(settings->mode == WiFiMode_AP ||settings->mode == WiFiMode_APSTA)
        tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_AP, settings->ap.network.hostname);

    if(settings->mode == WiFiMode_STA ||settings->mode == WiFiMode_APSTA)
        tcpip_adapter_set_hostname(TCPIP_ADAPTER_IF_STA, settings->sta.network.hostname);

    if(settings->mode == WiFiMode_APSTA)
        wifi_ap_scan();

    return true;
}

bool wifi_ap_connect (char *ssid, char *password)
{
    bool ok = !ssid || (strlen(ssid) > 0 && strlen(ssid) < sizeof(ssid_t) && strlen(password) < sizeof(password_t));

    if(!ok)
        return false;

    if(xEventGroupGetBits(wifi_event_group) & CONNECTED_BIT)
        esp_wifi_disconnect(); // TODO: delay until response is sent...

    if(xSemaphoreTake(aplist_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {

        ap_list.ap_selected = NULL;
        memset(&ap_list.ip_addr, 0, sizeof(ip4_addr_t));
        strcpy(ap_list.ap_status, ssid ? "Connecting..." : "");

        xSemaphoreGive(aplist_mutex);
    }

    memset(&wifi_sta_config, 0, sizeof(wifi_config_t));

    if(ssid) {

        strcpy((char *)wifi_sta_config.sta.ssid, ssid);
        strcpy((char *)wifi_sta_config.sta.password, password);

        ok = esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_sta_config) == ESP_OK &&
              esp_wifi_connect() == ESP_OK;
    }

    return ok;
}

bool wifi_stop (void)
{
    stop_services();

    esp_wifi_stop();

    return true;
}

#if WIFI_ENABLE

status_code_t wifi_setting (uint_fast16_t param, float value, char *svalue)
{
    status_code_t status = Status_Unhandled;

    if(svalue) switch(param) {

        case Setting_NetworkServices:
            if(isintf(value) && value >= 0.0f && value < 256.0f) {
                status = Status_OK;
                network_services_t is_available = {0};
#if TELNET_ENABLE
                is_available.telnet = On;
#endif
#if HTTP_ENABLE
                is_available.http = On;
#endif
#if WEBSOCKET_ENABLE
                is_available.websocket = On;
#endif
                driver_settings.wifi.sta.network.services.mask =
                 driver_settings.wifi.ap.network.services.mask = (uint8_t)value & is_available.mask;
                // TODO: fault if attempt to select services not available?
            } else
                status = Status_InvalidStatement; //out of range...
            break;

        case Setting_WifiMode:
            if(isintf(value) && value >= 0.0f && value < 4.0f) {
#if WIFI_SOFTAP
                status = Status_OK;
                driver_settings.wifi.mode = (grbl_wifi_mode_t)(uint8_t)value;
#else
                grbl_wifi_mode_t mode = (grbl_wifi_mode_t)(uint8_t)value;
                if((status = (mode == WiFiMode_AP || mode == WiFiMode_APSTA) ? Status_InvalidStatement : Status_OK) == Status_OK)
                    driver_settings.wifi.mode = mode;
#endif
            } else
                status = Status_InvalidStatement; //out of range...
            break;

        case Setting_WiFi_STA_SSID:
            if(is_valid_ssid(svalue) && strlcpy(driver_settings.wifi.sta.ssid, svalue, sizeof(ssid_t)) <= sizeof(ssid_t))
                status = Status_OK;
            else
                status = Status_InvalidStatement; // too long...
            break;

        case Setting_WiFi_STA_Password:
            if(strlcpy(driver_settings.wifi.sta.password, svalue, sizeof(password_t)) <= sizeof(password_t))
                status = Status_OK;
            else
                status = Status_InvalidStatement; // too long...
            break;

        case Setting_Hostname:
            if(is_valid_hostname(svalue) && strlcpy(driver_settings.wifi.sta.network.hostname, svalue, sizeof(hostname_t)) <= sizeof(hostname_t))
                status = Status_OK;
            else
                status = Status_InvalidStatement; // too long...
            break;

#if AUTH_ENABLE
        case Setting_AdminPassword:
            if(is_valid_password(svalue) && strlcpy(driver_settings.wifi.admin_password, svalue, sizeof(password_t)) <= sizeof(password_t))
                status = Status_OK;
            else
                status = Status_InvalidStatement; // too long...
            break;

        case Setting_UserPassword:
            if(is_valid_password(svalue) && strlcpy(driver_settings.wifi.user_password, svalue, sizeof(password_t)) <= sizeof(password_t))
                status = Status_OK;
            else
                status = Status_InvalidStatement; // too long...
            break;
#endif

#if NETWORK_IPMODE_STATIC || true

        case Setting_IpAddress:
            {
                ip4_addr_t addr;
                if(inet_pton(AF_INET, svalue, &addr) == 1) {
                    status = Status_OK;
                    *((ip4_addr_t *)driver_settings.wifi.sta.network.ip) = addr;
                } else
                    status = Status_InvalidStatement;
            }
            break;

        case Setting_Gateway:
            {
                ip4_addr_t addr;
                if(inet_pton(AF_INET, svalue, &addr) == 1) {
                    status = Status_OK;
                    *((ip4_addr_t *)driver_settings.wifi.sta.network.gateway) = addr;
                } else
                    status = Status_InvalidStatement;
            }
            break;

        case Setting_NetMask:
            {
                ip4_addr_t addr;
                if(inet_pton(AF_INET, svalue, &addr) == 1) {
                    status = Status_OK;
                    *((ip4_addr_t *)driver_settings.wifi.sta.network.mask) = addr;
                } else
                    status = Status_InvalidStatement;
            }
            break;

#endif

#if WIFI_SOFTAP

        case Setting_WiFi_AP_SSID:
            if(is_valid_ssid(svalue) && strlcpy(driver_settings.wifi.ap.ssid, svalue, sizeof(ssid_t)) <= sizeof(ssid_t))
                status = Status_OK;
            else
                status = Status_InvalidStatement; // too long...
            break;

        case Setting_WiFi_AP_Password:
            if(strlcpy(driver_settings.wifi.ap.password, svalue, sizeof(password_t)) <= sizeof(password_t))
                status = Status_OK;
            else
                status = Status_InvalidStatement; // too long...
            break;

        case Setting_Hostname2:
            if(strlcpy(driver_settings.wifi.ap.network.hostname, svalue, sizeof(hostname_t)) <= sizeof(hostname_t))
                status = Status_OK;
            else
                status = Status_InvalidStatement; // too long...
            break;

        case Setting_IpAddress2:
            {
                ip4_addr_t addr;
                if(inet_pton(AF_INET, svalue, &addr) == 1) {
                    status = Status_OK;
                    *((ip4_addr_t *)driver_settings.wifi.ap.network.ip) = addr;
                } else
                    status = Status_InvalidStatement;
            }
            break;

        case Setting_Gateway2:
            {
                ip4_addr_t addr;
                if(inet_pton(AF_INET, svalue, &addr) == 1) {
                    status = Status_OK;
                    *((ip4_addr_t *)driver_settings.wifi.ap.network.gateway) = addr;
                } else
                    status = Status_InvalidStatement;
            }
            break;

        case Setting_NetMask2:
            {
                ip4_addr_t addr;
                if(inet_pton(AF_INET, svalue, &addr) == 1) {
                    status = Status_OK;
                    *((ip4_addr_t *)driver_settings.wifi.ap.network.mask) = addr;
                } else
                    status = Status_InvalidStatement;
            }
            break;

#endif // WIFI_SOFTAP

#if TELNET_ENABLE
        case Setting_TelnetPort:
            if(isintf(value) && is_valid_port((uint16_t)value)) {
                status = Status_OK;
                driver_settings.wifi.sta.network.telnet_port =
                driver_settings.wifi.ap.network.telnet_port = (uint16_t)value;
            } else
                status = Status_InvalidStatement; //out of range...
            break;
#endif

#if HTTP_ENABLE
        case Setting_HttpPort:
            if(isintf(value) && is_valid_port((uint16_t)value)) {
                status = Status_OK;
                driver_settings.wifi.sta.network.http_port =
                driver_settings.wifi.ap.network.http_port = (uint16_t)value;
            } else
                status = Status_InvalidStatement; //out of range...
            break;
#endif

#if WEBSOCKET_ENABLE
        case Setting_WebSocketPort:
            if(isintf(value) && is_valid_port((uint16_t)value)) {
                status = Status_OK;
                driver_settings.wifi.sta.network.websocket_port =
                driver_settings.wifi.ap.network.websocket_port = (uint16_t)value;
            } else
                status = Status_InvalidStatement; //out of range...
            break;
#endif
    }

    return status;
}

void wifi_settings_report (setting_type_t setting)
{
    switch(setting) {

        case Setting_NetworkServices:
            report_uint_setting(setting, driver_settings.wifi.sta.network.services.mask);
            break;

        case Setting_WifiMode:
            report_uint_setting(setting, driver_settings.wifi.mode);
            break;

        case Setting_WiFi_STA_SSID:
// TODO:?           if(driver_settings.wifi.mode == WiFiMode_STA && driver_settings.wifi.mode == WiFiMode_APSTA)
            report_string_setting(setting, driver_settings.wifi.sta.ssid);
            break;

        case Setting_WiFi_STA_Password:
            report_string_setting(setting, driver_settings.wifi.sta.password);
            break;

        case Setting_Hostname:
            report_string_setting(setting, driver_settings.wifi.sta.network.hostname);
            break;

#if NETWORK_IPMODE_STATIC || true

        case Setting_IpAddress:
            {
                char ip[INET6_ADDRSTRLEN];
                report_string_setting(setting, inet_ntop(AF_INET, &driver_settings.wifi.sta.network.ip, ip, INET6_ADDRSTRLEN));
            }
            break;

        case Setting_Gateway:
            {
                char ip[INET6_ADDRSTRLEN];
                report_string_setting(setting, inet_ntop(AF_INET, &driver_settings.wifi.sta.network.gateway, ip, INET6_ADDRSTRLEN));
            }
            break;

        case Setting_NetMask:
            {
                char ip[INET6_ADDRSTRLEN];
                report_string_setting(setting, inet_ntop(AF_INET, &driver_settings.wifi.sta.network.mask, ip, INET6_ADDRSTRLEN));
            }
            break;
#endif

#if WIFI_SOFTAP

        case Setting_WiFi_AP_SSID:
            report_string_setting(setting, driver_settings.wifi.ap.ssid);
            break;

        case Setting_WiFi_AP_Password:
            report_string_setting(setting, driver_settings.wifi.ap.password);
            break;

        case Setting_Hostname2:
            report_string_setting(setting, driver_settings.wifi.ap.network.hostname);
            break;

        case Setting_IpAddress2:
            {
                char ip[INET6_ADDRSTRLEN];
                report_string_setting(setting, inet_ntop(AF_INET, &driver_settings.wifi.ap.network.ip, ip, INET6_ADDRSTRLEN));
            }
            break;

        case Setting_Gateway2:
            {
                char ip[INET6_ADDRSTRLEN];
                report_string_setting(setting, inet_ntop(AF_INET, &driver_settings.wifi.ap.network.gateway, ip, INET6_ADDRSTRLEN));
            }
            break;

        case Setting_NetMask2:
            {
                char ip[INET6_ADDRSTRLEN];
                report_string_setting(setting, inet_ntop(AF_INET, &driver_settings.wifi.ap.network.mask, ip, INET6_ADDRSTRLEN));
            }
            break;

#endif // WIFI_SOFTAP

#if TELNET_ENABLE
        case Setting_TelnetPort:
            report_uint_setting(setting, driver_settings.wifi.sta.network.telnet_port);
            break;
#endif

#if HTTP_ENABLE
        case Setting_HttpPort:
            report_uint_setting(setting, driver_settings.wifi.sta.network.http_port);
            break;
#endif

#if WEBSOCKET_ENABLE
        case Setting_WebSocketPort:
            report_uint_setting(setting, driver_settings.wifi.sta.network.websocket_port);
            break;
#endif

#if AUTH_ENABLE
        case Setting_AdminPassword:
            report_string_setting(setting, hal.stream.type == StreamType_Serial ? driver_settings.wifi.admin_password : HIDDEN_PASSWORD);
            break;

        case Setting_UserPassword:
            report_string_setting(setting, hal.stream.type == StreamType_Serial ? driver_settings.wifi.user_password : HIDDEN_PASSWORD);
            break;
#endif

        default:
            break;
    }
}

void wifi_settings_restore (void)
{
#if NETWORK_IPMODE_STATIC || WIFI_SOFTAP
    ip4_addr_t addr;
#endif

    driver_settings.wifi.mode = WIFI_MODE;

// Station

    strlcpy(driver_settings.wifi.sta.network.hostname, NETWORK_HOSTNAME, sizeof(driver_settings.wifi.sta.network.hostname));

#if NETWORK_IPMODE_STATIC

    driver_settings.wifi.sta.network.ip_mode = IpMode_Static;

    if(inet_pton(AF_INET, NETWORK_IP, &addr) == 1)
        *((ip4_addr_t *)driver_settings.wifi.sta.network.ip) = addr;

    if(inet_pton(AF_INET, NETWORK_GATEWAY, &addr) == 1)
        *((ip4_addr_t *)driver_settings.wifi.sta.network.gateway) = addr;

    if(inet_pton(AF_INET, NETWORK_MASK, &addr) == 1)
        *((ip4_addr_t *)driver_settings.wifi.sta.network.mask) = addr;

#else
    driver_settings.wifi.sta.network.ip_mode = IpMode_DHCP;
#endif

// Access Point

#if WIFI_SOFTAP

    driver_settings.wifi.ap.network.ip_mode = IpMode_Static;
    strlcpy(driver_settings.wifi.ap.network.hostname, NETWORK_AP_HOSTNAME, sizeof(driver_settings.wifi.ap.network.hostname));
    strlcpy(driver_settings.wifi.ap.ssid, WIFI_AP_SSID, sizeof(driver_settings.wifi.ap.ssid));
    strlcpy(driver_settings.wifi.ap.password, WIFI_AP_PASSWORD, sizeof(driver_settings.wifi.ap.password));

    if(inet_pton(AF_INET, NETWORK_AP_IP, &addr) == 1)
        *((ip4_addr_t *)driver_settings.wifi.ap.network.ip) = addr;

    if(inet_pton(AF_INET, NETWORK_AP_GATEWAY, &addr) == 1)
        *((ip4_addr_t *)driver_settings.wifi.ap.network.gateway) = addr;

    if(inet_pton(AF_INET, NETWORK_AP_MASK, &addr) == 1)
        *((ip4_addr_t *)driver_settings.wifi.ap.network.mask) = addr;

#endif

// Common

    driver_settings.wifi.sta.network.telnet_port =
    driver_settings.wifi.ap.network.telnet_port = NETWORK_TELNET_PORT;
    driver_settings.wifi.sta.network.http_port =
    driver_settings.wifi.ap.network.http_port = NETWORK_HTTP_PORT;
    driver_settings.wifi.sta.network.websocket_port =
    driver_settings.wifi.ap.network.websocket_port = NETWORK_WEBSOCKET_PORT;

#if TELNET_ENABLE
    driver_settings.wifi.sta.network.services.telnet =
    driver_settings.wifi.ap.network.services.telnet = On;
#endif

#if HTTP_ENABLE
    driver_settings.wifi.sta.network.services.http =
    driver_settings.wifi.ap.network.services.http = On;
#endif

#if WEBSOCKET_ENABLE
    driver_settings.wifi.sta.network.services.websocket =
    driver_settings.wifi.ap.network.services.websocket = On;
#endif
}

#endif // WIFI_ENABLE

