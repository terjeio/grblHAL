/*
  wifi.h - An embedded CNC Controller with rs274/ngc (g-code) support

  WiFi comms

  Part of grblHAL

  Copyright (c) 2018 Terje Io

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

#ifndef _grbl_wifi_h_
#define _grbl_wifi_h_

#include "driver.h"
#include "esp_wifi.h"

#define STREAM_POLL_INTERVAL 20 // Poll interval in milliseconds

typedef struct {
    uint16_t ap_num;
    wifi_ap_record_t *ap_records;
    uint8_t *ap_selected;
    ip4_addr_t ip_addr;
    char ap_status[20];
} ap_list_t;

bool wifi_init (void);
bool wifi_start (void);
bool wifi_stop (void);
bool wifi_ap_connect (char *ssid, char *password);
ap_list_t *wifi_get_aplist (void);
void wifi_release_aplist (void);
char *wifi_get_ipaddr (void);
char *wifi_get_mac (void);
bool wifi_dns_running (void);
wifi_settings_t *get_wifi_settings (void);

char *iptoa(void *ip);

#endif

