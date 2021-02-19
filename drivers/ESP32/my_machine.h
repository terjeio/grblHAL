/*
  my_machine.h - configuration for ESP32 processos

  NOTE: only in use if not compiling with cmake (idf.py)

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

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

// NOTE: Only one board may be enabled!
// If none is enabled pin mappings from generic_map.h will be used
//#define BOARD_ESPDUINO32
//#define BOARD_BDRING_V3P5
//#define BOARD_BDRING_V4
//#define BOARD_BDRING_I2S6A // NOT production ready!
//#define BOARD_MY_MACHINE // Add my_machine_map.h before enabling this!

// Configuration
// Uncomment to enable, for some a value > 1 may be assigned, if so the default value is shown.

//#define NETWORKING_ENABLE  1 // WiFi streaming. Requires networking plugin.
#if NETWORKING_ENABLE
//#define WIFI_SOFTAP        1 // Use Soft AP mode for WiFi.
//#define WEBUI_ENABLE       0 // Not yet available, do not change.
#endif
//#define SDCARD_ENABLE      1 // Run gcode programs from SD card, requires sdcard plugin.
//#define BLUETOOTH_ENABLE   1 // Enable Bloetooht streaming.
//#define MPG_MODE_ENABLE    1 // Enable MPG mode (secondary serial port)
//#define NOPROBE            1 // Comment out to disable porbe input.
//#define EEPROM_ENABLE      1 // I2C EEPROM support. Set to 1 for 24LC16(2K), 2 for larger sizes. Requires eeprom plugin.
//#define EEPROM_IS_FRAM     1 // Uncomment when EEPROM is enabled and chip is FRAM, this to remove write delay.

#if NETWORKING_ENABLE
#define NETWORK_PARAMETERS_OK   1
#define TELNET_ENABLE           1 // Telnet daemon - requires Ethernet streaming enabled.
#define WEBSOCKET_ENABLE        1 // Websocket daemon - requires Ethernet streaming enabled.
#define NETWORK_HOSTNAME        "Grbl"
#define NETWORK_IPMODE          1 // 0 = static, 1 = DHCP, 2 = AutoIP
#define NETWORK_IP              "192.168.5.1"
#define NETWORK_GATEWAY         "192.168.5.1"
#define NETWORK_MASK            "255.255.255.0"
#define NETWORK_TELNET_PORT     23
#define NETWORK_WEBSOCKET_PORT  81
#define NETWORK_HTTP_PORT       80

// WiFi Access Point (AP) settings
#if WIFI_SOFTAP
#define NETWORK_AP_HOSTNAME "GrblAP"
#define NETWORK_AP_IP       "192.168.5.1"
#define NETWORK_AP_GATEWAY  "192.168.5.1"
#define NETWORK_AP_MASK     "255.255.255.0"
#define WIFI_AP_SSID        "GRBL"
#define WIFI_AP_PASSWORD    "GrblPassword" // Minimum 8 characters, or blank for open
#define WIFI_MODE WiFiMode_AP; // OPTION: WiFiMode_APSTA
#else
#define WIFI_SOFTAP 0
#define WIFI_MODE WiFiMode_STA; // Do not change!
#endif

#endif
