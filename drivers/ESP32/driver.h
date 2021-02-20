/*
  driver.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of grblHAL

  Copyright (c) 2018-2020 Terje Io
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#ifndef __DRIVER_H__
#define __DRIVER_H__

#ifndef OVERRIDE_MY_MACHINE
//
// Set options from my_machine.h
//
#include "my_machine.h"

#if NETWORKING_ENABLE
#define WIFI_ENABLE 1
#endif

#if WEBUI_ENABLE
#error "WebUI is not available in this setup!"
#endif
//
#else
//
// Set options from CMakeLists.txt
//
#ifdef WEBUI_ENABLE
#undef WEBUI_ENABLE
#define WEBUI_ENABLE 1
#endif

#ifdef AUTH_ENABLE
#undef AUTH_ENABLE
#define AUTH_ENABLE 1
#endif

#ifdef SDCARD_ENABLE
#undef SDCARD_ENABLE
#define SDCARD_ENABLE 1
#endif

#ifdef BLUETOOTH_ENABLE
#undef BLUETOOTH_ENABLE
#define BLUETOOTH_ENABLE 1
#endif

#ifdef MPG_MODE_ENABLE
#undef MPG_MODE_ENABLE
#define MPG_MODE_ENABLE 1
#endif

#ifdef KEYPAD_ENABLE
#undef KEYPAD_ENABLE
#define KEYPAD_ENABLE 1
#endif

#ifdef TRINAMIC_ENABLE
#undef TRINAMIC_ENABLE
#define TRINAMIC_ENABLE 2130
#define TRINAMIC_I2C    1
#endif

#ifdef NETWORKING_ENABLE
#define WIFI_ENABLE      1
#define HTTP_ENABLE      0
#define TELNET_ENABLE    1
#define WEBSOCKET_ENABLE 1
#define NETWORK_TELNET_PORT     23
#define NETWORK_HTTP_PORT       80
#define NETWORK_WEBSOCKET_PORT  81
#endif

#if WEBUI_ENABLE
#undef HTTP_ENABLE
#define HTTP_ENABLE 1
#endif

#ifdef RS485_DIR_ENABLE
#undef RS485_DIR_ENABLE
#if SPINDLE_HUANYANG
#define RS485_DIR_ENABLE 1
#else
#define RS485_DIR_ENABLE 0
#endif
#endif

#ifndef EEPROM_ENABLE
#define EEPROM_ENABLE 0
#endif

#endif

#include "soc/rtc.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "driver/rmt.h"
#include "driver/i2c.h"

#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "grbl/hal.h"

static const DRAM_ATTR float FZERO = 0.0f;

#define PWM_RAMPED       0 // Ramped spindle PWM.
#ifdef NOPROBE
#define PROBE_ENABLE     0 // No probe input.
#else
#define PROBE_ENABLE     1 // Probe input.
#endif
#define PROBE_ISR        0 // Catch probe state change by interrupt TODO: needs verification!
#define TRINAMIC_DEV     0 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code

// DO NOT change settings here!

#ifndef IOEXPAND_ENABLE
#define IOEXPAND_ENABLE 0 // I2C IO expander for some output signals.
#endif

#ifndef WIFI_SOFTAP
#define WIFI_SOFTAP      0
#endif

#ifndef KEYPAD_ENABLE
#define KEYPAD_ENABLE    0
#endif

#ifndef NETWORKING_ENABLE
#define WIFI_ENABLE      0
#define HTTP_ENABLE      0
#define TELNET_ENABLE    0
#define WEBSOCKET_ENABLE 0
#endif

#ifndef BLUETOOTH_ENABLE
#define BLUETOOTH_ENABLE 0
#endif

#ifndef AUTH_ENABLE
#define AUTH_ENABLE      0
#endif

#ifndef SDCARD_ENABLE
#define SDCARD_ENABLE    0
#endif

#ifndef WEBUI_ENABLE
#define WEBUI_ENABLE     0
#endif

#ifndef TRINAMIC_ENABLE
#define TRINAMIC_ENABLE  0
#define TRINAMIC_I2C     0
#endif

#define IOEXPAND 0xFF   // Dummy pin number for I2C IO expander

// end configuration

#if !WIFI_ENABLE
  #if HTTP_ENABLE || TELNET_ENABLE || WEBSOCKET_ENABLE
  #error "Networking protocols requires networking enabled!"
  #endif // WIFI_ENABLE
#else

#if !NETWORK_PARAMETERS_OK

// WiFi Station (STA) settings
#define NETWORK_HOSTNAME    "Grbl"
#define NETWORK_IPMODE      1 // 0 = static, 1 = DHCP, 2 = AutoIP
#define NETWORK_IP          "192.168.5.1"
#define NETWORK_GATEWAY     "192.168.5.1"
#define NETWORK_MASK        "255.255.255.0"

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
#define WIFI_MODE WiFiMode_STA; // Do not change!
#endif

#if NETWORK_IPMODE < 0 || NETWORK_IPMODE > 2
#error "Invalid IP mode selected!"
#endif

#if NETWORK_IPMODE == 0 && WIFI_SOFTAP
#error "Cannot use static IP for station when soft AP is enabled!"
#endif

#endif // !NETWORK_PARAMETERS_OK
#endif // WIFI_ENABLE

#if BLUETOOTH_ENABLE
#define BLUETOOTH_DEVICE    "GRBL"
#define BLUETOOTH_SERVICE   "GRBL Serial Port" // Minimum 8 characters, or blank for open
#endif

// End configuration

#if IOEXPAND_ENABLE || KEYPAD_ENABLE || EEPROM_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)
#define I2C_ENABLE 1
#else
#define I2C_ENABLE 0
#endif

#if TRINAMIC_ENABLE
#ifndef TRINAMIC_MIXED_DRIVERS
#define TRINAMIC_MIXED_DRIVERS 1
#endif
#include "motors/trinamic.h"
#include "trinamic/common.h"
#endif

#ifdef SPINDLE_HUANYANG
#include "spindle/huanyang.h"
#endif

typedef struct
{
    grbl_wifi_mode_t mode;
    wifi_sta_settings_t sta;
    wifi_ap_settings_t ap;
//  network_settings_t network;
    password_t admin_password;
    password_t user_password;
} wifi_settings_t;

typedef struct {
    uint8_t action;
    void *params;
} i2c_task_t;

// End configuration

#ifdef BOARD_CNC_BOOSTERPACK
  #include "cnc_boosterpack_map.h"
#elif defined(BOARD_BDRING_V4)
  #include "bdring_v4_map.h"
#elif defined(BOARD_BDRING_V3P5)
  #include "bdring_v3.5_map.h"
#elif defined(BOARD_BDRING_I2S6A)
  #include "bdring_i2s_6_axis_map.h"
#elif defined(BOARD_ESPDUINO32)
  #include "espduino-32_wemos_d1_r32_uno_map.h"
#elif defined(BOARD_MY_MACHINE)
  #include "my_machine_map.h"
#else // default board - NOTE: NOT FINAL VERSION!
  #warning "Compiling for generic board!"
  #include "generic_map.h"
#endif

#ifndef GRBL_ESP32
#error "Add #define GRBL_ESP32 in grbl/config.h or update your CMakeLists.txt to the latest version!"
#endif

#if IOEXPAND_ENABLE == 0 && ((DIRECTION_MASK|STEPPERS_DISABLE_MASK|SPINDLE_MASK|COOLANT_MASK) & 0xC00000000ULL)
#error "Pins 34 - 39 are input only!"
#endif

#ifdef I2C_PORT
extern QueueHandle_t i2cQueue;
extern SemaphoreHandle_t i2cBusy;
#elif I2C_ENABLE == 1
#error "I2C port not available!"
#endif

#if MPG_MODE_ENABLE || MODBUS_ENABLE
#define SERIAL2_ENABLE 1
#endif

#if MPG_MODE_ENABLE
  #ifndef MPG_ENABLE_PIN
  #error "MPG_ENABLE_PIN must be defined when MPG mode is enabled!"
  #endif
  #ifndef UART2_RX_PIN
  #error "UART2_RX_PIN must be defined when MPG mode is enabled!"
  #endif
#endif

void selectStream (stream_type_t stream);

#endif // __DRIVER_H__
