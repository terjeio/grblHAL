/*
  driver.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of GrblHAL

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

#include "soc/rtc.h"
#include "driver/gpio.h"
#include "driver/timer.h"
#include "driver/ledc.h"
#include "driver/rmt.h"
#include "driver/i2c.h"

#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "grbl/grbl.h"

//
// Set config from compile definitions in CMakeLists.txt
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

#ifdef NETWORKING_ENABLE
#define WIFI_ENABLE      1
#define HTTP_ENABLE      1
#define TELNET_ENABLE    1
#define WEBSOCKET_ENABLE 1
#endif

#ifdef KEYPAD_ENABLE
#undef KEYPAD_ENABLE
#define KEYPAD_ENABLE 1
#endif

#ifdef TRINAMIC_ENABLE
#undef TRINAMIC_ENABLE
#define TRINAMIC_ENABLE 1
#define TRINAMIC_I2C    1
#endif

#ifdef MPG_MODE_ENABLE
#undef MPG_MODE_ENABLE
#define MPG_MODE_ENABLE 1
#endif

//

// Configuration
// Set value to 1 to enable, 0 to disable

#ifdef CNC_BOOSTERPACK
#undef CNC_BOOSTERPACK
#define CNC_BOOSTERPACK  1
#define EEPROM_ENABLE    1 // I2C EEPROM (24LC16) support.
#define IOEXPAND_ENABLE  1 // I2C IO expander for some output signals.
#else
#define EEPROM_ENABLE    0 // I2C EEPROM (24LC16) support.
#define IOEXPAND_ENABLE  0 // I2C IO expander for some output signals.
#endif
#define PWM_RAMPED       0 // Ramped spindle PWM.
#define PROBE_ENABLE     1 // Probe input
#define PROBE_ISR        0 // Catch probe state change by interrupt TODO: needs verification!
#define WIFI_SOFTAP      0 // Use Soft AP mode for WiFi.
#define BLUETOOTH_ENABLE 0 // Streaming over Bluetooth.
#define TRINAMIC_DEV     0 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code

// The following options should be set in CMakeLists.txt to ensure
// all relevant files are included for compilation
// DO NOT change settings here!

#ifndef KEYPAD_ENABLE
#define KEYPAD_ENABLE    0 // I2C keypad for jogging etc.
#endif

#ifndef NETWORKING_ENABLE
#define WIFI_ENABLE      0 // Streaming over WiFi.
#define HTTP_ENABLE      0 // Enable http daemon - requires WiFi enabled
#define TELNET_ENABLE    0 // Enable telnet daemon - requires WiFi enabled
#define WEBSOCKET_ENABLE 0 // Enable websocket daemon - requires WiFi enabled
#endif

#ifndef AUTH_ENABLE
#define AUTH_ENABLE      0 // Enable WebUI security
#endif
#ifndef SDCARD_ENABLE
#define SDCARD_ENABLE    0 // Run jobs from SD card.
#endif
#ifndef WEBUI_ENABLE
#define WEBUI_ENABLE     0 // Enables WebUi - requires WiFi enabled. Note: experimental - only partly implemented!
#endif
#ifndef TRINAMIC_ENABLE
#define TRINAMIC_ENABLE  0 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
#define TRINAMIC_I2C     0 // Trinamic I2C - SPI bridge interface.
#endif

// end configuration

#if WEBUI_ENABLE

#undef WIFI_ENABLE
#undef HTTP_ENABLE
#undef WEBSOCKET_ENABLE

#define WIFI_ENABLE      1
#define HTTP_ENABLE      1
#define WEBSOCKET_ENABLE 1

#endif

#if WIFI_ENABLE

#define NETWORK_TELNET_PORT     23
#define NETWORK_HTTP_PORT       80
#define NETWORK_WEBSOCKET_PORT  81

// WiFi Station (STA) settings
#define NETWORK_HOSTNAME        "Grbl"
#define NETWORK_IPMODE_STATIC   0
#if NETWORK_IPMODE_STATIC
#define NETWORK_IP              "192.168.5.1"
#define NETWORK_GATEWAY         "192.168.5.1"
#define NETWORK_MASK            "255.255.255.0"
#endif

#if NETWORK_IPMODE_STATIC && WIFI_SOFTAP
#error "Cannot use static IP for station when soft AP is enabled!"
#endif

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

#elif HTTP_ENABLE || TELNET_ENABLE || WEBSOCKET_ENABLE
#error "Networking protocols reqires WiFi enabled!"
#endif // WIFI_ENABLE

// End configuration

#if TRINAMIC_ENABLE
#include "tmc2130/trinamic.h"
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


#if WIFI_ENABLE || BLUETOOTH_ENABLE || TRINAMIC_ENABLE || KEYPAD_ENABLE

#define DRIVER_SETTINGS

typedef struct {
    wifi_settings_t wifi;
    bluetooth_settings_t bluetooth;
#if TRINAMIC_ENABLE
    trinamic_settings_t trinamic;
#endif
#if KEYPAD_ENABLE
    jog_settings_t jog;
#endif
} driver_settings_t;

extern driver_settings_t driver_settings;

#endif

#if MPG_MODE_ENABLE
#define MPG_ENABLE_PIN 13
#endif

#if CNC_BOOSTERPACK

#define IOEXPAND 0

#if SDCARD_ENABLE

// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define MPG_ENABLE_PIN 13

#endif // SDCARD_ENABLE


// timer definitions
#define STEP_TIMER_GROUP TIMER_GROUP_0
#define STEP_TIMER_INDEX TIMER_0

// Define step pulse output pins.
#define X_STEP_PIN      GPIO_NUM_26
#define Y_STEP_PIN      GPIO_NUM_27
#define Z_STEP_PIN      GPIO_NUM_14
#define STEP_MASK       (1ULL << X_STEP_PIN|1ULL << Y_STEP_PIN|1ULL << Z_STEP_PIN) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN     GPIO_NUM_2
#define Y_DIRECTION_PIN     GPIO_NUM_15
#define Z_DIRECTION_PIN     GPIO_NUM_12
#define DIRECTION_MASK      (1ULL << X_DIRECTION_PIN|1ULL << Y_DIRECTION_PIN|1ULL << Z_DIRECTION_PIN) // All direction bits

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_DISABLE_PIN    IOEXPAND
#define STEPPERS_DISABLE_MASK   (1ULL << STEPPERS_DISABLE_PIN)

// Define homing/hard limit switch input pins and limit interrupt vectors.
#define X_LIMIT_PIN     GPIO_NUM_4
#define Y_LIMIT_PIN     GPIO_NUM_16
#define Z_LIMIT_PIN     GPIO_NUM_32
#define LIMIT_MASK      (1ULL << X_LIMIT_PIN|1ULL << Y_LIMIT_PIN|1ULL << Z_LIMIT_PIN) // All limit bits

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PIN      IOEXPAND
#define SPINDLE_DIRECTION_PIN   IOEXPAND
#define SPINDLE_MASK            (1ULL << SPINDLE_ENABLE_PIN|1ULL << SPINDLE_DIRECTION_PIN)
#define SPINDLEPWMPIN           GPIO_NUM_17

// Define flood and mist coolant enable output pins.

#define COOLANT_FLOOD_PIN   IOEXPAND
#define COOLANT_MIST_PIN    IOEXPAND
#define COOLANT_MASK        (1UL << COOLANT_FLOOD_PIN|1ULL << COOLANT_MIST_PIN)

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PIN           GPIO_NUM_35
#define FEED_HOLD_PIN       GPIO_NUM_39
#define CYCLE_START_PIN     GPIO_NUM_36
#define SAFETY_DOOR_PIN     GPIO_NUM_34
#define CONTROL_MASK        (1UL << RESET_PIN|1UL << FEED_HOLD_PIN|1UL << CYCLE_START_PIN|1UL << SAFETY_DOOR_PIN)

// Define probe switch input pin.
#define PROBE_PIN       GPIO_NUM_13

#if KEYPAD_ENABLE
#define KEYPAD_STROBE_PIN   GPIO_NUM_33
#endif

#if IOEXPAND_ENABLE || KEYPAD_ENABLE || EEPROM_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)
// Define I2C port/pins
#define I2C_PORT  I2C_NUM_1
#define I2C_SDA   GPIO_NUM_21
#define I2C_SCL   GPIO_NUM_22
#define I2C_CLOCK 100000
#endif

#if IOEXPAND_ENABLE
typedef union {
    uint8_t mask;
    struct {
        uint8_t stepper_enable_z :1,
                stepper_enable_y :1,
                mist_on          :1,
                flood_on         :1,
                reserved         :1,
                spindle_dir      :1,
                stepper_enable_x :1,
                spindle_on       :1;
    };
} ioexpand_t;
#endif

#else  // ?? else no booster?

#if SDCARD_ENABLE

// Pin mapping when using SPI mode.
// With this mapping, SD card can be used both in SPI and 1-line SD mode.
// Note that a pull-up on CS line is required in SD mode.
#define PIN_NUM_MISO 19
#define PIN_NUM_MOSI 23
#define PIN_NUM_CLK  18
#define PIN_NUM_CS   5
#define MPG_ENABLE_PIN 13

#endif // SDCARD_ENABLE

// timer definitions
#define STEP_TIMER_GROUP TIMER_GROUP_0
#define STEP_TIMER_INDEX TIMER_0

// Define step pulse output pins.
#define X_STEP_PIN      GPIO_NUM_12
#define Y_STEP_PIN      GPIO_NUM_14
#define Z_STEP_PIN      GPIO_NUM_27
#define STEP_MASK       (1ULL << X_STEP_PIN|1ULL << Y_STEP_PIN|1ULL << Z_STEP_PIN) // All step bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define X_DIRECTION_PIN     GPIO_NUM_26
#define Y_DIRECTION_PIN     GPIO_NUM_25
#define Z_DIRECTION_PIN     GPIO_NUM_33
#define DIRECTION_MASK      (1ULL << X_DIRECTION_PIN|1ULL << Y_DIRECTION_PIN|1ULL << Z_DIRECTION_PIN) // All direction bits

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_DISABLE_PIN    GPIO_NUM_13
#define STEPPERS_DISABLE_MASK   (1ULL << STEPPERS_DISABLE_PIN)

// Define homing/hard limit switch input pins and limit interrupt vectors.
#define X_LIMIT_PIN     GPIO_NUM_2
#define Y_LIMIT_PIN     GPIO_NUM_4
#define Z_LIMIT_PIN     GPIO_NUM_15
#define LIMIT_MASK      (1ULL << X_LIMIT_PIN|1ULL << Y_LIMIT_PIN|1ULL << Z_LIMIT_PIN) // All limit bits

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PIN      GPIO_NUM_18
#define SPINDLE_DIRECTION_PIN   GPIO_NUM_5
#define SPINDLE_MASK            (1ULL << SPINDLE_ENABLE_PIN|1ULL << SPINDLE_DIRECTION_PIN)
#define SPINDLEPWMPIN           GPIO_NUM_17

// Define flood and mist coolant enable output pins.

#define COOLANT_FLOOD_PIN   GPIO_NUM_16
#define COOLANT_MIST_PIN    GPIO_NUM_21
#define COOLANT_MASK        (1UL << COOLANT_FLOOD_PIN|1ULL << COOLANT_MIST_PIN)

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PIN           GPIO_NUM_34
#define FEED_HOLD_PIN       GPIO_NUM_36
#define CYCLE_START_PIN     GPIO_NUM_39
#define SAFETY_DOOR_PIN     GPIO_NUM_35
#define CONTROL_MASK        (1UL << RESET_PIN|1UL << FEED_HOLD_PIN|1UL << CYCLE_START_PIN|1UL << SAFETY_DOOR_PIN)

// Define probe switch input pin.
#if PROBE_ENABLE
#define PROBE_PIN       GPIO_NUM_32
#else
#define PROBE_PIN       0xFF
#endif

#if KEYPAD_ENABLE
#error No free pins for keypad!
#endif

#if IOEXPAND_ENABLE || KEYPAD_ENABLE || EEPROM_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)
// Define I2C port/pins
#define I2C_PORT  I2C_NUM_1
#define I2C_SDA   GPIO_NUM_21
#define I2C_SCL   GPIO_NUM_22
#define I2C_CLOCK 100000
#endif

#if IOEXPAND_ENABLE
typedef union {
    uint8_t mask;
    struct {
        uint8_t spindle_on       :1,
                spindle_dir      :1,
                mist_on          :1,
                flood_on         :1,
                stepper_enable_z :1,
                stepper_enable_x :1,
                stepper_enable_y :1,
                reserved         :1;
    };
} ioexpand_t;
#endif

#endif

#ifdef I2C_PORT
extern QueueHandle_t i2cQueue;
extern SemaphoreHandle_t i2cBusy;
#endif

void selectStream (stream_type_t stream);

#endif // __DRIVER_H__
