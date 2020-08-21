/*
  driver.h - driver code for IMXRT1062 processor (on Teensy 4.0 board)

  Part of GrblHAL

  Copyright (c) 2020 Terje Io

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

#include "imxrt.h"
#include "core_pins.h"
#include "pins_arduino.h"

#include "src/grbl/hal.h"
#include "src/grbl/nuts_bolts.h"

#define UART_DEBUG // For development only - enable only with USB_SERIAL_GRBL enabled and SPINDLE_HUANYANG disabled

// NOTE: Only one board may be enabled! If none is enabled pin mappings from defaults below will be used
#define BOARD_T41U5XBB
//#define BOARD_CNC_BOOSTERPACK

// Configuration
// Set value to 1 or greater to enable, 0 to disable

#define USB_SERIAL_GRBL    2 // Set to 1 for Arduino class library, 2 for PJRC C library.
#define USB_SERIAL_WAIT    0 // Wait for USB connection before starting grblHAL.
#define SPINDLE_HUANYANG   0 // Set to 1 or 2 for Huanyang VFD spindle. Requires spindle plugin.
#define QEI_ENABLE         0 // Enable quadrature encoder interfaces. Max value is 1. Requires encoder plugin.
#define ETHERNET_ENABLE    0 // Ethernet streaming. Requires networking plugin.
#if ETHERNET_ENABLE
#define TELNET_ENABLE      1 // Telnet daemon - requires Ethernet streaming enabled.
#define WEBSOCKET_ENABLE   1 // Websocket daemon - requires Ethernet streaming enabled.
#else
#define TELNET_ENABLE      0 // Do not change!
#define WEBSOCKET_ENABLE   0 // Do not change!
#endif
#if COMPATIBILITY_LEVEL <= 1
#define ESTOP_ENABLE       1 // When enabled only real-time report requests will be executed when the reset pin is asserted.
#else
#define ESTOP_ENABLE       0 // Do not change!
#endif

#define SDCARD_ENABLE      0 // Run gcode programs from SD card, requires sdcard plugin.
#define KEYPAD_ENABLE      0 // I2C keypad for jogging etc., requires keypad plugin.

#ifndef BOARD_CNC_BOOSTERPACK
  #define EEPROM_ENABLE    1 // I2C EEPROM support. Set to 1 for 24LC16(2K), 2 for larger sizes.
                             // Requires eeprom plugin.
  #define TRINAMIC_ENABLE  0 // Do not enable!
  #define TRINAMIC_I2C     0 // Do not enable!
  #define TRINAMIC_DEV     0 // Do not enable!
#else
  #define EEPROM_ENABLE    1 // I2C EEPROM support. Set to 1 for 24LC16(2K), 2 for larger sizes.
                             // Requires eeprom plugin. The CNC BoostePack has an on board EEPROM.
  #define TRINAMIC_ENABLE  0 // Trinamic TMC2130 stepper driver support. NOTE: work in progress.
  #define TRINAMIC_I2C     0 // Trinamic I2C - SPI bridge interface.
  #define TRINAMIC_DEV     0 // Development mode, adds a few M-codes to aid debugging. Do not enable in production code.
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#define STEP_PULSE_LATENCY 0.2f // microseconds

// End configuration

#if ETHERNET_ENABLE
#define NETWORK_HOSTNAME        "GRBL"
#define NETWORK_IPMODE          1 // 0 = static, 1 = DHCP, 2 = AutoIP
#define NETWORK_IP              "192.168.5.1"
#define NETWORK_GATEWAY         "192.168.5.1"
#define NETWORK_MASK            "255.255.255.0"
#define NETWORK_TELNET_PORT     23
#define NETWORK_WEBSOCKET_PORT  80
#define NETWORK_HTTP_PORT       80
#if NETWORK_IPMODE < 0 || NETWORK_IPMODE > 2
#error "Invalid IP mode selected!"
#endif
#endif

#if TRINAMIC_ENABLE
#include "src/tmc2130/trinamic.h"
#endif

#if QEI_ENABLE
#include "src/encoder/encoder.h"
#endif

#if SPINDLE_HUANYANG
#if USB_SERIAL_GRBL == 0
#error "Huanyang VFD cannot be used with UART communications enabled!"
#endif
#include "src/spindle/huanyang.h"
#endif

#if TRINAMIC_ENABLE || KEYPAD_ENABLE || ETHERNET_ENABLE || QEI_ENABLE

#define DRIVER_SETTINGS

typedef struct {
#if ETHERNET_ENABLE
    network_settings_t network;
#endif
#if TRINAMIC_ENABLE
    trinamic_settings_t trinamic;
#endif
#if KEYPAD_ENABLE
    jog_settings_t jog;
#endif
#if QEI_ENABLE
    encoder_settings_t encoder[QEI_ENABLE];
#endif
} driver_settings_t;

extern driver_settings_t driver_settings;

#endif

#ifdef BOARD_CNC_BOOSTERPACK
  #include "cnc_boosterpack_map.h"
#elif defined(BOARD_T41U5XBB)
  #include "T41U5XBB_map.h"
#else // default board

// Define step pulse output pins.
#define X_STEP_PIN      (2u)
#define Y_STEP_PIN      (4u)
#define Z_STEP_PIN      (6u)

// Define step direction output pins.
#define X_DIRECTION_PIN (3u)
#define Y_DIRECTION_PIN (5u)
#define Z_DIRECTION_PIN (7u)

// Define stepper driver enable/disable output pin(s).
#define STEPPERS_ENABLE_PIN (10u)

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN     (20u)
#define Y_LIMIT_PIN     (21u)
#define Z_LIMIT_PIN     (22u)

#if N_AXIS > 3
#define A_STEP_PIN      (8u)
#define A_DIRECTION_PIN (9u)
#define A_LIMIT_PIN     (23u)
#endif

#if N_AXIS > 4
#define B_STEP_PIN      (26u)
#define B_DIRECTION_PIN (27u)
#define B_LIMIT_PIN     (28u)
#endif

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PIN      (12u)
#define SPINDLE_DIRECTION_PIN   (11u)
#define SPINDLEPWMPIN           (13u) // NOTE: only pin 12 or pin 13 can be assigned!

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PIN   (19u)
#define COOLANT_MIST_PIN    (18u)

// Define user-control CONTROLs (cycle start, reset, feed hold, door) input pins.
#define RESET_PIN           (14u)
#define FEED_HOLD_PIN       (16u)
#define CYCLE_START_PIN     (17u)
#define SAFETY_DOOR_PIN     (29u)

// Define probe switch input pin.
#define PROBE_PIN           (15U)

#if EEPROM_ENABLE || KEYPAD_ENABLE
#define I2C_PORT    4
#define I2C_SCL4    (24u) // Not used, for info only
#define I2C_SDA4    (25u) // Not used, for info only
#endif

#if QEI_ENABLE
#define QEI_A_PIN      (0)
#define QEI_B_PIN      (3)
// #define QEI_INDEX_PIN  GPIO2_PIN
#define QEI_SELECT_PIN (1)
#endif

#endif // default pin mappings

#ifndef IOPORTS_ENABLE
#define IOPORTS_ENABLE 0
#endif

#if KEYPAD_ENABLE && !defined(KEYPAD_STROBE_PIN)
#error "KEYPAD_ENABLE requires KEYPAD_STROBE_PIN to be defined!"
#endif

#ifndef I2C_PORT
  #if EEPROM_ENABLE
  #error "EEPROM_ENABLE requires I2C_PORT to be defined!"
  #endif
  #if KEYPAD_ENABLE
  #error "KEYPAD_ENABLE requires I2C_PORT to be defined!"
  #endif
#endif

#if !(SPINDLEPWMPIN == 12 || SPINDLEPWMPIN == 13)
  #error "SPINDLEPWMPIN can only be routed to pin 12 or 13!"
#endif

#if QEI_ENABLE > 1
  #error "Max number of quadrature interfaces is 1!"
#endif

#if QEI_ENABLE > 0 && !(defined(QEI_A_PIN) && defined(QEI_B_PIN))
  #error "QEI_ENABLE requires encoder input pins A and B to be defined!"
#endif

// The following struct is pulled from the Teensy Library core, Copyright (c) 2019 PJRC.COM, LLC.

typedef struct {
    const uint8_t pin;              // The pin number
    const uint32_t mux_val;         // Value to set for mux;
    volatile uint32_t *select_reg;  // Which register controls the selection
    const uint32_t select_val;      // Value for that selection
} pin_info_t;

//

void selectStream (stream_type_t stream);

uint32_t xTaskGetTickCount();

#ifdef UART_DEBUG
void uart_debug_write (char *s);
#endif

#endif // __DRIVER_H__
