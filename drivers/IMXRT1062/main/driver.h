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

//
// NOTE: do NOT change configuration here - edit my_machine.h instead!
//

#ifndef __DRIVER_H__
#define __DRIVER_H__

#include "imxrt.h"
#include "core_pins.h"
#include "pins_arduino.h"

#ifndef OVERRIDE_MY_MACHINE
#include "my_machine.h"
#endif

#include "src/grbl/hal.h"
#include "src/grbl/nuts_bolts.h"

#if USB_SERIAL_GRBL > 0
//#define UART_DEBUG // For development only - enable only with USB_SERIAL_GRBL enabled and SPINDLE_HUANYANG disabled
#endif

#ifndef USB_SERIAL_GRBL
#define USB_SERIAL_GRBL     0 // for UART comms
#endif
#ifndef USB_SERIAL_WAIT
#define USB_SERIAL_WAIT     0
#endif
#ifndef SPINDLE_HUANYANG
#define SPINDLE_HUANYANG    0
#endif
#ifndef QEI_ENABLE
#define QEI_ENABLE          0
#endif

#ifndef ETHERNET_ENABLE
#define ETHERNET_ENABLE     0
#endif
#ifndef TELNET_ENABLE
#define TELNET_ENABLE       0
#endif
#ifndef WEBSOCKET_ENABLE
#define WEBSOCKET_ENABLE    0
#endif

#ifndef SDCARD_ENABLE
#define SDCARD_ENABLE       0
#endif
#ifndef KEYPAD_ENABLE
#define KEYPAD_ENABLE       0
#endif
#ifndef EEPROM_ENABLE
#define EEPROM_ENABLE       0
#endif
#ifndef EEPROM_IS_FRAM
#define EEPROM_IS_FRAM      0
#endif
#ifndef TRINAMIC_ENABLE
#define TRINAMIC_ENABLE     0
#endif
#ifndef TRINAMIC_I2C
#define TRINAMIC_I2C        0
#endif
#ifndef TRINAMIC_DEV
#define TRINAMIC_DEV        0
#endif

#ifndef ESTOP_ENABLE
  #if COMPATIBILITY_LEVEL <= 1
    #define ESTOP_ENABLE    1
  #else
    #define ESTOP_ENABLE    0
  #endif
#elif ESTOP_ENABLE && COMPATIBILITY_LEVEL > 1
  #warning "Enabling ESTOP may not work with all senders!"
#endif

#if ETHERNET_ENABLE
#ifndef NETWORK_HOSTNAME
#define NETWORK_HOSTNAME        "GRBL"
#endif
#ifndef NETWORK_IPMODE
#define NETWORK_IPMODE          1 // 0 = static, 1 = DHCP, 2 = AutoIP
#endif
#ifndef NETWORK_IP
#define NETWORK_IP              "192.168.5.1"
#endif
#ifndef NETWORK_GATEWAY
#define NETWORK_GATEWAY         "192.168.5.1"
#endif
#ifndef NETWORK_MASK
#define NETWORK_MASK            "255.255.255.0"
#endif
#ifndef NETWORK_TELNET_PORT
#define NETWORK_TELNET_PORT     23
#endif
#ifndef NETWORK_WEBSOCKET_PORT
#define NETWORK_WEBSOCKET_PORT  80
#endif
#ifndef NETWORK_HTTP_PORT
#define NETWORK_HTTP_PORT       80
#endif
#if NETWORK_IPMODE < 0 || NETWORK_IPMODE > 2
#error "Invalid IP mode selected!"
#endif
#endif

// End configuration

#ifdef BOARD_CNC_BOOSTERPACK
  #include "cnc_boosterpack_map.h"
#elif defined(BOARD_T41U5XBB)
  #include "T41U5XBB_map.h"
#else // default board
#include "generic_map.h"
#endif

// Adjust STEP_PULSE_LATENCY to get accurate step pulse length when required, e.g if using high step rates.
// The default value is calibrated for 10 microseconds length.
// NOTE: step output mode, number of axes and compiler optimization settings may all affect this value.
#ifndef STEP_PULSE_LATENCY
#define STEP_PULSE_LATENCY 0.2f // microseconds
#endif

#ifndef IOPORTS_ENABLE
#define IOPORTS_ENABLE 0
#endif

#if EEPROM_ENABLE && !defined(EEPROM_IS_FRAM)
#define EEPROM_IS_FRAM  0
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
