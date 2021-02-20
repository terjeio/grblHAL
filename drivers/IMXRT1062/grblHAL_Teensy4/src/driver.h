/*
  driver.h - driver code for IMXRT1062 processor (on Teensy 4.0 board)

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

#include "grbl/hal.h"
#include "grbl/nuts_bolts.h"


#define DIGITAL_IN(gpio) (!!(gpio.reg->DR & gpio.bit))
#define DIGITAL_OUT(gpio, on) { if(on) gpio.reg->DR_SET = gpio.bit; else gpio.reg->DR_CLEAR = gpio.bit; }

#if USB_SERIAL_CDC > 0
//#define UART_DEBUG // For development only - enable only with USB_SERIAL_CDC enabled and SPINDLE_HUANYANG disabled
#endif

#ifndef USB_SERIAL_CDC
#define USB_SERIAL_CDC      0 // for UART comms
#endif
#ifndef USB_SERIAL_WAIT
#define USB_SERIAL_WAIT     0
#endif
#ifndef PLASMA_ENABLE
#define PLASMA_ENABLE       0
#endif
#ifndef PPI_ENABLE
#define PPI_ENABLE          0
#endif
#ifndef SPINDLE_HUANYANG
#define SPINDLE_HUANYANG    0
#endif
#ifndef QEI_ENABLE
#define QEI_ENABLE          0
#endif
#ifndef ODOMETER_ENABLE
#define ODOMETER_ENABLE     0
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

// Timer assignments (for reference, Arduino libs does not follow the CMSIS style...)

//#define STEPPER_TIMER     PIT0 (32 bit)
//#define PULSE_TIMER       TMR4
//#define SPINDLE_PWM_TIMER TMR1 (pin 12) or TMR2 (pin 3)
//#define DEBOUNCE_TIMER    TMR3
//#define PLASMA_TIMER      TMR2
//#define PPI_TIMER         inverse of SPINDLE_PWM_TIMER

// Timers used for spindle encoder if spindle sync is enabled:
//#define RPM_TIMER         GPT1
//#define RPM_COUNTER       GPT2

// End configuration

#ifdef BOARD_CNC_BOOSTERPACK
  #include "cnc_boosterpack_map.h"
#elif defined(BOARD_T40X101)
  #include "T40X101_map.h"
#elif defined(BOARD_T41U5XBB)
  #include "T41U5XBB_map.h"
#elif defined(BOARD_T41U5XSS)
  #include "T41U5XSS_map.h"
#elif defined(BOARD_T41PROBB)
  #include "T41ProBB_map.h"
#elif defined(BOARD_MY_MACHINE)
  #include "my_machine_map.h"
#else // default board
#include "generic_map.h"
#endif

#if SPINDLEPWMPIN == 12
#define PPI_TIMER       (IMXRT_TMR2)
#define PPI_TIMERIRQ    IRQ_QTIMER2
#else
#define PPI_TIMER       (IMXRT_TMR1)
#define PPI_TIMERIRQ    IRQ_QTIMER1
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
#include "tmc2130/trinamic.h"
#endif

#if QEI_ENABLE
#include "encoder/encoder.h"
#endif

#if SPINDLE_HUANYANG
#if USB_SERIAL_CDC == 0
#error "Huanyang VFD cannot be used with UART communications enabled!"
#endif
#include "spindle/huanyang.h"
#endif

#ifndef VFD_SPINDLE
#define VFD_SPINDLE 0
#endif

#if PLASMA_ENABLE
#include "plasma/thc.h"
#endif

#if ODOMETER_ENABLE
#include "odometer/odometer.h"
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

typedef enum {
    Input_Probe = 0,
    Input_Reset,
    Input_FeedHold,
    Input_CycleStart,
    Input_SafetyDoor,
    Input_LimitsOverride,
    Input_EStop,
    Input_ModeSelect,
    Input_LimitX,
    Input_LimitX_Max,
    Input_LimitY,
    Input_LimitY_Max,
    Input_LimitZ,
    Input_LimitZ_Max,
    Input_LimitA,
    Input_LimitA_Max,
    Input_LimitB,
    Input_LimitB_Max,
    Input_LimitC,
    Input_LimitC_Max,
    Input_KeypadStrobe,
    Input_QEI_A,
    Input_QEI_B,
    Input_QEI_Select,
    Input_QEI_Index,
    Input_SpindleIndex,
    Input_Aux0,
    Input_Aux1,
    Input_Aux2,
    Input_Aux3
} input_t;

typedef enum {
    IRQ_Mode_None    = 0b00,
    IRQ_Mode_Change  = 0b01,
    IRQ_Mode_Rising  = 0b10,
    IRQ_Mode_Falling = 0b11
} irq_mode_t;

typedef struct {
    volatile uint32_t DR;
    volatile uint32_t GDIR;
    volatile uint32_t PSR;
    volatile uint32_t ICR1;
    volatile uint32_t ICR2;
    volatile uint32_t IMR;
    volatile uint32_t ISR;
    volatile uint32_t EDGE_SEL;
    uint32_t unused[25];
    volatile uint32_t DR_SET;
    volatile uint32_t DR_CLEAR;
    volatile uint32_t DR_TOGGLE;
} gpio_reg_t;

typedef struct {
    gpio_reg_t *reg;
    uint32_t bit;
} gpio_t;

typedef struct {
    input_t id;
    uint8_t group;
    uint8_t pin;
    gpio_t *port;
    gpio_t gpio; // doubled up for now for speed...
    irq_mode_t irq_mode;
    uint8_t offset;
    volatile bool active;
    volatile bool debounce;
} input_signal_t;

// The following struct is pulled from the Teensy Library core, Copyright (c) 2019 PJRC.COM, LLC.

typedef struct {
    const uint8_t pin;              // The pin number
    const uint32_t mux_val;         // Value to set for mux;
    volatile uint32_t *select_reg;  // Which register controls the selection
    const uint32_t select_val;      // Value for that selection
} pin_info_t;

//

void selectStream (stream_type_t stream);
void pinModeOutput (gpio_t *gpio, uint8_t pin);

uint32_t xTaskGetTickCount();

#ifdef UART_DEBUG
void uart_debug_write (char *s);
#endif

#endif // __DRIVER_H__
