/*
  mcu.h - peripherals emulator code for simulator MCU

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

#ifndef _MCU_H_

#define _MCU_H_

#include <stdint.h>
#include <stdbool.h>

#define MCU_N_TIMERS 3
#define MCU_N_GPIO 10

typedef enum  {
    Systick_IRQ = 0,
    UART_IRQ,
    Timer0_IRQ,
    Timer1_IRQ,
    Timer2_IRQ,
    GPIO0_IRQ,
    GPIO1_IRQ,
    GPIO2_IRQ,
    GPIO3_IRQ,
    GPIO4_IRQ,
    GPIO5_IRQ,
    GPIO6_IRQ,
    GPIO7_IRQ,
    GPIO8_IRQ,
    GPIO9_IRQ,
    IRQ_N_HANDLERS
} irq_num_t;

typedef struct
{
    volatile bool enable;
    bool irq_enable;
    uint32_t value;
    uint32_t load;
    uint32_t prescale;
    uint32_t prescaler;
    uint32_t compare;
} mcu_timer_t;

typedef struct
{
    bool rx_irq;
    bool tx_irq;
    bool tx_flag;
    bool rx_irq_enable;
    bool tx_irq_enable;
    uint8_t rx_data;
    uint8_t tx_data;
    uint32_t cdiv;
} mcu_uart_t;

typedef union {
    uint8_t value;
    uint8_t mask;
    struct {
        uint8_t pin0 :1,
                pin1 :1,
                pin2 :1,
                pin3 :1,
                pin4 :1,
                pin5 :1,
                pin6 :1,
                pin7 :1;
    };
} gpio_pins_t;

typedef struct
{
    gpio_pins_t dir;
    gpio_pins_t state;
    gpio_pins_t irq_mask;
    gpio_pins_t irq_state;
    gpio_pins_t rising;
    gpio_pins_t falling;
    gpio_pins_t pullup;
    gpio_pins_t pulldown;
} gpio_port_t;

extern mcu_uart_t uart;
extern mcu_timer_t timer[MCU_N_TIMERS];
extern gpio_port_t gpio[MCU_N_GPIO];
extern mcu_timer_t systick_timer;

typedef void (*interrupt_handler)(void);

void mcu_reset (void);
void mcu_enable_interrupts (void);
void mcu_disable_interrupts (void);
void mcu_master_clock (void);
void mcu_register_irq_handler (interrupt_handler handler, irq_num_t irq_num);
void mcu_gpio_set (gpio_port_t *port, uint8_t pins, uint8_t mask);
uint8_t mcu_gpio_get (gpio_port_t *port, uint8_t mask);
void mcu_gpio_in (gpio_port_t *port, uint8_t pins, uint8_t mask);
void mcu_gpio_toggle_in (gpio_port_t *port, uint8_t pin);
void simulate_serial (void);

#endif
