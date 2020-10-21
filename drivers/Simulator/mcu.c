/*
  mcu.c - peripherals emulator code for simulator MCU

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

#include <stdio.h>
#include <string.h>

#include "mcu.h"
#include "simulator.h"
#include "grbl/hal.h"

static volatile bool irq_enable = false;
static bool booted = false;
static interrupt_handler isr[IRQ_N_HANDLERS];

mcu_uart_t uart;
mcu_timer_t timer[MCU_N_TIMERS];
mcu_timer_t systick_timer;
gpio_port_t gpio[MCU_N_GPIO];

static void default_handler (void)
{
    // NOOP - TODO: change to blocking loop?
}

void mcu_reset (void)
{
    uint_fast8_t i;

    irq_enable = false;

    for(i = 0; i < IRQ_N_HANDLERS; i++)
        isr[i] = default_handler;

    memset(&uart, 0, sizeof(mcu_uart_t));
    memset(&timer, 0, sizeof(mcu_timer_t) * MCU_N_TIMERS);
    memset(&systick_timer, 0, sizeof(mcu_timer_t));
    memset(&gpio, 0, sizeof(gpio_port_t) * MCU_N_GPIO);

    irq_enable = true;
    booted = true;
}

void mcu_register_irq_handler (interrupt_handler handler, irq_num_t irq_num)
{
    isr[irq_num] = handler == NULL ? default_handler : handler;
}

void mcu_enable_interrupts (void)
{
    irq_enable = true;
}

void mcu_disable_interrupts (void)
{
    irq_enable = false;
}

void mcu_master_clock (void)
{
    uint_fast8_t i;

    if(!booted)
        return;

    for(i = 0; i < MCU_N_TIMERS; i++) {

        if(timer[i].enable) {

            if(timer[i].prescaler) {
                if(timer[i].prescale == 0)
                    timer[i].prescale = timer[i].prescaler;
                else {
                    if(--timer[i].prescale != 0)
                        continue;
                    else
                        timer[i].prescale = timer[i].prescaler;
                }
            }

            if(timer[i].value == 0)
                timer[i].value = timer[i].load;
            else if(--timer[i].value == 0) {
                if(timer[i].irq_enable && irq_enable) {
                    switch(i)
                    {
                        case 0:
                            isr[Timer0_IRQ]();
                            break;
                        case 1:
                            isr[Timer1_IRQ]();
                            break;
                        case 2:
                            isr[Timer2_IRQ]();
                            break;                    }
                } 
                timer[i].value = timer[i].load;
            }
        }
    }

    for(i = 0; i < MCU_N_GPIO; i++) {
        if(gpio[i].irq_state.value & gpio[i].irq_mask.value)
            isr[GPIO0_IRQ + i]();
    }

    if(systick_timer.enable) {
        if(systick_timer.value == 0)
            systick_timer.value = systick_timer.load;
        else if(--systick_timer.value == 0) {
            if(systick_timer.irq_enable && irq_enable)
                isr[Systick_IRQ]();
            systick_timer.value = systick_timer.load;
        }
    }
}

void mcu_gpio_set (gpio_port_t *port, uint8_t pins, uint8_t mask)
{
    port->state.value = (port->state.value & ~mask) | (pins & mask);
}

uint8_t mcu_gpio_get (gpio_port_t *port, uint8_t mask)
{
    return port->state.value & mask;
}

void mcu_gpio_toggle_in (gpio_port_t *port, uint8_t pins)
{
    mcu_gpio_in(port, (port->state.value & pins) ^ pins, pins);
}

// Set input pin, trigger interrupt if enabled
void mcu_gpio_in (gpio_port_t *port, uint8_t pins, uint8_t mask)
{
    pins &= mask;

    uint8_t changed = (port->state.value & mask) ^ pins, bitflag = 1;

    do {
        if(changed & bitflag) {
            if(port->state.value & bitflag) {
                if(port->falling.value & bitflag)
                    port->irq_state.value |= bitflag;
            } else {
                if(port->rising.value & bitflag)
                    port->irq_state.value |= bitflag;
            }
            changed &= ~bitflag;
        }
        bitflag <<= 1;
    } while(changed);

    port->state.value = (port->state.value & ~mask) | pins;
}

// TODO: move to mcu_master_clock() above
void simulate_serial (void)
{
    if(!booted)
        return;

    if(uart.tx_flag) {
        sim.putchar(uart.tx_data);
        uart.tx_flag = 0;
    }

    if((uart.tx_irq = uart.tx_irq_enable))
        isr[UART_IRQ]();

    if(uart.rx_irq_enable && !uart.rx_irq && hal.stream.get_rx_buffer_available() > 100) {
        uint8_t char_in = sim.getchar();
        if (char_in) {
            uart.rx_data = char_in;
            uart.rx_irq = 1;
            isr[UART_IRQ]();
        }
    }
}
