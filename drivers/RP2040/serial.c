/*
  serial.c - driver code for RP2040 processor

  Part of grblHAL

  Copyright (c) 2021 Terje Io

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

#include "hardware/gpio.h"
#include "hardware/uart.h"
#include "hardware/irq.h"

#include "serial.h"
#include "driver.h"

#define UART_TX_PIN 0
#define UART_RX_PIN 1

#ifndef UART_PORT
#define UART_PORT uart0
#define UART ((uart_hw_t *)UART_PORT)
#define UART_IRQ UART0_IRQ
#endif

#define RX_BUFFER_MODULO RX_BUFFER_SIZE - 1
#define TX_BUFFER_MODULO TX_BUFFER_SIZE - 1
#define bufferIncPointer(ptr, size) ((ptr + 1) & size)

static uint16_t tx_fifo_size;
static stream_tx_buffer_t txbuffer = {0};
static stream_rx_buffer_t rxbuffer = {0}, rxbackup;

static void uart_interrupt_handler (void);

void serialInit (uint32_t baud_rate)
{
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    
    uart_init(UART_PORT, 2400);

    uart_set_hw_flow(UART_PORT, false, false);
    uart_set_format(UART_PORT, 8, 1, UART_PARITY_NONE);
    uart_set_baudrate(UART_PORT, 115200);
    uart_set_fifo_enabled(UART_PORT, true);

    irq_set_exclusive_handler(UART_IRQ, uart_interrupt_handler);
    irq_set_enabled(UART_IRQ, true);
    
    hw_set_bits(&UART->imsc, UART_UARTIMSC_RXIM_BITS|UART_UARTIMSC_RTIM_BITS);             
}

bool serialSetBaudRate (uint32_t baud_rate)
{
    static bool init_ok = false;

    if(!init_ok) {
        serialInit(baud_rate);
        init_ok = true;
    }

    return true;
}

//
// serialGetC - returns -1 if no data available
//
int16_t serialGetC (void)
{
    int16_t data;
    uint_fast16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head)
        return -1; // no data available else EOF

    data = rxbuffer.data[bptr];                   // Get next character, increment tmp pointer
    rxbuffer.tail = bufferIncPointer(bptr, RX_BUFFER_MODULO);    // and update pointer

    return data;
}

void serialTxFlush (void)
{
    txbuffer.tail = txbuffer.head;
}

uint16_t serialRxCount (void)
{
    uint_fast16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t serialRxFree (void)
{
    return RX_BUFFER_MODULO - serialRxCount();
}

void serialRxFlush (void)
{
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.overflow = false;
}

void serialRxCancel (void)
{
    serialRxFlush();
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.head = bufferIncPointer(rxbuffer.head, RX_BUFFER_MODULO);
}

bool serialPutC (const char c)
{
    uint_fast16_t next_head;

    if(!(UART->imsc & UART_UARTIMSC_TXIM_BITS)) {                   // If the transmit interrupt is deactivated
        if(!(UART->fr & UART_UARTFR_TXFF_BITS)) {                   // and if the TX FIFO is not full
            UART->dr = c;                                           // Write data in the TX FIFO
            return true;
        } else
            hw_set_bits(&UART->imsc, UART_UARTIMSC_TXIM_BITS);      // Enable transmit interrupt
    }

    // Write data in the Buffer is transmit interrupt activated or TX FIFO is                                                                
    next_head = bufferIncPointer(txbuffer.head, TX_BUFFER_MODULO);  // Get and update head pointer

    while(txbuffer.tail == next_head) {                             // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    txbuffer.data[txbuffer.head] = c;                               // Add data to buffer
    txbuffer.head = next_head;                                      // and update head pointer

    return true;
}

void serialWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

void serialWrite(const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serialPutC(*ptr++);
}

// "dummy" version of serialGetC
static int16_t serialGetNull (void)
{
    return -1;
}

bool serialSuspendInput (bool suspend)
{
    if(suspend)
        hal.stream.read = serialGetNull;
    else if(rxbuffer.backup)
        memcpy(&rxbuffer, &rxbackup, sizeof(stream_rx_buffer_t));

    return rxbuffer.tail != rxbuffer.head;
}

uint16_t serialTxCount(void) {

    uint_fast16_t head = txbuffer.head, tail = txbuffer.tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + ((UART->fr & UART_UARTFR_BUSY_BITS) ? 0 : 1);
}

static void uart_interrupt_handler (void)
{
    uint_fast16_t bptr;
    uint32_t data, ctrl = UART->mis;

    if(ctrl & (UART_UARTMIS_RXMIS_BITS | UART_UARTIMSC_RTIM_BITS)) {

        while (uart_is_readable(UART_PORT)) {

            bptr = bufferIncPointer(rxbuffer.head, RX_BUFFER_MODULO);  // Get next head pointer
            data = UART->dr & 0xFF;                             // and read input (use only 8 bits of data)

            if(bptr == rxbuffer.tail) {                         // If buffer full
                rxbuffer.overflow = true;                       // flag overflow
            } else {
#if MODBUS_ENABLE
                rxbuffer.data[rxbuffer.head] = (char)data;  // Add data to buffer
                rxbuffer.head = bptr;                       // and update pointer
#else
                if(data == CMD_TOOL_ACK && !rxbuffer.backup) {
                    memcpy(&rxbackup, &rxbuffer, sizeof(stream_rx_buffer_t));
                    rxbuffer.backup = true;
                    rxbuffer.tail = rxbuffer.head;
                    hal.stream.read = serialGetC; // restore normal input
                } else if(!hal.stream.enqueue_realtime_command((char)data)) {
                    rxbuffer.data[rxbuffer.head] = (char)data;  // Add data to buffer
                    rxbuffer.head = bptr;                       // and update pointer
                }
#endif
            }
        }
    }

    // Interrupt if the TX FIFO is lower or equal to the empty TX FIFO threshold
    if(ctrl & UART_UARTMIS_TXMIS_BITS)
    {
        bptr = txbuffer.tail;

        // As long as the TX FIFO is not full or the buffer is not empty
        while((!(UART->fr & UART_UARTFR_TXFF_BITS)) && (txbuffer.head != bptr)) {
            UART->dr = txbuffer.data[bptr];                         // Put character in TX FIFO
            bptr = bufferIncPointer(bptr, TX_BUFFER_MODULO);        // and update tmp tail pointer
        }

        txbuffer.tail = bptr;                                       //  Update tail pointer

        if(txbuffer.tail == txbuffer.head)						    // Disable TX interrupt when the TX buffer is empty
            hw_clear_bits(&UART->imsc, UART_UARTIMSC_TXIM_BITS);
    }
}