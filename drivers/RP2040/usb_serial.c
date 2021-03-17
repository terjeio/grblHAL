/*

  usb_serial.c - driver code for RP2040

  Part of grblHAL

  Some parts are copyright (c) 2021 Terje Io

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

***

  Some parts of the code is lifted from the Pico SDK pico_stdio_usb library and are

  Copyright (c) 2020 Raspberry Pi (Trading) Ltd.

  SPDX-License-Identifier: BSD-3-Clause
  
*/

#include "tusb.h"

#include "pico/time.h"
#include "pico/binary_info.h"
#include "hardware/irq.h"

#include <string.h>

#include "usb_serial.h"
#include "driver.h"

//#if USB_SERIAL_CDC == 2

#define BLOCK_RX_BUFFER_SIZE 20

static stream_block_tx_buffer_t txbuf = {0};
static char rxbuf[BLOCK_RX_BUFFER_SIZE];
static stream_rx_buffer_t usb_rxbuffer, usb_rxbackup;


// PICO_CONFIG: PICO_STDIO_USB_STDOUT_TIMEOUT_US, Number of microseconds to be blocked trying to write USB output before assuming the host has disappeared and discarding data, default=500000, group=pico_stdio_usb
#ifndef PICO_STDIO_USB_STDOUT_TIMEOUT_US
#define PICO_STDIO_USB_STDOUT_TIMEOUT_US 500000
#endif

// todo perhaps unnecessarily high?
// PICO_CONFIG: PICO_STDIO_USB_TASK_INTERVAL_US, Period of microseconds between calling tud_task in the background, default=1000, advanced=true, group=pico_stdio_usb
#ifndef PICO_STDIO_USB_TASK_INTERVAL_US
#define PICO_STDIO_USB_TASK_INTERVAL_US 1000
#endif

// PICO_CONFIG: PICO_STDIO_USB_LOW_PRIORITY_IRQ, low priority (non hardware) IRQ number to claim for tud_task() background execution, default=31, advanced=true, group=pico_stdio_usb
#ifndef PICO_STDIO_USB_LOW_PRIORITY_IRQ
#define PICO_STDIO_USB_LOW_PRIORITY_IRQ 31
#endif

static_assert(PICO_STDIO_USB_LOW_PRIORITY_IRQ > RTC_IRQ, ""); // note RTC_IRQ is currently the last one
static mutex_t stdio_usb_mutex;

static void low_priority_worker_irq() {
    // if the mutex is already owned, then we are in user code
    // in this file which will do a tud_task itself, so we'll just do nothing
    // until the next tick; we won't starve
    if (mutex_try_enter(&stdio_usb_mutex, NULL)) {
        tud_task();
        mutex_exit(&stdio_usb_mutex);
    }
}

static int64_t timer_task(__unused alarm_id_t id, __unused void *user_data) {
    irq_set_pending(PICO_STDIO_USB_LOW_PRIORITY_IRQ);
    return PICO_STDIO_USB_TASK_INTERVAL_US;
}

static void stdio_usb_out_chars(const char *buf, int length) {
    static uint64_t last_avail_time;
    uint32_t owner;
    if (!mutex_try_enter(&stdio_usb_mutex, &owner)) {
        if (owner == get_core_num()) return; // would deadlock otherwise
        mutex_enter_blocking(&stdio_usb_mutex);
    }
    if (tud_cdc_connected()) {
        for (int i = 0; i < length;) {
            int n = length - i;
            int avail = tud_cdc_write_available();
            if (n > avail) n = avail;
            if (n) {
                int n2 = tud_cdc_write(buf + i, n);
                tud_task();
                tud_cdc_write_flush();
                i += n2;
                last_avail_time = time_us_64();
            } else {
                tud_task();
                tud_cdc_write_flush();
                if (!tud_cdc_connected() ||
                    (!tud_cdc_write_available() && time_us_64() > last_avail_time + PICO_STDIO_USB_STDOUT_TIMEOUT_US)) {
                    break;
                }
            }
        }
    } else {
        // reset our timeout
        last_avail_time = 0;
    }
    mutex_exit(&stdio_usb_mutex);
}

int stdio_usb_in_chars(char *buf, int length) {
    uint32_t owner;
    if (!mutex_try_enter(&stdio_usb_mutex, &owner)) {
        if (owner == get_core_num()) return PICO_ERROR_NO_DATA; // would deadlock otherwise
        mutex_enter_blocking(&stdio_usb_mutex);
    }
    int rc = PICO_ERROR_NO_DATA;
    if (tud_cdc_connected() && tud_cdc_available()) {
        int count = tud_cdc_read(buf, length);
        rc =  count ? count : PICO_ERROR_NO_DATA;
    }
    mutex_exit(&stdio_usb_mutex);
    return rc;
}

bool usb_serialInit (void)
{
    // initialize TinyUSB
    tusb_init();

    irq_set_exclusive_handler(PICO_STDIO_USB_LOW_PRIORITY_IRQ, low_priority_worker_irq);
    irq_set_enabled(PICO_STDIO_USB_LOW_PRIORITY_IRQ, true);

    mutex_init(&stdio_usb_mutex);
    bool rc = add_alarm_in_us(PICO_STDIO_USB_TASK_INTERVAL_US, timer_task, NULL, true);

    txbuf.s = txbuf.data;
    txbuf.max_length = CFG_TUD_CDC_TX_BUFSIZE;
    txbuf.max_length = (txbuf.max_length > BLOCK_TX_BUFFER_SIZE ? BLOCK_TX_BUFFER_SIZE : txbuf.max_length) - 20;

    return rc;
}

//
// Returns number of characters in serial input buffer
//
uint16_t usb_serialRxCount (void)
{
    uint_fast16_t tail = usb_rxbuffer.tail, head = usb_rxbuffer.head;
    return (uint16_t)BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of free characters in serial input buffer
//
uint16_t usb_serialRxFree (void)
{
    uint_fast16_t tail = usb_rxbuffer.tail, head = usb_rxbuffer.head;
    return (uint16_t)((RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE));
}

//
// Flushes the serial input buffer (including the USB buffer)
//
void usb_serialRxFlush (void)
{
  //  usb_serial_flush_input();
    usb_rxbuffer.tail = usb_rxbuffer.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
void usb_serialRxCancel (void)
{
    usb_rxbuffer.data[usb_rxbuffer.head] = CMD_RESET;
    usb_rxbuffer.tail = usb_rxbuffer.head;
    usb_rxbuffer.head = (usb_rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
}

//
// Writes a character to the serial output stream
//
bool usb_serialPutC (const char c)
{
  //  usb_serial_putchar(c);

    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
void usb_serialWriteS (const char *s)
{
    if(*s == '\0')
        return;

    size_t length = strlen(s);

    if((length + txbuf.length) < BLOCK_TX_BUFFER_SIZE) {

        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;

        if(s[length - 1] == ASCII_LF || txbuf.length > txbuf.max_length) {

            size_t txfree;
            txbuf.s = txbuf.data;

            while(txbuf.length) {

                if(txfree = tud_cdc_write_available() > 10) {

                    length = txfree < txbuf.length ? txfree : txbuf.length;

                    stdio_usb_out_chars(txbuf.s, length); //

                    txbuf.length -= length;
                    txbuf.s += length;
                }

                if(txbuf.length && !hal.stream_blocking_callback()) {
                    txbuf.length = 0;
                    txbuf.s = txbuf.data;
                    return;
                }
            }
            txbuf.s = txbuf.data;
        }
    }
}

//
// Writes a null terminated string to the serial output stream followed by EOL, blocks if buffer full
//
void usb_serialWriteLn (const char *s)
{
    usb_serialWriteS(s);
    usb_serialWriteS(ASCII_EOL);
}

//
// Writes a number of characters from string to the serial output stream followed by EOL, blocks if buffer full
//
void usb_serialWrite (const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        usb_serialPutC(*ptr++);
}

//
// serialGetC - returns -1 if no data available
//
int16_t usb_serialGetC (void)
{
    uint16_t bptr = usb_rxbuffer.tail;

    if(bptr == usb_rxbuffer.head)
        return -1; // no data available else EOF

    char data = usb_rxbuffer.data[bptr++];              // Get next character, increment tmp pointer
    usb_rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);    // and update pointer

    return (int16_t)data;
}

bool usb_serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&usb_rxbuffer, suspend);
}

//
// This function get called from the systick interrupt handler,
// used here to get characters off the USB serial input stream and buffer
// them for processing by grbl. Real time command characters are stripped out
// and submitted for realtime processing.
//
void usb_execute_realtime (uint_fast16_t state)
{
    char c, *dp;
    int avail, free;

    if((avail = tud_cdc_available())) {

        dp = rxbuf;
        free = usb_serialRxFree();
        free = free > BLOCK_RX_BUFFER_SIZE ? BLOCK_RX_BUFFER_SIZE : free;

        avail = stdio_usb_in_chars(rxbuf, avail > free ? free : avail);

        while(avail--) {
            c = *dp++;
            if(c == CMD_TOOL_ACK && !usb_rxbuffer.backup) {
                stream_rx_backup(&usb_rxbuffer);
                hal.stream.read = usb_serialGetC; // restore normal input
            } else if(!hal.stream.enqueue_realtime_command(c)) {
                uint32_t bptr = (usb_rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1); // Get next head pointer
                if(bptr == usb_rxbuffer.tail)                                   // If buffer full
                    usb_rxbuffer.overflow = On;                                 // flag overflow,
                else {
                    usb_rxbuffer.data[usb_rxbuffer.head] = c;                   // else add character data to buffer
                    usb_rxbuffer.head = bptr;                                   // and update pointer
                }
            }
        }
    }
}
