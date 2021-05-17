/*

  usb_serial.c - USB serial port implementation for STM32F103C8 ARM processors

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

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

#include "driver.h"

#if USB_SERIAL_CDC

#include "serial.h"
#include "../grbl/grbl.h"

#include "main.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

static char txdata2[BLOCK_TX_BUFFER_SIZE]; // Secondary TX buffer (for double buffering)
static bool use_tx2data = false;
static stream_rx_buffer_t rxbuf = {0};
static stream_block_tx_buffer_t txbuf = {0};

// NOTE: USB interrupt priority should be set lower than stepper/step timer to avoid jitter
// It is set in HAL_PCD_MspInit() in usbd_conf.c
void usbInit (void)
{
    MX_USB_DEVICE_Init();

    txbuf.s = txbuf.data;
    txbuf.max_length = BLOCK_TX_BUFFER_SIZE;
}

//
// Returns number of free characters in the input buffer
//
uint16_t usbRxFree (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;
    return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the input buffer
//
void usbRxFlush (void)
{
    rxbuf.head = rxbuf.tail = 0;
}

//
// Flushes and adds a CAN character to the input buffer
//
void usbRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = (rxbuf.tail + 1) & (RX_BUFFER_SIZE - 1);
}

//
// Writes current buffer to the USB output stream, swaps buffers
//
static inline bool usb_write (void)
{
    static uint8_t dummy = 0;

    txbuf.s = use_tx2data ? txdata2 : txbuf.data;

    while(CDC_Transmit_FS((uint8_t *)txbuf.s, txbuf.length) == USBD_BUSY) {
        if(!hal.stream_blocking_callback())
            return false;
    }

    if(txbuf.length % 64 == 0) {
        while(CDC_Transmit_FS(&dummy, 0) == USBD_BUSY) {
            if(!hal.stream_blocking_callback())
                return false;
        }
    }

    use_tx2data = !use_tx2data;
    txbuf.s = use_tx2data ? txdata2 : txbuf.data;
    txbuf.length = 0;

    return true;
}

//
// Writes a single character to the USB output stream, blocks if buffer full
//
bool usbPutC (const char c)
{
    static uint8_t buf[1];

    *buf = c;

    while(CDC_Transmit_FS(buf, 1) == USBD_BUSY) {
        if(!hal.stream_blocking_callback())
            return false;
    }

    return true;
}

//
// Writes a null terminated string to the USB output stream, blocks if buffer full
// Buffers string up to EOL (LF) before transmitting
//
void usbWriteS (const char *s)
{
    size_t length = strlen(s);

    if((length + txbuf.length) > txbuf.max_length) {
        if(!usb_write())
            return;
    }

    memcpy(txbuf.s, s, length);
    txbuf.length += length;
    txbuf.s += length;

    if(s[length - 1] == ASCII_LF) {
        if(!usb_write())
            return;
    }
}

//
// usbGetC - returns -1 if no data available
//
int16_t usbGetC (void)
{
    uint16_t bptr = rxbuf.tail;

    if(bptr == rxbuf.head)
        return -1; // no data available else EOF

    char data = rxbuf.data[bptr++];             // Get next character, increment tmp pointer
    rxbuf.tail = bptr & (RX_BUFFER_SIZE - 1);   // and update pointer

    return (int16_t)data;
}

bool usbSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

// NOTE: add a call to this function as the first line CDC_Receive_FS() in usbd_cdc_if.c
void usbBufferInput (uint8_t *data, uint32_t length)
{
    while(length--) {

        uint_fast16_t next_head = (rxbuf.head + 1)  & (RX_BUFFER_SIZE - 1); // Get and increment buffer pointer

        if(rxbuf.tail == next_head) {                                       // If buffer full
            rxbuf.overflow = 1;                                             // flag overflow
        } else {
            if(*data == CMD_TOOL_ACK && !rxbuf.backup) {
                stream_rx_backup(&rxbuf);
                hal.stream.read = usbGetC; // restore normal input
            } else if(!hal.stream.enqueue_realtime_command(*data)) {        // Check and strip realtime commands,
                rxbuf.data[rxbuf.head] = *data;                             // if not add data to buffer
                rxbuf.head = next_head;                                     // and update pointer
            }
        }
        data++;                                                             // next
    }
}

#endif
