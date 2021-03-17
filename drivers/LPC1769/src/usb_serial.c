/*

  usb_serial.c - low level functions for transmitting bytes via the USB virtual serial port

  Part of grblHAL

  Copyright (c) 2017-2021 Terje Io

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

#ifndef __usb_serial_h__
#define __usb_serial_h__

#include "driver.h"

#if USB_SERIAL_CDC

#include <string.h>

#include "chip.h"
#include "app_usbd_cfg.h"
#include "cdc_vcom.h"
#include "grbl/hal.h"

extern const  USBD_HW_API_T hw_api;
extern const  USBD_CORE_API_T core_api;
extern const  USBD_CDC_API_T cdc_api;
static const  USBD_API_T g_usbApi = {
    &hw_api,
    &core_api,
    0,
    0,
    0,
    &cdc_api,
    0,
    0x02221101,
};

const USBD_API_T *g_pUsbApi = &g_usbApi;

static USBD_HANDLE_T g_hUsb;

/* Initialize pin and clocks for USB0/USB1 port */
static void usb_pin_clk_init(void)
{
    /* enable USB PLL and clocks */
    Chip_USB_Init();

    /* enable USB 1 port on the board */

    /* set a bias to this pin to enable the host system to detect and begin enumeration */
    Chip_IOCON_PinMux(LPC_IOCON, 2, 9, IOCON_MODE_INACT, IOCON_FUNC1);  /* USB_CONNECT */

    Chip_IOCON_PinMux(LPC_IOCON, 0, 29, IOCON_MODE_INACT, IOCON_FUNC1); /* P0.29 D1+, P0.30 D1- */
    Chip_IOCON_PinMux(LPC_IOCON, 0, 30, IOCON_MODE_INACT, IOCON_FUNC1);

    LPC_USB->USBClkCtrl = 0x12;                /* Dev, AHB clock enable */
    while ((LPC_USB->USBClkSt & 0x12) != 0x12);
}

/**
 * @brief   Handle interrupt from USB0
 * @return  Nothing
 */
void USB_IRQHandler(void)
{
    USBD_API->hw->ISR(g_hUsb);
}

/* Find the address of interface descriptor for given class type. */
USB_INTERFACE_DESCRIPTOR *find_IntfDesc(const uint8_t *pDesc, uint32_t intfClass)
{
    USB_COMMON_DESCRIPTOR *pD;
    USB_INTERFACE_DESCRIPTOR *pIntfDesc = 0;
    uint32_t next_desc_adr;

    pD = (USB_COMMON_DESCRIPTOR *) pDesc;
    next_desc_adr = (uint32_t) pDesc;

    while (pD->bLength) {
        /* is it interface descriptor */
        if (pD->bDescriptorType == USB_INTERFACE_DESCRIPTOR_TYPE) {

            pIntfDesc = (USB_INTERFACE_DESCRIPTOR *) pD;
            /* did we find the right interface descriptor */
            if (pIntfDesc->bInterfaceClass == intfClass) {
                break;
            }
        }
        pIntfDesc = 0;
        next_desc_adr = (uint32_t) pD + pD->bLength;
        pD = (USB_COMMON_DESCRIPTOR *) next_desc_adr;
    }

    return pIntfDesc;
}


#include "grbl/grbl.h"

#define USB_TXLEN 256

typedef struct {
    size_t length;
    char *s;
    char data[BLOCK_TX_BUFFER_SIZE];
} usb_tx_buf;

static usb_tx_buf txbuf = {0};
static stream_rx_buffer_t rxbuf = {0};

void usbInit (void)
{
    USBD_API_INIT_PARAM_T usb_param;
    USB_CORE_DESCS_T desc;
//  ErrorCode_t ret = LPC_OK;

    /* enable clocks and pinmux */
    usb_pin_clk_init();

    /* initialize call back structures */
    memset((void *) &usb_param, 0, sizeof(USBD_API_INIT_PARAM_T));
    usb_param.usb_reg_base = LPC_USB_BASE + 0x200;
    usb_param.max_num_ep = 3;
    usb_param.mem_base = USB_STACK_MEM_BASE;
    usb_param.mem_size = USB_STACK_MEM_SIZE;

    /* Set the USB descriptors */
    desc.device_desc = (uint8_t *) &USB_DeviceDescriptor[0];
    desc.string_desc = (uint8_t *) &USB_StringDescriptor[0];
    /* Note, to pass USBCV test full-speed only devices should have both
       descriptor arrays point to same location and device_qualifier set to 0.
     */
    desc.high_speed_desc = (uint8_t *) &USB_FsConfigDescriptor[0];
    desc.full_speed_desc = (uint8_t *) &USB_FsConfigDescriptor[0];
    desc.device_qualifier = 0;

    /* USB Initialization */
    if (USBD_API->hw->Init(&g_hUsb, &desc, &usb_param) == LPC_OK) {
        /* Init VCOM interface */
        if (vcom_init(g_hUsb, &desc, &usb_param) == LPC_OK) {
            /*  enable USB interrupts */
            NVIC_EnableIRQ(USB_IRQn);
            /* now connect */
            USBD_API->hw->Connect(g_hUsb, 1);
        }
    }

    txbuf.s = txbuf.data;
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
// Writes a null terminated string to the USB output stream, blocks if buffer full
// Buffers string up to EOL (LF) before transmitting
//
void usbWriteS (const char *s)
{
    size_t length = strlen(s);

    if(vcom_connected() && length + txbuf.length < sizeof(txbuf.data)) {
        memcpy(txbuf.s, s, length);
        txbuf.length += length;
        txbuf.s += length;
        if(s[length - 1] == '\n' || txbuf.length > 40) {
            length = txbuf.length;
            txbuf.s = txbuf.data;
            while(length) {
                txbuf.length = vcom_write((uint8_t *)txbuf.s, length > 64 ? 64 : length);
                txbuf.s += txbuf.length;
                length -= txbuf.length;
                if(!hal.stream_blocking_callback())
                    return;
            }
            txbuf.length = 0;
            txbuf.s = txbuf.data;
        }
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

#endif // __usb_serial_h__
