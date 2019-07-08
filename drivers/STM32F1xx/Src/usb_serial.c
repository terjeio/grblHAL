#include "driver.h"
#include "serial.h"
#include "../GRBL/grbl.h"

#include "main.h"
#include "usbd_cdc_if.h"
#include "usb_device.h"

static stream_rx_buffer_t rxbuf = {0};

#define USB_TXLEN 128

typedef struct {
	size_t length;
	char *s;
	char data[USB_TXLEN];
} usb_tx_buf;

usb_tx_buf txbuf = {0};

void usbInit (void)
{
	MX_USB_DEVICE_Init();

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

	if(length + txbuf.length < USB_TXLEN) {
		memcpy(txbuf.s, s, length);
		txbuf.length += length;
		txbuf.s += length;
		if(s[length - 1] == '\n') {
			length = txbuf.length;
			txbuf.length = 0;
			txbuf.s = txbuf.data;
			while(CDC_Transmit_FS((uint8_t *)txbuf.data, length) == USBD_BUSY) {
	            if(!hal.stream_blocking_callback())
	                return;
			}
		}
	}
//	while(CDC_Transmit_FS((uint8_t*)s, strlen(s)) == USBD_BUSY);
}

//
// usbGetC - returns -1 if no data available
//
int16_t usbGetC (void)
{
    uint16_t bptr = rxbuf.tail;

    if(bptr == rxbuf.head)
        return -1; // no data available else EOF

    char data = rxbuf.data[bptr++];           	// Get next character, increment tmp pointer
    rxbuf.tail = bptr & (RX_BUFFER_SIZE - 1);	// and update pointer

    return (int16_t)data;
}

void usbBufferInput (uint8_t *data, uint32_t length)
{
	while(length--) {

		uint_fast16_t next_head = (rxbuf.head + 1)  & (RX_BUFFER_SIZE - 1);	// Get and increment buffer pointer

		if(rxbuf.tail == next_head) {										// If buffer full
			rxbuf.overflow = 1;												// flag overflow
		} else {
			if(!hal.stream.enqueue_realtime_command(*data)) {				// Check and strip realtime commands,
				rxbuf.data[rxbuf.head] = *data;                  			// if not add data to buffer
				rxbuf.head = next_head;                    					// and update pointer
			}
		}
		data++;																// next
	}
}
