/*

  serial.c - LPC17xx low level functions for transmitting bytes via the serial port

  Part of Grbl

  Copyright (c) 2017-2019 Terje Io

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

#ifndef __serial_h__
#define __serial_h__

#include "driver.h"
#include "serial.h"
#include "grbl/grbl.h"
#include "VCOM_lib/usbSerial.h"

#define BUFCOUNT(head, tail, size) ((head >= tail) ? (head - tail) : (size - tail + head))

#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128
#endif
#ifndef TX_BUFFER_SIZE
  #define TX_BUFFER_SIZE 64
#endif

static char rxbuf[RX_BUFFER_SIZE];
static volatile uint16_t rx_head = 0, rx_tail = 0, rx_overflow = 0;

static char txbuf[TX_BUFFER_SIZE];
static volatile uint16_t tx_head = 0, tx_tail = 0;

#ifdef RTS_PORT
  static volatile uint8_t rts_state = 0;
#endif

#ifdef ENABLE_XONXOFF
  static volatile uint8_t flow_ctrl = XON_SENT; // Flow control state variable
#endif

void USB_StateCallback(bool dtr, bool rts)
{

}

// Receives serial data. Called by an interrupt.
void USB_ReadCallback(const char* data, unsigned len)
{
    uint16_t bptr;
    char rxdata;

	while (len--) {

		bptr = (rx_head + 1) & (RX_BUFFER_SIZE - 1);    // Get next head pointer

        if(bptr == rx_tail) {   // If buffer full
            rx_overflow = 1;    // flag overflow
        } else {
            rxdata = *data++;
            if(!hal.stream.enqueue_realtime_command(rxdata)) {
                rxbuf[rx_head] = rxdata;    // Add data to buffer
                rx_head = bptr;             // and update pointer
            }
        }
	}
}


void serialInit (void)
{
	#ifdef USE_USB
	usbSerialInit(USB_StateCallback, USB_ReadCallback);
	#else
	int32_t uartFlags = ARM_USART_MODE_ASYNCHRONOUS |
					  ARM_USART_DATA_BITS_8 |
					  ARM_USART_PARITY_NONE |
					  ARM_USART_STOP_BITS_1 |
					  ARM_USART_FLOW_CONTROL_NONE;

	serialDriver.Initialize(serialInterrupt);
	serialDriver.PowerControl(ARM_POWER_FULL);
	serialDriver.Control(uartFlags, 115200);
	serialDriver.Control(ARM_USART_CONTROL_TX, 1);
	serialDriver.Control(ARM_USART_CONTROL_RX, 1);

	//Issue first read
	serialDriver.Receive(arm_rx_buf, 1);
	#endif
#ifdef RTS_PORT
	RTS_PORT->DIR |= RTS_BIT;
    BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = 0;
#endif
}

//
// Returns number of characters in serial output buffer
//
uint16_t serialTxCount (void)
{
  uint16_t tail = tx_tail;
  return BUFCOUNT(tx_head, tail, TX_BUFFER_SIZE);
}

//
// Returns number of characters in serial input buffer
//
uint16_t serialRxCount (void)
{
  uint16_t tail = rx_tail, head = rx_head;
  return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of free characters in serial input buffer
//
uint16_t serialRxFree (void)
{
  unsigned int tail = rx_tail, head = rx_head;
  return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
void serialRxFlush (void)
{
    rx_head = rx_tail = 0;
#ifdef RTS_PORT
    BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = 0;
#endif
}

//
// Flushes and adds a CAN character to the serial input buffer
//
void serialRxCancel (void)
{
    rxbuf[rx_head] = ASCII_CAN;
    rx_tail = rx_head;
    rx_head = (rx_tail + 1) & (RX_BUFFER_SIZE - 1);
#ifdef RTS_PORT
    BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = 0;
#endif
}

//
// Attempt to send a character bypassing buffering
//
static inline bool serialPutCNonBlocking (const char c)
{
    bool ok;

//    if((ok = !(SERIAL_MODULE->IE & EUSCI_A_IE_TXIE) && !(SERIAL_MODULE->STATW & EUSCI_A_STATW_BUSY)))
//        SERIAL_MODULE->TXBUF = c;

    return ok;
}

//
// Writes a character to the serial output stream
//
bool serialPutC (const char c) {
#ifdef USE_USB
	while(VCOM_putchar(c) == EOF);
#else
    uint32_t next_head;

    if(tx_head != tx_tail || !serialPutCNonBlocking(c)) {   // Try to send character without buffering...

        next_head = (tx_head + 1) & (TX_BUFFER_SIZE - 1);   // .. if not, set and update head pointer

        while(tx_tail == next_head) {                       // While TX buffer full
            if(!serialBlockingCallback())                   // check if blocking for space,
                return false;                               // exit if not (leaves TX buffer in an inconsistent state)
        }

        txbuf[tx_head] = c;                                 // Add data to buffer
        tx_head = next_head;                                // and update head pointer

//        SERIAL_MODULE->IE |= EUSCI_A_IE_TXIE;               // Enable TX interrupts
    }
#endif
    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
void serialWriteS (const char *s)
{
    char c, *ptr = (char *)s;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

//
// Writes a null terminated string to the serial output stream followed by EOL, blocks if buffer full
//
void serialWriteLn (const char *s)
{
    serialWriteS(s);
    serialWriteS(ASCII_EOL);
}

//
// Writes a number of characters from string to the serial output stream followed by EOL, blocks if buffer full
//
void serialWrite(const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serialPutC(*ptr++);
}

//
// serialGetC - returns -1 if no data available
//
int16_t serialGetC (void)
{
    uint16_t bptr = rx_tail;

    if(bptr == rx_head)
        return -1; // no data available else EOF

    char data = rxbuf[bptr++];              // Get next character, increment tmp pointer
    rx_tail = bptr & (RX_BUFFER_SIZE - 1);  // and update pointer

#ifdef RTS_PORT
    if (rts_state && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM) // Clear RTS if below LWM
        BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = rts_state = 0;
#endif

    return (int16_t)data;
}

//
void SERIAL_IRQHandler (void)
{
	/*
    char data;
	uint16_t bptr;

	switch(SERIAL_MODULE->IV) {

        case 0x04:
            bptr = tx_tail;                             // Temp tail position (to avoid volatile overhead)
            SERIAL_MODULE->TXBUF = txbuf[bptr++];       // Send a byte from the buffer
            bptr &= (TX_BUFFER_SIZE - 1);               // and update
            tx_tail = bptr;                             // tail position
            if (bptr == tx_head)                        // Turn off TX interrupt
                SERIAL_MODULE->IE &= ~EUSCI_A_IE_TXIE;  // when buffer empty
            break;

        case 0x02:
            data = SERIAL_MODULE->RXBUF;                            // Read character received
            bptr = (rx_head + 1) & (RX_BUFFER_SIZE - 1);            // Temp head position (to avoid volatile overhead)
            if(bptr == rx_tail) {                                   // If buffer full
                rx_overflow = 1;                                    // flag overflow
            } else {
                if(!serialReceiveCallback || serialReceiveCallback(data)) {
                    rxbuf[rx_head] = data;                          // Add data to buffer
                    rx_head = bptr;                                 // and update pointer
            #ifdef RTS_PORT
                if (!rts_state && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) >= RX_BUFFER_HWM) // Set RTS if at or above HWM
                    BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = rts_state = 1;
            #endif
                }
            }
            break;
    }
    */
}

#endif // __serial_h__
