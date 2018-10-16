/*

  serial.c - MSP432 low level functions for transmitting bytes via the serial port

  Part of Grbl

  Copyright (c) 2017-2018 Terje Io

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
#include "GRBL/grbl.h"

#define BUFCOUNT(head, tail, size) ((head >= tail) ? (head - tail) : (size - tail + head))

#ifndef RX_BUFFER_SIZE
  #define RX_BUFFER_SIZE 128
#endif
#ifndef TX_BUFFER_SIZE
  #define TX_BUFFER_SIZE 64
#endif

typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    bool overflow;
    bool rts_state;
    bool backup;
    char data[RX_BUFFER_SIZE];
} serial_buffer_t;

static serial_buffer_t rxbuffer = {
  .head = 0,
  .tail = 0,
  .backup = false,
  .overflow = false,
  .rts_state = false
};

static serial_buffer_t rxbackup;

static char txbuf[TX_BUFFER_SIZE];
static volatile uint16_t tx_head = 0, tx_tail = 0;

static bool (*serialReceiveCallback)(char) = 0;
static bool (*serialBlockingCallback)(void) = 0;

#ifdef SERIAL2_MOD
static char rx2buf[RX_BUFFER_SIZE];
static volatile uint16_t rx2_head = 0, rx2_tail = 0, rx2_overflow = 0;
#endif

#ifdef RTS_PORT
  static volatile uint8_t rts_state = 0;
#endif

#ifdef ENABLE_XONXOFF
  static volatile uint8_t flow_ctrl = XON_SENT; // Flow control state variable
#endif

static bool serialBlockingCallbackDummy (void)
{
    return true;
}

void setSerialBlockingCallback (bool (*fn)(void)) {
    serialBlockingCallback = fn == 0 ? serialBlockingCallbackDummy : fn;
}

void setSerialReceiveCallback (bool (*fn)(char)) {
    serialReceiveCallback = fn;
}

void serialInit (void)
{
    SERIAL_MODULE->CTLW0 = EUSCI_A_CTLW0_SWRST|EUSCI_A_CTLW0_SSEL__SMCLK;
    SERIAL_MODULE->BRW = 6;
    SERIAL_MODULE->MCTLW = (0x20 << 8) | (8 << 4) | 1;
    SERIAL_MODULE->IFG = ~EUSCI_A_IFG_RXIFG;
    SERIAL_MODULE->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;
    SERIAL_MODULE->IE = EUSCI_A_IE_RXIE;

	NVIC_EnableIRQ(SERIAL_MODULE_INT);
    NVIC_SetPriority(SERIAL_MODULE_INT, 0x40);

	SERIAL_PORT->SEL0 = SERIAL_RX|SERIAL_TX;    // set 2-UART pins as second function

#ifdef SERIAL2_MOD
    SERIAL2_MODULE->CTLW0 = EUSCI_A_CTLW0_SWRST|EUSCI_A_CTLW0_SSEL__SMCLK;
    SERIAL2_MODULE->BRW = 6;
    SERIAL2_MODULE->MCTLW = (0x20 << 8) | (8 << 4) | 1;
    SERIAL2_MODULE->IFG = ~EUSCI_A_IFG_RXIFG;
    SERIAL2_MODULE->CTLW0 &= ~EUSCI_A_CTLW0_SWRST;

    NVIC_EnableIRQ(SERIAL2_MODULE_INT);
    NVIC_SetPriority(SERIAL2_MODULE_INT, 0x40);

    SERIAL2_PORT->SEL0 = SERIAL_RX|SERIAL_TX;    // set 2-UART pins as second function
#endif

	setSerialBlockingCallback(0);

	__enable_interrupts();

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
  uint16_t tail = rxbuffer.tail, head = rxbuffer.head;
  return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of free characters in serial input buffer
//
uint16_t serialRxFree (void)
{
  unsigned int tail = rxbuffer.tail, head = rxbuffer.head;
  return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
void serialRxFlush (void)
{
    rxbuffer.head = rxbuffer.tail = 0;
#ifdef RTS_PORT
    BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = 0;
#endif
}

//
// Flushes and adds a CAN character to the serial input buffer
//
void serialRxCancel (void)
{
    rxbuffer.data[rxbuffer.head] = CAN;
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
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

    if((ok = !(SERIAL_MODULE->IE & EUSCI_A_IE_TXIE) && !(SERIAL_MODULE->STATW & EUSCI_A_STATW_BUSY)))
        SERIAL_MODULE->TXBUF = c;

    return ok;
}

//
// Writes a character to the serial output stream
//
bool serialPutC (const char c) {

    uint32_t next_head;

    if(tx_head != tx_tail || !serialPutCNonBlocking(c)) {   // Try to send character without buffering...

        next_head = (tx_head + 1) & (TX_BUFFER_SIZE - 1);   // .. if not, set and update head pointer

        while(tx_tail == next_head) {                       // While TX buffer full
            SERIAL_MODULE->IE |= EUSCI_A_IE_TXIE;               // Enable TX interrupts???
            if(!serialBlockingCallback())                   // check if blocking for space,
                return false;                               // exit if not (leaves TX buffer in an inconsistent state)
        }

        txbuf[tx_head] = c;                                 // Add data to buffer
        tx_head = next_head;                                // and update head pointer

        SERIAL_MODULE->IE |= EUSCI_A_IE_TXIE;               // Enable TX interrupts
    }

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
    serialWriteS(EOL);
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
    uint16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head)
        return -1; // no data available else EOF

    char data = rxbuffer.data[bptr++];     // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);  // and update pointer

#ifdef RTS_PORT
    if (rts_state && BUFCOUNT(rxbuffer.head, rxbuffer.tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM) // Clear RTS if below LWM
        BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = rts_state = 0;
#endif

    return (int16_t)data;
}

// "dummy" version of serialGetC
static int16_t serialGetNull (void)
{
    return -1;
}

bool serialSuspendInput (bool suspend)
{
    if(suspend)
        hal.serial_read = serialGetNull;
    else if(rxbuffer.backup)
        memcpy(&rxbuffer, &rxbackup, sizeof(serial_buffer_t));

    return rxbuffer.tail != rxbuffer.head;
}

//
void SERIAL_IRQHandler (void)
{
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
            data = (char)SERIAL_MODULE->RXBUF;                  // Read character received
            if(data == CMD_TOOL_ACK && !rxbuffer.backup) {

                memcpy(&rxbackup, &rxbuffer, sizeof(serial_buffer_t));
                rxbuffer.backup = true;
                rxbuffer.tail = rxbuffer.head;
                hal.serial_read = serialGetC; // restore normal input

            } else if(!serialReceiveCallback || serialReceiveCallback(data)) {

                bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer

                if(bptr == rxbuffer.tail)                           // If buffer full
                    rxbuffer.overflow = 1;                          // flag overflow,
                else {
                    rxbuffer.data[rxbuffer.head] = (char)data;      // else add data to buffer
                    rxbuffer.head = bptr;                           // and update pointer
                }
            }
        #ifdef RTS_PORT
            if (!rts_state && BUFCOUNT(rxbuffer.head, rxbuffer.tail, RX_BUFFER_SIZE) >= RX_BUFFER_HWM) // Set RTS if at or above HWM
                BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = rts_state = 1;
        #endif
            break;
    }
}

#ifdef SERIAL2_MOD

void serialSelect (bool mpg)
{
    if(mpg) {
        SERIAL_MODULE->IE = 0;
        SERIAL2_MODULE->IE = EUSCI_A_IE_RXIE;
    } else {
        SERIAL_MODULE->IE = EUSCI_A_IE_RXIE;
        SERIAL2_MODULE->IE = 0;
    }
}

//
// Returns number of free characters in serial input buffer
//
uint16_t serial2RxFree (void)
{
  unsigned int tail = rx2_tail, head = rx2_head;
  return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
void serial2RxFlush (void)
{
    rx2_head = rx2_tail = 0;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
void serial2RxCancel (void)
{
    rx2buf[rx2_head] = CAN;
    rx2_tail = rx2_head;
    rx2_head = (rx2_tail + 1) & (RX_BUFFER_SIZE - 1);

}

//
// serialGetC - returns -1 if no data available
//
int16_t serial2GetC (void)
{
    uint16_t bptr = rx2_tail;

    if(bptr == rx2_head)
        return -1; // no data available else EOF

    char data = rx2buf[bptr++];     // Get next character, increment tmp pointer
    rx2_tail = bptr & (RX_BUFFER_SIZE - 1);  // and update pointer

    return (int16_t)data;
}

void SERIAL2_IRQHandler (void)
{
    char data;
    uint16_t bptr;

    switch(SERIAL2_MODULE->IV) {

        case 0x02:
            data = (char)SERIAL2_MODULE->RXBUF;             // Read character received
            bptr = (rx2_head + 1) & (RX_BUFFER_SIZE - 1);            // Temp head position (to avoid volatile overhead)
            if(bptr == rx2_tail) {                                   // If buffer full
                rx2_overflow = 1;                                    // flag overflow
            } else {
                if(!serialReceiveCallback || serialReceiveCallback(data)) {
                    rx2buf[rx2_head] = data;                          // Add data to buffer
                    rx2_head = bptr;                                 // and update pointer
                }
            }
            break;
    }
}
#endif

#endif // __serial_h__
