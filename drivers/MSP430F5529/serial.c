//
// serial.c - serial (UART) port library
//
// v1.0 / 2019-06-03 / Io Engineering / Terje
//
//

/*

Copyright (c) 2015-2019, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

· Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

· Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

· Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include <msp430.h>
#include <stdbool.h>
#include <stdint.h>

#include "serial.h"
#include "grbl/hal.h"

#define BUFCOUNT(head, tail, size) ((head >= tail) ? (head - tail) : (size - tail + head))

static char txbuf[TX_BUFFER_SIZE];
static char rxbuf[RX_BUFFER_SIZE];

const char eol[] = "\r\n";
static volatile uint16_t tx_head = 0, tx_tail = 0, rx_head = 0, rx_tail = 0, rx_overflow = 0;

#ifdef XONXOFF
    static volatile unsigned int rx_off = XONOK;
#endif

inline void setUCA1BR (uint16_t prescaler)
{
    UCA1BR0 = prescaler & 0xFF; // LSB
    UCA1BR1 = prescaler >> 8;   // MSB
}

void serialInit (void)
{
    UCA1CTL1 = UCSWRST;
    UCA1CTL1 |= UCSSEL_2;   // Use SMCLK
    setUCA1BR(217);         // Set baudrate to 115200 @ 25MHz SMCLK
    UCA1MCTL = 0;           // Modulation UCBRSx=0, UCBRFx=0
    UCA1CTL0 = 0;           // 8 bit, 1 stop bit, no parity
    SERIAL_SEL |= SERIAL_RXD|SERIAL_TXD;
    UCA1CTL1 &= ~UCSWRST;   // Initialize USCI state machine
    UCA1IE |= UCRXIE;       // Enable USCI_A0 RX interrupt

#ifndef XONXOFF
    SERIAL_RTS_PORT_DIR |= SERIAL_RTS_BIT;  // Enable RTS pin
    SERIAL_RTS_PORT_OUT &= ~SERIAL_RTS_BIT; // and drive it low
#endif

}

uint16_t serialTxCount (void)
{
  uint16_t tail = tx_tail;
  return BUFCOUNT(tx_head, tail, TX_BUFFER_SIZE);
}

uint16_t serialRxCount (void)
{
  uint16_t tail = rx_tail, head = rx_head;
  return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t serialRxFree (void)
{
  unsigned int tail = rx_tail, head = rx_head;
  return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

void serialRxFlush (void)
{
    rx_head = rx_tail = 0;
    SERIAL_RTS_PORT_OUT &= ~SERIAL_RTS_BIT;
}

void serialRxCancel (void)
{
    rxbuf[rx_head] = ASCII_CAN;
    rx_tail = rx_head;
    rx_head = (rx_tail + 1) & (RX_BUFFER_SIZE - 1);
    SERIAL_RTS_PORT_OUT &= ~SERIAL_RTS_BIT;;
}

bool serialPutC(const char data)
{
    unsigned int next_head = tx_head;

    if((UCA1IFG & UCTXIFG) && next_head == tx_tail)
        UCA1TXBUF = data;

    else {

        if (++next_head == TX_BUFFER_SIZE)
            next_head = 0;

        while(tx_tail == next_head);    // Buffer full, block until free room

        txbuf[tx_head] = data;          // Enter data into buffer
        tx_head = next_head;            // and increment pointer to next entry

        UCA1IE |= UCTXIE;               // Enable transmit interrupts
    }
    return true;
}

int16_t serialGetC (void) {

    if(rx_tail == rx_head)
        return -1;

    uint16_t tail;

//  _DINT();                                                // Disable interrupts (to avoid contention)

    tail = rx_tail;
    char data = rxbuf[tail++];                  // Get next character and increment pointer if data available else EOF
    rx_tail = tail == RX_BUFFER_SIZE ? 0 : tail;            // then update pointer

#ifdef XONXOFF
    if (rx_off == XOFFOK && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM) {
        rx_off = XON;                                       // Queue XON at front
        UC0IE |= UCA1TXIE;                                  // and enable UART TX interrupt
    }
#else

    if ((SERIAL_RTS_PORT_IN & SERIAL_RTS_BIT) && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM)    // Clear RTS if
        SERIAL_RTS_PORT_OUT &= ~SERIAL_RTS_BIT;                                     // buffer count is below low water mark

//    _EINT();                                              // Reenable interrupts

#endif

    return (int16_t)data;
}

void serialWriteS(const char *data) {

    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);

}

void serialWriteLn(const char *data) {
    serialWriteS(data);
    serialWriteS(eol);
}

void serialWrite(const char *data, uint16_t length) {

    char *ptr = (char *)data;

    while(length--)
        serialPutC(*ptr++);

}

#pragma vector=USCI_A1_VECTOR
__interrupt void USCI1RX_ISR(void)
{
    uint16_t iv = UCA1IV;

    if(iv == 0x02) {

        uint16_t next_head = rx_head + 1;       // Get and increment buffer pointer

        if (next_head == RX_BUFFER_SIZE)        // If at end
            next_head = 0;                      // wrap pointer around

        if(rx_tail == next_head) {              // If buffer full
            rx_overflow = 1;                    // flag overlow
            next_head = UCA1RXBUF;              // and do dummy read to clear interrupt
        } else {
            char data = UCA1RXBUF;
            if(!hal.stream.enqueue_realtime_command(data)) {
                rxbuf[rx_head] = data;                  // Add data to buffer
                rx_head = next_head;                    // and update pointer
            }
    #ifdef XONXOFF
            if (rx_off == XONOK && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) > RX_BUFFER_HWM) {
                rx_off = XOFF;                  // Queue XOFF at front
                UC0IE |= UCA1TXIE;              // and enable UART TX interrupt
            }
    #else
            if (!(SERIAL_RTS_PORT_IN & SERIAL_RTS_BIT) && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) >= RX_BUFFER_HWM)
                SERIAL_RTS_PORT_OUT &= ~SERIAL_RTS_BIT;
    #endif
        }
    }

    if(iv == 0x04) {

        uint16_t tail = tx_tail;               // Get buffer pointer

    #ifdef XONXOFF
        if(rx_off == XON || rx_off == XOFF) {   // If we have XOFF/XON to send
            UCA1TXBUF = rx_off;                 // send it
            rx_off |= 0x80;                     // and flag it sent
        } else {
    #endif
            UCA1TXBUF = txbuf[tail++];          // Send next character and increment pointer

            if(tail == TX_BUFFER_SIZE)          // If at end
                tail = 0;                       // wrap pointer around

            tx_tail = tail;                     // Update global pointer
    #ifdef XONXOFF
        }
    #endif
        if(tail == tx_head)                     // If buffer empty then
           UCA1IE &= ~UCTXIE;                   // disable UART TX interrupt
    }

}
