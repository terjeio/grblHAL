//
// serial.c - serial (UART) port library for MSP430G2553 etc.
//
// v1.5 / 2018-07-01 / Io Engineering / Terje
//
//

/*

Copyright (c) 2015-2018, Terje Io
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

#define BUFCOUNT(head, tail, size) ((head >= tail) ? (head - tail) : (size - tail + head))

char txbuf[TX_BUFFER_SIZE];
char rxbuf[RX_BUFFER_SIZE];

const char eol[] = "\r\n";
static volatile uint16_t tx_head = 0, tx_tail = 0, rx_head = 0, rx_tail = 0, rx_overflow = 0;

#ifdef XONXOFF
    static volatile unsigned int rx_off = XONOK;
#endif

inline static void setUARTBR (int prescaler)
{
    SERIAL_BR0 = prescaler & 0xFF;  // LSB
    SERIAL_BR1 = prescaler >> 8;    // MSB
}

void serialInit (void)
{
    SERIAL_SEL  |= RXD|TXD;         // P1.1 = RXD, P1.2=TXD
    SERIAL_SEL2 |= RXD|TXD;         // P1.1 = RXD, P1.2=TXD

    SERIAL_CTL1 = UCSWRST;
    SERIAL_CTL1 |= UCSSEL_2;        // Use SMCLK

    setUARTBR(26<<2);               // Set baudrate to 9600
    SERIAL_MCTL = UCBRF0|UCOS16;    // with oversampling

    SERIAL_CTL0 = 0;
    SERIAL_CTL1 &= ~UCSWRST;        // Initialize USCI state machine
    SERIAL_IE |= SERIAL_RXIE;       // Enable USCI_A0 RX interrupt

#ifndef XONXOFF
    RTS_PORT_DIR |= RTS_BIT;        // Enable RTS pin as output
    RTS_PORT_OUT &= ~RTS_BIT;       // and drive it low
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

void serialRxFlush (void)
{
    rx_head = 0;
    rx_tail = 0;
    RTS_PORT_OUT &= ~RTS_BIT;
}

void serialPutC (const char c)
{
    uint16_t next_head = tx_head;

    if((SERIAL_IFG & SERIAL_TXIE) && next_head == tx_tail)  // If no data pending in buffers
        SERIAL_TXD = c;                                     // send character immediately

    else {

        if (++next_head == TX_BUFFER_SIZE)  // Wrap buffer index
            next_head = 0;                  // if at end

        while(tx_tail == next_head);        // Buffer full, block until free room

        txbuf[tx_head] = c;                 // Enter data into buffer
        tx_head = next_head;                // and increment pointer to next entry

        SERIAL_IE |= SERIAL_TXIE;           // Enable transmit interrupts
    }
}

char serialRead (void)
{
    uint16_t c, tail;

    tail = rx_tail;
    c = tail != rx_head ? rxbuf[tail++] : EOF;      // Get next character and increment pointer if data available else EOF
    rx_tail = tail == RX_BUFFER_SIZE ? 0 : tail;    // then update pointer

#ifdef XONXOFF
    if (rx_off == XOFFOK && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM) {
        rx_off = XON;                               // Queue XON at front
        UC0IE |= SERIAL_TXIE;                       // and enable UART TX interrupt
    }
#else

    if ((RTS_PORT_IN & RTS_BIT) && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM)  // Clear RTS if
        RTS_PORT_OUT &= ~RTS_BIT;                                                               // buffer count is below low water mark

#endif

    return c;
}

void serialWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

void serialWriteLn (const char *data)
{
    serialWriteS(data);
    serialWriteS(eol);
}

void serialWrite (const char *data, uint16_t length)
{
    char *ptr = (char *)data;

    while(length--)
        serialPutC(*ptr++);
}

#pragma vector=USCIAB0RX_VECTOR
__interrupt void USCI0RX_ISR(void)
{
    uint16_t next_head = rx_head + 1;       // Get and increment buffer pointer

    if (next_head == RX_BUFFER_SIZE)        // If at end
        next_head = 0;                      // wrap pointer around

    if(rx_tail == next_head) {              // If buffer full
        rx_overflow = 1;                    // flag overlow
        next_head = SERIAL_RXD;             // and do dummy read to clear interrupt
    } else {
        rxbuf[rx_head] = SERIAL_RXD;        // Add data to buffer
        rx_head = next_head;                // and update pointer
#ifdef XONXOFF
        if (rx_off == XONOK && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) > RX_BUFFER_HWM) {
            rx_off = XOFF;                  // Queue XOFF at front
            SERIAL_IE |= SERIAL_TXIE;       // and enable UART TX interrupt
        }
#else
        if (!(RTS_PORT_IN & RTS_BIT) && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) >= RX_BUFFER_HWM)
            RTS_PORT_OUT |= RTS_BIT;
#endif
    }
}

#pragma vector=USCIAB0TX_VECTOR
__interrupt void USCI0TX_ISR(void)
{
    if((SERIAL_IFG & SERIAL_TXIE) && (SERIAL_IE & SERIAL_TXIE)) { // UART

        uint16_t tail = tx_tail;                // Get buffer pointer

    #ifdef XONXOFF
        if(rx_off == XON || rx_off == XOFF) {   // If we have XOFF/XON to send
            SERIAL_TXD = rx_off;                // send it
            rx_off |= 0x80;                     // and flag it sent
        } else {
    #endif
            SERIAL_TXD = txbuf[tail++];         // Send next character and increment pointer

            if(tail == TX_BUFFER_SIZE)          // If at end
                tail = 0;                       // wrap pointer around

            tx_tail = tail;                     // Update global pointer
    #ifdef XONXOFF
        }
    #endif
        if(tail == tx_head)                     // If buffer empty then
            SERIAL_IE &= ~SERIAL_TXIE;          // disable UART TX interrupt
    }
}
