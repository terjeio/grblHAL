/*

  serial.c - LPC17xx low level functions for transmitting bytes via the serial port

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

#ifndef __serial_h__
#define __serial_h__

#include <string.h>

#include "driver.h"
#include "serial.h"
#include "grbl/hal.h"

#include "chip.h"

static stream_rx_buffer_t rxbuffer = {0};
static stream_tx_buffer_t txbuffer = {0};

/* Pin muxing configuration */
static const PINMUX_GRP_T uart_pinmux[] = {
    {0, 2, IOCON_MODE_INACT | IOCON_FUNC1}, /* TXD0 */
    {0, 3, IOCON_MODE_INACT | IOCON_FUNC1}  /* RXD0 */
};

#ifdef RTS_PORT
  static volatile uint8_t rts_state = 0;
#endif

#ifdef ENABLE_XONXOFF
  static volatile uint8_t flow_ctrl = XON_SENT; // Flow control state variable
#endif

void serialInit (void)
{
    Chip_IOCON_SetPinMuxing(LPC_IOCON, uart_pinmux, sizeof(uart_pinmux) / sizeof(PINMUX_GRP_T));

    /* Setup UART for 115.2K8N1 */
    Chip_UART_Init(SERIAL_MODULE);
    Chip_UART_SetBaud(SERIAL_MODULE, 115200);
    Chip_UART_ConfigData(SERIAL_MODULE, (UART_LCR_WLEN8|UART_LCR_SBS_1BIT));
    Chip_UART_SetupFIFOS(SERIAL_MODULE, (UART_FCR_FIFO_EN|UART_FCR_TRG_LEV2));
    Chip_UART_TXEnable(SERIAL_MODULE);

    /* Reset and enable FIFOs, FIFO trigger level 3 (14 chars) */
    Chip_UART_SetupFIFOS(SERIAL_MODULE, (UART_FCR_FIFO_EN|UART_FCR_RX_RS|UART_FCR_TX_RS|UART_FCR_TRG_LEV3));

    /* Enable receive data and line status interrupt */
    Chip_UART_IntEnable(SERIAL_MODULE, UART_IER_RBRINT);
//  Chip_UART_IntEnable(SERIAL_MODULE, (UART_IER_RBRINT|UART_IER_RLSINT));

    /* preemption = 3, sub-priority = 1 */
    NVIC_SetPriority(SERIAL_MODULE_INT, 3);
    NVIC_EnableIRQ(SERIAL_MODULE_INT);

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
  uint16_t tail = txbuffer.tail;
  return BUFCOUNT(txbuffer.head, tail, TX_BUFFER_SIZE);
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
    SERIAL_MODULE->FCR |= UART_FCR_RX_RS; // Flush FIFO too

#ifdef RTS_PORT
    BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = 0;
#endif
}

//
// Flushes and adds a CAN character to the serial input buffer
//
void serialRxCancel (void)
{
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
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
    bool ok = false;

    if((ok = !(SERIAL_MODULE->IER & UART_IER_THREINT) && (Chip_UART_ReadLineStatus(SERIAL_MODULE) & UART_LSR_THRE)))
        SERIAL_MODULE->THR = c;

    return ok;
}

//
// Writes a character to the serial output stream
//
bool serialPutC (const char c)
{
    uint32_t next_head;

    if(txbuffer.head != txbuffer.tail || !serialPutCNonBlocking(c)) {   // Try to send character without buffering...

        next_head = (txbuffer.head + 1) & (TX_BUFFER_SIZE - 1);         // .. if not, set and update head pointer

        while(txbuffer.tail == next_head) {                             // While TX buffer full
            if(!hal.stream_blocking_callback())                         // check if blocking for space,
                return false;                                           // exit if not (leaves TX buffer in an inconsistent state)
        }

        txbuffer.data[txbuffer.head] = c;                               // Add data to buffer
        txbuffer.head = next_head;                                      // and update head pointer

        Chip_UART_IntEnable(SERIAL_MODULE, UART_IER_THREINT);           // Enable TX interrupts
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
    uint16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head)
        return -1; // no data available else EOF

    char data = rxbuffer.data[bptr++];              // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);    // and update pointer

#ifdef RTS_PORT
    if (rts_state && BUFCOUNT(rx_head, rx_tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM) // Clear RTS if below LWM
        BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = rts_state = 0;
#endif

    return (int16_t)data;
}

bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuffer, suspend);
}

//
void SERIAL_IRQHandler (void)
{
    uint32_t bptr = SERIAL_MODULE->IIR;

    switch(bptr & 0x07) {
/*
        case UART_IIR_INTID_RLS:
            bptr = SERIAL_MODULE->LSR;
            break;
*/
        case UART_IIR_INTID_THRE:
            bptr = txbuffer.tail;                                       // Temp tail position (to avoid volatile overhead)
            SERIAL_MODULE->SCR = UART_TX_FIFO_SIZE;                     // Use UART scratch pad register as
            while((--SERIAL_MODULE->SCR) && bptr != txbuffer.head) {    // counter variable for filling the transmit FIFO
                SERIAL_MODULE->THR = txbuffer.data[bptr++];             // Send a byte from the buffer
                bptr &= (TX_BUFFER_SIZE - 1);                           // and
                txbuffer.tail = bptr;                                   // update tail position
            }
            if (bptr == txbuffer.head)                                  // Turn off TX interrupt
                Chip_UART_IntDisable(SERIAL_MODULE, UART_IER_THREINT);  // when buffer empty
            break;

        case UART_IIR_INTID_RDA:
        case UART_IIR_INTID_CTI:;
            while(SERIAL_MODULE->LSR & UART_LSR_RDR) {
                uint32_t data = SERIAL_MODULE->RBR;
                if(data == CMD_TOOL_ACK && !rxbuffer.backup) {
                    stream_rx_backup(&rxbuffer);
                    hal.stream.read = serialGetC; // restore normal input
                } else if(!hal.stream.enqueue_realtime_command(data)) { // Read character received

                    bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Temp head position (to avoid volatile overhead)

                    if(bptr == rxbuffer.tail)                           // If buffer full
                        rxbuffer.overflow = 1;                          // flag overflow
                    else {
                        rxbuffer.data[rxbuffer.head] = data;            // Add data to buffer
                        rxbuffer.head = bptr;                           // and update pointer
                      #ifdef RTS_PORT
                        if (!rts_state && BUFCOUNT(rxbuffer.head, rxbuffer.tail, RX_BUFFER_SIZE) >= RX_BUFFER_HWM) // Set RTS if at or above HWM
                            BITBAND_PERI(RTS_PORT->OUT, RTS_PIN) = rts_state = 1;
                      #endif
                    }
                }
            }
            break;
    }
}

#endif // __serial_h__
