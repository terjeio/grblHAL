/*

  serial.c - Atmel SAM3X8E low level functions for transmitting bytes via the serial port

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/

#include "driver.h"
#include "serial.h"

static stream_tx_buffer_t txbuffer = {0};
static stream_rx_buffer_t rxbuffer = {0};

static void SERIAL_IRQHandler (void);

void serialInit (void)
{
    pmc_enable_periph_clk(SERIAL_ID);
    pmc_enable_periph_clk(ID_PIOA);
/*
    SERIAL_PORT->PIO_PDR  = SERIAL_RX|SERIAL_TX;
    SERIAL_PORT->PIO_OER  = SERIAL_TX;
    SERIAL_PORT->PIO_ABSR = SERIAL_RX|SERIAL_TX;
*/
#if SERIAL_DEVICE == -1
    SERIAL_PERIPH->UART_PTCR = UART_PTCR_RXTDIS | UART_PTCR_TXTDIS;
    SERIAL_PERIPH->UART_CR = UART_CR_RSTRX|UART_CR_RSTTX|UART_CR_RXDIS|UART_CR_TXDIS;

    SERIAL_PERIPH->UART_MR = UART_MR_PAR_NO;
    SERIAL_PERIPH->UART_BRGR = (SystemCoreClock / 115200) >> 4;
    SERIAL_PERIPH->UART_IER = UART_IER_RXRDY|UART_IER_OVRE|UART_IER_FRAME;

    SERIAL_PERIPH->UART_CR = UART_CR_RXEN|UART_CR_TXEN;
#else
    SERIAL_PERIPH->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS;
    SERIAL_PERIPH->US_CR = US_CR_RSTRX|US_CR_RSTTX|US_CR_RXDIS|US_CR_TXDIS;

    SERIAL_PERIPH->US_MR = US_MR_CHRL_8_BIT|US_MR_PAR_NO; // |US_MR_NBSTOP_2
    SERIAL_PERIPH->US_BRGR = (SystemCoreClock / 115200) >> 4;
    SERIAL_PERIPH->US_IER = US_IER_RXRDY|US_IER_OVRE|US_IER_FRAME;

    SERIAL_PERIPH->US_CR = US_CR_RXEN|US_CR_TXEN;
#endif

    IRQRegister(SERIAL_IRQ, SERIAL_IRQHandler);

    NVIC_EnableIRQ(SERIAL_IRQ);
    NVIC_SetPriority(SERIAL_IRQ, 1);
}

//
// Returns number of characters in serial output buffer
//
uint16_t serialTxCount (void)
{
    uint16_t tail = txbuffer.tail;

#if SERIAL_DEVICE == -1
    return BUFCOUNT(txbuffer.head, tail, TX_BUFFER_SIZE) + (SERIAL_PERIPH->UART_SR & UART_SR_TXEMPTY) ? 0 : 1;
#else
    return BUFCOUNT(txbuffer.head, tail, TX_BUFFER_SIZE) + (SERIAL_PERIPH->US_CSR & US_CSR_TXEMPTY) ? 0 : 1;
#endif
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

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
void serialRxFlush (void)
{
    rxbuffer.head = rxbuffer.tail = 0;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
void serialRxCancel (void)
{
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
}

//
// Attempt to send a character bypassing buffering
//
static inline bool serialPutCNonBlocking (const char c)
{
    bool ok = false;

#if SERIAL_DEVICE == -1
    if((ok = (SERIAL_PERIPH->UART_IMR & US_IMR_TXRDY) == 0 && SERIAL_PERIPH->UART_SR & UART_SR_TXEMPTY))
        SERIAL_PERIPH->UART_THR = c;
#else
    if((ok = (SERIAL_PERIPH->US_IMR & US_IMR_TXRDY) == 0 && SERIAL_PERIPH->US_CSR & US_CSR_TXEMPTY))
        SERIAL_PERIPH->US_THR = c;
#endif
    return ok;
}

//
// Writes a character to the serial output stream
//
bool serialPutC (const char c) {

    uint32_t next_head;

    if(txbuffer.head != txbuffer.tail || !serialPutCNonBlocking(c)) {   // Try to send character without buffering...

        next_head = (txbuffer.head + 1) & (TX_BUFFER_SIZE - 1);         // .. if not, set and update head pointer

        while(txbuffer.tail == next_head) {                             // While TX buffer full
      //      SERIAL_MODULE->IE |= EUSCI_A_IE_TXIE;                     // Enable TX interrupts???
            if(!hal.stream_blocking_callback())                         // check if blocking for space,
                return false;                                           // exit if not (leaves TX buffer in an inconsistent state)
        }

        txbuffer.data[txbuffer.head] = c;                               // Add data to buffer
        txbuffer.head = next_head;                                      // and update head pointer
#if SERIAL_DEVICE == -1
        SERIAL_PERIPH->UART_IER = UART_IER_TXRDY;                       // Enable TX interrupts
#else
        SERIAL_PERIPH->US_IER = US_IER_TXRDY;                           // Enable TX interrupts
#endif
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
void serialWrite (const char *s, uint16_t length)
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

    return (int16_t)data;
}

bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuffer, suspend);
}

//
static void SERIAL_IRQHandler (void)
{
    uint16_t bptr;

//uint8_t ifg = SERIAL_PERIPH->USART.INTFLAG.reg;
/*
    if(SERIAL_PERIPH->USART.STATUS.bit.FERR) {
        data = SERIAL_PERIPH->USART.DATA.bit.DATA;
        SERIAL_PERIPH->USART.STATUS.bit.FERR = 1;
        SERIAL_PERIPH->USART.INTFLAG.reg = ifg;
    }
*/
#if SERIAL_DEVICE == -1
    if(SERIAL_PERIPH->UART_SR & UART_SR_RXRDY) {
        char data = (char)SERIAL_PERIPH->UART_RHR;
#else
    if(SERIAL_PERIPH->US_CSR & US_CSR_RXRDY) {
        char data = (char)SERIAL_PERIPH->US_RHR;
#endif
        if(data == CMD_TOOL_ACK && !rxbuffer.backup) {
            stream_rx_backup(&rxbuffer);
            hal.stream.read = serialGetC; // restore normal input
        } else if(!hal.stream.enqueue_realtime_command(data)) {

            bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer

            if(bptr == rxbuffer.tail)                           // If buffer full
                rxbuffer.overflow = 1;                          // flag overflow,
            else {
                rxbuffer.data[rxbuffer.head] = data;            // else add data to buffer
                rxbuffer.head = bptr;                           // and update pointer
            }
        }           
    }
#if SERIAL_DEVICE == -1
    if(SERIAL_PERIPH->UART_SR & UART_SR_TXRDY) {
#else
    if(SERIAL_PERIPH->US_CSR & US_CSR_TXRDY) {
#endif
        bptr = txbuffer.tail;                                           // Temp tail position (to avoid volatile overhead)
        if(txbuffer.tail != txbuffer.head) {
#if SERIAL_DEVICE == -1
            SERIAL_PERIPH->UART_THR = (uint32_t)txbuffer.data[bptr++];  // Send a byte from the buffer
#else
            SERIAL_PERIPH->US_THR = (uint32_t)txbuffer.data[bptr++];    // Send a byte from the buffer
#endif
            bptr &= (TX_BUFFER_SIZE - 1);                               // and update
            txbuffer.tail = bptr;                                       // tail position
        }
        if (bptr == txbuffer.head)                                      // Turn off TX interrupt
#if SERIAL_DEVICE == -1
            SERIAL_PERIPH->UART_IDR = UART_IER_TXRDY;                   // when buffer empty
#else
            SERIAL_PERIPH->US_IDR = US_IER_TXRDY;                       // when buffer empty
#endif
    }
}

#ifdef SERIAL2_DEVICE

#if SERIAL2_DEVICE == -1
#error "Not supported!"
#endif

static stream_tx_buffer_t tx2buffer = {0};
static stream_rx_buffer_t rx2buffer = {0};

static void SERIAL2_IRQHandler (void);

void serial2Init (uint32_t baud_rate)
{
    pmc_enable_periph_clk(SERIAL2_ID);
    pmc_enable_periph_clk(ID_PIOA);
/*
    SERIAL_PORT->PIO_PDR  = SERIAL_RX|SERIAL_TX;
    SERIAL_PORT->PIO_OER  = SERIAL_TX;
    SERIAL_PORT->PIO_ABSR = SERIAL_RX|SERIAL_TX;
*/

    SERIAL2_PERIPH->US_PTCR = US_PTCR_RXTDIS | US_PTCR_TXTDIS;
    SERIAL2_PERIPH->US_CR = US_CR_RSTRX|US_CR_RSTTX|US_CR_RXDIS|US_CR_TXDIS;

    SERIAL2_PERIPH->US_MR = US_MR_CHRL_8_BIT|US_MR_PAR_NO; // |US_MR_NBSTOP_2
    SERIAL2_PERIPH->US_BRGR = (SystemCoreClock / baud_rate) >> 4;
    SERIAL2_PERIPH->US_IER = US_IER_RXRDY|US_IER_OVRE|US_IER_FRAME;

    SERIAL2_PERIPH->US_CR = US_CR_RXEN|US_CR_TXEN;

    IRQRegister(SERIAL2_IRQ, SERIAL2_IRQHandler);

    NVIC_EnableIRQ(SERIAL2_IRQ);
    NVIC_SetPriority(SERIAL2_IRQ, 1);
}

//
// Returns number of characters in serial output buffer
//
uint16_t serial2TxCount (void)
{
    uint16_t tail = tx2buffer.tail;

    return BUFCOUNT(tx2buffer.head, tail, TX_BUFFER_SIZE) + (SERIAL2_PERIPH->US_CSR & US_CSR_TXEMPTY ? 0 : 1);
}

//
// Returns number of characters in serial input buffer
//
uint16_t serial2RxCount (void)
{
    uint16_t tail = rx2buffer.tail, head = rx2buffer.head;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of free characters in serial input buffer
//
uint16_t serial2RxFree (void)
{
    unsigned int tail = rx2buffer.tail, head = rx2buffer.head;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
void serial2RxFlush (void)
{
    rx2buffer.tail = rx2buffer.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
void serial2RxCancel (void)
{
    rx2buffer.data[rx2buffer.head] = ASCII_CAN;
    rx2buffer.tail = rx2buffer.head;
    rx2buffer.head = (rx2buffer.tail + 1) & (RX_BUFFER_SIZE - 1);
}

//
// Flushes the serial output buffer
//
void serial2TxFlush (void)
{
    tx2buffer.tail = tx2buffer.head;
}


//
// Attempt to send a character bypassing buffering
//
static inline bool serial2PutCNonBlocking (const char c)
{
    bool ok = false;

    if((ok = (SERIAL2_PERIPH->US_IMR & US_IMR_TXRDY) == 0 && SERIAL2_PERIPH->US_CSR & US_CSR_TXEMPTY))
        SERIAL2_PERIPH->US_THR = c;

    return ok;
}

//
// Writes a character to the serial output stream
//
bool serial2PutC (const char c) {

    uint32_t next_head;

    if(tx2buffer.head != tx2buffer.tail || !serial2PutCNonBlocking(c)) {    // Try to send character without buffering...

        next_head = (tx2buffer.head + 1) & (TX_BUFFER_SIZE - 1);            // .. if not, set and update head pointer

#if MODBUS_ENABLE
        while(tx2buffer.tail == next_head);                                 // Block while TX buffer full
#else
        while(tx2buffer.tail == next_head) {                                // While TX buffer full
      //      SERIAL2_MODULE->IE |= EUSCI_A_IE_TXIE;                        // Enable TX interrupts???
            if(!hal.stream_blocking_callback())                             // check if blocking for space,
                return false;                                               // exit if not (leaves TX buffer in an inconsistent state)
        }
#endif
        tx2buffer.data[tx2buffer.head] = c;                                 // Add data to buffer
        tx2buffer.head = next_head;                                         // and update head pointer

        SERIAL2_PERIPH->US_IER = US_IER_TXRDY;                              // Enable TX interrupts
    }

    return true;
}

//
// Writes a null terminated string to the serial output stream, blocks if buffer full
//
void serial2WriteS (const char *s)
{
    char c, *ptr = (char *)s;

    while((c = *ptr++) != '\0')
        serial2PutC(c);
}

//
// Writes a null terminated string to the serial output stream followed by EOL, blocks if buffer full
//
void serial2WriteLn (const char *s)
{
    serial2WriteS(s);
    serial2WriteS(ASCII_EOL);
}

//
// Writes a number of characters from string to the serial output stream followed by EOL, blocks if buffer full
//
void serial2Write (const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serial2PutC(*ptr++);
}

//
// serialGetC - returns -1 if no data available
//
int16_t serial2GetC (void)
{
    uint16_t bptr = rx2buffer.tail;

    if(bptr == rx2buffer.head)
        return -1; // no data available else EOF

    char data = rx2buffer.data[bptr++];              // Get next character, increment tmp pointer
    rx2buffer.tail = bptr & (RX_BUFFER_SIZE - 1);    // and update pointer

    return (int16_t)data;
}

//
static void SERIAL2_IRQHandler (void)
{
    uint16_t bptr;

//uint8_t ifg = SERIAL_PERIPH->USART.INTFLAG.reg;
/*
    if(SERIAL_PERIPH->USART.STATUS.bit.FERR) {
        data = SERIAL_PERIPH->USART.DATA.bit.DATA;
        SERIAL_PERIPH->USART.STATUS.bit.FERR = 1;
        SERIAL_PERIPH->USART.INTFLAG.reg = ifg;
    }
*/

    if(SERIAL2_PERIPH->US_CSR & US_CSR_RXRDY) {

        char data = (char)SERIAL2_PERIPH->US_RHR;

#if MODBUS_ENABLE
        bptr = (rx2buffer.head + 1) & (RX_BUFFER_SIZE - 1); // Get next head pointer

        if(bptr == rx2buffer.tail)                          // If buffer full
            rx2buffer.overflow = 1;                         // flag overflow,
        else {
            rx2buffer.data[rx2buffer.head] = data;          // else add data to buffer
            rx2buffer.head = bptr;                          // and update pointer
        }
#else
        if(!hal.stream.enqueue_realtime_command(data)) {

            bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer

            if(bptr == rxbuffer.tail)                           // If buffer full
                rxbuffer.overflow = 1;                          // flag overflow,
            else {
                rxbuffer.data[rxbuffer.head] = data;            // else add data to buffer
                rxbuffer.head = bptr;                           // and update pointer
            }
        }
#endif
    }

    if(SERIAL2_PERIPH->US_CSR & US_CSR_TXRDY) {
        bptr = tx2buffer.tail;                                          // Temp tail position (to avoid volatile overhead)
        if(tx2buffer.tail != tx2buffer.head) {
            SERIAL2_PERIPH->US_THR = (uint32_t)tx2buffer.data[bptr++];  // Send a byte from the buffer
            bptr &= (TX_BUFFER_SIZE - 1);                               // and update
            tx2buffer.tail = bptr;                                      // tail position
        }
        if (bptr == tx2buffer.head)                                     // Turn off TX interrupt
            SERIAL2_PERIPH->US_IDR = US_IER_TXRDY;                      // when buffer empty

    }
}

#endif // SERIAL2_DEVICE
