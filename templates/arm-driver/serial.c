/*
  serial.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Template driver code for ARM processors

  Part of GrblHAL

  By Terje Io, public domain

*/

#include "grbl/grbl.h"

static stream_tx_buffer_t txbuffer = {0};
static stream_rx_buffer_t rxbuffer = {0}, rxbackup;

void serialInit (void)
{
    // Configure serial peripheral here including RX interrupt enable
}

//
// serialGetC - returns -1 if no data available
//
int16_t serialGetC (void)
{
    int16_t data;
    uint_fast16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head)
        return -1; // no data available else EOF

    data = rxbuffer.data[bptr++];                   // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);    // and update pointer

    return data;
}

inline uint16_t serialRxCount (void)
{
    uint_fast16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t serialRxFree (void)
{
    return (RX_BUFFER_SIZE - 1) - serialRxCount();
}

void serialRxFlush (void)
{
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.overflow = false;
}

void serialRxCancel (void)
{
    serialRxFlush();
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
}

bool serialPutC (const char c)
{
    uint_fast16_t next_head;

    // NOTE: If buffer and transmit register are empty buffering may be bypassed.
    //       See actual drivers for examples.

    next_head = (txbuffer.head + 1) & (TX_BUFFER_SIZE - 1);     // Get and update head pointer

    while(txbuffer.tail == next_head) {                         // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    txbuffer.data[txbuffer.head] = c;                           // Add data to buffer
    txbuffer.head = next_head;                                  // and update head pointer

    UART_TX_IRQ_ENABLE();                                       // Enable TX interrupts

    return true;
}

void serialWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

// "dummy" version of serialGetC
static int16_t serialGetNull (void)
{
    return -1;
}

bool serialSuspendInput (bool suspend)
{
    if(suspend)
        hal.stream.read = serialGetNull;
    else if(rxbuffer.backup)
        memcpy(&rxbuffer, &rxbackup, sizeof(stream_rx_buffer_t));

    return rxbuffer.tail != rxbuffer.head;
}

uint16_t serialTxCount(void) {

    uint_fast16_t head = txbuffer.head, tail = txbuffer.tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE);
}

static void uart_interrupt_handler (void)
{
    uint_fast16_t bptr;
    int32_t data;
    uint32_t iflags;

//    iflags = UART_GET_IRQSSTATE(); // Get UART interrupt flags

    if(iflags & UART_IRQ_TX) {

        bptr = txbuffer.tail;

        if(txbuffer.head != bptr) {

            // UART_TX_WRITE(UARTCH, txbuffer.data[bptr++]);                 // Put character in TXT register
            bptr &= (TX_BUFFER_SIZE - 1);                               // and update tmp tail pointer

            txbuffer.tail = bptr;                                       //  Update tail pinter

            if(bptr == txbuffer.head)                                   // Disable TX interrups
               // UART_TX_IRQ_DISABLE();                    // when TX buffer empty
        }
    }

    if(iflags & (UART_IRQ_R)) {

        bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer

        if(bptr == rxbuffer.tail) {                         // If buffer full
            rxbuffer.overflow = 1;                          // flag overflow
            // UART_RX_IRQ_DISABLE(); Clear RX interrupt, may be done by a dummy read of the RX register
        } else {
            // data = UART_GET(); Read received character to data varable, clear RX interrupt if not done automatically by read.
            if(data == CMD_TOOL_ACK && !rxbuffer.backup) {
                memcpy(&rxbackup, &rxbuffer, sizeof(stream_rx_buffer_t));
                rxbuffer.backup = true;
                rxbuffer.tail = rxbuffer.head;
                hal.stream.read = serialGetC; // restore normal input

            } else if(!hal.stream.enqueue_realtime_command((char)data)) {
                rxbuffer.data[rxbuffer.head] = (char)data;  // Add data to buffer
                rxbuffer.head = bptr;                       // and update pointer
            }
        }
    }
}
