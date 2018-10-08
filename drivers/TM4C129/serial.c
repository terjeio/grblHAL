//
// serial.c - (UART) port library for Tiva C (TM4C1294MCPDT)
//
// v1.01 / 2018-09-21 / Io Engineering / Terje
//

/*

Copyright (c) 2018, Terje Io
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

#include "tiva.h"
#include "serial.h"
#include "GRBL/grbl.h"

typedef struct {
    volatile uint16_t head;
    volatile uint16_t tail;
    bool overflow;
    bool rts_state;
    bool backup;
    char data[RX_BUFFER_SIZE];
} serial_buffer_t;

#define BUFCOUNT(head, tail, size) ((head >= tail) ? (head - tail) : (size - tail + head))

#if defined(RX_BUFFER_SIZE) || defined(TX_BUFFER_SIZE)

 static void uart_interrupt_handler (void);

 #ifdef RX_BUFFER_SIZE
//  static char rxbuf[RX_BUFFER_SIZE];
//  static volatile uint16_t rx_head = 0, rx_tail = 0, rx_overflow = 0, rts_state = 0;
  static serial_buffer_t rxbuffer = {
    .head = 0,
    .tail = 0,
    .backup = false,
    .overflow = false,
    .rts_state = false
  };
  static serial_buffer_t rxbackup;
  static bool (*serialReceiveCallback)(char) = 0;
 #endif

 #ifdef TX_BUFFER_SIZE
  static char txbuf[TX_BUFFER_SIZE];
  static volatile uint16_t tx_head = 0, tx_tail = 0;
 #endif

 #ifdef SERIAL2_MOD
   static char rx2buf[RX_BUFFER_SIZE];
   static volatile uint16_t rx2_head = 0, rx2_tail = 0, rx2_overflow = 0;

  static void uart2_interrupt_handler (void);
 #endif

#endif

static bool (*serialBlockingCallback)(void) = 0;

void serialInit (void)
{
    SysCtlPeripheralEnable(SERIAL1_PERIPH);
    SysCtlPeripheralEnable(SERIAL1_SYSCTL);
    SysCtlDelay(3);

    GPIOPinConfigure(SERIAL1_RX); // JP4.1
    GPIOPinConfigure(SERIAL1_TX);
    GPIOPinTypeUART(SERIAL1_PORT, SERIAL1_PINS);

    UARTClockSourceSet(SERIAL1_BASE, UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(SERIAL1_BASE, 16000000, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTFIFOEnable(SERIAL1_BASE);
    UARTFIFOLevelSet(SERIAL1_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

#if defined(RX_BUFFER_SIZE) || defined(TX_BUFFER_SIZE)
    IntPrioritySet(SERIAL1_INT, 0x40);
    UARTIntRegister(SERIAL1_BASE, uart_interrupt_handler);
  #ifdef RX_BUFFER_SIZE
    UARTIntEnable(SERIAL1_BASE, UART_INT_RX|UART_INT_RT);
  #endif
  #ifdef TX_BUFFER_SIZE
    UARTTxIntModeSet(SERIAL1_BASE, UART_TXINT_MODE_EOT);
  #endif
#endif

    UARTEnable(SERIAL1_BASE);

#ifdef RTS_PORT

    SysCtlPeripheralEnable(RTS_PERIPH);
    SysCtlDelay(3);

    GPIOPinTypeGPIOOutput(RTS_PORT, RTS_PIN);
    GPIOPinWrite(RTS_PORT, RTS_PIN, 0);

#endif

#ifdef SERIAL2_MOD

    SysCtlPeripheralEnable(SERIAL2_PERIPH);
    SysCtlPeripheralEnable(SERIAL2_SYSCTL);
    SysCtlDelay(3);

    GPIOPinConfigure(SERIAL2_RX);
    GPIOPinConfigure(SERIAL2_TX);
    GPIOPinTypeUART(SERIAL2_PORT, SERIAL2_PINS);

    UARTClockSourceSet(SERIAL2_BASE, UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(SERIAL2_BASE, 16000000, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTFIFOEnable(SERIAL2_BASE);
    UARTFIFOLevelSet(SERIAL2_BASE, UART_FIFO_TX1_8, UART_FIFO_RX1_8);
    IntPrioritySet(SERIAL2_INT, 0x40);
    UARTIntRegister(SERIAL2_BASE, uart2_interrupt_handler);
    UARTEnable(SERIAL2_BASE);
#endif
}

static bool serialBlockingCallbackDummy (void)
{
    return true;
}

void setSerialBlockingCallback (bool (*fn)(void))
{
    serialBlockingCallback = fn == 0 ? serialBlockingCallbackDummy : fn;
}

#ifdef RX_BUFFER_SIZE

//
// serialReceiveCallback - called before data is inserted into the buffer, return false to drop
//

void setSerialReceiveCallback (bool (*fn)(char))
{
    serialReceiveCallback = fn;
}

//
// serialGetC - returns -1 if no data available
//

int16_t serialGetC (void)
{
    int16_t data;
    uint16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head)
        return -1; // no data available else EOF

//    UARTIntDisable(SERIAL1_PORT, UART_INT_RX|UART_INT_RT);
    data = rxbuffer.data[bptr++];                   // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);  // and update pointer

//    UARTIntEnable(SERIAL1_PORT, UART_INT_RX|UART_INT_RT);
 #ifdef RTS_PORT
    if (rxbuffer.rts_state && BUFCOUNT(rxbuffer.head, rxbuffer.tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM)    // Clear RTS if
        GPIOPinWrite(RTS_PORT, RTS_PIN, rxbuffer.rts_state = 0);                             // buffer count is below low water mark
 #endif
    return data;
}

// "dummy" version of serialGetC
static int16_t serialGetNull (void)
{
    return -1;
}

inline uint16_t serialRxCount (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t serialRxFree (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

void serialRxFlush (void)
{
    rxbuffer.tail = rxbuffer.head;
 #ifdef RTS_PORT
    GPIOPinWrite(RTS_PORT, RTS_PIN, rxbuffer.rts_state = 0);
 #endif
}

void serialRxCancel (void)
{
    rxbuffer.data[rxbuffer.head] = CAN;
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
 #ifdef RTS_PORT
    GPIOPinWrite(RTS_PORT, RTS_PIN, rxbuffer.rts_state = 0);
 #endif
}

#else

int32_t serialGetC ()
{
    int32_t c = -1,  waitCount = 3000;

    while(waitCount && c == -1) {

        if(!UARTCharsAvail(SERIAL1_PORT)) {
            SysCtlDelay(500);
            waitCount--;
        } else
            c = UARTCharGetNonBlocking(SERIAL1_PORT);
    }

    return c;
}

#endif

void serialWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

void serialWriteLn (const char *data)
{
    serialWriteS(data);
    serialWriteS(EOL);
}

void serialWrite (const char *data, unsigned int length)
{
    char *ptr = (char *)data;

    while(length--)
        serialPutC(*ptr++);

}

bool serialSuspendInput (bool suspend)
{
    if(suspend)
        hal.serial_read = serialGetNull;
    else if(rxbuffer.backup)
        memcpy(&rxbuffer, &rxbackup, sizeof(serial_buffer_t));

    return rxbuffer.tail != rxbuffer.head;
}

#ifdef LINE_BUFFER_SIZE

char *serialReadLn (void)
{
    static char cmdbuf[LINE_BUFFER_SIZE];

    int32_t c = 0;
    uint32_t count = 0;

    while(c != CR) {
        if((c = serialGetC()) != -1) {

            if(c == CR)
                cmdbuf[count] = '\0';
            else if(c > 31 && count < sizeof(cmdbuf))
                cmdbuf[count++] = (char)c;
            else if(c == EOF)
                c = CR;
            else if(c == DEL && count > 0)
                count--;
        }
    }

    return count ? cmdbuf : 0;
}

#endif

#ifdef TX_BUFFER_SIZE

bool serialPutC (const char c)
{
    uint32_t next_head;

    if(tx_head != tx_tail || !UARTCharPutNonBlocking(SERIAL1_BASE, c)) {  // Send character without buffering if possible

        next_head = (tx_head + 1) & (TX_BUFFER_SIZE - 1);           // Get and update head pointer

        while(tx_tail == next_head) {                               // Buffer full, block until space is available...
            if(!serialBlockingCallback())
                return false;
        }

        txbuf[tx_head] = c;                                         // Add data to buffer
        tx_head = next_head;                                        // and update head pointer

        UARTIntEnable(SERIAL1_BASE, UART_INT_TX); // Enable interrupts
    }

    return true;
}

uint16_t serialTxCount (void)
{
    uint16_t head = tx_head, tail = tx_tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE);
}

#else

void serialPutC (const char c) {
    UARTCharPut(SERIAL1_PORT, c);
}

#endif

#if defined(RX_BUFFER_SIZE) || defined(TX_BUFFER_SIZE)

static void uart_interrupt_handler (void)
{
    uint16_t bptr;
    int32_t data;
    uint32_t iflags = UARTIntStatus(SERIAL1_BASE, true);

 #ifdef TX_BUFFER_SIZE

    if(iflags & UART_INT_TX) {

        bptr = tx_tail;

        if(tx_head != bptr) {

            UARTCharPut(SERIAL1_BASE, txbuf[bptr++]);                 // Put character in TXT FIFO
            bptr &= (TX_BUFFER_SIZE - 1);                       // and update tmp tail pointer

            while(tx_head != bptr && UARTSpaceAvail(SERIAL1_BASE)) {  // While data in TX buffer and free space in TX FIFO
                UARTCharPut(SERIAL1_BASE, txbuf[bptr++]);             // put next character
                bptr &= (TX_BUFFER_SIZE - 1);                   // and update tmp tail pointer
            }

            tx_tail = bptr;                                     //  Update tail pinter

            if(bptr == tx_head)                                 // Disable TX  interrups
                UARTIntDisable(SERIAL1_BASE, UART_INT_TX);            // when TX buffer empty

/*        } else {
            UARTIntDisable(SERIAL1_PORT, UART_INT_TX);
            UARTIntClear(SERIAL1_PORT, UART_INT_TX); */
        }

    }

 #endif

 #ifdef RX_BUFFER_SIZE

    if(iflags & (UART_INT_RX|UART_INT_RT)) {

        data = UARTCharGet(SERIAL1_BASE);

        if(data == CMD_TOOL_ACK && !rxbuffer.backup) {

            memcpy(&rxbackup, &rxbuffer, sizeof(serial_buffer_t));
            rxbuffer.backup = true;
            rxbuffer.tail = rxbuffer.head;
            hal.serial_read = serialGetC; // restore normal input

        } else if(!serialReceiveCallback || serialReceiveCallback((char)data)) {

            bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer

            if(bptr == rxbuffer.tail)                           // If buffer full
                rxbuffer.overflow = 1;                          // flag overflow,
            else {
                rxbuffer.data[rxbuffer.head] = (char)data;      // else add data to buffer
                rxbuffer.head = bptr;                           // and update pointer
            }
        }

   #ifdef RTS_PORT
        if (!rxbuffer.rts_state && BUFCOUNT(rxbuffer.head, rxbuffer.tail, RX_BUFFER_SIZE) >= RX_BUFFER_HWM)
            GPIOPinWrite(RTS_PORT, RTS_PIN, rxbuffer.rts_state = RTS_PIN);
   #endif
    }
 #endif
}

#ifdef SERIAL2_MOD

void serialSelect (bool mpg)
{
    if(mpg) {
        UARTIntDisable(SERIAL1_BASE, UART_INT_RX|UART_INT_RT);
        UARTIntEnable(SERIAL2_BASE, UART_INT_RX|UART_INT_RT);
    } else {
        UARTIntEnable(SERIAL1_BASE, UART_INT_RX|UART_INT_RT);
        UARTIntDisable(SERIAL2_BASE, UART_INT_RX|UART_INT_RT);
    }
}

//
// Returns number of free characters in serial input buffer
//
uint16_t serial2RxFree (void)
{
    uint16_t tail = rx2_tail, head = rx2_head;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
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

    char data = rx2buf[bptr++];             // Get next character, increment tmp pointer
    rx2_tail = bptr & (RX_BUFFER_SIZE - 1); // and update pointer

    return (int16_t)data;
}

static void uart2_interrupt_handler (void)
{
    uint32_t iflags = UARTIntStatus(SERIAL2_BASE, true);

 #ifdef RX_BUFFER_SIZE

    if(iflags & (UART_INT_RX|UART_INT_RT)) {

        uint16_t bptr = (rx2_head + 1) & (RX_BUFFER_SIZE - 1);   // Get next head pointer

        if(bptr == rx2_tail) {                                  // If buffer full
            rx2_overflow = 1;                                   // flag overlow
            UARTCharGet(SERIAL2_BASE);                          // and do dummy read to clear interrupt;
        } else {
            int32_t data = UARTCharGet(SERIAL2_BASE);
            if(!serialReceiveCallback || serialReceiveCallback((char)data)) {
                rx2buf[rx2_head] = (char)data;  // Add data to buffer
                rx2_head = bptr;                // and update pointer
            }
        }
    }

 #endif

}

#endif // SERIAL2_MOD

#endif // defined(RX_BUFFER_SIZE) || defined(TX_BUFFER_SIZE)
