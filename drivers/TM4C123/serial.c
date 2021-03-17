//
// serial.c - (UART) port library for Tiva
//
// v1.6 / 2021-03-14 / Io Engineering / Terje
//

/*

Copyright (c) 2017-2021, Terje Io
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

static void uart_interrupt_handler (void);

static stream_tx_buffer_t txbuffer = {0};
static stream_rx_buffer_t rxbuffer = {0};

void serialInit (void)
{
#ifdef BACKCHANNEL

    #define UARTCH UART0_BASE
    #define INT_UART INT_UART0

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    SysCtlDelay(3);

    GPIOPinConfigure(GPIO_PA0_U0RX);
    GPIOPinConfigure(GPIO_PA1_U0TX);
    GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

#else

    #define UARTCH UART1_BASE
    #define INT_UART INT_UART1

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlDelay(3);

    GPIOPinConfigure(GPIO_PB0_U1RX);
    GPIOPinConfigure(GPIO_PB1_U1TX);
    GPIOPinTypeGPIOOutput(RTS_PORT, RTS_PIN);
    GPIOPinWrite(RTS_PORT, RTS_PIN, 0);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_UART1);
    SysCtlDelay(3);

    GPIOPinTypeUART(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1);

#endif

    UARTClockSourceSet(UARTCH, UART_CLOCK_PIOSC);
    UARTConfigSetExpClk(UARTCH, 16000000, 115200, (UART_CONFIG_WLEN_8 | UART_CONFIG_STOP_ONE | UART_CONFIG_PAR_NONE));
    UARTFIFOEnable(UARTCH);
    UARTFIFOLevelSet(UARTCH, UART_FIFO_TX1_8, UART_FIFO_RX1_8);

    IntPrioritySet(INT_UART, 0x40);
    UARTIntRegister(UARTCH, uart_interrupt_handler);
    UARTIntEnable(UARTCH, UART_INT_RX|UART_INT_RT);
    UARTTxIntModeSet(UARTCH, UART_TXINT_MODE_EOT);

    UARTEnable(UARTCH);
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

//    UARTIntDisable(UARTCH, UART_INT_RX|UART_INT_RT);
    data = rxbuffer.data[bptr++];                   // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);    // and update pointer

//    UARTIntEnable(UARTCH, UART_INT_RX|UART_INT_RT);

    if (rxbuffer.rts_state && BUFCOUNT(rxbuffer.head, rxbuffer.tail, RX_BUFFER_SIZE) < RX_BUFFER_LWM) {   // Clear RTS if
        rxbuffer.rts_state = false;                                                                       // buffer count is below
        GPIOPinWrite(RTS_PORT, RTS_PIN, 0);                                                               // low water mark
    }

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
    rxbuffer.overflow = rxbuffer.rts_state = false;
    GPIOPinWrite(RTS_PORT, RTS_PIN, 0);
}

void serialRxCancel (void)
{
    serialRxFlush();
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
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
    serialWriteS(ASCII_EOL);
}

void serialWrite (const char *data, unsigned int length)
{
    char *ptr = (char *)data;

    while(length--)
        serialPutC(*ptr++);
}

bool serialPutC (const char c)
{
    uint_fast16_t next_head;

    if(txbuffer.head != txbuffer.tail || !UARTCharPutNonBlocking(UARTCH, c)) {  // Send character without buffering if possible

        next_head = (txbuffer.head + 1) & (TX_BUFFER_SIZE - 1);     // Get and update head pointer

        while(txbuffer.tail == next_head) {                         // Buffer full, block until space is available...
            if(!hal.stream_blocking_callback())
                return false;
        }

        txbuffer.data[txbuffer.head] = c;                           // Add data to buffer
        txbuffer.head = next_head;                                  // and update head pointer

        UARTIntEnable(UARTCH, UART_INT_TX);                         // Enable interrupts
    }

    return true;
}

bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuffer, suspend);
}

uint16_t serialTxCount(void) {

    uint_fast16_t head = txbuffer.head, tail = txbuffer.tail;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE);
}

static void uart_interrupt_handler (void)
{
    uint_fast16_t bptr;
    int32_t data;
    uint32_t iflags = UARTIntStatus(UARTCH, true);

    if(iflags & UART_INT_TX) {

        bptr = txbuffer.tail;

        if(txbuffer.head != bptr) {

            UARTCharPut(UARTCH, txbuffer.data[bptr++]);                 // Put character in TXT FIFO
            bptr &= (TX_BUFFER_SIZE - 1);                               // and update tmp tail pointer

            while(txbuffer.head != bptr && UARTSpaceAvail(UARTCH)) {    // While data in TX buffer and free space in TX FIFO
                UARTCharPut(UARTCH, txbuffer.data[bptr++]);             // put next character
                bptr &= (TX_BUFFER_SIZE - 1);                           // and update tmp tail pointer
            }

            txbuffer.tail = bptr;                                       //  Update tail pinter

            if(bptr == txbuffer.head)                                   // Disable TX  interrups
                UARTIntDisable(UARTCH, UART_INT_TX);                    // when TX buffer empty
        }
    }

    if(iflags & (UART_INT_RX|UART_INT_RT)) {

        bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer

        if(bptr == rxbuffer.tail) {                         // If buffer full
            rxbuffer.overflow = 1;                          // flag overflow
            UARTCharGet(UARTCH);                            // and do dummy read to clear interrupt;
        } else {
            data = UARTCharGet(UARTCH);
            if(data == CMD_TOOL_ACK && !rxbuffer.backup) {
                stream_rx_backup(&rxbuffer);
                hal.stream.read = serialGetC; // restore normal input
            } else if(!hal.stream.enqueue_realtime_command((char)data)) {
                rxbuffer.data[rxbuffer.head] = (char)data;  // Add data to buffer
                rxbuffer.head = bptr;                       // and update pointer
            }
        }

        if (!rxbuffer.rts_state && BUFCOUNT(rxbuffer.head, rxbuffer.tail, RX_BUFFER_SIZE) >= RX_BUFFER_HWM) {
            rxbuffer.rts_state = true;
            GPIOPinWrite(RTS_PORT, RTS_PIN, RTS_PIN);
        }
    }
}
