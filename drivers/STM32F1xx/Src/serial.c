/*

  serial.c - serial port implementation for STM32F103C8 ARM processors

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

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

#include <string.h>

#include "driver.h"
#include "serial.h"
#include "grbl.h"

#include "main.h"

#if !USB_SERIAL_CDC

static stream_rx_buffer_t rxbuf = {0};
static stream_tx_buffer_t txbuf = {0};

void serialInit (void)
{
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_InitStructure;

    GPIO_InitStructure.Pin = GPIO_PIN_9;
    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    GPIO_InitStructure.Pin = GPIO_PIN_10;
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART1->CR1 |= (USART_CR1_RE|USART_CR1_TE);
    USART1->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), 115200);
    USART1->CR1 |= (USART_CR1_UE|USART_CR1_RXNEIE);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
}

//
// Returns number of free characters in serial input buffer
//
uint16_t serialRxFree (void)
{
    uint16_t tail = rxbuf.tail, head = rxbuf.head;
    return RX_BUFFER_SIZE - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
void serialRxFlush (void)
{
    rxbuf.head = rxbuf.tail = 0;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
void serialRxCancel (void)
{
    rxbuf.data[rxbuf.head] = ASCII_CAN;
    rxbuf.tail = rxbuf.head;
    rxbuf.head = (rxbuf.tail + 1) & (RX_BUFFER_SIZE - 1);
}

//
// Attempt to send a character bypassing buffering
//
inline static bool serialPutCNonBlocking (const char c)
{
    bool ok;

    if((ok = !(USART1->CR1 & USART_CR1_TXEIE) && !(USART1->SR & USART_SR_TXE)))
        USART1->DR = c;

    return ok;
}

//
// Writes a character to the serial output stream
//
bool serialPutC (const char c)
{
//    if(txbuf.head != txbuf.tail || !serialPutCNonBlocking(c)) {           // Try to send character without buffering...

        uint16_t next_head = (txbuf.head + 1) & (TX_BUFFER_SIZE - 1);   // .. if not, get pointer to next free slot in buffer

        while(txbuf.tail == next_head) {                                // While TX buffer full
            if(!hal.stream_blocking_callback())                         // check if blocking for space,
                return false;                                           // exit if not (leaves TX buffer in an inconsistent state)
        }

        txbuf.data[txbuf.head] = c;                                     // Add data to buffer,
        txbuf.head = next_head;                                         // update head pointer and
        USART1->CR1 |= USART_CR1_TXEIE;                                 // enable TX interrupts
//    }

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
    uint16_t bptr = rxbuf.tail;

    if(bptr == rxbuf.head)
        return -1; // no data available else EOF

    char data = rxbuf.data[bptr++];             // Get next character, increment tmp pointer
    rxbuf.tail = bptr & (RX_BUFFER_SIZE - 1);   // and update pointer

    return (int16_t)data;
}

bool serialSuspendInput (bool suspend)
{
    return stream_rx_suspend(&rxbuf, suspend);
}

void USART1_IRQHandler (void)
{
    if(USART1->SR & USART_SR_RXNE) {

        uint16_t next_head = (rxbuf.head + 1) & (RX_BUFFER_SIZE - 1);   // Get and increment buffer pointer

        if(rxbuf.tail == next_head) {                                   // If buffer full
            rxbuf.overflow = 1;                                         // flag overflow
            next_head =  USART1->DR;                                    // and do dummy read to clear interrupt
        } else {
            char data = USART1->DR;
            if(data == CMD_TOOL_ACK && !rxbuf.backup) {
                stream_rx_backup(&rxbuf);
                hal.stream.read = serialGetC; // restore normal input
            } else if(!hal.stream.enqueue_realtime_command(data)) {     // Check and strip realtime commands,
                rxbuf.data[rxbuf.head] = data;                          // if not add data to buffer
                rxbuf.head = next_head;                                 // and update pointer
            }
        }
    }

    if((USART1->SR & USART_SR_TXE) && (USART1->CR1 & USART_CR1_TXEIE)) {

        uint16_t tail = txbuf.tail;             // Get buffer pointer

        USART1->DR = txbuf.data[tail++];        // Send next character and increment pointer

        if(tail == TX_BUFFER_SIZE)              // If at end
            tail = 0;                           // wrap pointer around

        txbuf.tail = tail;                      // Update global pointer

        if(tail == txbuf.head)                  // If buffer empty then
            USART1->CR1 &= ~USART_CR1_TXEIE;    // disable UART TX interrupt
   }
}

#endif
