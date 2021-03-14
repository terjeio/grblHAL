/*

  serial.c - serial port implementation for STM32F4xx ARM processors

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

#include "serial.h"
#include "grbl/hal.h"

#include "main.h"

static stream_rx_buffer_t rxbuf = {0};
static stream_tx_buffer_t txbuf = {0};

#ifdef SERIAL2_MOD
static stream_rx_buffer_t rxbuf2 = {0};
static stream_tx_buffer_t txbuf2 = {0};
#endif

void serialInit (void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    __HAL_RCC_GPIOA_CLK_ENABLE();

#if defined(NUCLEO_F411) || defined(NUCLEO_F446)

  #define USART USART2
  #define USART_IRQHandler USART2_IRQHandler

    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitStructure.Pin = GPIO_PIN_2|GPIO_PIN_3;
    GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART->CR1 = USART_CR1_RE|USART_CR1_TE;
    USART->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), 115200);
    USART->CR1 |= (USART_CR1_UE|USART_CR1_RXNEIE);

    HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);

#else

  #define USART USART1
  #define USART_IRQHandler USART1_IRQHandler

    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitStructure.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

    USART->CR1 = USART_CR1_RE|USART_CR1_TE;
    USART->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), 115200);
    USART->CR1 |= (USART_CR1_UE|USART_CR1_RXNEIE);

    HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);

#endif
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

    if((ok = !(USART->CR1 & USART_CR1_TXEIE) && !(USART->SR & USART_SR_TXE)))
        USART->DR = c;

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
        USART->CR1 |= USART_CR1_TXEIE;                                  // enable TX interrupts
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

void USART_IRQHandler (void)
{
    if(USART->SR & USART_SR_RXNE) {

        uint16_t next_head = (rxbuf.head + 1) & (RX_BUFFER_SIZE - 1);   // Get and increment buffer pointer

        if(rxbuf.tail == next_head) {                                   // If buffer full
            rxbuf.overflow = 1;                                         // flag overflow
            next_head =  USART->DR;                                     // and do dummy read to clear interrupt
        } else {
            char data = USART->DR;
            if(data == CMD_TOOL_ACK && !rxbuf.backup) {
                stream_rx_backup(&rxbuf);
                hal.stream.read = serialGetC; // restore normal input
            } else if(!hal.stream.enqueue_realtime_command(data)) {     // Check and strip realtime commands,
                rxbuf.data[rxbuf.head] = data;                          // if not add data to buffer
                rxbuf.head = next_head;                                 // and update pointer
            }
        }
    }

    if((USART->SR & USART_SR_TXE) && (USART->CR1 & USART_CR1_TXEIE)) {

        uint16_t tail = txbuf.tail;             // Get buffer pointer

        USART->DR = txbuf.data[tail++];         // Send next character and increment pointer

        if(tail == TX_BUFFER_SIZE)              // If at end
            tail = 0;                           // wrap pointer around

        txbuf.tail = tail;                      // Update global pointer

        if(tail == txbuf.head)                  // If buffer empty then
            USART->CR1 &= ~USART_CR1_TXEIE;     // disable UART TX interrupt
   }
}

#ifdef SERIAL2_MOD

void serial2Init (uint32_t baud_rate)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    GPIO_InitStructure.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStructure.Pull = GPIO_NOPULL;
    GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_VERY_HIGH;

    __HAL_RCC_GPIOA_CLK_ENABLE();

#if defined(NUCLEO_F411) || defined(NUCLEO_F446)

#define UART2 USART1
#define UART2_IRQHandler USART1_IRQHandler

  __HAL_RCC_USART1_CLK_ENABLE();

  GPIO_InitStructure.Pin = GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStructure.Alternate = GPIO_AF7_USART1;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  UART2->CR1 = USART_CR1_RE|USART_CR1_TE;
  UART2->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), baud_rate);
  UART2->CR1 |= (USART_CR1_UE|USART_CR1_RXNEIE);

  HAL_NVIC_SetPriority(USART1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART1_IRQn);

#else

#define UART2 USART2
#define UART2_IRQHandler USART2_IRQHandler

  __HAL_RCC_USART2_CLK_ENABLE();

  GPIO_InitStructure.Pin = GPIO_PIN_2|GPIO_PIN_3;
  GPIO_InitStructure.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);

  UART2->CR1 = USART_CR1_RE|USART_CR1_TE;
  UART2->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), baud_rate);
  UART2->CR1 |= (USART_CR1_UE|USART_CR1_RXNEIE);

  HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(USART2_IRQn);

#endif

#if MODBUS_ENABLE
//  UART2->IE = EUSCI_A_IE_RXIE;
#endif
}

bool serial2SetBaudRate (uint32_t baud_rate)
{
    static bool init_ok = false;

    if(!init_ok) {
        serial2Init(baud_rate);
        init_ok = true;
    }

    return true;
}

#if !(MODBUS_ENABLE || TRINAMIC_ENABLE == 2209)

void serialSelect (bool mpg)
{ /*
    if(mpg) {
        SERIAL_MODULE->IE = 0;
        SERIAL2_MODULE->IE = EUSCI_A_IE_RXIE;
    } else {
        SERIAL_MODULE->IE = EUSCI_A_IE_RXIE;
        SERIAL2_MODULE->IE = 0;
    } */
}

#endif

//
// Returns number of free characters in serial input buffer
//
uint16_t serial2RxFree (void)
{
    uint32_t tail = rxbuf2.tail, head = rxbuf2.head;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Returns number of characters in serial input buffer
//
uint16_t serial2RxCount (void)
{
    uint32_t tail = rxbuf2.tail, head = rxbuf2.head;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

//
// Flushes the serial input buffer
//
void serial2RxFlush (void)
{
    rxbuf2.tail = rxbuf2.head;
}

//
// Flushes and adds a CAN character to the serial input buffer
//
void serial2RxCancel (void)
{
    rxbuf2.data[rxbuf2.head] = ASCII_CAN;
    rxbuf2.tail = rxbuf2.head;
    rxbuf2.head = (rxbuf2.tail + 1) & (RX_BUFFER_SIZE - 1);
}

//
// Attempt to send a character bypassing buffering
//
static inline bool serial2PutCNonBlocking (const char c)
{
    bool ok;

    if((ok = !(UART2->CR1 & USART_CR1_TXEIE) && !(UART2->SR & USART_SR_TXE)))
        UART2->DR = c;

    return ok;
}

//
// Writes a character to the serial output stream
//
bool serial2PutC (const char c)
{
    uint32_t next_head;

//    if(txbuf2.head != txbuf2.tail || !serial2PutCNonBlocking(c)) {  // Try to send character without buffering...

        next_head = (txbuf2.head + 1) & (TX_BUFFER_SIZE - 1);       // .. if not, set and update head pointer

        while(txbuf2.tail == next_head) {                           // While TX buffer full
            if(!hal.stream_blocking_callback())                     // check if blocking for space,
                return false;                                       // exit if not (leaves TX buffer in an inconsistent state)
            UART2->CR1 |= USART_CR1_TXEIE;                          // Enable TX interrupts???
        }

        txbuf2.data[txbuf2.head] = c;                               // Add data to buffer
        txbuf2.head = next_head;                                    // and update head pointer

        UART2->CR1 |= USART_CR1_TXEIE;                              // Enable TX interrupts
//    }

    return true;
}

// Writes a number of characters from a buffer to the serial output stream, blocks if buffer full
//
void serial2Write(const char *s, uint16_t length)
{
    char *ptr = (char *)s;

    while(length--)
        serial2PutC(*ptr++);
}

//
// Flushes the serial output buffer
//
void serial2TxFlush (void)
{
    UART2->CR1 &= ~USART_CR1_TXEIE;     // Disable TX interrupts
    txbuf2.tail = txbuf2.head;
}

//
// Returns number of characters pending transmission
//
uint16_t serial2TxCount (void)
{
    uint32_t tail = txbuf2.tail, head = txbuf2.head;

    return BUFCOUNT(head, tail, TX_BUFFER_SIZE) + (UART2->SR & USART_SR_TC ? 0 : 1);
}

//
// serialGetC - returns -1 if no data available
//
int16_t serial2GetC (void)
{
    uint16_t bptr = rxbuf2.tail;

    if(bptr == rxbuf2.head)
        return -1; // no data available else EOF

    char data = rxbuf2.data[bptr++];             // Get next character, increment tmp pointer
    rxbuf2.tail = bptr & (RX_BUFFER_SIZE - 1);   // and update pointer

    return (int16_t)data;
}

void UART2_IRQHandler (void)
{
    if(UART2->SR & USART_SR_RXNE) {

        uint16_t next_head = (rxbuf2.head + 1) & (RX_BUFFER_SIZE - 1);  // Get and increment buffer pointer

        if(rxbuf2.tail == next_head) {                                  // If buffer full
            rxbuf.overflow = 1;                                         // flag overflow
            next_head = UART2->DR;                                      // and do dummy read to clear interrupt
        } else {
#if MODBUS_ENABLE || TRINAMIC_ENABLE == 2209
            rxbuf2.data[rxbuf2.head] = UART2->DR;                       // if not add data to buffer
            rxbuf2.head = next_head;                                    // and update pointer
#else
            char data = USART->DR;
            if(!hal.stream.enqueue_realtime_command(data)) {            // Check and strip realtime commands,
                rxbuf2.data[rxbuf2.head] = data;                        // if not add data to buffer
                rxbuf2.head = next_head;                                // and update pointer
            }
#endif
        }
    }

    if((UART2->SR & USART_SR_TXE) && (UART2->CR1 & USART_CR1_TXEIE)) {

        uint16_t tail = txbuf2.tail;            // Get buffer pointer

        UART2->DR = txbuf2.data[tail++];        // Send next character and increment pointer

        if(tail == TX_BUFFER_SIZE)              // If at end
            tail = 0;                           // wrap pointer around

        txbuf2.tail = tail;                     // Update global pointer

        if(tail == txbuf2.head)                 // If buffer empty then
            UART2->CR1 &= ~USART_CR1_TXEIE;     // disable UART TX interrupt
   }
}
#endif
