/*

  serial.c - Atmel SAMD21 low level functions for transmitting bytes via the serial port

  Part of grblHAL

  Copyright (c) 2017-2021 Terje Io
  Some parts:
  Copyright (c) 2015 Arduino LLC.  All right reserved.

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

#include "Arduino.h"

#include "sam.h"
#include "variant.h"
#include "wiring_private.h"

#define PIN_SERIAL1_RX (13ul)
#define PIN_SERIAL1_TX (14ul)
#define PAD_SERIAL1_TX (UART_TX_PAD_2)
#define PAD_SERIAL1_RX (SERCOM_RX_PAD_3)

#define SERCOM_FREQ_REF      48000000
#define SERCOM_NVIC_PRIORITY ((1<<__NVIC_PRIO_BITS) - 1)

#include "driver.h"
#include "serial.h"

typedef enum
{
    UART_TX_PAD_0 = 0x0ul,  // Only for UART
    UART_TX_PAD_2 = 0x1ul,  // Only for UART
    UART_TX_RTS_CTS_PAD_0_2_3 = 0x2ul,  // Only for UART with TX on PAD0, RTS on PAD2 and CTS on PAD3
} SercomUartTXPad;

typedef enum
{
    SAMPLE_RATE_x16 = 0x1,  //Fractional
    SAMPLE_RATE_x8 = 0x3,   //Fractional
} SercomUartSampleRate;


typedef enum
{
    SERCOM_RX_PAD_0 = 0,
    SERCOM_RX_PAD_1,
    SERCOM_RX_PAD_2,
    SERCOM_RX_PAD_3
} SercomRXPad;

typedef enum
{
    UART_EXT_CLOCK = 0,
    UART_INT_CLOCK = 0x1u
} SercomUartMode;
typedef enum
{
    MSB_FIRST = 0,
    LSB_FIRST
} SercomDataOrder;

static Sercom *sercom = SERCOM5;
static stream_rx_buffer_t rxbuffer = {0};
static stream_rx_buffer_t txbuffer = {0};

static void SERIAL_IRQHandler (void);

void initSerClockNVIC (Sercom *sercom)
{
  uint8_t clockId = 0;
  IRQn_Type IdNvic=PendSV_IRQn ; // Dummy init to intercept potential error later

  if(sercom == SERCOM0)
  {
    clockId = GCM_SERCOM0_CORE;
    IdNvic = SERCOM0_IRQn;
  }
  else if(sercom == SERCOM1)
  {
    clockId = GCM_SERCOM1_CORE;
    IdNvic = SERCOM1_IRQn;
  }
  else if(sercom == SERCOM2)
  {
    clockId = GCM_SERCOM2_CORE;
    IdNvic = SERCOM2_IRQn;
  }
  else if(sercom == SERCOM3)
  {
    clockId = GCM_SERCOM3_CORE;
    IdNvic = SERCOM3_IRQn;
  }
  #if defined(SERCOM4)
  else if(sercom == SERCOM4)
  {
    clockId = GCM_SERCOM4_CORE;
    IdNvic = SERCOM4_IRQn;
  }
  #endif // SERCOM4
  #if defined(SERCOM5)
  else if(sercom == SERCOM5)
  {
    clockId = GCM_SERCOM5_CORE;
    IdNvic = SERCOM5_IRQn;
  }
  #endif // SERCOM5

  if ( IdNvic == PendSV_IRQn )
  {
    // We got a problem here
    return ;
  }

  // Setting NVIC
  NVIC_EnableIRQ(IdNvic);
  NVIC_SetPriority (IdNvic, SERCOM_NVIC_PRIORITY);  /* set Priority */

  //Setting clock
  GCLK->CLKCTRL.reg = GCLK_CLKCTRL_ID( clockId ) | // Generic Clock 0 (SERCOMx)
                      GCLK_CLKCTRL_GEN_GCLK0 | // Generic Clock Generator 0 is source
                      GCLK_CLKCTRL_CLKEN ;

  while ( GCLK->STATUS.reg & GCLK_STATUS_SYNCBUSY )
  {
    /* Wait for synchronization */
  }
}

void serialInit (void)
{
    pinPeripheral(PIN_SERIAL1_RX, g_APinDescription[PIN_SERIAL1_RX].ulPinType);
    pinPeripheral(PIN_SERIAL1_TX, g_APinDescription[PIN_SERIAL1_TX].ulPinType);

    // sercom->initUART(UART_INT_CLOCK, SAMPLE_RATE_x16, BAUD_RATE);

    initSerClockNVIC(sercom);

    // reset
    sercom->USART.CTRLA.bit.SWRST = 1 ;
    while(sercom->USART.CTRLA.bit.SWRST || sercom->USART.SYNCBUSY.bit.SWRST);

    //Setting the CTRLA register
    sercom->USART.CTRLA.reg =  SERCOM_USART_CTRLA_MODE(UART_INT_CLOCK) |
                SERCOM_USART_CTRLA_SAMPR(SAMPLE_RATE_x16);

    // Internal clock, LSB first, no parity, 16x oversampling
    //sercom->USART.CTRLA.reg = SERCOM_USART_CTRLA_MODE_USART_INT_CLK|SERCOM_SPI_CTRLA_DORD;

    //Setting the Interrupt register
    sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_RXC |  //Received complete
                 SERCOM_USART_INTENSET_ERROR; //All others errors


    // Asynchronous fractional mode (Table 24-2 in datasheet)
    //   BAUD = fref / (sampleRateValue * fbaud)
    // (multiply by 8, to calculate fractional piece)
    uint32_t baudTimes8 = (SystemCoreClock * 8) / (16 * BAUD_RATE);

    sercom->USART.BAUD.FRAC.FP   = (baudTimes8 % 8);
    sercom->USART.BAUD.FRAC.BAUD = (baudTimes8 / 8);


    //

    //Setting the CTRLA register
    sercom->USART.CTRLA.reg |= SERCOM_USART_CTRLA_FORM(0) | LSB_FIRST << SERCOM_USART_CTRLA_DORD_Pos;

    //Setting the CTRLB register
    sercom->USART.CTRLB.reg |= SERCOM_USART_CTRLB_CHSIZE(8) | 1 << SERCOM_USART_CTRLB_SBMODE_Pos; //If no parity use default value

    //  sercom->initPads(uc_padTX, uc_padRX);

    //Setting the CTRLA register
    sercom->USART.CTRLA.reg |= SERCOM_USART_CTRLA_TXPO(PAD_SERIAL1_TX)|SERCOM_USART_CTRLA_RXPO(PAD_SERIAL1_RX);

    // Enable Transceiver and Receiver
    sercom->USART.CTRLB.reg = SERCOM_USART_CTRLB_TXEN|SERCOM_USART_CTRLB_RXEN;

    sercom->USART.CTRLA.bit.ENABLE = 1;
    while(sercom->USART.SYNCBUSY.bit.ENABLE);

    IRQRegister(SERCOM5_IRQn, SERIAL_IRQHandler);

    NVIC_EnableIRQ(SERCOM5_IRQn);
    NVIC_SetPriority(SERCOM5_IRQn, 1);

    //  __enable_interrupts();
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
    bool ok;

    if((ok = sercom->USART.INTFLAG.bit.DRE))
        sercom->USART.DATA.reg = c;

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

        sercom->USART.INTENSET.reg = SERCOM_USART_INTENSET_DRE;         // Enable TX interrupts
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
    char data;
    uint16_t bptr;
    
uint8_t ifg = sercom->USART.INTFLAG.reg;

    if(sercom->USART.STATUS.bit.FERR) {
        data = sercom->USART.DATA.bit.DATA;
        sercom->USART.STATUS.bit.FERR = 1;
        sercom->USART.INTFLAG.reg = ifg;
    }

    while(sercom->USART.INTFLAG.bit.RXC) {
        uint16_t sts = sercom->USART.STATUS.reg;
        data =  sercom->USART.DATA.bit.DATA;
        if(data == CMD_TOOL_ACK && !rxbuffer.backup) {
            stream_rx_backup(&rxbuffer);
            hal.stream.read = serialGetC; // restore normal input
        } else if(!hal.stream.enqueue_realtime_command(data)) {

            bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer

            if(bptr == rxbuffer.tail)                           // If buffer full
                rxbuffer.overflow = 1;                          // flag overflow,
            else {
                rxbuffer.data[rxbuffer.head] = (char)data;      // else add data to buffer
                rxbuffer.head = bptr;                           // and update pointer
            }
        }           
    }
    
    if(sercom->USART.INTFLAG.bit.DRE) {
        bptr = txbuffer.tail;                                           // Temp tail position (to avoid volatile overhead)
        if(txbuffer.tail != txbuffer.head) {
            sercom->USART.DATA.reg = (uint16_t)txbuffer.data[bptr++];   // Send a byte from the buffer
            bptr &= (TX_BUFFER_SIZE - 1);                               // and update
            txbuffer.tail = bptr;                                       // tail position
        }
        if (bptr == txbuffer.head)                                      // Turn off TX interrupt
            sercom->USART.INTENCLR.reg = SERCOM_USART_INTENCLR_DRE;     // when buffer empty
    }
}
