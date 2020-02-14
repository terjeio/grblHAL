/*
  uart.c - driver code for IMXRT1062 processor (on Teensy 4.0 board)

  Part of GrblHAL

  Some parts of this code is Copyright (c) 2020 Terje Io

  Some parts are derived from HardwareSerial.cpp in the Teensyduino Core Library

*/

/* Teensyduino Core Library
 * http://www.pjrc.com/teensy/
 * Copyright (c) 2019 PJRC.COM, LLC.
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * 1. The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * 2. If the Software is incorporated into a build system that allows
 * selection among a list of target devices, then similar target
 * devices manufactured by PJRC.COM must be included in the list of
 * target devices and selectable in the same manner.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "driver.h"

static stream_tx_buffer_t txbuffer = {0};
static stream_rx_buffer_t rxbuffer = {0}, rxbackup;

#define PIN_TO_BASEREG(pin)             (portOutputRegister(pin))
#define PIN_TO_BITMASK(pin)             (digitalPinToBitMask(pin))
#define IO_REG_TYPE uint32_t
#define IO_REG_BASE_ATTR
#define IO_REG_MASK_ATTR
#define DIRECT_READ(base, mask)         ((*((base)+2) & (mask)) ? 1 : 0)
#define DIRECT_MODE_INPUT(base, mask)   (*((base)+1) &= ~(mask))
#define DIRECT_MODE_OUTPUT(base, mask)  (*((base)+1) |= (mask))
#define DIRECT_WRITE_LOW(base, mask)    (*((base)+34) = (mask))
#define DIRECT_WRITE_HIGH(base, mask)   (*((base)+33) = (mask))

#define CTRL_ENABLE         (LPUART_CTRL_TE | LPUART_CTRL_RE | LPUART_CTRL_RIE | LPUART_CTRL_ILIE)
#define CTRL_TX_ACTIVE      (CTRL_ENABLE | LPUART_CTRL_TIE)
#define CTRL_TX_COMPLETING  (CTRL_ENABLE | LPUART_CTRL_TCIE)
#define CTRL_TX_INACTIVE    CTRL_ENABLE 

#define UART_CLOCK 24000000
IMXRT_LPUART_t *port;

uint8_t             rx_pin_index_ = 0x0;    // default is always first item
uint8_t             tx_pin_index_ = 0x0;

typedef struct {
    const uint8_t       pin;        // The pin number
    const uint32_t      mux_val;    // Value to set for mux;
    volatile uint32_t   *select_input_register; // Which register controls the selection
    const uint32_t      select_val; // Value for that selection
} pin_info_t;

typedef struct {
    uint8_t serial_index;   // which object are we? 0 based
    enum IRQ_NUMBER_t irq;
    void (*irq_handler)(void);
    volatile uint32_t *ccm_register;
    const uint32_t ccm_value;
    pin_info_t rx_pins[2];
    pin_info_t tx_pins[2];
    const uint16_t irq_priority;
} hardware_t;

static void uart_interrupt_handler (void);
    volatile uint32_t   *transmit_pin_baseReg_ = 0;
    uint32_t            transmit_pin_bitmask_ = 0;

static hardware_t hw =
{
    .serial_index = 0,
    .irq = IRQ_LPUART6,
    .irq_handler = uart_interrupt_handler,
    .irq_priority = 1,
    .ccm_register = &CCM_CCGR3,
    .ccm_value = CCM_CCGR3_LPUART6(CCM_CCGR_ON), 
    .rx_pins = {{0,2, &IOMUXC_LPUART6_RX_SELECT_INPUT, 1}, {0xff, 0xff, NULL, 0}},
    .tx_pins = {{1,2, NULL, 0}, {0xff, 0xff, NULL, 0}}
};

static hardware_t *hardware;
static bool transmitting_ = 0;

void transmitterEnable(uint8_t pin)
{
    while (transmitting_) ;
    pinMode(pin, OUTPUT);
    transmit_pin_baseReg_ = PIN_TO_BASEREG(pin);
    transmit_pin_bitmask_ = PIN_TO_BITMASK(pin);
    DIRECT_WRITE_LOW(transmit_pin_baseReg_, transmit_pin_bitmask_);
}

void serialInit (void)
{
    hardware = &hw;

    port = &IMXRT_LPUART6;

    *hardware->ccm_register |= hardware->ccm_value;

    *(portControlRegister(hardware->rx_pins[rx_pin_index_].pin)) = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
    *(portConfigRegister(hardware->rx_pins[rx_pin_index_].pin)) = hardware->rx_pins[rx_pin_index_].mux_val;
    if (hardware->rx_pins[rx_pin_index_].select_input_register) {
        *(hardware->rx_pins[rx_pin_index_].select_input_register) = hardware->rx_pins[rx_pin_index_].select_val;        
    }   

    *(portControlRegister(hardware->tx_pins[tx_pin_index_].pin)) = IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(3) | IOMUXC_PAD_SPEED(3);
    *(portConfigRegister(hardware->tx_pins[tx_pin_index_].pin)) = hardware->tx_pins[tx_pin_index_].mux_val;

    port->BAUD = 419430408UL; // 115200 @ UART CLOCK = 24 MHz
    port->PINCFG = 0;

    // Enable the transmitter, receiver and enable receiver interrupt
    NVIC_DISABLE_IRQ(hardware->irq);
    attachInterruptVector(hardware->irq, hardware->irq_handler);
    NVIC_SET_PRIORITY(hardware->irq, hardware->irq_priority);   // maybe should put into hardware...
    NVIC_ENABLE_IRQ(hardware->irq);

    uint16_t tx_fifo_size = (((port->FIFO >> 4) & 0x7) << 2);
    uint8_t tx_water = (tx_fifo_size < 16) ? tx_fifo_size >> 1 : 7;
    uint16_t rx_fifo_size = (((port->FIFO >> 0) & 0x7) << 2);
    uint8_t rx_water = (rx_fifo_size < 16) ? rx_fifo_size >> 1 : 7;
    /*
    Serial.printf("SerialX::begin stat:%x ctrl:%x fifo:%x water:%x\n", port->STAT, port->CTRL, port->FIFO, port->WATER );
    Serial.printf("  FIFO sizes: tx:%d rx:%d\n",tx_fifo_size, rx_fifo_size);    
    Serial.printf("  Watermark tx:%d, rx: %d\n", tx_water, rx_water);
    */
    port->WATER = LPUART_WATER_RXWATER(rx_water) | LPUART_WATER_TXWATER(tx_water);
    port->FIFO |= LPUART_FIFO_TXFE | LPUART_FIFO_RXFE;
    // lets configure up our CTRL register value
    uint32_t ctrl = CTRL_TX_INACTIVE;

uint16_t format = 0;

    // Now process the bits in the Format value passed in
    // Bits 0-2 - Parity plus 9  bit. 
    ctrl |= (format & (LPUART_CTRL_PT | LPUART_CTRL_PE) );  // configure parity - turn off PT, PE, M and configure PT, PE
    if (format & 0x04) ctrl |= LPUART_CTRL_M;       // 9 bits (might include parity)
    if ((format & 0x0F) == 0x04) ctrl |=  LPUART_CTRL_R9T8; // 8N2 is 9 bit with 9th bit always 1

    // Bit 5 TXINVERT
    if (format & 0x20) ctrl |= LPUART_CTRL_TXINV;       // tx invert

    // write out computed CTRL
    port->CTRL = ctrl;

    // Bit 3 10 bit - Will assume that begin already cleared it.
    // process some other bits which change other registers.
    if (format & 0x08)  port->BAUD |= LPUART_BAUD_M10;

    // Bit 4 RXINVERT 
    uint32_t c = port->STAT & ~LPUART_STAT_RXINV;
    if (format & 0x10) c |= LPUART_STAT_RXINV;      // rx invert
    port->STAT = c;

    // bit 8 can turn on 2 stop bit mote
    if ( format & 0x100) port->BAUD |= LPUART_BAUD_SBNS;

//transmitterEnable(1);
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

//  swr(c); return true;

//  if (transmit_pin_baseReg_) DIRECT_WRITE_HIGH(transmit_pin_baseReg_, transmit_pin_bitmask_);

    if(txbuffer.head == txbuffer.tail && ((port->WATER >> 8) & 0x7) < ((port->FIFO >> 4) & 0x7)) {
        port->DATA  = c;
        return true;
    } 

    next_head = (txbuffer.head + 1) & (TX_BUFFER_SIZE - 1);     // Get and update head pointer

    while(txbuffer.tail == next_head) {                         // Buffer full, block until space is available...
        if(!hal.stream_blocking_callback())
            return false;
    }

    txbuffer.data[txbuffer.head] = c;                           // Add data to buffer
    txbuffer.head = next_head;                                  // and update head pointer

    __disable_irq();
    transmitting_ = 1;
    port->CTRL |= LPUART_CTRL_TIE; // (may need to handle this issue)BITBAND_SET_BIT(LPUART0_CTRL, TIE_BIT); // Enable TX interrupts
    __enable_irq();

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
    uint32_t data, ctrl = port->CTRL;

    if ((ctrl & LPUART_CTRL_TIE) && (port->STAT & LPUART_STAT_TDRE))
    {
        bptr = txbuffer.tail;

        do {
            if(txbuffer.head != bptr) {

                port->DATA = txbuffer.data[bptr++]; // Put character in TXT register
                bptr &= (TX_BUFFER_SIZE - 1);       // and update tmp tail pointer

            } else
                break;

        } while(((port->WATER >> 8) & 0x7) < ((port->FIFO >> 4) & 0x7));

        txbuffer.tail = bptr;                                       //  Update tail pinter

        if(bptr == txbuffer.head)      {                             // Disable TX interrups
            port->CTRL &= ~LPUART_CTRL_TIE; 
//            port->CTRL |= LPUART_CTRL_TCIE; // Actually wondering if we can just leave this one on...
        }
    }

    if ((ctrl & LPUART_CTRL_TCIE) && (port->STAT & LPUART_STAT_TC))
    {
        transmitting_ = 0;
//      if (transmit_pin_baseReg_) DIRECT_WRITE_LOW(transmit_pin_baseReg_, transmit_pin_bitmask_);

        port->CTRL &= ~LPUART_CTRL_TCIE;
    }

    if (port->STAT & (LPUART_STAT_RDRF | LPUART_STAT_IDLE)) {

        while ((port->WATER >> 24) & 0x7) {

            bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer
            data = port->DATA & 0xFF;                           // and read input (use only 8 bits of data)

            if(bptr == rxbuffer.tail) {                         // If buffer full
                rxbuffer.overflow = true;                       // flag overflow
            } else {
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

        if (port->STAT & LPUART_STAT_IDLE)
            port->STAT |= LPUART_STAT_IDLE; // writing a 1 to idle should clear it. 
    }
}
