// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
// Copyright 2018-2021 Terje Io : Modifications for grbl
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp32/rom/ets_sys.h"
#include "esp_attr.h"
#include "esp32/rom/uart.h"
#include "soc/uart_reg.h"
#include "soc/uart_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/dport_reg.h"
#include "driver/uart.h"
#include "esp_intr_alloc.h"

#include "esp32-hal-uart.h"
#include "grbl/grbl.h"

#define TWO_STOP_BITS_CONF 0x3
#define ONE_STOP_BITS_CONF 0x1
#define CONFIG_DISABLE_HAL_LOCKS 1

#define UART_REG_BASE(u)    ((u==0)?DR_REG_UART_BASE:(      (u==1)?DR_REG_UART1_BASE:(    (u==2)?DR_REG_UART2_BASE:0)))
#define UART_RXD_IDX(u)     ((u==0)?U0RXD_IN_IDX:(          (u==1)?U1RXD_IN_IDX:(         (u==2)?U2RXD_IN_IDX:0)))
#define UART_TXD_IDX(u)     ((u==0)?U0TXD_OUT_IDX:(         (u==1)?U1TXD_OUT_IDX:(        (u==2)?U2TXD_OUT_IDX:0)))
#define UART_INTR_SOURCE(u) ((u==0)?ETS_UART0_INTR_SOURCE:( (u==1)?ETS_UART1_INTR_SOURCE:((u==2)?ETS_UART2_INTR_SOURCE:0)))

typedef void (*uart_isr_ptr)(void *arg);

typedef struct {
    uart_dev_t *dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
    intr_handle_t intr_handle;
} uart_t;

#if CONFIG_DISABLE_HAL_LOCKS
#define UART_MUTEX_LOCK(u)
#define UART_MUTEX_UNLOCK(u)

static uart_t _uart_bus_array[3] = {
    {(volatile uart_dev_t *)(DR_REG_UART_BASE), 0, NULL},
    {(volatile uart_dev_t *)(DR_REG_UART1_BASE), 1, NULL},
    {(volatile uart_dev_t *)(DR_REG_UART2_BASE), 2, NULL}
};
#else
#define UART_MUTEX_LOCK(u)    do {} while (xSemaphoreTake(u->lock, portMAX_DELAY) != pdPASS)
#define UART_MUTEX_UNLOCK(u)  xSemaphoreGive(u->lock)

static uart_t _uart_bus_array[3] = {
    {(volatile uart_dev_t *)(DR_REG_UART_BASE), NULL, 0, NULL},
    {(volatile uart_dev_t *)(DR_REG_UART1_BASE), NULL, 1, NULL},
    {(volatile uart_dev_t *)(DR_REG_UART2_BASE), NULL, 2, NULL}
};
#endif

static const DRAM_ATTR uint16_t RX_BUFFER_SIZE_MASK = RX_BUFFER_SIZE - 1;
static const DRAM_ATTR uint8_t ESP_CMD_TOOL_ACK = CMD_TOOL_ACK;

static uart_t *uart1 = NULL;
static stream_rx_buffer_t rxbuffer = {0};

#if SERIAL2_ENABLE
static uart_t *uart2 = NULL;
static stream_rx_buffer_t rxbuffer2 = {0};
#endif

IRAM_ATTR static void _uart1_isr (void *arg)
{
    uint8_t c;

    uart1->dev->int_clr.rxfifo_full = 1;
    uart1->dev->int_clr.frm_err = 1;
    uart1->dev->int_clr.rxfifo_tout = 1;

    while(uart1->dev->status.rxfifo_cnt || (uart1->dev->mem_rx_status.wr_addr != uart1->dev->mem_rx_status.rd_addr)) {

        c = uart1->dev->fifo.rw_byte;

        if(c == ESP_CMD_TOOL_ACK && !rxbuffer.backup) {
	        stream_rx_backup(&rxbuffer);
            hal.stream.read = serialRead; // restore normal input

        } else if(!hal.stream.enqueue_realtime_command(c)) {

            uint32_t bptr = (rxbuffer.head + 1) & RX_BUFFER_SIZE_MASK;  // Get next head pointer

            if(bptr == rxbuffer.tail)                   // If buffer full
                rxbuffer.overflow = 1;                  // flag overflow,
            else {
                rxbuffer.data[rxbuffer.head] = (char)c; // else add data to buffer
                rxbuffer.head = bptr;                   // and update pointer
            }
        }
    }
}

static void uartEnableInterrupt (uart_t *uart, uart_isr_ptr isr, bool enable_rx)
{
    UART_MUTEX_LOCK(uart);

    esp_intr_alloc(UART_INTR_SOURCE(uart->num), (int)ESP_INTR_FLAG_IRAM, isr, NULL, &uart->intr_handle);

    uart->dev->conf1.rxfifo_full_thrhd = 112;
    uart->dev->conf1.rx_tout_thrhd = 2;
    uart->dev->conf1.rx_tout_en = 1;
    uart->dev->int_ena.rxfifo_full = enable_rx;
    uart->dev->int_ena.frm_err = enable_rx;
    uart->dev->int_ena.rxfifo_tout = enable_rx;
    uart->dev->int_clr.val = 0xffffffff;

    UART_MUTEX_UNLOCK(uart);
}
/*
static void uartDisableInterrupt (uart_t *uart)
{
    UART_MUTEX_LOCK();
    rx_uart->dev->conf1.val = 0;
    rx_uart->dev->int_ena.val = 0;
    rx_uart->dev->int_clr.val = 0xffffffff;

    esp_intr_free(rx_uart->intr_handle);
    rx_uart->intr_handle = NULL;

    UART_MUTEX_UNLOCK();
}
*/
static void uartSetBaudRate (uart_t *uart, uint32_t baud_rate)
{
    if(uart == NULL)
        return;

    UART_MUTEX_LOCK(uart);
    uint32_t clk_div = ((UART_CLK_FREQ << 4) / baud_rate);
    uart->dev->clk_div.div_int = clk_div >> 4 ;
    uart->dev->clk_div.div_frag = clk_div & 0xf;
    UART_MUTEX_UNLOCK(uart);
}

static void uartConfig (uart_t *uart, uint32_t baud_rate)
{
#if !CONFIG_DISABLE_HAL_LOCKS
    if(uart->lock == NULL) {
        uart->lock = xSemaphoreCreateMutex();
        if(uart->lock == NULL)
            return;
    }
#endif

    switch(uart->num) {

        case 0:
            DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_UART_CLK_EN);
            DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART_RST);
            break;

        case 1:
            DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_UART1_CLK_EN);
            DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART1_RST);
            break;

        case 2:
            DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_UART2_CLK_EN);
            DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART2_RST);
            break;
    }

    uartSetBaudRate(uart, baud_rate);

    UART_MUTEX_LOCK(uart);
    uart->dev->conf0.val = SERIAL_8N2;

    if(uart->dev->conf0.stop_bit_num == TWO_STOP_BITS_CONF) {
        uart->dev->conf0.stop_bit_num = ONE_STOP_BITS_CONF;
        uart->dev->rs485_conf.dl1_en = 1;
    }

    // Note: UART0 pin mappings are set at boot, no need to set here unless override is required

#if SERIAL2_ENABLE
    if(uart->num == 1)
  #if MODBUS_ENABLE
        uart_set_pin(uart->num, UART2_TX_PIN, UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  #else
        uart_set_pin(uart->num, UART_PIN_NO_CHANGE, UART2_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  #endif
#endif

    UART_MUTEX_UNLOCK(uart);
}

IRAM_ATTR static void flush (uart_t *uart)
{
    UART_MUTEX_LOCK(uart);

    while(uart->dev->status.txfifo_cnt);

    //Due to hardware issue, we can not use fifo_rst to reset uart fifo.
    //See description about UART_TXFIFO_RST and UART_RXFIFO_RST in <<esp32_technical_reference_manual>> v2.6 or later.

    // we read the data out and make `fifo_len == 0 && rd_addr == wr_addr`.
    while(uart->dev->status.rxfifo_cnt || (uart->dev->mem_rx_status.wr_addr != uart->dev->mem_rx_status.rd_addr))
        READ_PERI_REG(UART_FIFO_REG(uart->num));

    UART_MUTEX_UNLOCK(uart);
}

void serialInit (void)
{
    uart1 = &_uart_bus_array[0]; // use UART 0

    uartConfig(uart1, BAUD_RATE);

    serialFlush();
    uartEnableInterrupt(uart1, _uart1_isr, true);
}

uint32_t serialAvailable (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t serialRXFree (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint32_t serialAvailableForWrite (void)
{
    return uart1 ? 0x7f - uart1->dev->status.txfifo_cnt : 0;
}

int16_t serialRead (void)
{
    UART_MUTEX_LOCK(uart1);
    int16_t data;
    uint16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head) {
        UART_MUTEX_UNLOCK(uart1);
        return -1; // no data available else EOF
    }
    data = rxbuffer.data[bptr++];                 // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);  // and update pointer
    UART_MUTEX_UNLOCK(uart1);

    return data;
}

bool serialPutC (const char c)
{
    UART_MUTEX_LOCK(uart1);

    while(uart1->dev->status.txfifo_cnt == 0x7F) {
        if(!hal.stream_blocking_callback())
            return false;
    }

    uart1->dev->fifo.rw_byte = c;
    UART_MUTEX_UNLOCK(uart1);

    return true;
}

void serialWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        serialPutC(c);
}

IRAM_ATTR void serialFlush (void)
{
    flush(uart1);

    rxbuffer.tail = rxbuffer.head;
}

IRAM_ATTR void serialCancel (void)
{
//    UART_MUTEX_LOCK(uart1);
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
//    UART_MUTEX_UNLOCK(uart1);
}

IRAM_ATTR bool serialSuspendInput (bool suspend)
{
    bool ok;
    UART_MUTEX_LOCK(uart1);
    ok = stream_rx_suspend(&rxbuffer, suspend);
    UART_MUTEX_UNLOCK(uart1);

    return ok;
}

#if SERIAL2_ENABLE

static void IRAM_ATTR _uart2_isr (void *arg)
{
    uint8_t c;

    uart2->dev->int_clr.rxfifo_full = 1;
    uart2->dev->int_clr.frm_err = 1;
    uart2->dev->int_clr.rxfifo_tout = 1;

#if MODBUS_ENABLE && defined(MODBUS_DIRECTION_PIN)
    if(uart2->dev->int_st.tx_done) {
    //    uart2->dev->int_clr.tx_done = 1;
        uart2->dev->int_ena.tx_done = 0;
        gpio_set_level(MODBUS_DIRECTION_PIN, false);
    }
#endif

    while(uart2->dev->status.rxfifo_cnt || (uart2->dev->mem_rx_status.wr_addr != uart2->dev->mem_rx_status.rd_addr)) {

        c = uart2->dev->fifo.rw_byte;

#if MODBUS_ENABLE
        uint32_t bptr = (rxbuffer2.head + 1) & RX_BUFFER_SIZE_MASK;  // Get next head pointer

        if(bptr == rxbuffer2.tail)                    // If buffer full
            rxbuffer2.overflow = 1;                   // flag overflow,
        else {
            rxbuffer2.data[rxbuffer2.head] = (char)c; // else add data to buffer
            rxbuffer2.head = bptr;                    // and update pointer
        }
#else
        if(c == ESP_CMD_TOOL_ACK && !rxbuffer.backup) {
            stream_rx_backup(&rxbuffer2);
            hal.stream.read = serial2Read; // restore normal input
        } else if(!hal.stream.enqueue_realtime_command(c)) {

            uint32_t bptr = (rxbuffer2.head + 1) & RX_BUFFER_SIZE_MASK;  // Get next head pointer

            if(bptr == rxbuffer2.tail)                    // If buffer full
                rxbuffer2.overflow = 1;                   // flag overflow,
            else {
                rxbuffer2.data[rxbuffer2.head] = (char)c; // else add data to buffer
                rxbuffer2.head = bptr;                    // and update pointer
            }
        }
#endif
    }

/*
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
    */
}

#if MODBUS_ENABLE && defined(MODBUS_DIRECTION_PIN)

IRAM_ATTR void serial2Direction(bool tx)
{
    gpio_set_level(MODBUS_DIRECTION_PIN, tx);
}

#else

IRAM_ATTR void serialSelect(bool mpg_mode)
{
    uart_t *uart_on = mpg_mode ? uart2 : uart1;
    uart_t *uart_off = mpg_mode ? uart1 : uart2;

    // Disable interrupts
    uart_off->dev->int_ena.rxfifo_full = 0;
    uart_off->dev->int_ena.frm_err = 0;
    uart_off->dev->int_ena.rxfifo_tout = 0;

    flush(uart_on);

    // Clear and enable interrupts
    uart_on->dev->int_clr.rxfifo_full = 1;
    uart_on->dev->int_clr.frm_err = 1;
    uart_on->dev->int_clr.rxfifo_tout = 1;
    uart_on->dev->int_ena.rxfifo_full = 1;
    uart_on->dev->int_ena.frm_err = 1;
    uart_on->dev->int_ena.rxfifo_tout = 1;
}

#endif

void serial2Init (uint32_t baud_rate)
{
    uart2 = &_uart_bus_array[1]; // use UART 1

    uartConfig(uart2, baud_rate);

    serial2Flush();
#if MODBUS_ENABLE
    uartEnableInterrupt(uart2, _uart2_isr, true);
  #ifdef MODBUS_DIRECTION_PIN
    gpio_config_t gpioConfig = {
        .pin_bit_mask = (1UL << MODBUS_DIRECTION_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&gpioConfig);
    serial2Direction(false);
  #endif
#else
    uartEnableInterrupt(uart2, _uart2_isr, false);
#endif
}

bool serial2SetBaudRate (uint32_t baud_rate)
{
    static bool init_ok = false;

    if(!init_ok) {
        serial2Init(baud_rate);
        init_ok = true;
    }

    uartSetBaudRate(uart2, baud_rate);

    return true;
}

uint16_t serial2Available (void)
{
    uint16_t head = rxbuffer2.head, tail = rxbuffer2.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t serial2txCount (void)
{
    return (uint16_t)uart2->dev->status.txfifo_cnt + (uart2->dev->int_raw.tx_done ? 0 : 1);
}

uint16_t serial2RXFree (void)
{
    uint16_t head = rxbuffer2.head, tail = rxbuffer2.tail;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

bool serial2PutC (const char c)
{
    UART_MUTEX_LOCK(uart2);

    while(uart2->dev->status.txfifo_cnt == 0x7F);

    uart2->dev->fifo.rw_byte = c;
    UART_MUTEX_UNLOCK(uart2);

    return true;
}

// Writes a number of characters from a buffer to the serial output stream, blocks if buffer full
//
void serial2Write (const char *s, uint16_t length)
{
    char *ptr = (char *)s;

#if MODBUS_ENABLE && defined(MODBUS_DIRECTION_PIN)
    gpio_set_level(MODBUS_DIRECTION_PIN, true);
#endif

    while(length--)
        serial2PutC(*ptr++);

#if MODBUS_ENABLE && defined(MODBUS_DIRECTION_PIN)
    uart2->dev->int_clr.tx_done = 1;
    uart2->dev->int_ena.tx_done = 1;
#endif
}


int16_t serial2Read (void)
{
    UART_MUTEX_LOCK(uart2);
    int16_t data;
    uint16_t bptr = rxbuffer2.tail;

    if(bptr == rxbuffer2.head) {
        UART_MUTEX_UNLOCK(uart2);
        return -1; // no data available else EOF
    }

    data = rxbuffer2.data[bptr++];                 // Get next character, increment tmp pointer
    rxbuffer2.tail = bptr & (RX_BUFFER_SIZE - 1);  // and update pointer
    UART_MUTEX_UNLOCK(uart2);

    return data;
}

IRAM_ATTR void serial2Flush (void)
{
    flush(uart2);

    rxbuffer2.tail = rxbuffer2.head;
}

IRAM_ATTR void serial2Cancel (void)
{
//    UART_MUTEX_LOCK(uart2);
    rxbuffer2.data[rxbuffer2.head] = ASCII_CAN;
    rxbuffer2.tail = rxbuffer2.head;
    rxbuffer2.head = (rxbuffer2.tail + 1) & (RX_BUFFER_SIZE - 1);
//    UART_MUTEX_UNLOCK(uart2);
}

bool serial2SuspendInput (bool suspend)
{
    bool ok;
    UART_MUTEX_LOCK(uart2);
    ok = stream_rx_suspend(&rxbuffer2, suspend);
    UART_MUTEX_UNLOCK(uart2);

    return ok;
}

#endif
