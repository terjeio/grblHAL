// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
// Copyright 2018 Terje Io : Modifications for grbl
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

/*
TX0 - GPIO 01
RX0 - GPIO 03

perhaps use UART_PIN_NO_CHANGE since the uart is already configured maybe there is no need to use a pin number.

Bug in queue - LF & CR get eaten most of the time, or is the OS interfering in our business?

 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "rom/ets_sys.h"
#include "esp_attr.h"
#include "esp_intr.h"
#include "rom/uart.h"
#include "soc/uart_reg.h"
#include "soc/uart_struct.h"
#include "soc/io_mux_reg.h"
#include "soc/gpio_sig_map.h"
#include "soc/dport_reg.h"
#include "esp_intr_alloc.h"

#include "serial.h"
#include "esp32-hal-uart.h"
#include "GRBL/grbl.h"

#define UART_REG_BASE(u)    ((u==0)?DR_REG_UART_BASE:(      (u==1)?DR_REG_UART1_BASE:(    (u==2)?DR_REG_UART2_BASE:0)))
#define UART_RXD_IDX(u)     ((u==0)?U0RXD_IN_IDX:(          (u==1)?U1RXD_IN_IDX:(         (u==2)?U2RXD_IN_IDX:0)))
#define UART_TXD_IDX(u)     ((u==0)?U0TXD_OUT_IDX:(         (u==1)?U1TXD_OUT_IDX:(        (u==2)?U2TXD_OUT_IDX:0)))
#define UART_INTR_SOURCE(u) ((u==0)?ETS_UART0_INTR_SOURCE:( (u==1)?ETS_UART1_INTR_SOURCE:((u==2)?ETS_UART2_INTR_SOURCE:0)))

typedef struct uart_struct_t uart_t;

static serial_rx_buffer_t rxbuffer = {
	.head = 0,
	.tail = 0,
	.backup = false,
	.overflow = false,
	.rts_state = false
};

static serial_rx_buffer_t rxbackup;

struct uart_struct_t {
    uart_dev_t * dev;
#if !CONFIG_DISABLE_HAL_LOCKS
    xSemaphoreHandle lock;
#endif
    uint8_t num;
    intr_handle_t intr_handle;
};

#if CONFIG_DISABLE_HAL_LOCKS
#define UART_MUTEX_LOCK()
#define UART_MUTEX_UNLOCK()

static uart_t _uart_bus_array[3] = {
    {(volatile uart_dev_t *)(DR_REG_UART_BASE), 0, NULL, NULL},
    {(volatile uart_dev_t *)(DR_REG_UART1_BASE), 1, NULL, NULL},
    {(volatile uart_dev_t *)(DR_REG_UART2_BASE), 2, NULL, NULL}
};
#else
#define UART_MUTEX_LOCK()    do {} while (xSemaphoreTake(port->lock, portMAX_DELAY) != pdPASS)
#define UART_MUTEX_UNLOCK()  xSemaphoreGive(port->lock)

static uart_t _uart_bus_array[3] = {
    {(volatile uart_dev_t *)(DR_REG_UART_BASE), NULL, 0, NULL, NULL},
    {(volatile uart_dev_t *)(DR_REG_UART1_BASE), NULL, 1, NULL, NULL},
    {(volatile uart_dev_t *)(DR_REG_UART2_BASE), NULL, 2, NULL, NULL}
};
#endif

static uart_t *port = NULL;
static bool (*uartReceiveCallback)(char) = 0;
static bool (*uartBlockingCallback)(void) = 0;

static bool uartBlockingCallbackDummy (void)
{
    return true;
}

void setUARTBlockingCallback (bool (*fn)(void))
{
    uartBlockingCallback = fn == 0 ? uartBlockingCallbackDummy : fn;
}

//
// uartReceiveCallback - called before data is inserted into the buffer, return false to drop
//
void setUARTReceiveCallback (bool (*fn)(char))
{
    uartReceiveCallback = fn;
}

static void IRAM_ATTR _uart_isr (void *arg)
{
    uint8_t i, c;
    uart_t* uart;

    for(i=0;i<3;i++){
        uart = &_uart_bus_array[i];
        if(uart->intr_handle == NULL){
            continue;
        }
        uart->dev->int_clr.rxfifo_full = 1;
        uart->dev->int_clr.frm_err = 1;
        uart->dev->int_clr.rxfifo_tout = 1;
        while(uart->dev->status.rxfifo_cnt || (uart->dev->mem_rx_status.wr_addr != uart->dev->mem_rx_status.rd_addr)) {

        	c = uart->dev->fifo.rw_byte;

/*        	if(c == CMD_JOG_CANCEL)
        		uartCancel();
        	else*/ if(c == CMD_TOOL_ACK && !rxbuffer.backup) {

				memcpy(&rxbackup, &rxbuffer, sizeof(serial_rx_buffer_t));
				rxbuffer.backup = true;
				rxbuffer.tail = rxbuffer.head;
				hal.serial_read = uartRead; // restore normal input

			} else if(uartReceiveCallback && uartReceiveCallback(c)) {

                uint32_t bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer

                if(bptr == rxbuffer.tail)                           // If buffer full
                    rxbuffer.overflow = 1;                          // flag overflow,
                else {
                    rxbuffer.data[rxbuffer.head] = (char)c;      // else add data to buffer
                    rxbuffer.head = bptr;                           // and update pointer
                }
            }
        }
    }
/*
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
    */
}

static void uartEnableInterrupt (uart_t* uart)
{
    UART_MUTEX_LOCK();
    uart->dev->conf1.rxfifo_full_thrhd = 112;
    uart->dev->conf1.rx_tout_thrhd = 2;
    uart->dev->conf1.rx_tout_en = 1;
    uart->dev->int_ena.rxfifo_full = 1;
    uart->dev->int_ena.frm_err = 1;
    uart->dev->int_ena.rxfifo_tout = 1;
    uart->dev->int_clr.val = 0xffffffff;

    esp_intr_alloc(UART_INTR_SOURCE(uart->num), (int)ESP_INTR_FLAG_IRAM, _uart_isr, NULL, &uart->intr_handle);
    UART_MUTEX_UNLOCK();
}

static void uartDisableInterrupt (uart_t* uart)
{
    UART_MUTEX_LOCK();
    uart->dev->conf1.val = 0;
    uart->dev->int_ena.val = 0;
    uart->dev->int_clr.val = 0xffffffff;

    esp_intr_free(uart->intr_handle);
    uart->intr_handle = NULL;

    UART_MUTEX_UNLOCK();
}

static void uartSetBaudRate (uart_t* uart, uint32_t baud_rate)
{
    if(uart == NULL)
        return;

    UART_MUTEX_LOCK();
    uint32_t clk_div = ((UART_CLK_FREQ << 4) / baud_rate);
    uart->dev->clk_div.div_int = clk_div >> 4 ;
    uart->dev->clk_div.div_frag = clk_div & 0xf;
    UART_MUTEX_UNLOCK();
}

void uartBegin(uint8_t uart_nr, uint32_t baudrate, uint32_t config, int8_t rxPin, int8_t txPin, bool inverted)
{
    port = &_uart_bus_array[uart_nr];

#if !CONFIG_DISABLE_HAL_LOCKS
    if(port->lock == NULL) {
    	port->lock = xSemaphoreCreateMutex();
        if(port->lock == NULL)
            return;
    }
#endif

    if(uart_nr == 1){
        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_UART1_CLK_EN);
        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART1_RST);
    } else if(uart_nr == 2){
        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_UART2_CLK_EN);
        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART2_RST);
    } else {
        DPORT_SET_PERI_REG_MASK(DPORT_PERIP_CLK_EN_REG, DPORT_UART_CLK_EN);
        DPORT_CLEAR_PERI_REG_MASK(DPORT_PERIP_RST_EN_REG, DPORT_UART_RST);
    }

    uartFlush();
    uartSetBaudRate(port, baudrate);

    UART_MUTEX_LOCK();
    port->dev->conf0.val = config;
    #define TWO_STOP_BITS_CONF 0x3
    #define ONE_STOP_BITS_CONF 0x1

    if ( port->dev->conf0.stop_bit_num == TWO_STOP_BITS_CONF) {
    	port->dev->conf0.stop_bit_num = ONE_STOP_BITS_CONF;
    	port->dev->rs485_conf.dl1_en = 1;
    }
    UART_MUTEX_UNLOCK();

//    pinMode(rxPin, INPUT);
//    pinMatrixInAttach(rxPin, UART_RXD_IDX(uart->num), inverted);
//    pinMode(txPin, OUTPUT);
//    pinMatrixOutAttach(txPin, UART_TXD_IDX(uart->num), inverted, false);
	uartEnableInterrupt(port);
}

void uartInit (void)
{
	uartBegin(0, 115200, SERIAL_8N2, 1, 2, false);
}

uint32_t uartAvailable (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t uartRXFree (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint32_t uartAvailableForWrite ()
{
    if(port == NULL)
        return 0;

    return 0x7f - port->dev->status.txfifo_cnt;
}

// "dummy" version of serialGetC
static int16_t uartGetNull (void)
{
    return -1;
}

int16_t uartRead (void)
{
    UART_MUTEX_LOCK();
    int16_t data;
    uint16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head) {
        UART_MUTEX_UNLOCK();
        return -1; // no data available else EOF
    }
    data = rxbuffer.data[bptr++];                 // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);  // and update pointer
    UART_MUTEX_UNLOCK();
    return data;
}

bool uartPutC (const char c)
{
    UART_MUTEX_LOCK();
    while(port->dev->status.txfifo_cnt == 0x7F);
    port->dev->fifo.rw_byte = c;
    UART_MUTEX_UNLOCK();

    return true;
}

void uartWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
    	uartPutC(c);
}

void uartFlush (void)
{
    UART_MUTEX_LOCK();
    while(port->dev->status.txfifo_cnt);

    //Due to hardware issue, we can not use fifo_rst to reset uart fifo.
    //See description about UART_TXFIFO_RST and UART_RXFIFO_RST in <<esp32_technical_reference_manual>> v2.6 or later.

    // we read the data out and make `fifo_len == 0 && rd_addr == wr_addr`.
    while(port->dev->status.rxfifo_cnt || (port->dev->mem_rx_status.wr_addr != port->dev->mem_rx_status.rd_addr))
        READ_PERI_REG(UART_FIFO_REG(port->num));

    rxbuffer.tail = rxbuffer.head;
    UART_MUTEX_UNLOCK();
}

IRAM_ATTR void uartCancel (void)
{
//    UART_MUTEX_LOCK();
    rxbuffer.data[rxbuffer.head] = CAN;
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
//    UART_MUTEX_UNLOCK();
}

bool uartSuspendInput (bool suspend)
{
    UART_MUTEX_LOCK();
    if(suspend)
        hal.serial_read = uartGetNull;
    else if(rxbuffer.backup)
        memcpy(&rxbuffer, &rxbackup, sizeof(serial_rx_buffer_t));
    UART_MUTEX_UNLOCK();
    return rxbuffer.tail != rxbuffer.head;
}

