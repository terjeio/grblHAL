/*
  i2c.c - I2C interface

  Driver code for Atmel SAMD21 ARM processor

  Part of grblHAL

  Copyright (c) 2018-2020 Terje Io

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

#include "driver.h"

#ifdef I2C_PORT

#include <Arduino.h>

#include "sam.h"
#include "variant.h"
#include "wiring_private.h"

#include "i2c.h"
#include "serial.h"

#if TRINAMIC_ENABLE && TRINAMIC_I2C
#define I2C_ADR_I2CBRIDGE 0x47
#endif

#define i2cIsBusy (!(i2c.state == I2CState_Idle || i2c.state == I2CState_Error) || !(i2c_port->I2CM.STATUS.bit.BUSSTATE == 0x01 || i2c_port->I2CM.STATUS.bit.BUSSTATE == 0x02))

typedef enum
{
    I2C_SLAVE_OPERATION = 0x4u,
    I2C_MASTER_OPERATION = 0x5u
} SercomI2CMode;

static Sercom *i2c_port = SERCOM2; // Alt mode C

typedef enum {
    I2CState_Idle = 0,
    I2CState_SendNext,
    I2CState_SendLast,
    I2CState_SendRegisterAddress,
    I2CState_Restart,
    I2CState_AwaitCompletion,
    I2CState_ReceiveNext,
    I2CState_ReceiveNextToLast,
    I2CState_ReceiveLast,
    I2CState_Error
} i2c_state_t;

typedef struct {
    volatile i2c_state_t state;
    uint8_t addr;
    uint16_t count;
    uint8_t *data;
#if KEYPAD_ENABLE
    keycode_callback_ptr keycode_callback;
#endif
    uint8_t buffer[8];
} i2c_trans_t;

static i2c_trans_t i2c = {0};

static void I2C_interrupt_handler (void);

#define WIRE_RISE_TIME_NANOSECONDS 125

// get bytes (max 8), waits for result
uint8_t *I2C_Receive (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes, bool block)
{
    i2c.data  = buf ? buf : i2c.buffer;
    i2c.count = bytes;
    i2c.state = bytes == 1 ? I2CState_ReceiveLast : (bytes == 2 ? I2CState_ReceiveNextToLast : I2CState_ReceiveNext);

    // Send start and address
    i2c_port->I2CM.ADDR.bit.ADDR = (i2cAddr << 1) | 0x01;
    while(i2c_port->I2CM.SYNCBUSY.bit.SYSOP);

    if(block)
        while(i2cIsBusy);

    return i2c.buffer;
}

void I2C_Send (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes, bool block)
{
//    while(i2cIsBusy);

    i2c.count = bytes;
    i2c.data  = buf ? buf : i2c.buffer;
    i2c.state = bytes == 0 ? I2CState_AwaitCompletion : (bytes == 1 ? I2CState_SendLast : I2CState_SendNext);

    i2c_port->I2CM.ADDR.bit.ADDR = i2cAddr << 1;
    while(i2c_port->I2CM.SYNCBUSY.bit.SYSOP);

    if(block)
        while(i2cIsBusy);
}

uint8_t *I2C_ReadRegister (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes, bool block)
{
    while(i2cIsBusy);

    i2c.count = bytes;
    i2c.data  = buf ? buf : i2c.buffer;
    i2c.state = I2CState_SendRegisterAddress;

    i2c_port->I2CM.ADDR.bit.ADDR = i2cAddr << 1;
    while(i2c_port->I2CM.SYNCBUSY.bit.SYSOP);

    if(block)
        while(i2cIsBusy);

    return i2c.buffer;
}

#if EEPROM_ENABLE

nvs_transfer_result_t i2c_nvs_transfer (nvs_transfer_t *transfer, bool read)
{
    static uint8_t txbuf[34];

    while(i2cIsBusy);

    if(read) {
        transfer->data[0] = transfer->word_addr; // !!
        I2C_ReadRegister(transfer->address, transfer->data, transfer->count, true);
    } else {
        memcpy(&txbuf[1], transfer->data, transfer->count);
        txbuf[0] = transfer->word_addr;
        I2C_Send(transfer->address, txbuf, transfer->count + 1, true);
#if !EEPROM_IS_FRAM
        hal.delay_ms(5, NULL);
#endif
    }

    return NVS_TransferResult_OK;
}

#endif

#if KEYPAD_ENABLE

void I2C_GetKeycode (uint32_t i2cAddr, keycode_callback_ptr callback)
{
    while(i2cIsBusy);

    i2c.keycode_callback = callback;

    I2C_Receive(i2cAddr, NULL, 1, false);
}

#endif

#if TRINAMIC_ENABLE && TRINAMIC_I2C

static uint8_t axis = 0xFF;

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    uint8_t *res;
    TMC_spi_status_t status = 0;

    if(driver.axis != axis) {
        i2c.buffer[0] = driver.axis;
        I2C_Send(I2C_ADR_I2CBRIDGE, NULL, 1, true);

        axis = driver.axis;
    }

    memset(i2c.buffer, 0, sizeof(i2c.buffer));

    while(i2cIsBusy);

    i2c.buffer[0] = datagram->addr.idx;
    i2c.buffer[1] = 0;
    i2c.buffer[2] = 0;
    i2c.buffer[3] = 0;
    i2c.buffer[4] = 0;

    res = I2C_ReadRegister(I2C_ADR_I2CBRIDGE, NULL, 5, true);

    status = (uint8_t)*res++;
    datagram->payload.value = ((uint8_t)*res++ << 24);
    datagram->payload.value |= ((uint8_t)*res++ << 16);
    datagram->payload.value |= ((uint8_t)*res++ << 8);
    datagram->payload.value |= (uint8_t)*res++;

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    uint8_t *res;
    TMC_spi_status_t status = 0;

    if(driver.axis != axis) {
        i2c.buffer[0] = driver.axis;
        I2C_Send(I2C_ADR_I2CBRIDGE, NULL, 1, true);

        axis = driver.axis;
    }

    datagram->addr.write = On;
    i2c.buffer[0] = datagram->addr.value;
    i2c.buffer[1] = (datagram->payload.value >> 24) & 0xFF;
    i2c.buffer[2] = (datagram->payload.value >> 16) & 0xFF;
    i2c.buffer[3] = (datagram->payload.value >> 8) & 0xFF;
    i2c.buffer[4] = datagram->payload.value & 0xFF;
    datagram->addr.write = Off;

    I2C_Send(I2C_ADR_I2CBRIDGE, NULL, 5, true);

    return status;
}

#endif

void i2c_init (void)
{
    static bool init_ok = false;

    if(!init_ok) {

        init_ok = true;

        pinPeripheral(I2C_SDA_PIN, g_APinDescription[I2C_SDA_PIN].ulPinType); // PIO_SERCOM
        pinPeripheral(I2C_SCL_PIN, g_APinDescription[I2C_SCL_PIN].ulPinType);

        initSerClockNVIC(i2c_port);

        NVIC_SetPriority(SERCOM2_IRQn, 0);
        IRQRegister(SERCOM2_IRQn, I2C_interrupt_handler);

        /* Enable the peripherals used to drive the SDC on SSI */

        i2c_port->I2CM.CTRLA.bit.ENABLE = 0;
        while(i2c_port->I2CM.SYNCBUSY.bit.ENABLE);

        i2c_port->I2CM.CTRLA.bit.SWRST = 1;
        //Wait both bits Software Reset from CTRLA and SYNCBUSY are equal to 0
        while(i2c_port->I2CM.CTRLA.bit.SWRST || i2c_port->I2CM.SYNCBUSY.bit.SWRST);

        i2c_port->I2CM.CTRLA.reg = SERCOM_I2CM_CTRLA_MODE(I2C_MASTER_OPERATION) /*| SERCOM_I2CM_CTRLA_SCLSM*/ ;

        // Enable Smart mode and Quick Command
        //i2c_port->I2CM.CTRLB.reg =  SERCOM_I2CM_CTRLB_SMEN /*| SERCOM_I2CM_CTRLB_QCEN*/ ;

        // Enable all interrupts
        i2c_port->I2CM.INTENSET.reg = SERCOM_I2CM_INTENSET_MB | SERCOM_I2CM_INTENSET_SB | SERCOM_I2CM_INTENSET_ERROR ;

        // Synchronous arithmetic baudrate
        i2c_port->I2CM.BAUD.bit.BAUD = SystemCoreClock / (2 * I2C_CLOCK) - 5 - (((SystemCoreClock / 1000000) * WIRE_RISE_TIME_NANOSECONDS) / (2 * 1000));

        i2c_port->I2CM.CTRLA.bit.ENABLE = 1;
        while(i2c_port->I2CM.SYNCBUSY.bit.ENABLE);

        // Setting bus idle mode
        i2c_port->I2CM.STATUS.bit.BUSSTATE = 1;
        while(i2c_port->I2CM.SYNCBUSY.bit.SYSOP);
    }
}

static void I2C_interrupt_handler (void)
{
    uint8_t ifg = i2c_port->I2CM.INTFLAG.reg;

    i2c_port->I2CM.INTFLAG.reg = ifg;

    if(i2c_port->I2CM.INTFLAG.bit.SB && i2c_port->I2CM.INTFLAG.bit.MB) {
        i2c_port->I2CM.CTRLB.bit.CMD = 3; // Stop condition
        while(i2c_port->I2CM.SYNCBUSY.bit.SYSOP);
        i2c.state = I2CState_Error;
    }

  //ACK received (0: ACK, 1: NACK)
//  if(i2c_port->I2CM.STATUS.bit.RXNACK

    switch(i2c.state) {

        case I2CState_Idle:
        case I2CState_Error:
            break;

        case I2CState_SendNext:
            i2c_port->I2CM.DATA.reg = *i2c.data++;
            while(i2c_port->I2CM.SYNCBUSY.bit.SYSOP);
            if(--i2c.count == 1)
                i2c.state = I2CState_SendLast;
            break;

        case I2CState_SendLast:
            i2c_port->I2CM.DATA.reg = *i2c.data;
            while(i2c_port->I2CM.SYNCBUSY.bit.SYSOP);
            i2c.state = I2CState_AwaitCompletion;
            break;

        case I2CState_AwaitCompletion:
            //i2c_port->I2CM.CTRLB.bit.ACKACT = 1;
            i2c_port->I2CM.CTRLB.bit.CMD = 3; // Stop condition
            while(i2c_port->I2CM.SYNCBUSY.bit.SYSOP);
            i2c.count = 0;
            i2c.state = I2CState_Idle;
            break;

        case I2CState_SendRegisterAddress:
            i2c_port->I2CM.DATA.reg = *i2c.data;
            while(i2c_port->I2CM.SYNCBUSY.bit.SYSOP);
            i2c.state = I2CState_Restart;
            break;

        case I2CState_Restart:
            i2c_port->I2CM.ADDR.reg |= 0x01;
            i2c_port->I2CM.CTRLB.bit.CMD = 0x1;
            while(i2c_port->I2CM.SYNCBUSY.bit.SYSOP);
            i2c.state = i2c.count == 1 ? I2CState_ReceiveLast : (i2c.count == 2 ? I2CState_ReceiveNextToLast : I2CState_ReceiveNext);
            break;

        case I2CState_ReceiveNext:
            *i2c.data++ = i2c_port->I2CM.DATA.reg;
            i2c_port->I2CM.CTRLB.bit.ACKACT = 0;
            i2c_port->I2CM.CTRLB.bit.CMD = 0x2;
            while(i2c_port->I2CM.SYNCBUSY.bit.SYSOP);
            if(--i2c.count == 2)
                i2c.state = I2CState_ReceiveNextToLast;
            break;

        case I2CState_ReceiveNextToLast:
            *i2c.data++ = i2c_port->I2CM.DATA.reg;
            i2c_port->I2CM.CTRLB.bit.ACKACT = 0;
            i2c_port->I2CM.CTRLB.bit.CMD = 0x2;
            while(i2c_port->I2CM.SYNCBUSY.bit.SYSOP);
            i2c.count--;
            i2c.state = I2CState_ReceiveLast;
            break;

        case I2CState_ReceiveLast:
            *i2c.data = i2c_port->I2CM.DATA.reg;
            i2c_port->I2CM.CTRLB.bit.ACKACT = 1;
            i2c_port->I2CM.CTRLB.bit.CMD = 3; // Stop condition
            while(i2c_port->I2CM.SYNCBUSY.bit.SYSOP);
            i2c.count = 0;
            i2c.state = I2CState_Idle;
          #if KEYPAD_ENABLE
            if(i2c.keycode_callback) {
                //  if(GPIOIntStatus(KEYINTR_PORT, KEYINTR_PIN) != 0) { // only add keycode when key is still pressed
                i2c.keycode_callback(*i2c.data);
                i2c.keycode_callback = NULL;
            }
          #endif
            break;
    }
}

#endif
