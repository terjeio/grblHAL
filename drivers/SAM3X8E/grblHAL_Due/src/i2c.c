/*
  i2c.c - I2C interface

  Driver code for Atmel SAM3X8E ARM processor

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

#include "driver.h"

#if I2C_ENABLE

#include <Arduino.h>

#include "sam.h"
#include "variant.h"
#include "wiring_private.h"

#include "i2c.h"
#include "serial.h"

#define i2cIsBusy (!(i2c.state == I2CState_Idle || i2c.state == I2CState_Error) || !(I2C_PERIPH->TWI_SR & TWI_SR_TXCOMP))

typedef enum {
    I2CState_Idle = 0,
    I2CState_SendNext,
    I2CState_SendLast,
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

static i2c_trans_t i2c;

static void I2C_interrupt_handler (void);

void i2c_init (void)
{
    static bool init_ok = false;

    if(!init_ok) {

        init_ok = true;

        pmc_enable_periph_clk(I2C_ID);
        pmc_enable_periph_clk(I2C_PERIPH == TWI0 ? ID_PIOA : ID_PIOB);

        I2C_PORT->PIO_PDR = I2C_SDA_BIT|I2C_SCL_BIT;
        if(I2C_PERIPH == TWI0)
            I2C_PORT->PIO_ABSR &= ~(I2C_SDA_BIT|I2C_SCL_BIT);
        else
            I2C_PORT->PIO_ABSR |= (I2C_SDA_BIT|I2C_SCL_BIT);

        I2C_PERIPH->TWI_CR = TWI_CR_SWRST;
        I2C_PERIPH->TWI_RHR;

        hal.delay_ms(10, NULL);        

        I2C_PERIPH->TWI_CR = TWI_CR_SVDIS|TWI_CR_MSEN;

        TWI_SetClock(I2C_PERIPH, I2C_CLOCK, SystemCoreClock);

        IRQRegister(I2C_IRQ, I2C_interrupt_handler);

        NVIC_SetPriority(I2C_IRQ, 0);
        NVIC_EnableIRQ(I2C_IRQ);
    }
}

// get bytes (max 8), waits for result
uint8_t *I2C_Receive (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes, bool block)
{
    i2c.data  = buf ? buf : i2c.buffer;
    i2c.count = bytes;
    i2c.state = bytes == 1 ? I2CState_ReceiveLast : (bytes == 2 ? I2CState_ReceiveNextToLast : I2CState_ReceiveNext);

    // Send start and address
    I2C_PERIPH->TWI_MMR = TWI_MMR_DADR(i2cAddr)|TWI_MMR_MREAD;
    I2C_PERIPH->TWI_CR = bytes == 1 ? TWI_CR_START : TWI_CR_START|TWI_CR_STOP; // Start condition +  stop condition if reading one byte
    I2C_PERIPH->TWI_IER = TWI_IER_TXRDY|TWI_IER_RXRDY;

    if(block)
        while(i2cIsBusy);

    return i2c.buffer;
}

void I2C_Send (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes, bool block)
{
    i2c.count = bytes;
    i2c.data  = buf ? buf : i2c.buffer;
    i2c.state = bytes == 0 ? I2CState_AwaitCompletion : (bytes == 1 ? I2CState_SendLast : I2CState_SendNext);

    I2C_PERIPH->TWI_MMR = TWI_MMR_DADR(i2cAddr);
    I2C_PERIPH->TWI_THR = *i2c.data++;
    I2C_PERIPH->TWI_IER = TWI_IER_TXRDY|TWI_IER_NACK;

    if(block)
        while(i2cIsBusy);
}

uint8_t *I2C_ReadRegister (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes, bool block)
{
    while(i2cIsBusy);

    i2c.count = bytes;
    i2c.data  = buf ? buf : i2c.buffer;
    i2c.state = bytes == 1 ? I2CState_ReceiveLast : (bytes == 2 ? I2CState_ReceiveNextToLast : I2CState_ReceiveNext);

    I2C_PERIPH->TWI_MMR = TWI_MMR_DADR(i2cAddr)|TWI_MMR_MREAD|TWI_MMR_IADRSZ_1_BYTE;
    I2C_PERIPH->TWI_IADR = *i2c.data;
    I2C_PERIPH->TWI_CR = bytes == 1 ? TWI_CR_START|TWI_CR_STOP : TWI_CR_START; // Start condition + stop condition if reading one byte
    I2C_PERIPH->TWI_IER = TWI_IER_RXRDY;

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
        I2C_Send(transfer->address, txbuf, transfer->count, true);
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

#if TRINAMIC_ENABLE == 2130 && TRINAMIC_I2C

static TMC2130_status_t I2C_TMC_ReadRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    uint8_t *res, i2creg;
    TMC2130_status_t status = {0};
;
    if((i2creg = TMCI2C_GetMapAddress((uint8_t)(driver ? (uint32_t)driver->axis : 0), reg->addr).value) == 0xFF)
        return status; // unsupported register

    while(i2cIsBusy);

    i2c.buffer[0] = i2creg;
    i2c.buffer[1] = 0;
    i2c.buffer[2] = 0;
    i2c.buffer[3] = 0;
    i2c.buffer[4] = 0;

    res = I2C_ReadRegister(I2C_ADR_I2CBRIDGE, NULL, 5, true);

    status.value = (uint8_t)*res++;
    reg->payload.value = ((uint8_t)*res++ << 24);
    reg->payload.value |= ((uint8_t)*res++ << 16);
    reg->payload.value |= ((uint8_t)*res++ << 8);
    reg->payload.value |= (uint8_t)*res++;

    return status;
}

static TMC2130_status_t I2C_TMC_WriteRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    TMC2130_status_t status = {0};

    while(i2cIsBusy);

    reg->addr.write = 1;
    i2c.buffer[0] = TMCI2C_GetMapAddress((uint8_t)(driver ? (uint32_t)driver->axis : 0), reg->addr).value;
    reg->addr.write = 0;

    if(i2c.buffer[0] == 0xFF)
        return status; // unsupported register

    i2c.buffer[1] = (reg->payload.value >> 24) & 0xFF;
    i2c.buffer[2] = (reg->payload.value >> 16) & 0xFF;
    i2c.buffer[3] = (reg->payload.value >> 8) & 0xFF;
    i2c.buffer[4] = reg->payload.value & 0xFF;

    I2C_Send(I2C_ADR_I2CBRIDGE, NULL, 5, true);

    return status;
}

void I2C_DriverInit (TMC_io_driver_t *driver)
{
    i2c_init();
    driver->WriteRegister = I2C_TMC_WriteRegister;
    driver->ReadRegister = I2C_TMC_ReadRegister;
}

#endif

static void I2C_interrupt_handler (void)
{
    uint8_t ifg = I2C_PERIPH->TWI_SR;

    if(ifg & TWI_SR_ARBLST) {
        I2C_PERIPH->TWI_CR = TWI_CR_STOP; // Stop condition
        i2c.state = I2CState_Error;
    }

    if(ifg & TWI_SR_NACK)
        i2c.state = I2CState_Error;

    switch(i2c.state) {

        case I2CState_Idle:
        case I2CState_Error:
            I2C_PERIPH->TWI_IDR = TWI_IDR_TXRDY|TWI_IDR_RXRDY|TWI_IDR_NACK;
            break;

        case I2CState_SendNext:
            I2C_PERIPH->TWI_THR = *i2c.data++;
            if(--i2c.count == 1)
                i2c.state = I2CState_SendLast;
            break;

        case I2CState_SendLast:
            I2C_PERIPH->TWI_THR = *i2c.data;
            I2C_PERIPH->TWI_CR = TWI_CR_STOP; // Stop condition
            i2c.state = I2CState_AwaitCompletion;
            break;

        case I2CState_AwaitCompletion:
            I2C_PERIPH->TWI_IDR = TWI_IDR_TXRDY|TWI_IDR_RXRDY|TWI_IDR_NACK;
            i2c.count = 0;
            i2c.state = I2CState_Idle;
            break;

        case I2CState_ReceiveNext:
            *i2c.data++ = I2C_PERIPH->TWI_RHR;
            if(--i2c.count == 2)
                i2c.state = I2CState_ReceiveNextToLast;
            break;

        case I2CState_ReceiveNextToLast:
            *i2c.data++ = I2C_PERIPH->TWI_RHR;
            I2C_PERIPH->TWI_CR = TWI_CR_STOP;
            i2c.count--;
            i2c.state = I2CState_ReceiveLast;
            break;

        case I2CState_ReceiveLast:
            I2C_PERIPH->TWI_IDR = TWI_IDR_TXRDY|TWI_IDR_RXRDY|TWI_IDR_NACK;
            *i2c.data = I2C_PERIPH->TWI_RHR;
            i2c.count = 0;
            i2c.state = I2CState_Idle;
          #if KEYPAD_ENABLE
            if(i2c.keycode_callback) {
                i2c.keycode_callback(*i2c.data);
                i2c.keycode_callback = NULL;
            }
          #endif
            break;
    }
}

#endif
