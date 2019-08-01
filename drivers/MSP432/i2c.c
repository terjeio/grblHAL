/*
  i2c.c - I2C bridge interface for Trinamic TMC2130 stepper drivers

  For Texas Instruments SimpleLink ARM processors/LaunchPads

  Part of Grbl

  Copyright (c) 2018-2019 Terje Io

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

#include "i2c.h"

typedef enum {
    I2CState_Idle = 0,
    I2CState_SendNext,
    I2CState_SendLast,
    I2CState_SendRegisterAddress,
    I2CState_AwaitCompletion,
    I2CState_ReceiveNext,
    I2CState_ReceiveNextToLast,
    I2CState_ReceiveLast,
} i2c_state_t;

typedef struct {
    volatile i2c_state_t state;
    uint8_t count;
    uint8_t *data;
#if KEYPAD_ENABLE
    keycode_callback_ptr keycode_callback;
#endif
    uint8_t buffer[8];
} i2c_tr_trans_t;

static i2c_tr_trans_t i2c;

#define i2cIsBusy ((i2c.state != I2CState_Idle) || (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTP))

void I2CInit (void)
{
    memset(&i2c, 0, sizeof(i2c_tr_trans_t));

    P6->SEL0 |= BIT4|BIT5;                                                          // Assign I2C pins to USCI_B0

    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_SWRST;                                         // Put EUSCI_B1 in reset state
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_MODE_3|EUSCI_B_CTLW0_MST| EUSCI_B_CTLW0_SYNC;  // I2C master mode, SMCLK
    EUSCI_B1->BRW = 240;                                                            // baudrate 100 KHZ (SMCLK = 48MHz)
    EUSCI_B1->CTLW0 &=~ EUSCI_B_CTLW0_SWRST;                                        // clear reset register
//    EUSCI_B1->IE = EUSCI_B_IE_NACKIE;                                             // NACK interrupt enable
    NVIC_EnableIRQ(EUSCIB1_IRQn);       // Enable I2C interrupt and
    NVIC_SetPriority(EUSCIB1_IRQn, 3);  // set priority

}

// get bytes (max 8), waits for result
static uint8_t *I2CReceive (uint32_t i2cAddr, uint32_t bytes, bool block)
{
    while(i2cIsBusy);

    i2c.data  = i2c.buffer;
    i2c.count = bytes;
    i2c.state = bytes == 1 ? I2CState_ReceiveLast : (bytes == 2 ? I2CState_ReceiveNextToLast : I2CState_ReceiveNext);

    EUSCI_B1->I2CSA = i2cAddr;
    EUSCI_B1->CTLW0 &= ~EUSCI_B_CTLW0_TR;                       // Set read mode
    EUSCI_B1->IFG &= ~(EUSCI_B_IFG_TXIFG0|EUSCI_B_IFG_RXIFG0);  // Clear interrupt flags
    EUSCI_B1->IE |= EUSCI_B_IE_RXIE0;
    if(bytes == 1)
        EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT|EUSCI_B_CTLW0_TXSTP;
    else
        EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;

    if(block)
        while(i2cIsBusy);

    return i2c.buffer;
}

static void I2CSend (uint32_t i2cAddr, uint8_t bytes, bool block)
{
    while(i2cIsBusy);

    i2c.count = bytes;
    i2c.data  = i2c.buffer;
    i2c.state = bytes == 1 ? I2CState_SendLast : (bytes == 2 ? I2CState_SendLast : I2CState_SendNext);
    EUSCI_B1->IFG &= ~(EUSCI_B_IFG_TXIFG0|EUSCI_B_IFG_RXIFG0);          // Clear interrupt flags
    EUSCI_B1->IE |= EUSCI_B_IE_TXIE0;
    EUSCI_B1->I2CSA = i2cAddr;
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TR|EUSCI_B_CTLW0_TXSTT;

    if(block)
        while(i2cIsBusy);
}

static uint8_t *I2C_ReadRegister (uint32_t i2cAddr, uint8_t bytes, bool block)
{
    while(i2cIsBusy);

    i2c.count = bytes;
    i2c.data  = i2c.buffer;
    i2c.state = I2CState_SendRegisterAddress;
    EUSCI_B1->IFG &= ~(EUSCI_B_IFG_TXIFG0|EUSCI_B_IFG_RXIFG0);          // Clear interrupt flags
    EUSCI_B1->IE |= (EUSCI_B_IE_TXIE0|EUSCI_B_IE_RXIE0);
    EUSCI_B1->I2CSA = i2cAddr;
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TR|EUSCI_B_CTLW0_TXSTT;

    if(block)
        while(i2cIsBusy);

    return i2c.buffer;
}

#if KEYPAD_ENABLE

void I2C_GetKeycode (uint32_t i2cAddr, keycode_callback_ptr callback)
{
    while(i2cIsBusy);

    i2c.keycode_callback = callback;

    I2CReceive(i2cAddr, 1, false);
}

#endif

#if TRINAMIC_ENABLE && TRINAMIC_I2C

TMC2130_status_t TMC_I2C_ReadRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    uint8_t *res;
    TMC2130_status_t status = {0};

    memset(i2c.buffer, 0, sizeof(i2c.buffer));

    if((i2c.buffer[0] = TMCI2C_GetMapAddress((uint8_t)(driver ? (uint32_t)driver->cs_pin : 0), reg->addr).value) == 0xFF)
        return status; // unsupported register

    res = I2C_ReadRegister(I2C_ADR_I2CBRIDGE, 5, true);

    EUSCI_B1->IE &= ~(EUSCI_B_IE_TXIE0|EUSCI_B_IE_RXIE0);
/*
    __delay_cycles(1200); // Delay to allow I2C bridge to get data from driver

    res = I2CReceiveMany(I2C_ADR_I2CBRIDGE, 5);

    EUSCI_B1->IE &= ~EUSCI_B_IE_RXIE0;
*/
    status.value = (uint8_t)*res++;
    reg->payload.value = ((uint8_t)*res++ << 24);
    reg->payload.value |= ((uint8_t)*res++ << 16);
    reg->payload.value |= ((uint8_t)*res++ << 8);
    reg->payload.value |= (uint8_t)*res++;

    return status;
}


TMC2130_status_t TMC_I2C_WriteRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    TMC2130_status_t status = {0};

    while(i2cIsBusy);

    reg->addr.write = 1;
    i2c.buffer[0] = TMCI2C_GetMapAddress((uint8_t)(driver ? (uint32_t)driver->cs_pin : 0), reg->addr).value;
    reg->addr.write = 0;

    if(i2c.buffer[0] == 0xFF)
        return status; // unsupported register

    i2c.buffer[1] = (reg->payload.value >> 24) & 0xFF;
    i2c.buffer[2] = (reg->payload.value >> 16) & 0xFF;
    i2c.buffer[3] = (reg->payload.value >> 8) & 0xFF;
    i2c.buffer[4] = reg->payload.value & 0xFF;

    I2CSend(I2C_ADR_I2CBRIDGE, 5, true);

    EUSCI_B1->IE &= ~EUSCI_B_IE_TXIE0;

    return status;
}

void I2C_DriverInit (TMC_io_driver_t *driver)
{
    driver->WriteRegister = TMC_I2C_WriteRegister;
    driver->ReadRegister = TMC_I2C_ReadRegister;
}

#endif

void I2C_interrupt_handler (void)
{
    // based on code from https://e2e.ti.com/support/microcontrollers/tiva_arm/f/908/t/169882

//    I2CMasterIntClear(I2C0_BASE);

    uint32_t ifg = EUSCI_B1->IFG;

    EUSCI_B1->IFG &= ~(EUSCI_B_IFG_TXIFG0|EUSCI_B_IFG_RXIFG0);          // Clear interrupt flags



//    if(I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE)

    if(ifg & (EUSCI_B_IFG_TXIFG0|EUSCI_B_IFG_RXIFG0))
    switch(i2c.state) {

        case I2CState_Idle:
            break;

        case I2CState_SendNext:
            EUSCI_B1->TXBUF = *i2c.data++;
            if(--i2c.count == 1)
                i2c.state = I2CState_SendLast;
            break;

        case I2CState_SendLast:
            EUSCI_B1->TXBUF = *i2c.data++;
            while(!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));   // Wait for start of TX
            EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;

            i2c.state = I2CState_AwaitCompletion;
            break;

        case I2CState_SendRegisterAddress:
            EUSCI_B1->IE &= ~EUSCI_B_IE_TXIE0;
            EUSCI_B1->TXBUF = *i2c.data;
            i2c.state = i2c.count == 1 ? I2CState_ReceiveLast : (i2c.count == 2 ? I2CState_ReceiveNextToLast : I2CState_ReceiveNext);
            while(!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));   // Wait for start of TX
            EUSCI_B1->CTLW0 &= ~EUSCI_B_CTLW0_TR;                           // Set read mode
            if(i2c.state == I2CState_ReceiveLast)           // and issue
                EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT|EUSCI_B_CTLW0_TXSTP; // restart and stop condition if single byte read
            else                                                            // else
                EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;                     // restart condition only
            break;

        case I2CState_AwaitCompletion:
            i2c.count = 0;
            i2c.state = I2CState_Idle;
            //            EUSCI_B1->IFG &= ~EUSCI_B_IFG_TXIFG0;
            break;

        case I2CState_ReceiveNext:
            *i2c.data++ = EUSCI_B1->RXBUF;
            if(--i2c.count == 2)
                i2c.state = I2CState_ReceiveNextToLast;
            break;

        case I2CState_ReceiveNextToLast:
            *i2c.data++ = EUSCI_B1->RXBUF;
            EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
            i2c.count--;
            i2c.state = I2CState_ReceiveLast;
            break;

        case I2CState_ReceiveLast:
            *i2c.data = EUSCI_B1->RXBUF;
            i2c.count = 0;
            i2c.state = I2CState_Idle;
#if KEYPAD_ENABLE
            if(i2c.keycode_callback) {
                i2c.keycode_callback(i2c.data[0]);
                i2c.keycode_callback = NULL;
            }
#endif
            break;
    }
}
