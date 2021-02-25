/*
  i2c.c - driver code for IMXRT1062 processor (on Teensy 4.0 board)

  Part of grblHAL

  Some parts of this code is Copyright (c) 2020 Terje Io

  Some parts are derived/pulled from WireIMXRT.cpp in the Teensyduino Core Library (no copyright header)

*/

#include "driver.h"

#ifdef I2C_PORT

#include <string.h>

#include "i2c.h"

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#define i2cIsBusy (!(i2c.state == I2CState_Idle || i2c.state == I2CState_Error) || (port->MSR & (LPI2C_MSR_BBF|LPI2C_MSR_MBF)))

// Timeout if a device stretches SCL this long, in microseconds
#define CLOCK_STRETCH_TIMEOUT 15000
#define PINCONFIG (IOMUXC_PAD_ODE | IOMUXC_PAD_SRE | IOMUXC_PAD_DSE(4) | IOMUXC_PAD_SPEED(1) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3))

typedef struct {
    volatile uint32_t *clock_gate_register;
    uint32_t clock_gate_mask;
    pin_info_t sda_pin;
    pin_info_t scl_pin;
    IMXRT_LPI2C_t *port;
    enum IRQ_NUMBER_t irq;
} i2c_hardware_t;

static const i2c_hardware_t i2c1_hardware = {
    .clock_gate_register = &CCM_CCGR2,
    .clock_gate_mask = CCM_CCGR2_LPI2C1(CCM_CCGR_ON),
    .port = &IMXRT_LPI2C1,
    .irq = IRQ_LPI2C1,
    .sda_pin = {
        .pin = 18,
        .mux_val = 3 | 0x10,
        .select_reg = &IOMUXC_LPI2C1_SDA_SELECT_INPUT,
        .select_val = 1
    },
    .scl_pin = {
        .pin = 19,
        .mux_val = 3 | 0x10,
        .select_reg = &IOMUXC_LPI2C1_SCL_SELECT_INPUT,
        .select_val = 1
    }
};

// NOTE: port 3 has alternative mapping to pin 36 and 37
static const i2c_hardware_t i2c3_hardware = {
    .clock_gate_register = &CCM_CCGR2,
    .clock_gate_mask = CCM_CCGR2_LPI2C3(CCM_CCGR_ON),
    .port = &IMXRT_LPI2C3,
    .irq = IRQ_LPI2C3,
    .sda_pin = {
        .pin = 16,
        .mux_val = 1 | 0x10,
        .select_reg = &IOMUXC_LPI2C3_SDA_SELECT_INPUT,
        .select_val = 1
    },
    .scl_pin = {
        .pin = 17,
        .mux_val = 1 | 0x10,
        .select_reg = &IOMUXC_LPI2C3_SCL_SELECT_INPUT,
        .select_val = 1
    }
};

static const i2c_hardware_t i2c4_hardware = {
    .clock_gate_register = &CCM_CCGR6,
    .clock_gate_mask = CCM_CCGR6_LPI2C4_SERIAL(CCM_CCGR_ON),
    .port = &IMXRT_LPI2C4,
    .irq = IRQ_LPI2C4,
    .sda_pin = {
        .pin = 25,
        .mux_val = 0 | 0x10,
        .select_reg = &IOMUXC_LPI2C4_SDA_SELECT_INPUT,
        .select_val = 1
    },
    .scl_pin = {
        .pin = 24,
        .mux_val = 0 | 0x10,
        .select_reg = &IOMUXC_LPI2C4_SCL_SELECT_INPUT,
        .select_val = 1
    }
};

static bool force_clock (const i2c_hardware_t *hardware)
{
    bool ret = false;
    uint32_t sda_pin = hardware->sda_pin.pin;
    uint32_t scl_pin = hardware->scl_pin.pin;
    uint32_t sda_mask = digitalPinToBitMask(sda_pin);
    uint32_t scl_mask = digitalPinToBitMask(scl_pin);
    // take control of pins with GPIO
    *portConfigRegister(sda_pin) = 5 | 0x10;
    *portSetRegister(sda_pin) = sda_mask;
    *portModeRegister(sda_pin) |= sda_mask;
    *portConfigRegister(scl_pin) = 5 | 0x10;
    *portSetRegister(scl_pin) = scl_mask;
    *portModeRegister(scl_pin) |= scl_mask;
    delayMicroseconds(10);
    for (int i=0; i < 9; i++) {
        if ((*portInputRegister(sda_pin) & sda_mask) && (*portInputRegister(scl_pin) & scl_mask)) {
            // success, both pins are high
            ret = true;
            break;
        }
        *portClearRegister(scl_pin) = scl_mask;
        delayMicroseconds(5);
        *portSetRegister(scl_pin) = scl_mask;
        delayMicroseconds(5);
    }
    // return control of pins to I2C
    *(portConfigRegister(sda_pin)) = hardware->sda_pin.mux_val;
    *(portConfigRegister(scl_pin)) = hardware->scl_pin.mux_val;

    return ret;
}

static void set_clock (IMXRT_LPI2C_t *port, uint32_t frequency)
{
    port->MCR = 0;

    if (frequency < 400000) {
        // 100 kHz
        port->MCCR0 = LPI2C_MCCR0_CLKHI(55) | LPI2C_MCCR0_CLKLO(59) | LPI2C_MCCR0_DATAVD(25) | LPI2C_MCCR0_SETHOLD(40);
        port->MCFGR1 = LPI2C_MCFGR1_PRESCALE(1);
        port->MCFGR2 = LPI2C_MCFGR2_FILTSDA(5) | LPI2C_MCFGR2_FILTSCL(5) | LPI2C_MCFGR2_BUSIDLE(3000); // idle timeout 250 us
        port->MCFGR3 = LPI2C_MCFGR3_PINLOW(CLOCK_STRETCH_TIMEOUT * 12 / 256 + 1);
    } else if (frequency < 1000000) {
        // 400 kHz
        port->MCCR0 = LPI2C_MCCR0_CLKHI(26) | LPI2C_MCCR0_CLKLO(28) | LPI2C_MCCR0_DATAVD(12) | LPI2C_MCCR0_SETHOLD(18);
        port->MCFGR1 = LPI2C_MCFGR1_PRESCALE(0);
        port->MCFGR2 = LPI2C_MCFGR2_FILTSDA(2) | LPI2C_MCFGR2_FILTSCL(2) | LPI2C_MCFGR2_BUSIDLE(3600); // idle timeout 150 us
        port->MCFGR3 = LPI2C_MCFGR3_PINLOW(CLOCK_STRETCH_TIMEOUT * 24 / 256 + 1);
    } else {
        // 1 MHz
        port->MCCR0 = LPI2C_MCCR0_CLKHI(9) | LPI2C_MCCR0_CLKLO(10) | LPI2C_MCCR0_DATAVD(4) | LPI2C_MCCR0_SETHOLD(7);
        port->MCFGR1 = LPI2C_MCFGR1_PRESCALE(0);
        port->MCFGR2 = LPI2C_MCFGR2_FILTSDA(1) | LPI2C_MCFGR2_FILTSCL(1) | LPI2C_MCFGR2_BUSIDLE(2400); // idle timeout 100 us
        port->MCFGR3 = LPI2C_MCFGR3_PINLOW(CLOCK_STRETCH_TIMEOUT * 24 / 256 + 1);
    }
    port->MCCR1 = port->MCCR0;
    port->MCFGR0 = 0;
    port->MFCR = LPI2C_MFCR_RXWATER(0) | LPI2C_MFCR_TXWATER(1);
    port->MCR = LPI2C_MCR_MEN;
}

typedef enum {
    I2CState_Idle = 0,
    I2CState_Restart,
    I2CState_SendAddr,
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
    volatile uint16_t count;
    volatile uint16_t rcount;
    volatile uint8_t acount;
    uint8_t *data;
    uint8_t regaddr[2];
#if KEYPAD_ENABLE
    keycode_callback_ptr keycode_callback;
#endif
    uint8_t buffer[8];
} i2c_trans_t;

static i2c_trans_t i2c;
static uint8_t tx_fifo_size;
static const i2c_hardware_t *hardware;
static IMXRT_LPI2C_t *port = NULL;

static void I2C_interrupt_handler (void);

void i2c_init (void)
{
    static bool init_ok = false;

    if(!init_ok) {

        init_ok = true;

#ifndef I2C_PORT
#error "I2C port is undefined!"
#endif

#if I2C_PORT == 0
        hardware = &i2c1_hardware;
#elif I2C_PORT == 3
        hardware = &i2c3_hardware;
#elif  I2C_PORT == 4
        hardware = &i2c4_hardware;
#else
#error "No such I2C port!"
#endif

        port = hardware->port;
        tx_fifo_size = 1 << (port->PARAM & 0b1111);

        CCM_CSCDR2 = (CCM_CSCDR2 & ~CCM_CSCDR2_LPI2C_CLK_PODF(63)) | CCM_CSCDR2_LPI2C_CLK_SEL;
        *hardware->clock_gate_register |= hardware->clock_gate_mask;
        port->MCR = LPI2C_MCR_RST;

        set_clock(port, 100000);

        // Setup SDA register
        *(portControlRegister(hardware->sda_pin.pin)) = PINCONFIG;
        *(portConfigRegister(hardware->sda_pin.pin)) = hardware->sda_pin.mux_val;
        *(hardware->sda_pin.select_reg) =  hardware->sda_pin.select_val;

        // setup SCL register
        *(portControlRegister(hardware->scl_pin.pin)) = PINCONFIG;
        *(portConfigRegister(hardware->scl_pin.pin)) = hardware->scl_pin.mux_val;
        *(hardware->scl_pin.select_reg) =  hardware->scl_pin.select_val;

        attachInterruptVector(hardware->irq, I2C_interrupt_handler);

        NVIC_SET_PRIORITY(hardware->irq, 1);
        NVIC_ENABLE_IRQ(hardware->irq);
    }
}

// wait until ready for transfer, try peripheral reset if bus hangs
inline static bool wait_ready (void)
{
    while(i2cIsBusy) {
        if(port->MSR & LPI2C_MSR_PLTF) {
            if(force_clock(hardware)) {
                port->MCR = LPI2C_MCR_RST;
                set_clock(port, 100000);
            } else
                return false;
        }
    }

    return true;
}

// get bytes (max 8 if local buffer, else max 255), waits for result
uint8_t *I2C_Receive (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes, bool block)
{
    i2c.data  = buf ? buf : i2c.buffer;
    i2c.count = bytes;
    i2c.rcount = 0;
    i2c.state = bytes == 1 ? I2CState_ReceiveLast : (bytes == 2 ? I2CState_ReceiveNextToLast : I2CState_ReceiveNext);

    port->MSR = 0;
    port->MTDR = LPI2C_MTDR_CMD_START | (i2cAddr << 1) | 1;
    port->MTDR = LPI2C_MTDR_CMD_RECEIVE | ((uint32_t)i2c.count - 1);
    port->MIER = LPI2C_MIER_NDIE|LPI2C_MIER_RDIE;

    if(block)
        while(i2cIsBusy);

    return i2c.buffer;
}

void I2C_Send (uint32_t i2cAddr, uint8_t *buf, uint16_t bytes, bool block)
{
    i2c.count = bytes;
    i2c.data  = buf ? buf : i2c.buffer;

    port->MSR = 0;
    port->MTDR = LPI2C_MTDR_CMD_START | (i2cAddr << 1);

    while(i2c.count && (port->MFSR & 0b111) < tx_fifo_size) {
        port->MTDR = LPI2C_MTDR_CMD_TRANSMIT | (uint32_t)(*i2c.data++);
        i2c.count--;
    }

    port->MIER = LPI2C_MIER_NDIE|LPI2C_MIER_TDIE;

    i2c.state = i2c.count == 0 ? I2CState_AwaitCompletion : (i2c.count == 1 ? I2CState_SendLast : I2CState_SendNext);

    if(block)
        while(i2cIsBusy);
}

uint8_t *I2C_ReadRegister (uint32_t i2cAddr, uint8_t *buf, uint8_t abytes, uint16_t bytes, bool block)
{
    while(i2cIsBusy);

    i2c.addr   = i2cAddr;
    i2c.count  = bytes;
    i2c.rcount = 0;
    i2c.acount = abytes;
    i2c.data   = buf ? buf : i2c.buffer;
    i2c.state  = I2CState_SendAddr;

    port->MSR = 0;
    port->MTDR = LPI2C_MTDR_CMD_START | (i2cAddr << 1);
    port->MIER = LPI2C_MIER_NDIE|LPI2C_MIER_TDIE;

    if(block)
        while(i2cIsBusy);

    return i2c.buffer;
}

#if EEPROM_ENABLE

nvs_transfer_result_t i2c_nvs_transfer (nvs_transfer_t *transfer, bool read)
{
    static uint8_t txbuf[NVS_SIZE + 2];

    while(i2cIsBusy);

    if(read) {
        if(transfer->word_addr_bytes == 1)
            i2c.regaddr[0] = transfer->word_addr;
        else {
            i2c.regaddr[0] = transfer->word_addr & 0xFF;
            i2c.regaddr[1] = transfer->word_addr >> 8;
        }
        I2C_ReadRegister(transfer->address, transfer->data, transfer->word_addr_bytes, transfer->count, true);
    } else {
        memcpy(&txbuf[transfer->word_addr_bytes], transfer->data, transfer->count);
        if(transfer->word_addr_bytes == 1)
            txbuf[0] = transfer->word_addr;
        else {
            txbuf[0] = transfer->word_addr >> 8;
            txbuf[1] = transfer->word_addr & 0xFF;
        }
        I2C_Send(transfer->address, txbuf, transfer->count + transfer->word_addr_bytes, true);
#if !EEPROM_IS_FRAM
        hal.delay_ms(7, NULL);
#endif
    }

    return NVS_TransferResult_OK;
}

#endif

#if KEYPAD_ENABLE

void I2C_GetKeycode (uint32_t i2cAddr, keycode_callback_ptr callback)
{
    if(wait_ready()) {
        i2c.keycode_callback = callback;
        I2C_Receive(i2cAddr, NULL, 1, false);
    }
}

#endif

#if TRINAMIC_ENABLE && TRINAMIC_I2C

static TMC2130_status_t I2C_TMC_ReadRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    uint8_t *res, i2creg;
    TMC2130_status_t status = {0};

    if((i2creg = TMCI2C_GetMapAddress((uint8_t)(driver ? (uint32_t)driver->cs_pin : 0), reg->addr).value) == 0xFF)
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
    i2c.buffer[0] = TMCI2C_GetMapAddress((uint8_t)(driver ? (uint32_t)driver->cs_pin : 0), reg->addr).value;
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
    uint32_t ifg = port->MSR & 0xFFFF;

    port->MSR &= ~ifg;

if((ifg & port->MIER) == 0) return;

//if(port->MSR & LPI2C_MSR_MBF) return;
/*
hal.stream.write("I:");
hal.stream.write(uitoa(ifg));
hal.stream.write(" ");
hal.stream.write(uitoa(i2c.state));
hal.stream.write(ASCII_EOL);
*/
    if(ifg & LPI2C_MSR_ALF) {
        port->MTDR = LPI2C_MTDR_CMD_STOP;
        i2c.state = I2CState_Error;
    }

    if(ifg & LPI2C_MSR_NDF)
        i2c.state = I2CState_Error;

    switch(i2c.state) {

        case I2CState_Idle:
        case I2CState_Error:
            port->MIER = 0;
            port->MCR |= (LPI2C_MCR_RTF|LPI2C_MCR_RRF);
            break;

        case I2CState_SendNext:
            port->MTDR = LPI2C_MTDR_CMD_TRANSMIT | (uint32_t)(*i2c.data++);
            if(--i2c.count == 1)
                i2c.state = I2CState_SendLast;
            break;

        case I2CState_SendLast:
            port->MTDR = LPI2C_MTDR_CMD_TRANSMIT | (uint32_t)(*i2c.data++);
            i2c.state = I2CState_AwaitCompletion;
            break;

        case I2CState_AwaitCompletion:
            port->MIER &= ~LPI2C_MIER_TDIE;
            port->MTDR = LPI2C_MTDR_CMD_STOP;
            i2c.count = 0;
            i2c.state = I2CState_Idle;
            break;

        case I2CState_SendAddr:
            port->MTDR = LPI2C_MTDR_CMD_TRANSMIT | (uint32_t)(i2c.regaddr[--i2c.acount]);
            if(i2c.acount == 0)
                i2c.state = I2CState_Restart;  
            break;

        case I2CState_Restart:
            if(port->MIER & LPI2C_MIER_TDIE) {
                port->MIER &= ~LPI2C_MIER_TDIE;
                port->MIER |= LPI2C_MIER_EPIE;
                port->MTDR = LPI2C_MTDR_CMD_START | (i2c.addr << 1) | 1;
            } else if(port->MIER & LPI2C_MIER_EPIE) {
                port->MIER &= ~LPI2C_MIER_EPIE;
                port->MIER |= LPI2C_MIER_RDIE;
                port->MTDR = LPI2C_MTDR_CMD_RECEIVE | ((uint32_t)i2c.count - 1);
                i2c.state = i2c.count == 1 ? I2CState_ReceiveLast : (i2c.count == 2 ? I2CState_ReceiveNextToLast : I2CState_ReceiveNext);
            }
            break;

        case I2CState_ReceiveNext: // superfluous, to be removed...
            *i2c.data++ = port->MRDR & 0xFF;
            if(--i2c.count == 1) {
                i2c.state = I2CState_ReceiveLast;
            }
            ++i2c.rcount;
            break;

        case I2CState_ReceiveNextToLast:
            *i2c.data++ = port->MRDR & 0xFF;
//            port->MTDR = LPI2C_MTDR_CMD_STOP;
            i2c.count--;
            i2c.state = I2CState_ReceiveLast;
            break;

        case I2CState_ReceiveLast:
            *i2c.data = port->MRDR & 0xFF;
            i2c.count = 0;
            i2c.state = I2CState_Idle;
            port->MTDR = LPI2C_MTDR_CMD_STOP;
          #if KEYPAD_ENABLE
            if(i2c.keycode_callback) {
                i2c.keycode_callback(*i2c.data);
                i2c.keycode_callback = NULL;
            }
          #endif
            break;

        default:
            break;
    }
}

#endif
