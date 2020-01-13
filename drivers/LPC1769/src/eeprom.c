/*

  eeprom.c - driver code for NXP LPC176x ARM processors

  for 8K EEPROM on OM13085 dev board (Microchip 24LC64) connected to I2C1

  Part of GrblHAL

  Copyright (c) 2019 Terje Io

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

#include "chip.h"
#include "driver.h"
#include "grbl/grbl.h"

#define EEPROM_I2C_ADDRESS (0xA0 >> 1)
#define EEPROM_PAGE_SIZE 32

typedef struct {
    uint16_t addr;
    volatile int16_t count;
    uint8_t *data;
    uint16_t word_addr;
} i2c_trans_t;

#if EEPROM_ENABLE

static i2c_trans_t i2c;
static I2C_XFER_T xfer;

void eepromInit (void)
{
    Chip_IOCON_PinMux(LPC_IOCON, 0, 19, IOCON_MODE_INACT, IOCON_FUNC3);
    Chip_IOCON_PinMux(LPC_IOCON, 0, 20, IOCON_MODE_INACT, IOCON_FUNC3);
    Chip_IOCON_EnableOD(LPC_IOCON, 0, 19);
    Chip_IOCON_EnableOD(LPC_IOCON, 0, 20);

    Chip_I2C_Init(I2C1);
    Chip_I2C_SetClockRate(I2C1, 400000);

    Chip_I2C_SetMasterEventHandler(I2C1, Chip_I2C_EventHandlerPolling);
}

static void I2C_EEPROM (i2c_trans_t *i2c, bool read)
{
    static uint8_t txbuf[EEPROM_PAGE_SIZE + 2];

    txbuf[0] = i2c->word_addr >> 8;
    txbuf[1] = i2c->word_addr & 0xFF;

    xfer.slaveAddr = i2c->addr;
    xfer.rxSz = read ? i2c->count : 0;
    xfer.rxBuff = read ? i2c->data : NULL;
    xfer.txSz = 2;
    xfer.txBuff = txbuf;

    if(!read) {
        xfer.txSz += i2c->count;
        memcpy(&txbuf[2], i2c->data, i2c->count);
    }

    Chip_I2C_MasterTransfer(I2C1, &xfer);

    if(!read)
        hal.delay_ms(15, NULL);

    i2c->data += i2c->count;
}

uint8_t eepromGetByte (uint32_t addr)
{
    static uint8_t value;

    i2c.addr = EEPROM_I2C_ADDRESS;
    i2c.word_addr = addr;
    i2c.data = &value;
    i2c.count = 1;
    I2C_EEPROM(&i2c, true);

    return value;
}

void eepromPutByte (uint32_t addr, uint8_t new_value)
{
    i2c.addr = EEPROM_I2C_ADDRESS;
    i2c.word_addr = addr;
    i2c.data = &new_value;
    i2c.count = 1;

    I2C_EEPROM(&i2c, false);
}

void eepromWriteBlockWithChecksum (uint32_t destination, uint8_t *source, uint32_t size)
{
    uint32_t bytes = size;

    i2c.word_addr = destination;
    i2c.data = source;

    while(bytes > 0) {
        i2c.count = EEPROM_PAGE_SIZE - (destination & (EEPROM_PAGE_SIZE - 1));
        i2c.count = bytes < i2c.count ? bytes : i2c.count;
        i2c.addr = EEPROM_I2C_ADDRESS;
        bytes -= i2c.count;
        destination += i2c.count;

        I2C_EEPROM(&i2c, false);

        i2c.word_addr = destination;
    }

    if(size > 0)
        eepromPutByte(destination, calc_checksum(source, size));
}

bool eepromReadBlockWithChecksum (uint8_t *destination, uint32_t source, uint32_t size)
{
    i2c.addr = EEPROM_I2C_ADDRESS;
    i2c.word_addr = source;
    i2c.count = size;
    i2c.data = destination;

    I2C_EEPROM(&i2c, true);

    return calc_checksum(destination, size) == eepromGetByte(source + size);
}

#endif

