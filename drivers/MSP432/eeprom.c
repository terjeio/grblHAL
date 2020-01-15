/*

  eeprom.c - driver code for Texas Instruments MSP432 ARM processor

  for 2K EEPROM on CNC Boosterpack (Microchip 24LC16B)

  Part of Grbl

  Copyright (c) 2017-2019 Terje Io

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

#include <msp.h>
#include <stdint.h>
#include <stdbool.h>

#include "driver.h"

#include "grbl/grbl.h"

#define EEPROM_I2C_ADDRESS (0xA0 >> 1)
#define EEPROM_ADDR_BITS_LO 8
#define EEPROM_BLOCK_SIZE (2 ^ EEPROM_LO_ADDR_BITS)
#define EEPROM_PAGE_SIZE 16

typedef struct {
    uint8_t addr;
    volatile int16_t count;
    uint8_t *data;
    uint8_t word_addr;
} i2c_trans_t;

static i2c_trans_t i2c;

/* could not get ACK polling to work...
static void WaitForACK (void)
{
    while(EUSCI_B1->STATW & EUSCI_B_STATW_BBUSY);

    do {
        EUSCI_B1->IFG &= ~(EUSCI_B_IFG_TXIFG0|EUSCI_B_IFG_RXIFG0);
        EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TR|EUSCI_B_CTLW0_TXSTT;     // I2C TX, start condition

        while (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTT) {               // Ensure stop condition got sent
            if(!(EUSCI_B1->IFG & EUSCI_B_IFG_NACKIFG))           // Break out if ACK received
              break;
        }
//        EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
//        while (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTP);               // Ensure stop condition got sent
        __delay_cycles(5000);
    } while(EUSCI_B1->IFG & EUSCI_B_IFG_NACKIFG);
//    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
//    while (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTP);               // Ensure stop condition got sent
}
*/
static void StartI2C (bool read)
{
    bool single = i2c.count == 1;

    EUSCI_B1->I2CSA = i2c.addr;                                         // Set EEPROM address and MSB part of data address
    EUSCI_B1->IE &= ~(EUSCI_B_IE_TXIE0|EUSCI_B_IE_TXIE0);               // Diasable and
    EUSCI_B1->IFG &= ~(EUSCI_B_IFG_TXIFG0|EUSCI_B_IFG_RXIFG0);          // clear interrupts
    EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TR|EUSCI_B_CTLW0_TXSTT;            // Transmit start condition and address
    while(!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));                       // Wait for TX completed
    EUSCI_B1->TXBUF = i2c.word_addr;                                    // Transmit data address LSB
    while(!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));                       // Wait for TX completed

    if(read) {                                                          // Read data from EEPROM:
        EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;                         // Transmit STOP condtition
        while (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTP);                  // and wait for it to complete
        EUSCI_B1->CTLW0 &= ~EUSCI_B_CTLW0_TR;                           // Set read mode
        if(single)                                                      // and issue
            EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT|EUSCI_B_CTLW0_TXSTP; // restart and stop condition if single byte read
        else                                                            // else
            EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTT;                     // restart condition only

        while(i2c.count) {                                              // Read data...
            if(!single && i2c.count == 1) {
                EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;
                while (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTP) {
                    while(!(EUSCI_B1->IFG & EUSCI_B_IFG_RXIFG0));
                }
            } else
                while(!(EUSCI_B1->IFG & EUSCI_B_IFG_RXIFG0));
            i2c.count--;
            *i2c.data++ = EUSCI_B1->RXBUF;
        }
    } else {                                                            // Write data to EEPROM:
        while (i2c.count--) {
            EUSCI_B1->TXBUF = *i2c.data++;
            while(!(EUSCI_B1->IFG & EUSCI_B_IFG_TXIFG0));
        }
        EUSCI_B1->CTLW0 |= EUSCI_B_CTLW0_TXSTP;                         // I2C stop condition
 //       WaitForACK();
        hal.delay_ms(5, 0);                                             // Wait a bit for the write cycle to complete
    }
    while (EUSCI_B1->CTLW0 & EUSCI_B_CTLW0_TXSTP);                      // Ensure stop condition got sent
}

uint8_t eepromGetByte (uint32_t addr)
{
    uint8_t value = 0;

    i2c.addr = EEPROM_I2C_ADDRESS | (addr >> 8);
    i2c.word_addr = addr & 0xFF;
    i2c.data = &value;
    i2c.count = 1;
    StartI2C(true);

    return value;
}

void eepromPutByte (uint32_t addr, uint8_t new_value)
{
    i2c.addr = EEPROM_I2C_ADDRESS | (addr >> 8);
    i2c.word_addr = addr & 0xFF;
    i2c.data = &new_value;
    i2c.count = 1;

    StartI2C(false);
}

void eepromWriteBlockWithChecksum (uint32_t destination, uint8_t *source, uint32_t size)
{
    uint32_t bytes = size;

    i2c.word_addr = destination & 0xFF;
    i2c.data = source;

    while(bytes > 0) {
        i2c.count = EEPROM_PAGE_SIZE - (destination & (EEPROM_PAGE_SIZE - 1));
        i2c.count = bytes < i2c.count ? bytes : i2c.count;
        i2c.addr = EEPROM_I2C_ADDRESS | (destination >> EEPROM_ADDR_BITS_LO);
        bytes -= i2c.count;
        destination += i2c.count;

        StartI2C(false);

        i2c.word_addr = destination & 0xFF;
    }

    if(size > 0)
        eepromPutByte(destination, calc_checksum(source, size));
}

bool eepromReadBlockWithChecksum (uint8_t *destination, uint32_t source, uint32_t size)
{
    i2c.addr = EEPROM_I2C_ADDRESS | (source >> 8);
    i2c.word_addr = source & 0xFF;
    i2c.count = size;
    i2c.data = destination;

    StartI2C(true);

    return calc_checksum(destination, size) == eepromGetByte(source + size);
}
