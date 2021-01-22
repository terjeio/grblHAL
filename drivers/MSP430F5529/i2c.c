/*

  i2c.c - driver code for Texas Instruments MSP430F5529 processor

  Part of grblHAL

  Copyright (c) 2017-2020 Terje Io

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

#if EEPROM_ENABLE

#include <msp430.h>

#include "grbl/hal.h"
#include "grbl/plugins.h"

#define SDA BIT1
#define SCL BIT2

#define EEPROM_I2C_ADDRESS (0xA0 >> 1)
#define EEPROM_ADDR_BITS_LO 8
#define EEPROM_BLOCK_SIZE (2 ^ EEPROM_LO_ADDR_BITS)
#define EEPROM_PAGE_SIZE 16

void i2c_init (void)
{
    if((P4IN & SDA) == 0) {
        P4DIR |= SCL;                          // Assign I2C pins to USCI_B0
        while((P4IN & SDA) == 0) {
            P4OUT &= ~SCL;
            _delay_cycles(2500);
            P4OUT |= SCL;
        }
        P4DIR &= ~SCL;                      // Assign I2C pins to USCI_B0
    }

    P4SEL |= SDA|SCL;                   // Assign I2C pins to USCI_B1

    UCB1CTL1 = UCSWRST;                 // Put EUSCI_B1 in reset state
    UCB1CTL1 |= UCSSEL_2;               // Put EUSCI_B1 in reset state
    UCB1CTL0 = UCMODE_3|UCMST|UCSYNC;   // I2C master mode, SMCLK
    UCB1BR0 = 250;                       // baudrate 100 KHZ (SMCLK = 48MHz)
    UCB1BR1 = 0;                       // baudrate 100 KHZ (SMCLK = 48MHz)
    UCB1CTL1 &= ~UCSWRST;               // clear reset register
//    UCB1IE = EUSCI_B_IE_NACKIE;       // NACK interrupt enable
}

/* could not get ACK polling to work...
static void WaitForACK (void)
{
    while(UCB1STATW & EUSCI_B_STATW_BBUSY);

    do {
        UCB1IFG &= ~(UCTXIFG|UCRXIFG);
        UCB1CTL0 |= UCTR|UCTXSTT;     // I2C TX, start condition

        while (UCB1CTL0 & UCTXSTT) {               // Ensure stop condition got sent
            if(!(UCB1IFG & EUSCI_B_IFG_NACKIFG))           // Break out if ACK received
              break;
        }
//        UCB1CTL0 |= UCTXSTP;
//        while (UCB1CTL0 & UCTXSTP);               // Ensure stop condition got sent
        __delay_cycles(5000);
    } while(UCB1IFG & EUSCI_B_IFG_NACKIFG);
//    UCB1CTL0 |= UCTXSTP;
//    while (UCB1CTL0 & UCTXSTP);               // Ensure stop condition got sent
}
*/
nvs_transfer_result_t i2c_nvs_transfer (nvs_transfer_t *i2c, bool read)
{
    UCB1I2CSA = i2c->address;            // Set EEPROM address and MSB part of data address
    UCB1IFG &= ~(UCTXIFG|UCRXIFG);          // Clear interrupt flags
    UCB1CTL1 |= UCTR|UCTXSTT;               // Transmit start condition and address
    while(!(UCB1IFG & UCTXIFG));            // Wait for TX
    UCB1TXBUF = i2c->word_addr;          // Transmit data address LSB
    while(!(UCB1IFG & UCTXIFG));            // wait for transmit complete

    if(read) {                              // Read data from EEPROM:
        UCB1CTL1 |= UCTXSTP;                // Transmit STOP condition
        while (UCB1CTL1 & UCTXSTP);         // and wait for it to complete
        UCB1CTL1 &= ~UCTR;                  // Set read mode,
        UCB1CTL1 |= UCTXSTT;                // issue a restart and
        while (UCB1CTL1 & UCTXSTT);         // wait for it to complete

        while(i2c->count) {              // Read data...
            if(i2c->count == 1) {        // If last byte to read
                UCB1CTL1 |= UCTXSTP;        // issue and wait for stop condition
                while (UCB1CTL1 & UCTXSTP) {
                    while(!(UCB1IFG & UCRXIFG));
                }
            } else
                while(!(UCB1IFG & UCRXIFG));
            i2c->count--;
            *i2c->data++ = UCB1RXBUF;
        }
    } else {                                // Write data to EEPROM:
        while (i2c->count--) {
            UCB1TXBUF = *i2c->data++;
            while(!(UCB1IFG & UCTXIFG));
        }
        UCB1CTL1 |= UCTXSTP;                // I2C stop condition
#if !EEPROM_IS_FRAM
        hal.delay_ms(5, NULL);              // Wait a bit for the write cycle to complete
#endif
    }
    while (UCB1CTL1 & UCTXSTP);             // Ensure stop condition got sent

    return NVS_TransferResult_OK;
}

#endif
