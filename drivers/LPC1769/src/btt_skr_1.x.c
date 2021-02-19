/*
  btt_skr_1.x.c - driver code for NXP LPC176x ARM processors

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

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

#if defined(BOARD_BTT_SKR_13) || defined(BOARD_BTT_SKR_14_TURBO)

#include "chip.h"

#if TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160

#include "trinamic/common.h"

#define TRINAMIC_MOSI_BIT (1<<TRINAMIC_MOSI_PIN)
#define TRINAMIC_MISO_BIT (1<<TRINAMIC_MISO_PIN)
#define TRINAMIC_SCK_BIT (1<<TRINAMIC_SCK_PIN)
#define TRINAMIC_CSX_BIT (1<<TRINAMIC_CSX_PIN)
#define TRINAMIC_CSY_BIT (1<<TRINAMIC_CSY_PIN)
#define TRINAMIC_CSZ_BIT (1<<TRINAMIC_CSZ_PIN)
#ifdef A_AXIS
#define TRINAMIC_CSA_BIT (1<<TRINAMIC_CSA_PIN)
#endif
#ifdef B_AXIS
#define TRINAMIC_CSB_BIT (1<<TRINAMIC_CSB_PIN)
#endif

#define spi_get_byte() sw_spi_xfer(0)
#define spi_put_byte(d) sw_spi_xfer(d)

static uint32_t cs_bit[N_AXIS];

inline static void delay (void)
{
    volatile uint32_t dly = 10;

    while(--dly)
        __ASM volatile ("nop");
}

static uint8_t sw_spi_xfer (uint8_t byte)
{
    uint_fast8_t msk = 0x80, res = 0;

    TRINAMIC_SCK_PORT->CLR = TRINAMIC_SCK_BIT;

    do {
        DIGITAL_OUT(TRINAMIC_MOSI_PORT, TRINAMIC_MOSI_BIT, byte & msk);
        msk >>= 1;
        delay();
        res = (res << 1) | DIGITAL_IN(TRINAMIC_MISO_PORT, TRINAMIC_MISO_BIT);
        TRINAMIC_SCK_PORT->SET = TRINAMIC_SCK_BIT;
        delay();
        if(msk)
            TRINAMIC_SCK_PORT->CLR = TRINAMIC_SCK_BIT;
    } while (msk);

    return (uint8_t)res;
}

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;

    TRINAMIC_CS_PORT->CLR = cs_bit[driver.axis];

    datagram->payload.value = 0;

    datagram->addr.write = 0;
    spi_put_byte(datagram->addr.value);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);

    TRINAMIC_CS_PORT->SET = cs_bit[driver.axis];
    delay();
    TRINAMIC_CS_PORT->CLR = cs_bit[driver.axis];

    status = spi_put_byte(datagram->addr.value);
    datagram->payload.data[3] = spi_get_byte();
    datagram->payload.data[2] = spi_get_byte();
    datagram->payload.data[1] = spi_get_byte();
    datagram->payload.data[0] = spi_get_byte();

    TRINAMIC_CS_PORT->SET = cs_bit[driver.axis];

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram)
{
    TMC_spi_status_t status;

    TRINAMIC_CS_PORT->CLR = cs_bit[driver.axis];

    datagram->addr.write = 1;
    status = spi_put_byte(datagram->addr.value);
    spi_put_byte(datagram->payload.data[3]);
    spi_put_byte(datagram->payload.data[2]);
    spi_put_byte(datagram->payload.data[1]);
    spi_put_byte(datagram->payload.data[0]);

    TRINAMIC_CS_PORT->SET = cs_bit[driver.axis];

    return status;
}

#endif

void board_init (void)
{
#if TRINAMIC_ENABLE == 2130 || TRINAMIC_ENABLE == 5160

    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_MOSI_PN, TRINAMIC_MOSI_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_MISO_PN, TRINAMIC_MISO_PIN, IOCON_MODE_PULLUP, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_SCK_PN, TRINAMIC_SCK_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_CS_PN, TRINAMIC_CSX_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_CS_PN, TRINAMIC_CSY_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_CS_PN, TRINAMIC_CSZ_PIN, IOCON_MODE_INACT, IOCON_FUNC0);

    Chip_IOCON_DisableOD((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_MOSI_PN, TRINAMIC_MOSI_PIN);
    Chip_IOCON_DisableOD((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_MISO_PN, TRINAMIC_MOSI_PIN);
    Chip_IOCON_DisableOD((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_SCK_PN, TRINAMIC_MOSI_PIN);
    Chip_IOCON_DisableOD((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_CS_PN, TRINAMIC_CSX_PIN);
    Chip_IOCON_DisableOD((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_CS_PN, TRINAMIC_CSY_PIN);
    Chip_IOCON_DisableOD((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_CS_PN, TRINAMIC_CSZ_PIN);

    TRINAMIC_MOSI_PORT->DIR |= TRINAMIC_MOSI_BIT;
    TRINAMIC_MISO_PORT->DIR &= ~TRINAMIC_MISO_BIT;

    TRINAMIC_SCK_PORT->DIR |= TRINAMIC_SCK_BIT;
    TRINAMIC_SCK_PORT->SET = TRINAMIC_SCK_BIT;

    cs_bit[X_AXIS] = TRINAMIC_CSX_BIT;
    TRINAMIC_CS_PORT->DIR |= TRINAMIC_CSX_BIT;
    TRINAMIC_CS_PORT->SET = TRINAMIC_CSX_BIT;

    cs_bit[Y_AXIS] = TRINAMIC_CSY_BIT;
    TRINAMIC_CS_PORT->DIR |= TRINAMIC_CSY_BIT;
    TRINAMIC_CS_PORT->SET = TRINAMIC_CSY_BIT;

    cs_bit[Z_AXIS] = TRINAMIC_CSZ_BIT;
    TRINAMIC_CS_PORT->DIR |= TRINAMIC_CSZ_BIT;
    TRINAMIC_CS_PORT->SET = TRINAMIC_CSZ_BIT;

#ifdef A_AXIS
    Chip_IOCON_DisableOD((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_CS_PN, TRINAMIC_CSA_PIN);
    cs_bit[A_AXIS] = TRINAMIC_CSA_PIN;
    TRINAMIC_CS_PORT->DIR |= TRINAMIC_CSA_BIT;
    TRINAMIC_CS_PORT->SET = TRINAMIC_CSA_BIT;
#endif
#ifdef B_AXIS
    Chip_IOCON_DisableOD((LPC_IOCON_T *)LPC_IOCON_BASE, TRINAMIC_CS_PN, TRINAMIC_CSB_PIN);
    cs_bit[B_AXIS] = TRINAMIC_CSB_PIN;
    TRINAMIC_CS_PORT->DIR |= TRINAMIC_CSB_BIT;
    TRINAMIC_CS_PORT->SET = TRINAMIC_CSB_BIT;
#endif

#endif
}

#endif

