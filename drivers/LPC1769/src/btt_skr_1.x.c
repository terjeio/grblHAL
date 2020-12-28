/*
  btt_skr_1.x.c - driver code for NXP LPC176x ARM processors

  Part of grblHAL

  Copyright (c) 2020 Terje Io

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
#include <trinamic/tmc2130.h>

#if TRINAMIC_ENABLE == 2130

#define spi_get_byte() sw_spi_xfer(0)
#define spi_put_byte(d) sw_spi_xfer(d)

static uint8_t cs_pin[N_AXIS];

static uint8_t sw_spi_xfer (uint8_t byte)
{
    uint8_t idx = 8, msk = 0x80, res = 0;

    BITBAND_GPIO(TRINAMIC_SCK_PORT->PIN, TRINAMIC_SCK_PIN) = 0;

    do {
        BITBAND_GPIO(TRINAMIC_MOSI_PORT->PIN, TRINAMIC_MOSI_PIN) = !!(byte & msk);
        msk >>= 1;
        BITBAND_GPIO(TRINAMIC_SCK_PORT->PIN, TRINAMIC_SCK_PIN) = 1;
        res = (res << 1) | BITBAND_GPIO(TRINAMIC_MISO_PORT->PIN, TRINAMIC_MISO_PIN);
        if(idx != 1)
            BITBAND_GPIO(TRINAMIC_SCK_PORT->PIN, TRINAMIC_SCK_PIN) = 0;
    } while (--idx);

    return res;
}

static TMC2130_status_t TMC_SPI_ReadRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    static TMC2130_status_t status = {0};

    BITBAND_GPIO(TRINAMIC_CS_PORT->PIN, cs_pin[driver->axis]) = 0;

    reg->addr.write = 0;
    spi_put_byte(reg->addr.value);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);
    spi_put_byte(0);

    BITBAND_GPIO(TRINAMIC_CS_PORT->PIN, cs_pin[driver->axis]) = 1;
    BITBAND_GPIO(TRINAMIC_CS_PORT->PIN, cs_pin[driver->axis]) = 0;

    status.value = spi_put_byte(reg->addr.value);
    reg->payload.data[3] = spi_get_byte();
    reg->payload.data[2] = spi_get_byte();
    reg->payload.data[1] = spi_get_byte();
    reg->payload.data[0] = spi_get_byte();

    BITBAND_GPIO(TRINAMIC_CS_PORT->PIN, cs_pin[driver->axis]) = 1;

    return status;
}

static TMC2130_status_t TMC_SPI_WriteRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    static TMC2130_status_t status = {0};

    BITBAND_GPIO(TRINAMIC_CS_PORT->PIN, cs_pin[driver->axis]) = 0;

    reg->addr.write = 1;
    status.value = spi_put_byte(reg->addr.value);
    spi_put_byte(reg->payload.data[3]);
    spi_put_byte(reg->payload.data[2]);
    spi_put_byte(reg->payload.data[1]);
    spi_put_byte(reg->payload.data[0]);

    BITBAND_GPIO(TRINAMIC_CS_PORT->PIN, cs_pin[driver->axis]) = 1;

    return status;
}

#endif

void board_init (void)
{
#if TRINAMIC_ENABLE == 2130

    trinamic_driver_if_t driver = {
        .interface.WriteRegister = TMC_SPI_WriteRegister,
        .interface.ReadRegister = TMC_SPI_ReadRegister
    };

    trinamic_if_init(&driver);

    BITBAND_GPIO(TRINAMIC_MOSI_PORT->DIR, TRINAMIC_MOSI_PIN) = 1;

    BITBAND_GPIO(TRINAMIC_MISO_PORT->DIR, TRINAMIC_MISO_PIN) = 0;
    gpio_pinmode(TRINAMIC_MISO_PORT, TRINAMIC_MISO_PIN, true); // enable pullup

    BITBAND_GPIO(TRINAMIC_SCK_PORT->DIR, TRINAMIC_SCK_PIN) = 1;
    BITBAND_GPIO(TRINAMIC_SCK_PORT->PIN, TRINAMIC_SCK_PIN) = 1;

    cs_pin[X_AXIS] = TRINAMIC_CSX_PIN;
    BITBAND_GPIO(TRINAMIC_CS_PORT->DIR, TRINAMIC_CSX_PIN) = 1;
    BITBAND_GPIO(TRINAMIC_CS_PORT->PIN, TRINAMIC_CSX_PIN) = 1;

    cs_pin[Y_AXIS] = TRINAMIC_CSY_PIN;
    BITBAND_GPIO(TRINAMIC_CS_PORT->DIR, TRINAMIC_CSY_PIN) = 1;
    BITBAND_GPIO(TRINAMIC_CS_PORT->PIN, TRINAMIC_CSY_PIN) = 1;

    cs_pin[Z_AXIS] = TRINAMIC_CSZ_PIN;
    BITBAND_GPIO(TRINAMIC_CS_PORT->DIR, TRINAMIC_CSZ_PIN) = 1;
    BITBAND_GPIO(TRINAMIC_CS_PORT->PIN, TRINAMIC_CSZ_PIN) = 1;
#ifdef A_AXIS
    cs_pin[A_AXIS] = TRINAMIC_CSA_PIN;
    BITBAND_GPIO(TRINAMIC_CS_PORT->DIR, TRINAMIC_CSA_PIN) = 1;
    BITBAND_GPIO(TRINAMIC_CS_PORT->PIN, TRINAMIC_CSB_PIN) = 1;
#endif
#ifdef B_AXIS
    cs_pin[B_AXIS] = TRINAMIC_CSB_PIN;
    BITBAND_GPIO(TRINAMIC_CS_PORT->DIR, TRINAMIC_CSB_PIN) = 1;
    BITBAND_GPIO(TRINAMIC_CS_PORT->PIN, TRINAMIC_CSB_PIN) = 1;
#endif

#endif
}

#endif

