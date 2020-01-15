/*
  spi.c - SPI interface for Trinamic TMC2130 stepper drivers

  For Texas Instruments SimpleLink ARM processors/LaunchPads

  Part of GrblHAL

  Copyright (c) 2018 Terje Io

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

#if TRINAMIC_ENABLE && !TRINAMIC_I2C

#include "spi.h"

#define F_SPI 4000000
#define SPI_DELAY _delay_cycles(5)

static TMC2130_status_t SPI_ReadRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    uint32_t data;
    TMC2130_status_t status;

    chip_select_t *cs = (chip_select_t *)driver->cs_pin;

    GPIOPinWrite(cs->port, cs->pin, 0);

    SPI_DELAY;

    // dummy write to prepare data
    SSIDataPut(SPI_BASE, reg->addr.value);
    SSIDataPut(SPI_BASE, 0);
    SSIDataPut(SPI_BASE, 0);
    SSIDataPut(SPI_BASE, 0);
    SSIDataPut(SPI_BASE, 0);
    while(SSIBusy(SPI_BASE));

    // Ditch data in FIFO
    while(SSIDataGetNonBlocking(SPI_BASE, &data));

    GPIOPinWrite(cs->port, cs->pin, cs->pin);

    SPI_DELAY;

    // get register data

    GPIOPinWrite(cs->port, cs->pin, 0);

    SPI_DELAY;

    SSIDataPut(SPI_BASE, reg->addr.value);
    SSIDataPut(SPI_BASE, 0);
    SSIDataPut(SPI_BASE, 0);
    SSIDataPut(SPI_BASE, 0);
    SSIDataPut(SPI_BASE, 0);
    while(SSIBusy(SPI_BASE));

    // Read values from FIFO
    SSIDataGetNonBlocking(SPI_BASE, &data);
    status.value = (uint8_t)data;
    SSIDataGetNonBlocking(SPI_BASE, &data);
    reg->payload.value = ((uint8_t)data << 24);
    SSIDataGetNonBlocking(SPI_BASE, &data);
    reg->payload.value |= ((uint8_t)data << 16);
    SSIDataGetNonBlocking(SPI_BASE, &data);
    reg->payload.value |= ((uint8_t)data << 8);
    SSIDataGetNonBlocking(SPI_BASE, &data);
    reg->payload.value |= (uint8_t)data;

    GPIOPinWrite(cs->port, cs->pin, cs->pin);

    return status;
}

static TMC2130_status_t SPI_WriteRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    TMC2130_status_t status = {0};

    chip_select_t *cs = (chip_select_t *)driver->cs_pin;

    GPIOPinWrite(cs->port, cs->pin, 0);

    // assume FIFO is empty?
    reg->addr.write = 1;
    SSIDataPut(SPI_BASE, reg->addr.value);
    reg->addr.write = 0;
    SSIDataPut(SPI_BASE, (reg->payload.value >> 24) & 0xFF);
    SSIDataPut(SPI_BASE, (reg->payload.value >> 16) & 0xFF);
    SSIDataPut(SPI_BASE, (reg->payload.value >> 8) & 0xFF);
    SSIDataPut(SPI_BASE, reg->payload.value & 0xFF);
    while(SSIBusy(SPI_BASE));

    GPIOPinWrite(cs->port, cs->pin, cs->pin);

    return status;
}

void SPI__DriverInit (SPI_driver_t *driver)
{
    driver->WriteRegister = SPI_WriteRegister;
    driver->ReadRegister = SPI_ReadRegister;

    // NOTE: GPIO port(s) used for chip select must be enabled/set up earlier!

    PREF(SysCtlPeripheralEnable(SPI_PORT_PERIPH));
    PREF(SysCtlPeripheralEnable(SPI_PERIPH));

    PREF(GPIOPinConfigure(SPI_CLK));
    PREF(GPIOPinTypeSSI(SPI_PORT, SPI_SCLK_PIN));

    PREF(GPIOPinConfigure(SPI_TX));
    PREF(GPIOPinTypeSSI(SPI_PORT, SPI_MOSI_PIN));

    PREF(GPIOPinConfigure(SPI_RX));
    PREF(GPIOPinTypeSSI(SPI_PORT, SPI_MISO_PIN));

    SSIClockSourceSet(SPI_BASE, SSI_CLOCK_SYSTEM);
    PREF(SSIConfigSetExpClk(SPI_BASE, 120000000, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, F_SPI, 8));
    PREF(SSIEnable(SPI_BASE));
}

#endif
