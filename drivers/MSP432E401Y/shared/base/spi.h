/*
  spi.h - SPI interface for Trinamic TMC2130 stepper drivers

  For Texas Instruments SimpleLink ARM processors/LaunchPads

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

#ifndef __SPI_DRIVER_H__
#define __SPI_DRIVER_H__

#define SPI_SCLK_PIN GPIO_PIN_0
#define SPI_MOSI_PIN GPIO_PIN_2
#define SPI_MISO_PIN GPIO_PIN_3

#define SPI_CS_PIN_X GPIO_PIN_7
#define SPI_CS_PORT_X GPIO_PORTA_BASE
#define SPI_CS_PIN_Y GPIO_PIN_4
#define SPI_CS_PORT_Y GPIO_PORTA_BASE
#define SPI_CS_PIN_Z GPIO_PIN_5
#define SPI_CS_PORT_Z GPIO_PORTA_BASE

#define SPI_PORT_PERIPH GPIO_PORTQ_BASE
#define SPI_PORT GPIO_PORTQ_BASE
#define SPI_PERIPH SYSCTL_PERIPH_SSI3
#define SPI_BASE SSI3_BASE
#define SPI_CLK GPIO_PQ0_SSI3CLK
#define SPI_TX GPIO_PQ2_SSI3XDAT0
#define SPI_RX GPIO_PQ3_SSI3XDAT1

void SPI_Init (void);

#endif

