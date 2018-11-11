/*
  trinamic.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Texas Instruments Tiva C (TM4C123GH6PM) ARM processor
   for Trinamic TMC2130 integration

  Part of Grbl

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

#ifndef _TRINAMIC_H_
#define _TRINAMIC_H_

#include "tiva.h"
#include "trinamic\trinamic2130.h"

typedef struct {
    uint16_t current; // mA
    uint16_t r_sense; // mOhm
    tmc2130_microsteps_t microsteps;
} motor_settings_t;

void trinamic_init (void);
void trinamic_configure (void);
bool trinamic_setting (uint_fast16_t setting, float value, char *svalue);
void trinamic_settings_restore (uint8_t restore_flag);
void trinamic_settings_report (bool axis_settings, axis_setting_type_t setting_type, uint8_t axis_idx);

uint_fast16_t trimamic_MCodeCheck (uint_fast16_t mcode);
status_code_t trimamic_MCodeValidate (parser_block_t *gc_block, uint_fast16_t *value_words);
void trimamic_MCodeExecute (uint_fast16_t state, parser_block_t *gc_block);

//

#define SPI_SCLK_PIN GPIO_PIN_0
#define SPI_MOSI_PIN GPIO_PIN_2 // TX
#define SPI_MISO_PIN GPIO_PIN_3 // RX

#define SPI_CS_PIN GPIO_PIN_7
#define SPI_CS_PORT GPIO_PORTA_BASE
#define SPI_SELECT GPIOPinWrite(SPI_CS_PORT, SPI_CS_PIN, 0)
#define SPI_DESELECT GPIOPinWrite(SPI_CS_PORT, SPI_CS_PIN, SPI_CS_PIN)

#define SPI_PERIPH SYSCTL_PERIPH_SSI3
#define SPI_BASE SSI3_BASE
#define SPI_PORT GPIO_PORTQ_BASE
#define SPI_CLK GPIO_PQ0_SSI3CLK
#define SPI_TX GPIO_PQ2_SSI3XDAT0
#define SPI_RX GPIO_PQ3_SSI3XDAT1

void SPI_DriverInit (SPI_driver_t *drv);
TMC2130_status_t SPI_ReadRegister (TMC2130_t *driver, TMC2130_datagram_t *reg);
TMC2130_status_t SPI_WriteRegister (TMC2130_t *driver, TMC2130_datagram_t *reg);

#endif
