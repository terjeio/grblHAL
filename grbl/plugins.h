/*
  plugins.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Some data structures and function declarations for plugins that require driver code

  These are NOT referenced in the core grbl code

  Part of GrblHAL

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

#ifndef __PLUGINS_H__
#define __PLUGINS_H__

#include <stdint.h>
#include <stdbool.h>

// Quadrature encoder interface

typedef enum {
    Encoder_Universal = 0,
    Encoder_FeedRate,
    Encoder_Spindle_RPM,
    Encoder_Spindle_Position,
    Encoder_MPG,
    Encoder_MPG_X,
    Encoder_MPG_Y,
    Encoder_MPG_Z,
    Encoder_MPG_A,
    Encoder_MPG_B,
    Encoder_MPG_C
} encoder_mode_t;

typedef union {
    uint_fast8_t signals;
    struct {
        uint_fast8_t position :1,
                     select   :1,
                     index    :1,
                     home     :1;
    };
} encoder_signals_t;

typedef struct {
    encoder_mode_t mode;
    int32_t position;
    encoder_signals_t changed;
    encoder_signals_t state;
} encoder_t;

// EEPROM/FRAM interface

typedef struct {
    uint8_t address;
    uint8_t word_addr_bytes;
    uint16_t word_addr;
    volatile uint_fast16_t count;
    uint8_t *data;
} i2c_eeprom_trans_t;

extern void i2c_init (void);
extern void i2c_eeprom_transfer (i2c_eeprom_trans_t *i2c, bool read);

#endif
