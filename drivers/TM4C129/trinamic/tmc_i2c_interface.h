/*
 * tmc_i2c_interface.h - I2C <> SPI command mappings and datagrams for Trinamic TMC2130 stepper driver
 *
 * v0.0.3 / 2019-07-23 / (c) Io Engineering / Terje
 */

/*

Copyright (c) 2018-2019, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef _TRINAMIC_I2C_H_
#define _TRINAMIC_I2C_H_

#include <stdint.h>

#include "common.h"

#pragma pack(push, 1)

typedef union {
    uint8_t value;
    struct {
        uint8_t
        mapaddr :5,
        axis    :3;
    };
} TMCI2C_map_addr_t;

typedef union {
    uint8_t mask;
    struct {
        uint8_t x :1,
                y :1,
                z :1,
                a :1,
                b :1,
                c :1,
       unassigned :2;
    };
} tmc_axes_t;

typedef union {
    uint8_t value;
    tmc_axes_t stalled;
} TMCI2C_status_t;

typedef union {
    uint32_t value;
    uint8_t data[4];
    struct {
        tmc_axes_t enable;
        tmc_axes_t monitor;
        tmc_axes_t u1; // R
        tmc_axes_t u2;
    };
} TMCI2C_enable_reg_t;

// I2C_MONSTAT : R

typedef union {
    uint32_t value;
    uint8_t data[4];
    struct {
        tmc_axes_t ot;
        tmc_axes_t otpw;
        tmc_axes_t otpw_cnt;
        tmc_axes_t error;
    };
} TMCI2C_monitor_status_reg_t;

typedef struct {
    TMC_addr_t addr;
    TMCI2C_enable_reg_t reg;
} TMCI2C_enable_dgr_t;

typedef struct {
    TMC_addr_t addr;
    TMCI2C_monitor_status_reg_t reg;
} TMCI2C_monitor_status_dgr_t;

#pragma pack(pop)

#endif
