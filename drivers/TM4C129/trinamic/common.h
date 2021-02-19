/*
 * common.h - shared code for Trinamic drivers
 *
 * v0.0.1 / 2020-02-04 / (c) Io Engineering / Terje
 */

/*

Copyright (c) 2021, Terje Io
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
specific prior written permission..

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

#pragma once

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint8_t axis; // axis index
    void *cs_pin; // the CS pin for the stepper driver
} trinamic_motor_t;

typedef enum {
    TMC2209 = 0,
    TMC2130,
    TMC5160
} trinamic_driver_t;

typedef struct {
    uint32_t f_clk;
    uint16_t microsteps;
    uint16_t r_sense;           // mOhm
    uint16_t current;           // mA
    uint8_t hold_current_pct;   // percent
    uint8_t addr;
    bool cool_step_enabled;
} trinamic_config_t;

#pragma pack(push, 1)

typedef union {
    uint8_t value;
    struct {
        uint8_t
        idx   :7,
        write :1;
    };
} TMC_addr_t;

typedef union {
    uint32_t value;
    uint8_t data[4];
} TMC_payload_t;

typedef struct {
    TMC_addr_t addr;
    TMC_payload_t payload;
} TMC_spi_datagram_t;

typedef union {
    uint8_t data[8];
    struct {
        uint8_t sync;
        uint8_t slave;
        TMC_addr_t addr;
        TMC_payload_t payload;
        uint8_t crc;
    } msg;
} TMC_uart_write_datagram_t;

typedef union {
    uint8_t data[4];
    struct {
        uint8_t sync;
        uint8_t slave;
        TMC_addr_t addr;
        uint8_t crc;
    } msg;
} TMC_uart_read_datagram_t;

// Custom registers used by I2C <> SPI bridge
typedef enum {
    TMC_I2CReg_MON_STATE = 0x7D,
    TMC_I2CReg_ENABLE = 0x7E
} TMC_i2c_registers_t;

#pragma pack(pop)

typedef uint8_t TMC_spi_status_t;

bool tmc_microsteps_validate (uint16_t microsteps);
uint8_t tmc_microsteps_to_mres (uint16_t microsteps);
uint32_t tmc_calc_tstep (trinamic_config_t *config, float mm_sec, float steps_mm);

extern TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *datagram);
extern TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *datagram);

extern void tmc_uart_write (trinamic_motor_t driver, TMC_uart_write_datagram_t *datagram);
extern TMC_uart_write_datagram_t *tmc_uart_read (trinamic_motor_t driver, TMC_uart_read_datagram_t *datagram);

