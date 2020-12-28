/*
 * tmc26x.h - interface for Trinamic TRM26x stepper drivers
 *
 * v0.0.1 / 2018-10-27 / (c) Io Engineering / Terje
 */

/*

Copyright (c) 2018, Terje Io
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

#ifndef _TRINAMIC_TM26X_H_
#define _TRINAMIC_TM26X_H_

#include <stdint.h>
#include <stdbool.h>

typedef enum {
    TMC26x_Microsteps_1 = 1,
    TMC26x_Microsteps_2 = 2,
    TMC26x_Microsteps_4 = 4,
    TMC26x_Microsteps_8 = 8,
    TMC26x_Microsteps_16 = 16,
    TMC26x_Microsteps_32 = 32,
    TMC26x_Microsteps_64 = 64,
    TMC26x_Microsteps_128 = 128,
    TMC26x_Microsteps_256 = 256
} tmc26x_microsteps_t;

/*
 *        stepper_tos_100[0].cs_pin = 1;
          stepper_tos_100[0].microsteps = 16;
          stepper_tos_100[0].number_of_steps=200;
          stepper_tos_100[0].current = 800;
          stepper_tos_100[0].resistor = 150;
          TMC26XStepper_init( &stepper_tos_100[0]);
 *
 */

typedef union {
    uint32_t value;
    struct {
        uint32_t cb   :8,
                 phb  :1,
                 ca   :8,
                 pha  :1,
                 addr :2;
    };
} TMC26x_drvctrl_sdoff_reg_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t mres      :4,
                 reserved1 :4,
                 dedge     :1,
                 intpol    :1,
                 reserved2 :8,
                 addr      :2;
    };
} TMC26x_drvctrl_sdon_reg_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t toff  :4,
                 hstrt :3,
                 hend  :4,
                 hdec  :2,
                 rndtf :1,
                 chm   :1,
                 tbl   :2,
                 addr  :3;
    };
} TMC26x_chopconf_reg_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t semin     :4,
                 reserved1 :1,
                 seup      :2,
                 reserved2 :1,
                 semax     :4,
                 reserved3 :1,
                 sedn      :2,
                 seimin    :1,
                 reserved4 :1,
                 addr      :3;
    };
} TMC26x_smarten_reg_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t cs        :5,
                 reserved1 :3,
                 sgt       :7,
                 reserved2 :1,
                 sflt      :1,
                 addr      :3;
    };
} TMC26x_sgcsconf_reg_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t reserved1 :4,
                 rdsel     :2,
                 vsense    :1,
                 sdoff     :1,
                 ts2g      :2,
                 diss2g    :1,
                 reserved2 :1,
                 slpl      :2,
                 slph      :2,
                 tst       :1,
                 addr      :3;
    };
} TMC26x_drvconf_reg_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t mstep     :10,
                 reserved2 :2,
                 stst      :1,
                 olb       :1,
                 ola       :1,
                 s2gb      :1,
                 s2ga      :1,
                 otpw      :1,
                 ot        :1,
                 sg        :1;
    } rdsel00;
    struct {
        uint32_t sgv       :10,
                 reserved2 :2,
                 stst      :1,
                 olb       :1,
                 ola       :1,
                 s2gb      :1,
                 s2ga      :1,
                 otpw      :1,
                 ot        :1,
                 sg        :1;
    } rdsel01;
    struct {
        uint32_t sgv       :5,
                 sev       :5,
                 reserved2 :2,
                 stst      :1,
                 olb       :1,
                 ola       :1,
                 s2gb      :1,
                 s2ga      :1,
                 otpw      :1,
                 ot        :1,
                 sg        :1;
    } rdsel10;
} TMC26x_status_reg_t;

typedef union {
    TMC26x_drvctrl_sdon_reg_t sdon;
    TMC26x_drvctrl_sdoff_reg_t sdoff;
} TMC26x_drvctrl_reg_t;

typedef union {
    uint32_t value;
    TMC26x_drvctrl_sdon_reg_t drvctrl_sdon;
    TMC26x_drvctrl_sdoff_reg_t drvctrl_sdoff;
//    TMC26x_drvctrl_reg_t drvctrl;
    TMC26x_chopconf_reg_t chopconf;
    TMC26x_smarten_reg_t smarten;
    TMC26x_sgcsconf_reg_t sgcsconf;
    TMC26x_drvconf_reg_t drvconf;
} TMC26x_datagram_t;

typedef struct {
    TMC26x_drvctrl_sdon_reg_t drvctrl_sdon;
    TMC26x_drvctrl_sdoff_reg_t drvctrl_sdoff;
//    TMC26x_drvctrl_reg_t drvctrl;
    TMC26x_chopconf_reg_t chopconf;
    TMC26x_smarten_reg_t smarten;
    TMC26x_sgcsconf_reg_t sgcsconf;
    TMC26x_drvconf_reg_t drvconf;

    TMC26x_status_reg_t driver_status_result;

    void  *cs_pin;
    tmc26x_microsteps_t microsteps;
    uint16_t resistor; // mOhm
    uint16_t current;  // mA
    int16_t constant_off_time;
    bool cool_step_enabled;
} TMC26x_t;

typedef struct {
    TMC26x_status_reg_t (*WriteRegister)(TMC26x_t *driver, TMC26x_datagram_t reg);
    TMC26x_status_reg_t (*ReadRegister)(TMC26x_t *driver, TMC26x_datagram_t reg);
} SPI_driver_t;

void TMC26X_Init (TMC26x_t *driver);
void TMC26X_SetDefaults (TMC26x_t *driver, bool sdoff);
void TMC26X_SetCurrent (TMC26x_t *driver, uint16_t mA);
void TMC26X_SetMicrosteps (TMC26x_t *driver, tmc26x_microsteps_t usteps);
void TMC26X_SetConstantOffTimeChopper (TMC26x_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator);

extern void SPI_DriverInit (SPI_driver_t *drv);

#endif
