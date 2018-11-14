/*
 * TMC2130_I2C_map.c - I1C <> SPI command mapping for Trinamic TMC2130 stepper driver
 *
 * v0.0.1 / 2018-10-27 / ©Io Engineering / Terje
 */

/*

Copyright (c) 2018, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

· Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

· Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

· Neither the name of the copyright holder nor the names of its contributors may
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

#include "TMC2130_I2C_map.h"
#include "trinamic2130.h"

#define WR 0x80

const uint8_t TMC2130_I2C_regmap[] = {
    TMC2130Reg_GCONF,
    TMC2130Reg_GCONF|WR,
    TMC2130Reg_GSTAT,
    TMC2130Reg_IOIN,
    TMC2130Reg_IHOLD_IRUN|WR,
    TMC2130Reg_TPOWERDOWN|WR,
    TMC2130Reg_TSTEP,
    TMC2130Reg_TPWMTHRS|WR,
    TMC2130Reg_TCOOLTHRS|WR,
    TMC2130Reg_THIGH|WR,
    TMC2130Reg_VDCMIN|WR,
    TMC2130Reg_MSCNT,
    TMC2130Reg_MSCURACT,
    TMC2130Reg_CHOPCONF,
    TMC2130Reg_CHOPCONF|WR,
    TMC2130Reg_COOLCONF|WR,
    TMC2130Reg_DCCTRL|WR,
    TMC2130Reg_DRV_STATUS,
    TMC2130Reg_PWMCONF|WR,
    TMC2130Reg_PWM_SCALE,
    TMC2130Reg_ENCM_CTRL|WR,
    TMC2130Reg_LOST_STEPS,
    0xFF // terminator
};
