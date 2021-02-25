/*
 * tmc26x.c - interface for Trinamic TRM26x stepper drivers
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

#include <string.h>

#include <trinamic/tmc26x.h>

static SPI_driver_t io;

//default values

#define CONSTANT_OFF_TIME 3 // 7
#define FAST_DECAY_TIME 4 // 13
#define SINE_WAVE_OFFSET 1 // 15
#define CHOPPER_MODE 0 // 1
#define BLANK_TIME 2 // 3
#define RANDOM_TOFF 0 // 1

static const TMC26x_t tmc26x_sdon_defaults = {
    .cool_step_enabled = false,
    .resistor = 110,
    .current = 500,
    .microsteps = TMC26x_Microsteps_16,

    .chopconf.addr = 0x01,
    .chopconf.toff = CONSTANT_OFF_TIME,
    .chopconf.hdec = (FAST_DECAY_TIME & 0x08) >> 3,
    .chopconf.hstrt = FAST_DECAY_TIME & 0x07,
    .chopconf.hend = SINE_WAVE_OFFSET,
    .chopconf.chm = CHOPPER_MODE,
    .chopconf.tbl = BLANK_TIME,
    .chopconf.rndtf = RANDOM_TOFF,

    .smarten.addr = 0x05,
    .sgcsconf.addr = 0x03,
    .drvconf.addr = 0x07
};

static const TMC26x_t tmc26x_sdoff_defaults = {
    .cool_step_enabled = false,
    .resistor = 110,
    .current = 500,

    .chopconf.addr = 0x01,
    .chopconf.toff = CONSTANT_OFF_TIME,
    .chopconf.hdec = (FAST_DECAY_TIME & 0x08) >> 3,
    .chopconf.hstrt = FAST_DECAY_TIME & 0x07,
    .chopconf.hend = SINE_WAVE_OFFSET,
    .chopconf.chm = CHOPPER_MODE,
    .chopconf.tbl = BLANK_TIME,
    .chopconf.rndtf = RANDOM_TOFF,

    .smarten.addr = 0x05,
    .sgcsconf.addr = 0x03,
    .drvconf.addr = 0x07,
    .drvconf.sdoff = 1
};

void TMC26X_SetDefaults (TMC26x_t *driver, bool sdoff)
{
    memcpy(driver, sdoff ? &tmc26x_sdoff_defaults : &tmc26x_sdon_defaults, sizeof(TMC26x_t));
}

void TMC26X_Init (TMC26x_t *driver)
{
    static bool ioint_ok = false;

    if(!ioint_ok) {
        SPI_DriverInit(&io);
        ioint_ok = true;
    }

    driver->cool_step_enabled = false;

    TMC26X_SetCurrent(driver, driver->current);
    TMC26X_SetConstantOffTimeChopper(driver, 7, 54, 13, 12, 1);
    TMC26X_SetMicrosteps(driver, driver->microsteps);

    if(driver->drvconf.sdoff)
        io.WriteRegister(driver, (TMC26x_datagram_t)driver->drvctrl_sdoff);
    else
        io.WriteRegister(driver, (TMC26x_datagram_t)driver->drvctrl_sdon);
    io.WriteRegister(driver, (TMC26x_datagram_t)driver->chopconf);
    io.WriteRegister(driver, (TMC26x_datagram_t)driver->smarten);
    io.WriteRegister(driver, (TMC26x_datagram_t)driver->sgcsconf);
    io.WriteRegister(driver, (TMC26x_datagram_t)driver->drvconf);
}

void TMC26X_SetCurrent (TMC26x_t *driver, uint16_t mA)
{
    driver->current = mA;

    float maxv = ((float)driver->resistor * (float)driver->current * 32.0f) / 1000000.0f;

    // Calculate the current scaling from the max current setting (in mA),
    // this is derived from I=(cs+1)/32*(Vsense/Rsense)
    // leading to cs = CS = 32*R*I/V (with V = 0.31V or 0.165V and I = 1000*current)
    // with Rsense=0.15
    // For vsense = 0.310V (VSENSE not set)
    // or vsense = 0.165V (VSENSE set)

    uint8_t current_scaling = (uint8_t)((maxv / 0.31f) - 0.5f);

    // If the current scaling is too low
    // set the vsense bit to get a use half the sense voltage (to support lower motor currents)
    // and recalculate the current setting
    if ((driver->drvconf.vsense = (current_scaling < 16)))
        current_scaling = (uint8_t)((maxv / 0.165f) - 0.5f);

    driver->sgcsconf.cs = current_scaling > 31 ? 31 : current_scaling;

    io.WriteRegister(driver, (TMC26x_datagram_t)driver->drvconf);
    io.WriteRegister(driver, (TMC26x_datagram_t)driver->sgcsconf);
}

void TMC26X_SetMicrosteps (TMC26x_t *driver, tmc26x_microsteps_t usteps)
{
    if(driver->drvconf.sdoff)
        return;

    uint8_t value = 0;

    driver->microsteps = usteps = (usteps == 0 ? TMC26x_Microsteps_1 : usteps);

    while((usteps & 0x01) == 0) {
        value++;
        usteps >>= 1;
    }

    driver->drvctrl_sdon.mres = 8 - (value > 8 ? 8 : value);

    io.WriteRegister(driver, (TMC26x_datagram_t)driver->drvctrl_sdoff);
}

void TMC26X_SetConstantOffTimeChopper (TMC26x_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator)
{
  //calculate the value acc to the clock cycles
  if (blank_time >= 54)
      blank_time = 3;
  else if (blank_time >= 36)
      blank_time = 2;
  else if (blank_time >= 24)
      blank_time = 1;
  else
      blank_time = 0;

  if (fast_decay_time > 15)
    fast_decay_time = 15;

  driver->chopconf.chm = 1;
  driver->chopconf.tbl = blank_time;
  driver->chopconf.toff = constant_off_time < 2 ? 2 : (constant_off_time > 15 ? 15 : constant_off_time);
  driver->chopconf.hdec = (fast_decay_time & 0x8) >> 3;
  driver->chopconf.hstrt = fast_decay_time & 0x7;
  driver->chopconf.hend = (sine_wave_offset < -3 ? -3 : (sine_wave_offset > 12 ? 12 : sine_wave_offset)) + 3;
  driver->chopconf.rndtf = !use_current_comparator;

  io.WriteRegister(driver, (TMC26x_datagram_t)driver->chopconf);
}
