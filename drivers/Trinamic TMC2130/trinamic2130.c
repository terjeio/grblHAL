/*
 * trinamic2130.c - interface for Trinamic TRM2130 stepper driver
 *
 * v0.0.1 / 2018-11-10 / ©Io Engineering / Terje
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

#include <string.h>

#include "trinamic2130.h"

static SPI_driver_t io;

static const TMC2130_t tmc2130_defaults = {
    .cool_step_enabled = false,
    .r_sense = TMC2130_R_SENSE,
    .current = TMC2130_CURRENT,
    .hold_current_pct = 50,
    .microsteps = TMC2130_MICROSTEPS,

    // register adresses
    .gconf.addr.reg = TMC2130Reg_GCONF,
    .stat.addr.reg = TMC2130Reg_GSTAT,
    .ioin.addr.reg = TMC2130Reg_IOIN,
    .ihold_irun.addr.reg = TMC2130Reg_IHOLD_IRUN,
    .tpowerdown.addr.reg = TMC2130Reg_TPOWERDOWN,
    .tstep.addr.reg = TMC2130Reg_TSTEP,
    .tpwmthrs.addr.reg = TMC2130Reg_TPWMTHRS,
    .tcoolthrs.addr.reg = TMC2130Reg_TCOOLTHRS,
    .thigh.addr.reg = TMC2130Reg_THIGH,
    .vdcmin.addr.reg = TMC2130Reg_VDCMIN,
    .mscnt.addr.reg = TMC2130Reg_MSCNT,
    .mscuract.addr.reg = TMC2130Reg_MSCURACT,
    .chopconf.addr.reg = TMC2130Reg_CHOPCONF,
    .coolconf.addr.reg = TMC2130Reg_COOLCONF,
    .dcctrl.addr.reg = TMC2130Reg_DCCTRL,
    .drv_status.addr.reg = TMC2130Reg_DRV_STATUS,
    .pwmconf.addr.reg = TMC2130Reg_PWMCONF,
    .pwm_scale.addr.reg = TMC2130Reg_PWM_SCALE,
    .lost_steps.addr.reg = TMC2130Reg_LOST_STEPS,
#ifdef TMC2130_COMPLETE
    .xdirect.addr.reg = TMC2130Reg_XDIRECT,
    .mslut[0].addr.reg = TMC2130Reg_MSLUT_BASE,
    .mslut[1].addr.reg = (tmc2130_regaddr_t)(TMC2130Reg_MSLUT_BASE + 1),
    .mslut[2].addr.reg = (tmc2130_regaddr_t)(TMC2130Reg_MSLUT_BASE + 2),
    .mslut[3].addr.reg = (tmc2130_regaddr_t)(TMC2130Reg_MSLUT_BASE + 3),
    .mslut[4].addr.reg = (tmc2130_regaddr_t)(TMC2130Reg_MSLUT_BASE + 4),
    .mslut[5].addr.reg = (tmc2130_regaddr_t)(TMC2130Reg_MSLUT_BASE + 5),
    .mslut[6].addr.reg = (tmc2130_regaddr_t)(TMC2130Reg_MSLUT_BASE + 6),
    .mslut[7].addr.reg = (tmc2130_regaddr_t)(TMC2130Reg_MSLUT_BASE + 7),
    .mslutsel.addr.reg = TMC2130Reg_MSLUTSEL,
    .mslutstart.addr.reg = TMC2130Reg_MSLUTSTART,
    .encm_ctrl.addr.reg = TMC2130Reg_ENCM_CTRL,
#endif

    .chopconf.reg.toff = TMC2130_CONSTANT_OFF_TIME,
    .chopconf.reg.fd3 = (TMC2130_FAST_DECAY_TIME & 0x08) >> 3,
    .chopconf.reg.hstrt = TMC2130_FAST_DECAY_TIME & 0x07,
    .chopconf.reg.hend = TMC2130_SINE_WAVE_OFFSET,
    .chopconf.reg.chm = TMC2130_CHOPPER_MODE,
    .chopconf.reg.tbl = TMC2130_BLANK_TIME,
    .chopconf.reg.rndtf = TMC2130_RANDOM_TOFF,

    .ihold_irun.reg.irun = TMC2130_IRUN,
    .ihold_irun.reg.ihold = TMC2130_IHOLD,
    .ihold_irun.reg.iholddelay = TMC2130_IHOLDDELAY,

    .tpowerdown.reg.tpowerdown = TMC2130_TPOWERDOWN,

    .gconf.reg.en_pwm_mode = TMC2130_EN_PWM_MODE,

    .tpwmthrs.reg.tpwmthrs = TMC2130_TPWM_THRS,

    .pwmconf.reg.pwm_autoscale = TMC2130_PWM_AUTOSCALE,
    .pwmconf.reg.pwm_ampl = TMC2130_PWM_AMPL,
    .pwmconf.reg.pwm_grad = TMC2130_PWM_GRAD,
    .pwmconf.reg.pwm_freq = 0
};

static uint8_t to_mres (tmc2130_microsteps_t msteps)
{
    uint8_t value = 0;

    msteps = msteps == 0 ? TMC2130_Microsteps_1 : msteps;

    while((msteps & 0x01) == 0) {
      value++;
      msteps >>= 1;
    }

    return 8 - (value > 8 ? 8 : value);
}


void TMC2130_SetDefaults (TMC2130_t *driver)
{
    memcpy(driver, &tmc2130_defaults, sizeof(TMC2130_t));

    driver->chopconf.reg.mres = to_mres(driver->microsteps);
}

void TMC2130_Init (TMC2130_t *driver)
{
    static bool ioint_ok = false;

    if(!ioint_ok) {
        SPI_DriverInit(&io);
        ioint_ok = true;
    }

    TMC2130_status_t status = {0};

    // Perform a status register read to clear reset flag
    status = io.ReadRegister(driver, (TMC2130_datagram_t *)&driver->stat);


//    TMC2130_status_t status = {0};
/*
    while(1) {
        status = io.ReadRegister(driver, (TMC2130_datagram_t *)&driver->ioin);
    }
*/

    driver->chopconf.reg.mres = to_mres(driver->microsteps);
    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->chopconf);
    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->ihold_irun);
    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->gconf);
    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->tpowerdown);
    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->tpwmthrs);
    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->pwmconf);

    TMC2130_SetCurrent(driver, driver->current, driver->hold_current_pct);

    //set to a conservative start value
    //TMC2130_SetConstantOffTimeChopper(driver, 7, 54, 13, 12, true); // move to default values
}

void TMC2130_SetCurrent (TMC2130_t *driver, uint16_t mA, uint8_t hold_pct)
{
    driver->current = mA;
    driver->hold_current_pct = hold_pct;

    float maxv = ((float)driver->r_sense * (float)driver->current * 32.0f) / 1000000.0f;

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
    if ((driver->chopconf.reg.vsense = (current_scaling < 16)))
        current_scaling = (uint8_t)((maxv / 0.165f) - 0.5f);

    driver->ihold_irun.reg.irun = current_scaling > 31 ? 31 : current_scaling;
    driver->ihold_irun.reg.ihold = (driver->ihold_irun.reg.irun * driver->hold_current_pct) / 100;

    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->chopconf);
    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->ihold_irun);
}

void TMC2130_SetMicrosteps (TMC2130_t *driver, tmc2130_microsteps_t msteps)
{
    driver->chopconf.reg.mres = to_mres(msteps);

    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->chopconf);
}

void TMC2130_SetConstantOffTimeChopper (TMC2130_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator)
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

    //calculate the register setting
    //first of all delete all the values for this
    driver->chopconf.reg.chm = On;
    driver->chopconf.reg.tbl = blank_time;
    driver->chopconf.reg.toff = constant_off_time < 2 ? 2 : (constant_off_time > 15 ? 15 : constant_off_time);
    driver->chopconf.reg.fd3 = (fast_decay_time & 0x8) >> 3;
    driver->chopconf.reg.hstrt = fast_decay_time & 0x7;
    driver->chopconf.reg.hend = (sine_wave_offset < -3 ? -3 : (sine_wave_offset > 12 ? 12 : sine_wave_offset)) + 3;
    driver->chopconf.reg.rndtf = !use_current_comparator;

    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->chopconf);
}

TMC2130_status_t TMC2130_WriteRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    return io.WriteRegister(driver, reg);
}

TMC2130_status_t TMC2130_ReadRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    return io.ReadRegister(driver, reg);
}
