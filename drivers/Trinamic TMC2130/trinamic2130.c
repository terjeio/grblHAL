/*
 * trinamic2130.c - interface for Trinamic TMC2130 stepper driver
 *
 * v0.0.1 / 2018-11-14 / ©Io Engineering / Terje
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

/*
 * Reference for calculations:
 * https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC5130_TMC2130_TMC2100_Calculations.xlsx
 *
 */

#include <string.h>

#include "trinamic2130.h"

static SPI_driver_t io;

static const TMC2130_t tmc2130_defaults = {
    .f_clk = TMC2130_F_CLK,
    .cool_step_enabled = TMC2130_COOLSTEP_ENABLE,
    .r_sense = TMC2130_R_SENSE,
    .current = TMC2130_CURRENT,
    .hold_current_pct = TMC2130_HOLD_CURRENT_PCT,
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

#if TMC2130_COOLSTEP_ENABLE == 1
    .coolconf.reg.semin = TMC2130_COOLSTEP_SEMIN | (TMC2130_COOLSTEP_ENABLE << 3),
    .coolconf.reg.semax = TMC2130_COOLSTEP_SEMAX,
#endif

    .chopconf.reg.intpol = TMC2130_INTERPOLATE,
    .chopconf.reg.toff = TMC2130_CONSTANT_OFF_TIME,
    .chopconf.reg.chm = TMC2130_CHOPPER_MODE,
    .chopconf.reg.tbl = TMC2130_BLANK_TIME,
    .chopconf.reg.rndtf = TMC2130_RANDOM_TOFF,
#if TMC2130_CHOPPER_MODE == 0
    .chopconf.reg.hstrt = TMC2130_HSTRT,
    .chopconf.reg.hend = TMC2130_HEND,
#else
    .chopconf.reg.fd3 = (TMC2130_FAST_DECAY_TIME & 0x08) >> 3,
    .chopconf.reg.hstrt = TMC2130_FAST_DECAY_TIME & 0x07,
    .chopconf.reg.hend = TMC2130_SINE_WAVE_OFFSET,
#endif

    .ihold_irun.reg.irun = TMC2130_IRUN,
    .ihold_irun.reg.ihold = TMC2130_IHOLD,
    .ihold_irun.reg.iholddelay = TMC2130_IHOLDDELAY,

    .tpowerdown.reg.tpowerdown = TMC2130_TPOWERDOWN,

    .gconf.reg.en_pwm_mode = TMC2130_EN_PWM_MODE,

#if TMC2130_EN_PWM_MODE == 1 // stealthChop
    .pwmconf.reg.pwm_autoscale = TMC2130_PWM_AUTOSCALE,
    .pwmconf.reg.pwm_ampl = TMC2130_PWM_AMPL,
    .pwmconf.reg.pwm_grad = TMC2130_PWM_GRAD,
    .pwmconf.reg.pwm_freq = TMC2130_PWM_FREQ,
#endif

    .tpwmthrs.reg.tpwmthrs = TMC2130_TPWM_THRS
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

static void set_tfd (TMC2130_chopconf_reg_t *chopconf, uint8_t fast_decay_time)
{
    chopconf->chm = 1;
    chopconf->fd3 = (fast_decay_time & 0x8) >> 3;
    chopconf->hstrt = fast_decay_time & 0x7;
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

    // Perform a status register read to clear reset flag
    io.ReadRegister(driver, (TMC2130_datagram_t *)&driver->stat);

    driver->chopconf.reg.mres = to_mres(driver->microsteps);
    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->chopconf);
    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->ihold_irun);
    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->gconf);
    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->tpowerdown);
    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->tpwmthrs);
    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->pwmconf);

    TMC2130_SetCurrent(driver, driver->current, driver->hold_current_pct);

    //set to a conservative start value
    //TMC2130_SetConstantOffTimeChopper(driver, 5, 24, 13, 12, true); // move to default values
}

// r_sense = mOhm, Vsense = mV, current = mA (peak)
void TMC2130_SetCurrent (TMC2130_t *driver, uint16_t mA, uint8_t hold_pct)
{
    driver->current = mA;
    driver->hold_current_pct = hold_pct;

    float maxv = (((float)(driver->r_sense + 20)) * (float)(32UL * driver->current)) / 1000.0f;

    uint8_t current_scaling = (uint8_t)((maxv / 320.0f) - 0.5f);

    // If the current scaling is too low set the vsense bit and recalculate the current setting
    if ((driver->chopconf.reg.vsense = (current_scaling < 16)))
        current_scaling = (uint8_t)((maxv / 180.0f) - 0.5f);

    driver->ihold_irun.reg.irun = current_scaling > 31 ? 31 : current_scaling;
    driver->ihold_irun.reg.ihold = (driver->ihold_irun.reg.irun * driver->hold_current_pct) / 100;

    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->chopconf);
    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->ihold_irun);
}

// threshold = velocity in mm/s
void TMC2130_SetHybridThreshold (TMC2130_t *driver, uint32_t threshold, float steps_mm)
{
    driver->tpwmthrs.reg.tpwmthrs = threshold == 0.0f ? 0UL : driver->f_clk * driver->microsteps / (256 * (uint32_t)((float)threshold * steps_mm));
    io.WriteRegister(driver, (TMC2130_datagram_t *)&driver->tpwmthrs);
}

void TMC2130_SetMicrosteps (TMC2130_t *driver, tmc2130_microsteps_t msteps)
{
    driver->chopconf.reg.mres = to_mres(msteps);
    driver->microsteps = (tmc2130_microsteps_t)(1 << (8 - driver->chopconf.reg.mres));
// TODO: recalc and set hybrid threshold if enabled?
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

    set_tfd(&driver->chopconf.reg, fast_decay_time);

    driver->chopconf.reg.tbl = blank_time;
    driver->chopconf.reg.toff = constant_off_time < 2 ? 2 : (constant_off_time > 15 ? 15 : constant_off_time);
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
