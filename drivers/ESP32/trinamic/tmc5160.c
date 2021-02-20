/*
 * tmc5160.c - interface for Trinamic TMC5160 stepper driver
 *
 * v0.0.2 / 2021-02-04 / (c) Io Engineering / Terje
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

/*
 * Reference for calculations:
 * https://www.trinamic.com/fileadmin/assets/Products/ICs_Documents/TMC5160_Calculations.xlsx
 *
 */

#include <string.h>

#include "tmc5160.h"

static const TMC5160_t tmc5160_defaults = {
    .config.f_clk = TMC5160_F_CLK,
    .config.cool_step_enabled = TMC5160_COOLSTEP_ENABLE,
    .config.r_sense = TMC5160_R_SENSE,
    .config.current = TMC5160_CURRENT,
    .config.hold_current_pct = TMC5160_HOLD_CURRENT_PCT,
    .config.microsteps = TMC5160_MICROSTEPS,

    // register adresses
    .gconf.addr.reg = TMC5160Reg_GCONF,
    .gstat.addr.reg = TMC5160Reg_GSTAT,
    .ioin.addr.reg = TMC5160Reg_IOIN,
    .global_scaler.addr.reg = TMC5160Reg_GLOBAL_SCALER,
    .ihold_irun.addr.reg = TMC5160Reg_IHOLD_IRUN,
    .tpowerdown.addr.reg = TMC5160Reg_TPOWERDOWN,
    .tstep.addr.reg = TMC5160Reg_TSTEP,
    .tpwmthrs.addr.reg = TMC5160Reg_TPWMTHRS,
    .tcoolthrs.addr.reg = TMC5160Reg_TCOOLTHRS,
    .thigh.addr.reg = TMC5160Reg_THIGH,
    .vdcmin.addr.reg = TMC5160Reg_VDCMIN,
    .mscnt.addr.reg = TMC5160Reg_MSCNT,
    .mscuract.addr.reg = TMC5160Reg_MSCURACT,
    .chopconf.addr.reg = TMC5160Reg_CHOPCONF,
    .coolconf.addr.reg = TMC5160Reg_COOLCONF,
    .dcctrl.addr.reg = TMC5160Reg_DCCTRL,
    .drv_status.addr.reg = TMC5160Reg_DRV_STATUS,
    .pwmconf.addr.reg = TMC5160Reg_PWMCONF,
    .pwm_scale.addr.reg = TMC5160Reg_PWM_SCALE,
    .lost_steps.addr.reg = TMC5160Reg_LOST_STEPS,
#ifdef TMC5160_COMPLETE
    .xdirect.addr.reg = TMC5160Reg_XDIRECT,
    .mslut[0].addr.reg = TMC5160Reg_MSLUT_BASE,
    .mslut[1].addr.reg = (tmc5160_regaddr_t)(TMC5160Reg_MSLUT_BASE + 1),
    .mslut[2].addr.reg = (tmc5160_regaddr_t)(TMC5160Reg_MSLUT_BASE + 2),
    .mslut[3].addr.reg = (tmc5160_regaddr_t)(TMC5160Reg_MSLUT_BASE + 3),
    .mslut[4].addr.reg = (tmc5160_regaddr_t)(TMC5160Reg_MSLUT_BASE + 4),
    .mslut[5].addr.reg = (tmc5160_regaddr_t)(TMC5160Reg_MSLUT_BASE + 5),
    .mslut[6].addr.reg = (tmc5160_regaddr_t)(TMC5160Reg_MSLUT_BASE + 6),
    .mslut[7].addr.reg = (tmc5160_regaddr_t)(TMC5160Reg_MSLUT_BASE + 7),
    .mslutsel.addr.reg = TMC5160Reg_MSLUTSEL,
    .mslutstart.addr.reg = TMC5160Reg_MSLUTSTART,
    .encm_ctrl.addr.reg = TMC5160Reg_ENCM_CTRL,
#endif

#if TMC5160_COOLSTEP_ENABLE
    .coolconf.reg.semin = TMC5160_COOLSTEP_SEMIN,
    .coolconf.reg.semax = TMC5160_COOLSTEP_SEMAX,
#endif

    .chopconf.reg.intpol = TMC5160_INTERPOLATE,
    .chopconf.reg.toff = TMC5160_CONSTANT_OFF_TIME,
    .chopconf.reg.chm = TMC5160_CHOPPER_MODE,
    .chopconf.reg.tbl = TMC5160_BLANK_TIME,
#if TMC5160_CHOPPER_MODE == 0
    .chopconf.reg.hstrt = TMC5160_HSTRT,
    .chopconf.reg.hend = TMC5160_HEND,
#else
    .chopconf.reg.fd3 = (TMC5160_FAST_DECAY_TIME & 0x08) >> 3,
    .chopconf.reg.hstrt = TMC5160_FAST_DECAY_TIME & 0x07,
    .chopconf.reg.hend = TMC5160_SINE_WAVE_OFFSET,
#endif

    .ihold_irun.reg.irun = TMC5160_IRUN,
    .ihold_irun.reg.ihold = TMC5160_IHOLD,
    .ihold_irun.reg.iholddelay = TMC5160_IHOLDDELAY,

    .tpowerdown.reg.tpowerdown = TMC5160_TPOWERDOWN,

    .gconf.reg.en_pwm_mode = TMC5160_EN_PWM_MODE,

#if TMC5160_EN_PWM_MODE == 1 // stealthChop
    .pwmconf.reg.pwm_lim = 12,
    .pwmconf.reg.pwm_reg = 8,
    .pwmconf.reg.pwm_autograd = true,
    .pwmconf.reg.pwm_autoscale = true,
    .pwmconf.reg.pwm_freq = 0b01,
    .pwmconf.reg.pwm_grad = 14,
    .pwmconf.reg.pwm_ofs = 36,
#endif

    .tpwmthrs.reg.tpwmthrs = TMC5160_TPWM_THRS
};

static void set_tfd (TMC5160_chopconf_reg_t *chopconf, uint8_t fast_decay_time)
{
    chopconf->chm = 1;
    chopconf->fd3 = (fast_decay_time & 0x8) >> 3;
    chopconf->hstrt = fast_decay_time & 0x7;
}

void TMC5160_SetDefaults (TMC5160_t *driver)
{
    memcpy(driver, &tmc5160_defaults, sizeof(TMC5160_t));

    driver->chopconf.reg.mres = tmc_microsteps_to_mres(driver->config.microsteps);
}

bool TMC5160_Init (TMC5160_t *driver)
{
    // Read drv_status to check if driver is online
    tmc_spi_read(driver->motor, (TMC_spi_datagram_t *)&driver->drv_status);
    if(driver->drv_status.reg.value == 0 || driver->drv_status.reg.value == 0xFFFFFFFF)
        return false;

    // Perform a status register read to clear reset flag
    tmc_spi_read(driver->motor, (TMC_spi_datagram_t *)&driver->gstat);

    driver->chopconf.reg.mres = tmc_microsteps_to_mres(driver->config.microsteps);
    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->gconf);
    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->chopconf);
    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->coolconf);
    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->pwmconf);
    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->ihold_irun);
    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->tpowerdown);
    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->tpwmthrs);

    TMC5160_SetCurrent(driver, driver->config.current, driver->config.hold_current_pct);

    //set to a conservative start value
    //TMC5160_SetConstantOffTimeChopper(driver->motor, 5, 24, 13, 12, true); // move to default values

    // Read back chopconf to check if driver is online
    uint32_t chopconf = driver->chopconf.reg.value;
    tmc_spi_read(driver->motor, (TMC_spi_datagram_t *)&driver->chopconf);

    return driver->chopconf.reg.value == chopconf;
}

uint_fast16_t cs2rms (TMC5160_t *driver, uint8_t CS)
{
    uint32_t numerator = (driver->global_scaler.reg.scaler ? driver->global_scaler.reg.scaler : 256) * (CS + 1);
    numerator *= 325;
    numerator >>= (8 + 5); // Divide by 256 and 32
    numerator *= 1000000;
    uint32_t denominator = driver->config.r_sense;
    denominator *= 1414;

    return numerator / denominator;
}

uint16_t TMC5160_GetCurrent (TMC5160_t *driver)
{
    return cs2rms(driver, driver->ihold_irun.reg.irun);
}

// r_sense = mOhm, Vsense = mV, current = mA (RMS)
void TMC5160_SetCurrent (TMC5160_t *driver, uint16_t mA, uint8_t hold_pct)
{
    driver->config.current = mA;
    driver->config.hold_current_pct = hold_pct;

    const uint32_t V_fs = 325; // 0.325 * 1000
    uint_fast8_t CS = 31;
    uint32_t scaler = 0; // = 256

    uint16_t RS_scaled = ((float)driver->config.r_sense / 1000.f) * 0xFFFF; // Scale to 16b
    uint32_t numerator = 11585; // 32 * 256 * sqrt(2)
    numerator *= RS_scaled;
    numerator >>= 8;
    numerator *= mA;

    do {
        uint32_t denominator = V_fs * 0xFFFF >> 8;
        denominator *= CS + 1;
        scaler = numerator / denominator;
        if (scaler > 255)
            scaler = 0; // Maximum
        else if (scaler < 128)
            CS--;  // Try again with smaller CS
    } while(scaler && scaler < 128);

    driver->global_scaler.reg.scaler = scaler;
    driver->ihold_irun.reg.irun = CS > 31 ? 31 : CS;
    driver->ihold_irun.reg.ihold = (driver->ihold_irun.reg.irun * driver->config.hold_current_pct) / 100;

    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->global_scaler);
    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->ihold_irun);
}

uint32_t TMC5160_GetTPWMTHRS (TMC5160_t *driver, float steps_mm)
{
    return (uint32_t)((driver->config.microsteps * TMC5160_F_CLK) / (256 * driver->tpwmthrs.reg.tpwmthrs * steps_mm));
}

void TMC5160_SetHybridThreshold (TMC5160_t *driver, float mm_sec, float steps_mm) // -> pwm threshold
{
    driver->tpwmthrs.reg.tpwmthrs = tmc_calc_tstep(&driver->config, mm_sec, steps_mm);
    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->tpwmthrs);
}

void TMC5160_SetTHIGH (TMC5160_t *driver, float mm_sec, float steps_mm) // -> pwm threshold
{
    driver->thigh.reg.thigh = tmc_calc_tstep(&driver->config, mm_sec, steps_mm);
    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->thigh);
}

void TMC5160_SetTCOOLTHRS (TMC5160_t *driver, float mm_sec, float steps_mm) // -> pwm threshold
{
    driver->tcoolthrs.reg.tcoolthrs = tmc_calc_tstep(&driver->config, mm_sec, steps_mm);
    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->tcoolthrs);
}

// 1 - 256 in steps of 2^value is valid for TMC5160
bool TMC5160_MicrostepsIsValid (uint16_t usteps)
{
    return tmc_microsteps_validate(usteps);
}

void TMC5160_SetMicrosteps (TMC5160_t *driver, tmc5160_microsteps_t msteps)
{
    driver->chopconf.reg.mres = tmc_microsteps_to_mres(msteps);
    driver->config.microsteps = (tmc5160_microsteps_t)(1 << (8 - driver->chopconf.reg.mres));
// TODO: recalc and set hybrid threshold if enabled?
    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->chopconf);
}

void TMC5160_SetConstantOffTimeChopper (TMC5160_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator)
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

    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->chopconf);
}

TMC5160_status_t TMC5160_WriteRegister (TMC5160_t *driver, TMC5160_datagram_t *reg)
{
    TMC5160_status_t status;

    status.value = tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)reg);

    return status;
}

TMC5160_status_t TMC5160_ReadRegister (TMC5160_t *driver, TMC5160_datagram_t *reg)
{
    TMC5160_status_t status;

    status.value = tmc_spi_read(driver->motor, (TMC_spi_datagram_t *)reg);

    return status;
}

// Returns pointer to shadow register or NULL if not found
TMC5160_datagram_t *TMC5160_GetRegPtr (TMC5160_t *driver, tmc5160_regaddr_t reg)
{
    TMC5160_datagram_t *ptr = (TMC5160_datagram_t *)driver;

    while(ptr && ptr->addr.reg != reg) {
        ptr++;
        if(ptr->addr.reg == TMC5160Reg_LOST_STEPS && ptr->addr.reg != reg)
            ptr = NULL;
    }

    return ptr;
}

