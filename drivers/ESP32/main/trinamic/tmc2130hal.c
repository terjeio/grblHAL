/*
 * tmc2130hal.c - interface for Trinamic TMC2130 stepper driver
 *
 * v0.0.1 / 2021-02-07 / (c) Io Engineering / Terje
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

#include <stdlib.h>
#include <string.h>

#include "tmc2130.h"
#include "tmchal.h"

static TMC2130_t *tmcdriver[6];

static trinamic_config_t *getConfig (uint8_t axis)
{
    return &tmcdriver[axis]->config;
}

static bool MicrostepsIsValid (uint8_t axis, uint16_t msteps)
{
    return tmc_microsteps_validate((tmc2130_microsteps_t)msteps);
}

static void SetMicrosteps (uint8_t axis, uint16_t msteps)
{
   TMC2130_SetMicrosteps(tmcdriver[axis], (tmc2130_microsteps_t)msteps);
}

static void SetCurrent (uint8_t axis, uint16_t mA, uint8_t hold_pct)
{
    TMC2130_SetCurrent(tmcdriver[axis], mA, hold_pct);
}

static uint16_t GetCurrent (uint8_t axis)
{
    return TMC2130_GetCurrent(tmcdriver[axis]);
}

static TMC_chopconf_t GetChopconf (uint8_t axis)
{
    TMC_chopconf_t chopconf;

    tmc_spi_read(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->chopconf);

    chopconf.mres = tmcdriver[axis]->chopconf.reg.mres;
    chopconf.toff = tmcdriver[axis]->chopconf.reg.toff;
    chopconf.tbl = tmcdriver[axis]->chopconf.reg.tbl;
    chopconf.hend = tmcdriver[axis]->chopconf.reg.hend;
    chopconf.hstrt = tmcdriver[axis]->chopconf.reg.hstrt;

    return chopconf;
}

static TMC_drv_status_t GetDriverStatus (uint8_t axis)
{
    TMC_drv_status_t drv_status;
    TMC2130_status_t status;

    status.value = tmc_spi_read(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->drv_status);

    drv_status.driver_error = status.driver_error;
    drv_status.sg_result = tmcdriver[axis]->drv_status.reg.sg_result;
    drv_status.ot = tmcdriver[axis]->drv_status.reg.ot;
    drv_status.otpw = tmcdriver[axis]->drv_status.reg.otpw;
    drv_status.cs_actual = tmcdriver[axis]->drv_status.reg.cs_actual;
    drv_status.stst = tmcdriver[axis]->drv_status.reg.stst;
    drv_status.fsactive = tmcdriver[axis]->drv_status.reg.fsactive;
    drv_status.ola = tmcdriver[axis]->drv_status.reg.ola;
    drv_status.olb = tmcdriver[axis]->drv_status.reg.olb;
    drv_status.s2ga = tmcdriver[axis]->drv_status.reg.s2ga;
    drv_status.s2gb = tmcdriver[axis]->drv_status.reg.s2gb;

    return drv_status;
}

static TMC_ihold_irun_t GetIholdIrun (uint8_t axis)
{
    TMC_ihold_irun_t ihold_irun;

    ihold_irun.ihold = tmcdriver[axis]->ihold_irun.reg.ihold;
    ihold_irun.irun = tmcdriver[axis]->ihold_irun.reg.irun;
    ihold_irun.iholddelay = tmcdriver[axis]->ihold_irun.reg.iholddelay;

    return ihold_irun;
}

static uint32_t GetDriverStatusRaw (uint8_t axis)
{
    tmc_spi_read(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->drv_status);

    return tmcdriver[axis]->drv_status.reg.value;
}

static uint32_t GetTStep (uint8_t axis)
{
    tmc_spi_read(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->tstep);

    return (uint32_t)tmcdriver[axis]->tstep.reg.tstep;
}

static void SetTHigh (uint8_t axis, float mm_sec, float steps_mm)
{
    TMC2130_SetTHIGH(tmcdriver[axis], mm_sec, steps_mm);
}

static void SetTHighRaw (uint8_t axis, uint32_t value)
{
    tmcdriver[axis]->thigh.reg.thigh = value;
    tmc_spi_write(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->thigh);
}

static void SetTCoolThrs (uint8_t axis, float mm_sec, float steps_mm)
{
    TMC2130_SetTCOOLTHRS(tmcdriver[axis], mm_sec, steps_mm);
}

static void SetTCoolThrsRaw (uint8_t axis, uint32_t value)
{
    tmcdriver[axis]->tcoolthrs.reg.tcoolthrs = value;
    tmc_spi_write(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->tcoolthrs);
}

static void stallGuardEnable (uint8_t axis, bool enable, uint8_t sensitivity)
{
    TMC2130_t *driver = tmcdriver[axis];

    driver->gconf.reg.diag1_stall = enable;
    driver->gconf.reg.en_pwm_mode = !enable; // stealthChop
    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->gconf);

    driver->pwmconf.reg.pwm_autoscale = !enable;
    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->pwmconf);

    driver->tcoolthrs.reg.tcoolthrs = enable ? (1 << 20) - 1 : 0;
    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->tcoolthrs);

    driver->coolconf.reg.sgt = sensitivity & 0x7F; // 7-bits signed value
    tmc_spi_write(driver->motor, (TMC_spi_datagram_t *)&driver->coolconf);
}

static uint32_t GetTPWMThrs (uint8_t axis, float steps_mm)
{
    return TMC2130_GetTPWMTHRS(tmcdriver[axis], steps_mm);
}

static uint32_t GetTPWMThrsRaw (uint8_t axis)
{
    tmc_spi_read(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->tpwmthrs);

    return tmcdriver[axis]->tpwmthrs.reg.tpwmthrs;
}

// gconf

static void stealthChop (uint8_t axis, bool val)
{
    tmcdriver[axis]->gconf.reg.en_pwm_mode = val;
    tmc_spi_write(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->gconf);
}

static bool stealthChopGet (uint8_t axis)
{
    return tmcdriver[axis]->gconf.reg.en_pwm_mode;
}

// coolconf

static void sg_filter (uint8_t axis, bool val)
{
    tmcdriver[axis]->coolconf.reg.sfilt = val;
    tmc_spi_write(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->coolconf);
}

static void sg_stall_value (uint8_t axis, uint8_t val)
{
    tmcdriver[axis]->coolconf.reg.sgt = val;
    tmc_spi_write(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->coolconf);
}

static uint8_t get_sg_stall_value (uint8_t axis)
{
    return tmcdriver[axis]->coolconf.reg.sgt;
}

static void sedn (uint8_t axis, uint8_t val)
{
    tmcdriver[axis]->coolconf.reg.sedn = val;
    tmc_spi_write(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->coolconf);
}

static void semin (uint8_t axis, uint8_t val)
{
    tmcdriver[axis]->coolconf.reg.semin = val;
    tmc_spi_write(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->coolconf);
}

static void semax (uint8_t axis, uint8_t val)
{
    tmcdriver[axis]->coolconf.reg.semax = val;
    tmc_spi_write(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->coolconf);
}

// chopconf

static void toff (uint8_t axis, uint8_t val)
{
    tmcdriver[axis]->chopconf.reg.toff = val;
    tmc_spi_write(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->chopconf);
}

static void tbl (uint8_t axis, uint8_t val)
{
    tmcdriver[axis]->chopconf.reg.tbl = val;
    tmc_spi_write(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->chopconf);
}

static void chopper_mode (uint8_t axis, uint8_t val)
{
    tmcdriver[axis]->chopconf.reg.chm = val;
    tmc_spi_write(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->chopconf);
}

static void hysteresis_start (uint8_t axis, uint8_t val)
{
    tmcdriver[axis]->chopconf.reg.hstrt = (uint8_t)(val - 1) & 0x07;
    tmc_spi_write(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->chopconf);
}

static void hysteresis_end (uint8_t axis, int8_t val)
{
    tmcdriver[axis]->chopconf.reg.hend = (uint8_t)(val + 3);
    tmc_spi_write(tmcdriver[axis]->motor, (TMC_spi_datagram_t *)&tmcdriver[axis]->chopconf);
}

static const tmchal_t hal = {
    .driver = TMC2130,
    .name = "TMC2130",

    .get_config = getConfig,

    .microsteps_isvalid = MicrostepsIsValid,
    .set_microsteps = SetMicrosteps,
    .set_current = SetCurrent,
    .get_current = GetCurrent,
    .get_chopconf = GetChopconf,
    .get_tstep = GetTStep,
    .get_drv_status = GetDriverStatus,
    .get_drv_status_raw = GetDriverStatusRaw,
    .set_tcoolthrs = SetTCoolThrs,
    .set_tcoolthrs_raw = SetTCoolThrsRaw,
    .set_thigh = SetTHigh,
    .set_thigh_raw = SetTHighRaw,
    .stallguard_enable = stallGuardEnable,
    .get_tpwmthrs = GetTPWMThrs,
    .get_tpwmthrs_raw = GetTPWMThrsRaw,
    .get_en_pwm_mode = stealthChopGet,
    .get_ihold_irun = GetIholdIrun,

    .stealthChop = stealthChop,
    .sg_filter = sg_filter,
    .sg_stall_value = sg_stall_value,
    .get_sg_stall_value = get_sg_stall_value,
    .sedn = sedn,
    .semin = semin,
    .semax = semax,
    .toff = toff,
    .tbl = tbl,
    .chopper_mode = chopper_mode,
    .hysteresis_start = hysteresis_start,
    .hysteresis_end = hysteresis_end
};

const tmchal_t *TMC2130_AddAxis (uint8_t axis, uint16_t current, uint8_t microsteps, uint8_t r_sense)
{
    bool ok = !!tmcdriver[axis];

    if(!ok && (ok = (tmcdriver[axis] = malloc(sizeof(TMC2130_t))) != NULL)) {
        TMC2130_SetDefaults(tmcdriver[axis]);
        tmcdriver[axis]->motor.axis = axis;
        tmcdriver[axis]->config.current = current;
        tmcdriver[axis]->config.microsteps = microsteps;
        tmcdriver[axis]->config.r_sense = r_sense;
        tmcdriver[axis]->chopconf.reg.mres = tmc_microsteps_to_mres(microsteps);
    }

    if(ok && !(ok = TMC2130_Init(tmcdriver[axis])))
        free(tmcdriver[axis]);

    return ok ? &hal : NULL;
}
