/*
  trinamic.c - Trinamic TMC2130 integration

  Part of Grbl

  Copyright (c) 2018 Terje Io

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

#include "driver.h"

#ifdef TRINAMIC_ENABLE

#include <stdio.h>

#ifdef TRINAMIC_I2C
#include "i2c.h"
#else
#include "spi.h"
#endif

#define ENABLE_DRIVER_SETTING 90
#define ENABLE_HOMING_SETTING 91

char const *const axis_letter[] = {
    "X",
    "Y",
    "Z",
    "A",
    "B",
    "C"
};

static TMC2130_t stepper[N_AXIS];
static axes_signals_t homing = {0};
static limits_get_state_ptr limits_get_state = NULL;

// Wrapper for initializing physical interface (since two alternatives are provided)
void SPI_DriverInit (SPI_driver_t *driver)
{
#ifdef __SPI_DRIVER_H__
    SPI__DriverInit(driver);
#else
    I2C_DriverInit(driver);
#endif
}

void trinamic_init (void)
{
    uint_fast8_t idx = N_AXIS;

#ifdef __SPI_DRIVER_H__
    static chip_select_t cs[N_AXIS];

    cs[0].port = SPI_CS_PORT_X;
    cs[0].pin = SPI_CS_PIN_X;
    cs[1].port = SPI_CS_PORT_Y;
    cs[1].pin = SPI_CS_PIN_Y;
    cs[2].port = SPI_CS_PORT_Z;
    cs[2].pin = SPI_CS_PIN_Z;

    // TODO: add definitions for other axes if more than three!
#endif

    do {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(--idx))) {

            TMC2130_SetDefaults(&stepper[idx]);
#ifdef __SPI_DRIVER_H__
            stepper[idx].cs_pin = &cs[idx];
            GPIOPinTypeGPIOOutput(cs[idx].port, cs[idx].pin);
            GPIOPinWrite(cs[idx].port, cs[idx].pin, cs[idx].pin);
#else
            stepper[idx].cs_pin = (void *)idx;
#endif
            stepper[idx].current = driver_settings.trinamic.driver[idx].current;
            stepper[idx].microsteps = driver_settings.trinamic.driver[idx].microsteps;
            stepper[idx].r_sense = driver_settings.trinamic.driver[idx].r_sense;

            TMC2130_Init(&stepper[idx]);
        }
    } while(idx);
}

void trinamic_configure (void)
{
    uint_fast8_t idx = N_AXIS;

    do {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(--idx))) {
            stepper[idx].r_sense = driver_settings.trinamic.driver[idx].r_sense;
            TMC2130_SetCurrent(&stepper[idx], driver_settings.trinamic.driver[idx].current, stepper[idx].hold_current_pct);
            TMC2130_SetMicrosteps(&stepper[idx], driver_settings.trinamic.driver[idx].microsteps);
        }
    } while(idx);
}

// 1 - 256 in steps of 2^value is valid for TMC2130 TODO: move code to Trinamic core driver?
static bool validate_microstepping (uint16_t value)
{
    uint_fast8_t i = 8, count = 0;

    do {
        if(value & 0x01)
            count++;
        value >>= 1;
    } while(i--);

    return count == 1;
}

bool trinamic_setting (uint_fast16_t setting, float value, char *svalue)
{
    bool ok = false;

    if((setting_type_t)setting >= Setting_AxisSettingsBase && (setting_type_t)setting <= Setting_AxisSettingsMax) {

        uint_fast16_t base_idx = (uint_fast16_t)setting - (uint_fast16_t)Setting_AxisSettingsBase;
        uint_fast8_t idx = base_idx % AXIS_SETTINGS_INCREMENT;

        if(idx < N_AXIS) switch((base_idx - idx) / AXIS_SETTINGS_INCREMENT) {

            case AxisSetting_StepperCurrent:
                ok = true;
                driver_settings.trinamic.driver[idx].current = (uint16_t)value;
                break;

            case AxisSetting_MicroSteps:
                if((ok = validate_microstepping((uint16_t)value)))
                    driver_settings.trinamic.driver[idx].microsteps = (tmc2130_microsteps_t)value;
                break;
        }
    } else switch(setting) {

        case ENABLE_DRIVER_SETTING:
            ok = true;
            driver_settings.trinamic.driver_enable.mask = (uint8_t)value & AXES_BITMASK;
            break;

        case ENABLE_HOMING_SETTING:
            ok = true;
            driver_settings.trinamic.homing_enable.mask = (uint8_t)value & AXES_BITMASK;
            break;
    }

    return ok;
}

void trinamic_settings_restore (uint8_t restore_flag)
{
    if(restore_flag & SETTINGS_RESTORE_DRIVER_PARAMETERS) {

        uint_fast8_t idx = N_AXIS;

        driver_settings.trinamic.driver_enable.mask = 0;
        driver_settings.trinamic.homing_enable.mask = 0;

        do {
            idx--;
            driver_settings.trinamic.driver[idx].current = TMC2130_CURRENT;
            driver_settings.trinamic.driver[idx].microsteps = TMC2130_MICROSTEPS;
            driver_settings.trinamic.driver[idx].r_sense = TMC2130_R_SENSE;
            driver_settings.trinamic.driver[idx].homing_sensitivity = 0;
        } while(idx);
    }
}

void trinamic_settings_report (bool axis_settings, axis_setting_type_t setting_type, uint8_t axis_idx)
{
    if(axis_settings) {

        setting_type_t basetype = (setting_type_t)(Setting_AxisSettingsBase + setting_type * AXIS_SETTINGS_INCREMENT);

        switch(setting_type) {

            case AxisSetting_StepperCurrent:
                report_uint_setting((setting_type_t)(basetype + axis_idx), driver_settings.trinamic.driver[axis_idx].current);
                break;

            case AxisSetting_MicroSteps:
                report_uint_setting((setting_type_t)(basetype + axis_idx), driver_settings.trinamic.driver[axis_idx].microsteps);
                break;
        }
    } else {
        report_uint_setting((setting_type_t)ENABLE_DRIVER_SETTING, driver_settings.trinamic.driver_enable.mask);
        report_uint_setting((setting_type_t)ENABLE_HOMING_SETTING, driver_settings.trinamic.homing_enable.mask);
    }
}

static char *append (char *s)
{
    while(*s) s++;

    return s;
}

static void write_line (char *s)
{
    strcat(s, "\r\n");
    hal.stream.write(s);
}

static void write_report (void)
{
    /* Marlin format debug report

     SENDING:M122
    X   Y
    Enabled     false   false
    Set current 850 850
    RMS current 826 826
    MAX current 1165    1165
    Run current 26/31   26/31
    Hold current    13/31   13/31
    CS actual       13/31   13/31
    PWM scale   41  41
    vsense      1=.18   1=.18
    stealthChop true    true
    msteps      16  16
    tstep       1048575 1048575
    pwm
    threshold       0   0
    [mm/s]      -   -
    OT prewarn  false   false
    OT prewarn has
    been triggered  false   false
    off time        5   5
    blank time  24  24
    hysterisis
    -end        2   2
    -start      3   3
    Stallguard thrs 0   0
    DRVSTATUS   X   Y
    stallguard
    sg_result       0   0
    fsactive
    stst        X   X
    olb
    ola
    s2gb
    s2ga
    otpw
    ot
    'Driver registers:'
        X = 0x80:0D:00:00
        Y = 0x80:0D:00:00
*/

    char buf[65];
    uint_fast8_t idx = N_AXIS;

    hal.stream.write("Trinamic TMC2130:\r\n");

    do {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(--idx))) {
            TMC2130_ReadRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].chopconf);
            TMC2130_ReadRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].drv_status);
            TMC2130_ReadRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].pwm_scale);
            TMC2130_ReadRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].tstep);
        }
    } while(idx);

    sprintf(buf, "%-13s", "");
    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%8s", axis_letter[idx]);
    }
    write_line(buf);

    sprintf(buf, "%-13s", "run current");
    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%5d/31", stepper[idx].ihold_irun.reg.irun);
    }
    write_line(buf);

    sprintf(buf, "%-13s", "hold current");
    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%5d/31", stepper[idx].ihold_irun.reg.ihold);
    }
    write_line(buf);

    sprintf(buf, "%-13s", "CS actual");
    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%5d/31", stepper[idx].drv_status.reg.cs_actual);
    }
    write_line(buf);

    sprintf(buf, "%-13s", "PWM scale");
    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%8d", stepper[idx].pwm_scale.reg.pwm_scale);
    }
    write_line(buf);

    sprintf(buf, "%-13s", "vsense");
    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%8s", stepper[idx].chopconf.reg.vsense ? "1=0.18" : "0=0.32");
    }
    write_line(buf);

    sprintf(buf, "%-13s", "mstep");
    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%8d", 1 << (8 - stepper[idx].chopconf.reg.mres));
    }
    write_line(buf);

    sprintf(buf, "%-13s", "tstep");
    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%8d", stepper[idx].tstep.reg.tstep);
    }

    write_line(buf);

    hal.stream.write("PWM:\n\r");

    sprintf(buf, "%-13s", "pwm autoscale");

    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%8d", stepper[idx].pwmconf.reg.pwm_autoscale);
    }
    write_line(buf);

    sprintf(buf, "%-13s", "pwm ampl");
    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%8d", stepper[idx].pwmconf.reg.pwm_ampl);
    }
    write_line(buf);

    sprintf(buf, "%-13s", "pwm grad");
    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%8d", stepper[idx].pwmconf.reg.pwm_grad);
    }
    write_line(buf);

    sprintf(buf, "%-13s", "off time");
    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%8d", stepper[idx].chopconf.reg.toff);
    }
    write_line(buf);


    sprintf(buf, "%-13s", "blank time");
    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%8d", stepper[idx].chopconf.reg.tbl);
    }
    write_line(buf);

    hal.stream.write("Hysteresis:\r\n");

    sprintf(buf, "%-13s", "-end");
    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%8d", stepper[idx].chopconf.reg.hend);
    }
    write_line(buf);

    sprintf(buf, "%-13s", "-start");
    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%8d", stepper[idx].chopconf.reg.hstrt);
    }
    write_line(buf);

    hal.stream.write("DRIVER STATUS:\r\n");

    sprintf(buf, "%-13s", "otpw");
    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%8s", stepper[idx].drv_status.reg.otpw ? "*" : "");
    }
    write_line(buf);

    sprintf(buf, "%-13s", "ot");
    for(idx = 0; idx < N_AXIS; idx++) {
        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx)))
            sprintf(append(buf), "%8s", stepper[idx].drv_status.reg.ot ? "*" : "");
    }
    write_line(buf);
}

static bool check_params (parser_block_t *gc_block, uint_fast16_t *value_words)
{
    bool ok = false;
    uint_fast8_t idx = N_AXIS;

    static const uint8_t wordmap[] = {
       Word_X,
       Word_Y,
       Word_Z
#if N_AXIS > 3
      ,Word_A,
       Word_B,
       Word_C
#endif
    };

    do {
        idx--;
        if(bit_istrue(*value_words, bit(wordmap[idx])) && bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx))) {
            ok = true;
            bit_false(*value_words, bit(wordmap[idx]));
        } else
            gc_block->values.xyz[idx] = NAN;
    } while(idx);

    return ok;
}

uint_fast16_t trimamic_MCodeCheck (uint_fast16_t mcode)
{
    return driver_settings.trinamic.driver_enable.mask &&
            (mcode == 121 || mcode == 906 || mcode == 911 || mcode == 912 || mcode == 913 || mcode == 914) ? mcode : 0;
}

status_code_t trimamic_MCodeValidate (parser_block_t *gc_block, uint_fast16_t *value_words)
{
    status_code_t state = Status_GcodeValueWordMissing;

    switch(gc_block->driver_mcode) {

        case 121:
            state = Status_OK;
//            gc_block->driver_mcode_sync = true;
            break;

        case 906:
            if(check_params(gc_block, value_words)) {
                state = Status_OK;
                gc_block->driver_mcode_sync = true;
                if(bit_isfalse(*value_words, bit(Word_Q)))
                    gc_block->values.q = NAN;
                else // TODO: add range check?
                    bit_false(*value_words, bit(Word_Q));
            }
            break;

        case 911:
        case 912:
            state = Status_OK;
            break;

        case 913:
            if(check_params(gc_block, value_words)) {
                state = Status_OK;
                gc_block->driver_mcode_sync = true;
            }
            break;

        case 914: // TODO: allow negative settings parameters?
            if(check_params(gc_block, value_words)) {
                uint_fast8_t idx = N_AXIS;
                state = Status_OK;
                do {
                    idx--;
                    if(!isnan(gc_block->values.xyz[idx]) && (gc_block->values.xyz[idx] < -64.0f || gc_block->values.xyz[idx] > 63.0f))
                        state = Status_BadNumberFormat;
                } while(idx && state == Status_OK);
            }
            break;
    }

    return state;
}

void trimamic_MCodeExecute (uint_fast16_t state, parser_block_t *gc_block)
{
    uint_fast8_t idx = N_AXIS;

    switch(gc_block->driver_mcode) {

        case 121:
            write_report();
            break;

        case 906:
            do {
                idx--;
                if(!isnan(gc_block->values.xyz[idx]))
                    TMC2130_SetCurrent(&stepper[idx], (uint16_t)gc_block->values.xyz[idx],
                                        isnan(gc_block->values.q) ? stepper[idx].hold_current_pct : (uint8_t)gc_block->values.q);
            } while(idx);
            break;

        case 911:; // TODO: format grbl style?
            char buf[15];
            TMC2130_status_t status;
            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx))) {
                    status = TMC2130_ReadRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].drv_status);
                    strcpy(buf, axis_letter[idx]);
                    strcat(buf, ":");
                    if(status.driver_error)
                        strcat(buf, "E");
                    else if(stepper[idx].drv_status.reg.ot)
                        strcat(buf, "O");
                    else if(stepper[idx].drv_status.reg.otpw)
                        strcat(buf, "W");
                    write_line(buf);
                }
            }
            break;

        case 912:
            // TODO: clear prewarn flags - Marlin has a counter for otpw, implement similar autotune? M906S
            break;

        case 913:
            do {
                idx--;
                if(!isnan(gc_block->values.xyz[idx]))
                    TMC2130_SetHybridThreshold(&stepper[idx], (uint32_t)gc_block->values.xyz[idx], settings.steps_per_mm[idx]);
            } while(idx);
            break;

        case 914:
            do {
                idx--;
                if(!isnan(gc_block->values.xyz[idx]))
                    driver_settings.trinamic.driver[idx].homing_sensitivity = (uint8_t)gc_block->values.xyz[idx];
            } while(idx);
            break;
    }
}

// hal.limits_get_state is redirecte here when homing
static axes_signals_t trinamic_limits (void)
{
    axes_signals_t signals = limits_get_state(); // read from switches first

    signals.mask &= ~homing.mask;

    uint_fast8_t idx = N_AXIS;
    do {
        if(bit_istrue(homing.mask, bit(--idx))) {
            // TODO: read or just set flag in irq handler instead of reading here to reduce overhead?
            // DIAG1 output can be used for that but requires wiring
            TMC2130_ReadRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].drv_status);
            if(stepper[idx].drv_status.reg.stallGuard)
                bit_true(signals.mask, idx);
        }
    } while(idx);

    return signals;
}

void trinamic_homing (bool enable)
{
    uint_fast8_t idx = N_AXIS;

    homing.mask = driver_settings.trinamic.driver_enable.mask & driver_settings.trinamic.homing_enable.mask;

    enable = enable & homing.mask;

    do {
        if(bit_istrue(homing.mask, bit(--idx))) {
            stepper[idx].gconf.reg.diag1_stall = enable;
            stepper[idx].gconf.reg.en_pwm_mode = !enable; // stealthChop
            TMC2130_WriteRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].gconf);

            stepper[idx].tcoolthrs.reg.tcoolthrs = enable ? (1 << 20) - 1 : 0;
            TMC2130_WriteRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].tcoolthrs);

            stepper[idx].coolconf.reg.sgt = driver_settings.trinamic.driver[idx].homing_sensitivity;
            TMC2130_WriteRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].coolconf);
        }
    } while(idx);

    if(enable) {
        if(limits_get_state == NULL) {
            limits_get_state = hal.limits_get_state;
            hal.limits_get_state = trinamic_limits;
        }
    } else if(limits_get_state != NULL) {
        hal.limits_get_state = limits_get_state;
        limits_get_state = NULL;
    }
}

#endif
