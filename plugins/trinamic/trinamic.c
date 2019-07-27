/*
  trinamic.c - Trinamic TMC2130 plugin

  Part of Grbl

  Copyright (c) 2018-2019 Terje Io

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

#include "../driver.h"

#if TRINAMIC_ENABLE

#include <stdio.h>
#include <math.h>

#if TRINAMIC_I2C
#include "i2c.h"
#else
#include "spi.h"
#endif

static bool warning = false, is_homing = false;
static volatile uint_fast16_t diag1_poll = 0;
static char sbuf[65]; // string buffer for reports
static TMC2130_t stepper[N_AXIS];
static axes_signals_t homing = {0}, otpw_triggered = {0};
static limits_get_state_ptr limits_get_state = NULL;
static void (*hal_stepper_pulse_start)(stepper_t *stepper) = NULL;
static void (*hal_execute_realtime)(uint_fast16_t state) = NULL;
static struct {
    axes_signals_t axes;
    bool raw;
    bool sg_status_enable;
    volatile bool sg_status;
    bool sfilt;
    uint32_t sg_status_axis;
    uint32_t msteps;
} report = {0};
static TMC2130_datagram_t *reg_ptr = NULL;

#if TRINAMIC_I2C
TMCI2C_enable_dgr_t dgr_enable = {
    .addr.reg = TMC_I2CReg_ENABLE
};

TMCI2C_monitor_status_dgr_t dgr_monitor = {
    .addr.reg = TMC_I2CReg_MON_STATE
};
#endif

static void write_debug_report (void);

// Wrapper for initializing physical interface (since two alternatives are provided)
void TMC_DriverInit (TMC_io_driver_t *driver)
{
#if TRINAMIC_I2C
    I2C_DriverInit(driver);
#else
    SPI_DriverInit(driver);
#endif
}

void trinamic_init (void)
{
    uint_fast8_t idx = N_AXIS;

#if !TRINAMIC_I2C
    static chip_select_t cs[N_AXIS];

    cs[X_AXIS].port = SPI_CS_PORT_X;
    cs[X_AXIS].pin = SPI_CS_PIN_X;
    cs[Y_AXIS].port = SPI_CS_PORT_Y;
    cs[Y_AXIS].pin = SPI_CS_PIN_Y;
    cs[Z_AXIS].port = SPI_CS_PORT_Z;
    cs[Z_AXIS].pin = SPI_CS_PIN_Z;

    // TODO: add definitions for other axes if more than three!
#endif

    TMC_IOInit();

    do {

        if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(--idx))) {

            TMC2130_SetDefaults(&stepper[idx]); // Init shadow registers to default values

#if !TRINAMIC_I2C
            stepper[idx].cs_pin = &cs[idx];
            GPIOPinTypeGPIOOutput(cs[idx].port, cs[idx].pin);
            GPIOPinWrite(cs[idx].port, cs[idx].pin, cs[idx].pin);
#else
            stepper[idx].cs_pin = (void *)idx;
#endif
            stepper[idx].current = driver_settings.trinamic.driver[idx].current;
            stepper[idx].microsteps = driver_settings.trinamic.driver[idx].microsteps;
            stepper[idx].r_sense = driver_settings.trinamic.driver[idx].r_sense;

            switch(idx) {

                case X_AXIS:
                  #ifdef TMC_X_ADVANCED
                    TMC_X_ADVANCED
                  #endif
                  #if TRINAMIC_I2C && TMC_X_MONITOR
                    dgr_enable.reg.monitor.x = TMC_X_MONITOR;
                  #endif
                    break;

                case Y_AXIS:
                  #ifdef TMC_Y_ADVANCED
                    TMC_Y_ADVANCED
                  #endif
                  #if TRINAMIC_I2C && TMC_Y_MONITOR
                    dgr_enable.reg.monitor.x = TMC_Y_MONITOR;
                  #endif
                    break;

                case Z_AXIS:
                  #ifdef TMC_Z_ADVANCED
                    TMC_Z_ADVANCED
                  #endif
                  #if TRINAMIC_I2C && TMC_Z_MONITOR
                    dgr_enable.reg.monitor.x = TMC_Z_MONITOR;
                  #endif
                    break;
            }

            TMC2130_Init(&stepper[idx]);
          #if TRINAMIC_I2C
            TMC2130_WriteRegister(NULL, (TMC2130_datagram_t *)&dgr_enable);
          #endif
        }
    } while(idx);
}

// Update driver settings on changes
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

// Parse and set driver specific parameters
bool trinamic_setting (setting_type_t setting, float value, char *svalue)
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
                if((ok = TMC2130_MicrostepsIsValid((uint16_t)value)))
                    driver_settings.trinamic.driver[idx].microsteps = (tmc2130_microsteps_t)value;
                break;
        }
    } else switch((setting_type_t)setting) {

        case Setting_TrinamicDriver:
            ok = true;
            driver_settings.trinamic.driver_enable.mask = (uint8_t)value & AXES_BITMASK;
            break;

        case Setting_TrinamicHoming:
            ok = true;
            driver_settings.trinamic.homing_enable.mask = (uint8_t)value & AXES_BITMASK;
            break;
    }

    return ok;
}

// Initialize default EEPROM settings
void trinamic_settings_restore (uint8_t restore_flag)
{
    if(restore_flag & SETTINGS_RESTORE_DRIVER_PARAMETERS) {

        uint_fast8_t idx = N_AXIS;

        driver_settings.trinamic.driver_enable.mask = 0;
        driver_settings.trinamic.homing_enable.mask = 0;

        do {
            switch(--idx) {

                case X_AXIS:
                    driver_settings.trinamic.driver_enable.x = TMC_X_ENABLE;
                    driver_settings.trinamic.driver[idx].current = TMC_X_CURRENT;
                    driver_settings.trinamic.driver[idx].microsteps = TMC_X_MICROSTEPS;
                    driver_settings.trinamic.driver[idx].r_sense = TMC_X_R_SENSE;
                    driver_settings.trinamic.driver[idx].homing_sensitivity = TMC_X_SGT;
                    break;

                case Y_AXIS:
                    driver_settings.trinamic.driver[idx].current = TMC_Y_CURRENT;
                    driver_settings.trinamic.driver[idx].microsteps = TMC_Y_MICROSTEPS;
                    driver_settings.trinamic.driver[idx].r_sense = TMC_Y_R_SENSE;
                    driver_settings.trinamic.driver[idx].homing_sensitivity = TMC_Y_SGT;
                    break;

                case Z_AXIS:
                    driver_settings.trinamic.driver[idx].current = TMC_Z_CURRENT;
                    driver_settings.trinamic.driver[idx].microsteps = TMC_Z_MICROSTEPS;
                    driver_settings.trinamic.driver[idx].r_sense = TMC_Z_R_SENSE;
                    driver_settings.trinamic.driver[idx].homing_sensitivity = TMC_Z_SGT;
                    break;
            }
        } while(idx);
    }
}

// Append Trinamic settings to '$$' report
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
        report_uint_setting(Setting_TrinamicDriver, driver_settings.trinamic.driver_enable.mask);
        report_uint_setting(Setting_TrinamicHoming, driver_settings.trinamic.homing_enable.mask);
    }
}

// Add warning info to next realtime report when warning flag set by drivers
void trinamic_RTReport (stream_write_ptr stream_write)
{
    if(warning) {
        warning = false;
        TMCI2C_status_t status = (TMCI2C_status_t)TMC2130_ReadRegister(NULL, (TMC2130_datagram_t *)&dgr_monitor).value;
        otpw_triggered.mask |= dgr_monitor.reg.otpw.mask;
        sprintf(sbuf, "|TMCMON:%d:%d:%d:%d:%d", status.value, dgr_monitor.reg.ot.mask, dgr_monitor.reg.otpw.mask, dgr_monitor.reg.otpw_cnt.mask, dgr_monitor.reg.error.mask);
        stream_write(sbuf);
    }
}

// Return pointer to end of string
static char *append (char *s)
{
    while(*s) s++;

    return s;
}

// Write CRLF terminated string to current stream
static void write_line (char *s)
{
    strcat(s, "\r\n");
    hal.stream.write(s);
}

//
static void report_sg_status (uint_fast16_t state)
{
    if(report.sg_status) {
        report.sg_status = false;
        TMC2130_ReadRegister(&stepper[report.sg_status_axis], (TMC2130_datagram_t *)&stepper[report.sg_status_axis].drv_status);
        hal.stream.write("[SG:");
        hal.stream.write(uitoa((uint32_t)stepper[report.sg_status_axis].drv_status.reg.sg_result));
        hal.stream.write("]\r\n");
    }
    if(hal_execute_realtime)
        hal_execute_realtime(state);
}

static void report_sg_params (void)
{
    sprintf(sbuf, "[SGPARAMS:%d:%d:%d:%d]\r\n", report.sg_status_axis, stepper[report.sg_status_axis].coolconf.reg.sfilt, stepper[report.sg_status_axis].coolconf.reg.semin, stepper[report.sg_status_axis].coolconf.reg.semax);
    hal.stream.write(sbuf);
}

static void stepper_pulse_start (stepper_t *motors)
{
    static uint32_t step_count = 0;
    hal_stepper_pulse_start(motors);
    report.sg_status_axis = 0;

    if(motors->step_outbits.x) {
        step_count++;
        if(step_count == report.msteps) {
            step_count = 0;
            report.sg_status = true;
        }
    }
}

// Enable/disable stallGuard
static void stallGuard_enable (uint32_t axis, bool enable)
{
    stepper[axis].gconf.reg.diag1_stall = enable;
    stepper[axis].gconf.reg.en_pwm_mode = !enable; // stealthChop
    TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].gconf);

    stepper[axis].tcoolthrs.reg.tcoolthrs = enable ? (1 << 20) - 1 : 0;
    TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].tcoolthrs);

    stepper[axis].coolconf.reg.sgt = driver_settings.trinamic.driver[axis].homing_sensitivity & 0x7F; // 7-bits signed value
    TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].coolconf);
}

// Validate M-code axis parameters
// Sets value to NAN (Not A Number) if driver not installed
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

#define Trinamic_StallGuardParams 123
#define Trinamic_WriteRegister 124

// Check if given M-code is handled here
user_mcode_t trinamic_MCodeCheck (user_mcode_t mcode)
{
#if TRINAMIC_DEV
    if(mcode == Trinamic_StallGuardParams || mcode == Trinamic_WriteRegister)
        return mcode;
#endif

    return driver_settings.trinamic.driver_enable.mask &&
            (mcode == Trinamic_DebugReport || mcode == Trinamic_StepperCurrent || mcode == Trinamic_ReportPrewarnFlags ||
		      mcode == Trinamic_ClearPrewarnFlags || mcode == Trinamic_HybridThreshold || mcode == Trinamic_HomingSensivity) ? mcode : UserMCode_Ignore;
}

// Validate driver specific M-code parameters
status_code_t trinamic_MCodeValidate (parser_block_t *gc_block, uint_fast16_t *value_words)
{
    status_code_t state = Status_GcodeValueWordMissing;

    switch(gc_block->user_mcode) {

#if TRINAMIC_DEV

        case Trinamic_StallGuardParams:
            state = Status_OK;
            break;

        case Trinamic_WriteRegister:
            if(bit_istrue(*value_words, bit(Word_R)) && bit_istrue(*value_words, bit(Word_Q)) /* && getAdr() != 0xFF)*/) {
                reg_ptr = TMC2130_GetRegPtr(&stepper[report.sg_status_axis], (tmc2130_regaddr_t)gc_block->values.r);
                state = reg_ptr == NULL ? Status_GcodeValueOutOfRange : Status_OK;
                bit_false(*value_words, bit(Word_R|Word_Q));
                *value_words = 0;
            }
            break;

#endif

        case Trinamic_DebugReport:
            state = Status_OK;
            report.axes.x = bit_istrue(*value_words, bit(Word_X)) && gc_block->values.xyz[X_AXIS] != 0.0f;
            report.axes.y = bit_istrue(*value_words, bit(Word_Y)) && gc_block->values.xyz[Y_AXIS] != 0.0f;
            report.axes.z = bit_istrue(*value_words, bit(Word_Z)) && gc_block->values.xyz[Z_AXIS] != 0.0f;

            if(bit_istrue(*value_words, bit(Word_Q)))
                report.raw = bit_istrue(*value_words, bit(Word_Q)) && gc_block->values.q != 0.0f;

            if(bit_istrue(*value_words, bit(Word_H)))
                report.sfilt = gc_block->values.h != 0.0f;

            if(bit_istrue(*value_words, bit(Word_S))) {
                report.sg_status_enable = gc_block->values.s != 0.0f;
                report.msteps = driver_settings.trinamic.driver[report.sg_status_axis].microsteps;
            }

            if(report.axes.mask) {
                report.axes.mask &= driver_settings.trinamic.driver_enable.mask;
                uint32_t axis = 0, mask = report.axes.mask;
                while(mask) {
                    if(mask & 0x01) {
                        report.sg_status_axis = axis;
                        break;
                    }
                    axis++;
                    mask >>= 1;
                }
            } else
                report.axes.mask = driver_settings.trinamic.driver_enable.mask;

            bit_false(*value_words, bit(Word_Q|Word_S|bit(Word_X)|bit(Word_Y)|bit(Word_Z)|bit(Word_H)));

//            gc_block->user_mcode_sync = true;
            break;

        case Trinamic_StepperCurrent:
            if(check_params(gc_block, value_words)) {
                state = Status_OK;
                gc_block->user_mcode_sync = true;
                if(bit_isfalse(*value_words, bit(Word_Q)))
                    gc_block->values.q = NAN;
                else // TODO: add range check?
                    bit_false(*value_words, bit(Word_Q));
            }
            break;

        case Trinamic_ReportPrewarnFlags:
        case Trinamic_ClearPrewarnFlags:
            state = Status_OK;
            break;

        case Trinamic_HybridThreshold:
            if(check_params(gc_block, value_words)) {
                state = Status_OK;
                gc_block->user_mcode_sync = true;
            }
            break;

        case Trinamic_HomingSensivity:
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

// Execute driver specific M-code
void trinamic_MCodeExecute (uint_fast16_t state, parser_block_t *gc_block)
{
    uint_fast8_t idx = N_AXIS;

    switch(gc_block->user_mcode) {

#if TRINAMIC_DEV

        case Trinamic_StallGuardParams:
            report_sg_params();
            break;

        case Trinamic_WriteRegister:
            reg_ptr->payload.value = (uint32_t)gc_block->values.q;
            TMC2130_WriteRegister(&stepper[report.sg_status_axis], reg_ptr);
            break;

#endif

        case Trinamic_DebugReport:
            if(report.sg_status_enable) {
                if(hal.execute_realtime != report_sg_status) {
                    hal_execute_realtime = hal.execute_realtime;
                    hal.execute_realtime = report_sg_status;
                    hal_stepper_pulse_start = hal.stepper_pulse_start;
                    hal.stepper_pulse_start = stepper_pulse_start;
                }
                stepper[idx].coolconf.reg.sfilt = report.sfilt;
                TMC2130_WriteRegister(&stepper[report.sg_status_axis], (TMC2130_datagram_t *)&stepper[report.sg_status_axis].coolconf);
            } else if(hal.execute_realtime == report_sg_status) {
                hal.execute_realtime = NULL;
                hal.execute_realtime = hal_execute_realtime;
                hal.stepper_pulse_start = hal_stepper_pulse_start;
            }
            write_debug_report();
            break;

        case Trinamic_StepperCurrent:
            do {
                idx--;
                if(!isnan(gc_block->values.xyz[idx]))
                    TMC2130_SetCurrent(&stepper[idx], (uint16_t)gc_block->values.xyz[idx],
                                        isnan(gc_block->values.q) ? stepper[idx].hold_current_pct : (uint8_t)gc_block->values.q);
            } while(idx);
            break;

        case Trinamic_ReportPrewarnFlags:; // TODO: format grbl style?
            TMC2130_status_t status;
            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic.driver_enable.mask, bit(idx))) {
                    status = TMC2130_ReadRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].drv_status);
                    strcpy(sbuf, axis_letter[idx]);
                    strcat(sbuf, ":");
                    if(status.driver_error)
                        strcat(sbuf, "E");
                    else if(stepper[idx].drv_status.reg.ot)
                        strcat(sbuf, "O");
                    else if(stepper[idx].drv_status.reg.otpw)
                        strcat(sbuf, "W");
                    write_line(sbuf);
                }
            }
            break;

        case Trinamic_ClearPrewarnFlags:
            otpw_triggered.mask = 0;
#if TRINAMIC_DEV
            stallGuard_enable(report.sg_status_axis, true); // TODO: (re)move this...
#endif
            break;

        case Trinamic_HybridThreshold:
            do {
                idx--;
                if(!isnan(gc_block->values.xyz[idx]))
                    TMC2130_SetHybridThreshold(&stepper[idx], (uint32_t)gc_block->values.xyz[idx], settings.steps_per_mm[idx]);
            } while(idx);
            break;

        case Trinamic_HomingSensivity:
            do {
                idx--;
                if(!isnan(gc_block->values.xyz[idx])) {
                    driver_settings.trinamic.driver[idx].homing_sensitivity = (int8_t)gc_block->values.xyz[idx];
                    stepper[idx].coolconf.reg.sfilt = report.sfilt;
                    stepper[idx].coolconf.reg.sgt = driver_settings.trinamic.driver[idx].homing_sensitivity; // 7 bits signed
                    TMC2130_WriteRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].coolconf);
                }
            } while(idx);
            break;
    }
}

#if TRINAMIC_I2C
// Called by stepperEnable in driver.c
axes_signals_t trinamic_stepper_enable (axes_signals_t enable)
{
    dgr_enable.reg.enable.mask = enable.mask & driver_settings.trinamic.driver_enable.mask;

    TMC2130_WriteRegister(NULL, (TMC2130_datagram_t *)&dgr_enable);

    return driver_settings.trinamic.driver_enable;
}
#endif

// hal.limits_get_state is redirected here when homing
static axes_signals_t trinamic_limits (void)
{
    axes_signals_t signals = limits_get_state(); // read from switches first

    signals.mask &= ~homing.mask;

    if(hal.clear_bits_atomic(&diag1_poll, 0)) {
        // TODO: read I2C bridge status register instead of polling drivers when using I2C comms
        uint_fast8_t idx = N_AXIS;
        do {
            if(bit_istrue(homing.mask, bit(--idx))) {
                TMC2130_ReadRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].drv_status);
                if(stepper[idx].drv_status.reg.stallGuard)
                    bit_true(signals.mask, idx);
            }
        } while(idx);
    }

    return signals;
}

// Configure sensorless homing for enabled axes
void trinamic_homing (bool enable)
{
    uint_fast8_t idx = N_AXIS;

    homing.mask = driver_settings.trinamic.driver_enable.mask & driver_settings.trinamic.homing_enable.mask;

    is_homing = enable;
    enable = enable && homing.mask;

    do {
        if(bit_istrue(homing.mask, bit(--idx)))
            stallGuard_enable(idx, enable);
    } while(idx);

    if(enable) {
        if(limits_get_state == NULL) {
            limits_get_state = hal.limits_get_state;
            hal.limits_get_state = trinamic_limits;
        }
        diag1_poll = 0;
    } else if(limits_get_state != NULL) {
        hal.limits_get_state = limits_get_state;
        limits_get_state = NULL;
    }
}

// Write Marlin style driver debug report to output stream (M122)
// NOTE: this output is not in a parse friendly format for grbl senders
static void write_debug_report (void)
{
    uint_fast8_t idx = N_AXIS;

    hal.stream.write("[TRINAMIC]\r\n");

    do {
        if(bit_istrue(report.axes.mask, bit(--idx))) {
            TMC2130_ReadRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].chopconf);
            TMC2130_ReadRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].drv_status);
            TMC2130_ReadRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].pwm_scale);
            TMC2130_ReadRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].tstep);
            if(stepper[idx].drv_status.reg.otpw)
                otpw_triggered.mask |= bit(idx);
        }
    } while(idx);

    if(report.raw) {

    } else {

        sprintf(sbuf, "%-15s", "");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8s", axis_letter[idx]);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "Set current");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8d", stepper[idx].current);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "RMS current");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8d", TMC2130_GetCurrent(&stepper[idx]));
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "Peak current");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8d", (uint32_t)((float)TMC2130_GetCurrent(&stepper[idx]) * sqrtf(2)));
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "Run current");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%5d/31", stepper[idx].ihold_irun.reg.irun);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "Hold current");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%5d/31", stepper[idx].ihold_irun.reg.ihold);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "CS actual");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%5d/31", stepper[idx].drv_status.reg.cs_actual);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "PWM scale");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8d", stepper[idx].pwm_scale.reg.pwm_scale);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "vsense");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8s", stepper[idx].chopconf.reg.vsense ? "1=0.180" : "0=0.325");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "stealthChop");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8s", stepper[idx].gconf.reg.en_pwm_mode ? "true" : "false");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "msteps");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8d", 1 << (8 - stepper[idx].chopconf.reg.mres));
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "tstep");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8d", stepper[idx].tstep.reg.tstep);
        }
        write_line(sbuf);

        hal.stream.write("pwm\r\n");

        sprintf(sbuf, "%-15s", "threshold");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8d", stepper[idx].tpwmthrs.reg.tpwmthrs);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "[mm/s]");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx))) {
                if(stepper[idx].tpwmthrs.reg.tpwmthrs)
                    sprintf(append(sbuf), "%8d", TMC2130_GetTPWMTHRS(&stepper[idx], settings.steps_per_mm[idx]));
                else
                    sprintf(append(sbuf), "%8s", "-");
            }
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "OT prewarn");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8s", stepper[idx].drv_status.reg.otpw ? "true" : "false");
        }
        write_line(sbuf);

        hal.stream.write("OT prewarn has\r\n");
        sprintf(sbuf, "%-15s", "been triggered");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8s", bit_istrue(otpw_triggered.mask, bit(idx)) ? "true" : "false");
        }
        write_line(sbuf);

/*
        sprintf(sbuf, "%-15s", "pwm autoscale");

        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8d", stepper[idx].pwmconf.reg.pwm_autoscale);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "pwm ampl");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8d", stepper[idx].pwmconf.reg.pwm_ampl);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "pwm grad");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8d", stepper[idx].pwmconf.reg.pwm_grad);
        }
        write_line(sbuf);
*/

        sprintf(sbuf, "%-15s", "off time");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8d", stepper[idx].chopconf.reg.toff);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "blank time");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8d", stepper[idx].chopconf.reg.tbl);
        }
        write_line(sbuf);

        hal.stream.write("hysteresis\r\n");

        sprintf(sbuf, "%-15s", "-end");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8d", (int)stepper[idx].chopconf.reg.hend - 3);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "-start");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8d", stepper[idx].chopconf.reg.hstrt + 1);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "Stallguard thrs");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx))) {
                int8_t x = stepper[idx].coolconf.reg.sgt;
                x |= (x & 0x40) ? 0x80 : 0x00;
                sprintf(append(sbuf), "%8d", (int)x);
            }
        }
        write_line(sbuf);

        hal.stream.write("DRIVER STATUS:\r\n");

        sprintf(sbuf, "%-15s", "stallguard");
        write_line(sbuf);
        sprintf(sbuf, "%-15s", "sg_result");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8d", stepper[idx].drv_status.reg.sg_result);
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "stst");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8s", stepper[idx].drv_status.reg.stst ? "*" : "");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "fsactive");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8s", stepper[idx].drv_status.reg.fsactive ? "*" : "");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "olb");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8s", stepper[idx].drv_status.reg.olb ? "*" : "");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "ola");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8s", stepper[idx].drv_status.reg.ola ? "*" : "");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "s2gb");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8s", stepper[idx].drv_status.reg.s2gb ? "*" : "");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "s2ga");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8s", stepper[idx].drv_status.reg.s2ga ? "*" : "");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "otpw");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8s", stepper[idx].drv_status.reg.otpw ? "*" : "");
        }
        write_line(sbuf);

        sprintf(sbuf, "%-15s", "ot");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx)))
                sprintf(append(sbuf), "%8s", stepper[idx].drv_status.reg.ot ? "*" : "");
        }
        write_line(sbuf);

        hal.stream.write("STATUS REGISTERS:\r\n");
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx))) {
                uint32_t reg = stepper[idx].drv_status.reg.value;
                sprintf(sbuf, " %s = 0x%02X:%02X:%02X:%02X", axis_letter[idx], reg >> 24, (reg >> 16) & 0xFF, (reg >> 8) & 0xFF, reg & 0xFF);
                write_line(sbuf);
            }
        }
    }
}

// Interrupt handler for DIAG1 signal(s)
void trinamic_fault_handler (void)
{
    if(is_homing)
        diag1_poll = 1;
    else
        hal.limit_interrupt_callback((axes_signals_t){AXES_BITMASK});
}

#if TRINAMIC_I2C
// Interrupt handler for warning event from I2C bridge
// Sets flag to add realtime report message
void trinamic_warn_handler (void)
{
    warning = true;
}
#endif

#endif
