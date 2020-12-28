/*
  trinamic.c - Trinamic TMC2130 plugin

  Part of grblHAL

  Copyright (c) 2018-2020 Terje Io

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

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#if TRINAMIC_ENABLE == 2130

#include <stdio.h>
#include <string.h>
#include <math.h>

#include "trinamic.h"

#ifdef ARDUINO
#include "../grbl/nvs_buffer.h"
#else
#include "grbl/nvs_buffer.h"
#include "grbl/protocol.h"
#endif

#include "grbl/report.h"
#ifdef ARDUINO
  #if TRINAMIC_I2C
    #include "../i2c.h"
  #else
    #include "../spi.h"
#endif
#else
  #if TRINAMIC_I2C
    #include "i2c.h"
  #else
    #include "spi.h"
  #endif
#endif

static bool warning = false, is_homing = false;
static volatile uint_fast16_t diag1_poll = 0;
static char sbuf[65]; // string buffer for reports
static TMC2130_t stepper[N_AXIS];
static axes_signals_t homing = {0}, otpw_triggered = {0};
static limits_get_state_ptr limits_get_state = NULL;
static stepper_pulse_start_ptr hal_stepper_pulse_start = NULL;
static driver_setting_ptrs_t driver_settings;
static on_realtime_report_ptr on_realtime_report;
static on_report_options_ptr on_report_options;
static user_mcode_ptrs_t user_mcode;
static trinamic_settings_t trinamic;
static struct {
    axes_signals_t axes;
    bool raw;
    bool sg_status_enable;
    volatile bool sg_status;
    bool sfilt;
    uint32_t sg_status_axis;
    uint32_t msteps;
} report = {0};

#if TRINAMIC_DEV
static TMC2130_datagram_t *reg_ptr = NULL;
#endif

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
    SPI_DriverInit(driver, trinamic.driver_enable);
#endif
}

// Update driver settings on changes
static void trinamic_configure (void)
{
    uint_fast8_t idx = N_AXIS;

    do {
        if(bit_istrue(trinamic.driver_enable.mask, bit(--idx))) {
            stepper[idx].r_sense = trinamic.driver[idx].r_sense;
            TMC2130_SetCurrent(&stepper[idx], trinamic.driver[idx].current, stepper[idx].hold_current_pct);
            TMC2130_SetMicrosteps(&stepper[idx], trinamic.driver[idx].microsteps);
        }
    } while(idx);
}

static const setting_detail_t trinamic_settings[] = {
    { Setting_TrinamicDriver, Group_MotorDriver, "Trinamic driver", NULL, Format_AxisMask, NULL, NULL, NULL },
    { Setting_TrinamicHoming, Group_MotorDriver, "Sensorless homing", NULL, Format_AxisMask, NULL, NULL, NULL },
    { Setting_AxisStepperCurrentBase, Group_Axis0, "?-axis motor current", "mA", Format_Integer, "###0", NULL, NULL },
    { Setting_AxisMicroStepsBase, Group_Axis0, "?-axis microsteps", "steps", Format_Integer, "###0", NULL, NULL }
};

static setting_details_t details = {
    .settings = trinamic_settings,
    .n_settings = sizeof(trinamic_settings) / sizeof(setting_detail_t)
};

static setting_details_t *on_report_settings (void)
{
    return &details;
}

// Parse and set driver specific parameters
status_code_t trinamic_setting (setting_type_t setting, float value, char *svalue)
{
    status_code_t status = Status_OK;

    if((setting_type_t)setting >= Setting_AxisSettingsBase && (setting_type_t)setting <= Setting_AxisSettingsMax) {

        uint_fast16_t base_idx = (uint_fast16_t)setting - (uint_fast16_t)Setting_AxisSettingsBase;
        uint_fast8_t idx = base_idx % AXIS_SETTINGS_INCREMENT;

        if(idx < N_AXIS) switch((base_idx - idx) / AXIS_SETTINGS_INCREMENT) {

            case AxisSetting_StepperCurrent:
                trinamic.driver[idx].current = (uint16_t)value;
                TMC2130_SetCurrent(&stepper[idx], trinamic.driver[idx].current, stepper[idx].hold_current_pct);
                break;

            case AxisSetting_MicroSteps:
                if(TMC2130_MicrostepsIsValid((uint16_t)value)) {
                    trinamic.driver[idx].microsteps = (tmc2130_microsteps_t)value;
                    TMC2130_SetMicrosteps(&stepper[idx], trinamic.driver[idx].microsteps);
                }
                else
                    status = Status_InvalidStatement;
                break;

            default:
                status = Status_Unhandled;
                break;
        }
    } else switch((setting_type_t)setting) {

        case 256: // TODO: remove, for migrating from settings version 18 or earlier
        case Setting_TrinamicDriver:
            trinamic.driver_enable.mask = (uint8_t)value & AXES_BITMASK;
            break;

        case 257: // TODO: remove, for migrating from settings version 18 or earlier
        case Setting_TrinamicHoming:
            trinamic.homing_enable.mask = (uint8_t)value & AXES_BITMASK;
            break;

        default:
            status = Status_Unhandled;
            break;
    }

    if(status == Status_OK)
        hal.nvs.memcpy_to_nvs(driver_settings.nvs_address, (uint8_t *)&trinamic, sizeof(trinamic_settings_t), true);

    return status == Status_Unhandled && driver_settings.set ? driver_settings.set(setting, value, svalue) : status;
}

// Initialize default EEPROM settings
static void trinamic_settings_restore (void)
{
    uint_fast8_t idx = N_AXIS;

    trinamic.driver_enable.mask = 0;
    trinamic.homing_enable.mask = 0;

    do {
        switch(--idx) {

            case X_AXIS:
                trinamic.driver_enable.x = TMC_X_ENABLE;
                trinamic.driver[idx].current = TMC_X_CURRENT;
                trinamic.driver[idx].microsteps = TMC_X_MICROSTEPS;
                trinamic.driver[idx].r_sense = TMC_X_R_SENSE;
                trinamic.driver[idx].homing_sensitivity = TMC_X_SGT;
                break;

            case Y_AXIS:
                trinamic.driver[idx].current = TMC_Y_CURRENT;
                trinamic.driver[idx].microsteps = TMC_Y_MICROSTEPS;
                trinamic.driver[idx].r_sense = TMC_Y_R_SENSE;
                trinamic.driver[idx].homing_sensitivity = TMC_Y_SGT;
                break;

            case Z_AXIS:
                trinamic.driver[idx].current = TMC_Z_CURRENT;
                trinamic.driver[idx].microsteps = TMC_Z_MICROSTEPS;
                trinamic.driver[idx].r_sense = TMC_Z_R_SENSE;
                trinamic.driver[idx].homing_sensitivity = TMC_Z_SGT;
                break;
        }
    } while(idx);

    hal.nvs.memcpy_to_nvs(driver_settings.nvs_address, (uint8_t *)&trinamic, sizeof(trinamic_settings_t), true);

    if(driver_settings.restore)
        driver_settings.restore();
}

static void trinamic_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&trinamic, driver_settings.nvs_address, sizeof(trinamic_settings_t), true) != NVS_TransferResult_OK)
        trinamic_settings_restore();

    if(driver_settings.load)
        driver_settings.load();
}

// Append Trinamic settings to '$$' report
static void trinamic_settings_report (setting_type_t setting)
{
    bool reported = true;

    switch(setting) {

        case Setting_TrinamicDriver:
            report_uint_setting(setting, trinamic.driver_enable.mask);
            break;

        case Setting_TrinamicHoming:
            report_uint_setting(setting, trinamic.homing_enable.mask);
            break;

        default:
            reported = false;
            break;
    }

    if(!reported && driver_settings.report)
        driver_settings.report(setting);
}

static void trinamic_axis_settings_report (axis_setting_type_t setting, uint8_t axis_idx)
{
    bool reported = true;
    setting_type_t basetype = (setting_type_t)(Setting_AxisSettingsBase + setting * AXIS_SETTINGS_INCREMENT);

    switch(setting) {

        case AxisSetting_StepperCurrent:
            report_uint_setting((setting_type_t)(basetype + axis_idx), trinamic.driver[axis_idx].current);
            break;

        case AxisSetting_MicroSteps:
            report_uint_setting((setting_type_t)(basetype + axis_idx), trinamic.driver[axis_idx].microsteps);
            break;

        default:
            reported = false;
            break;
    }

    if(!reported && driver_settings.axis_report)
        driver_settings.axis_report(setting, axis_idx);
}

// Add warning info to next realtime report when warning flag set by drivers
static void trinamic_realtime_report (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if(warning) {
        warning = false;
#if TRINAMIC_I2C
        TMCI2C_status_t status = (TMCI2C_status_t)TMC2130_ReadRegister(NULL, (TMC2130_datagram_t *)&dgr_monitor).value;
        otpw_triggered.mask |= dgr_monitor.reg.otpw.mask;
        sprintf(sbuf, "|TMCMON:%d:%d:%d:%d:%d", status.value, dgr_monitor.reg.ot.mask, dgr_monitor.reg.otpw.mask, dgr_monitor.reg.otpw_cnt.mask, dgr_monitor.reg.error.mask);
        stream_write(sbuf);
#endif
    }

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
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
    strcat(s, ASCII_EOL);
    hal.stream.write(s);
}

//
static void report_sg_status (uint_fast16_t state)
{
    TMC2130_ReadRegister(&stepper[report.sg_status_axis], (TMC2130_datagram_t *)&stepper[report.sg_status_axis].drv_status);
    hal.stream.write("[SG:");
    hal.stream.write(uitoa((uint32_t)stepper[report.sg_status_axis].drv_status.reg.sg_result));
    hal.stream.write("]" ASCII_EOL);
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
            protocol_enqueue_rt_command(report_sg_status);
        }
    }
}

#if TRINAMIC_DEV

static void report_sg_params (void)
{
    sprintf(sbuf, "[SGPARAMS:%ld:%d:%d:%d]" ASCII_EOL, report.sg_status_axis, stepper[report.sg_status_axis].coolconf.reg.sfilt, stepper[report.sg_status_axis].coolconf.reg.semin, stepper[report.sg_status_axis].coolconf.reg.semax);
    hal.stream.write(sbuf);
}

#endif

// Enable/disable stallGuard
static void stallGuard_enable (uint32_t axis, bool enable)
{
    stepper[axis].gconf.reg.diag1_stall = enable;
    stepper[axis].gconf.reg.en_pwm_mode = !enable; // stealthChop
    TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].gconf);

    stepper[axis].tcoolthrs.reg.tcoolthrs = enable ? (1 << 20) - 1 : 0;
    TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].tcoolthrs);

    stepper[axis].coolconf.reg.sgt = trinamic.driver[axis].homing_sensitivity & 0x7F; // 7-bits signed value
    TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].coolconf);
}

// Validate M-code axis parameters
// Sets value to NAN (Not A Number) if driver not installed
static bool check_params (parser_block_t *gc_block, uint32_t *value_words)
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
        if(bit_istrue(*value_words, bit(wordmap[idx])) && bit_istrue(trinamic.driver_enable.mask, bit(idx))) {
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
static user_mcode_t trinamic_MCodeCheck (user_mcode_t mcode)
{
#if TRINAMIC_DEV
    if(mcode == Trinamic_StallGuardParams || mcode == Trinamic_WriteRegister)
        return mcode;
#endif

    return trinamic.driver_enable.mask &&
            (mcode == Trinamic_DebugReport || mcode == Trinamic_StepperCurrent || mcode == Trinamic_ReportPrewarnFlags ||
              mcode == Trinamic_ClearPrewarnFlags || mcode == Trinamic_HybridThreshold || mcode == Trinamic_HomingSensivity)
              ? mcode
              : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore);
}

// Validate driver specific M-code parameters
static status_code_t trinamic_MCodeValidate (parser_block_t *gc_block, uint32_t *value_words)
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
                report.msteps = trinamic.driver[report.sg_status_axis].microsteps;
            }

            if(report.axes.mask) {
                report.axes.mask &= trinamic.driver_enable.mask;
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
                report.axes.mask = trinamic.driver_enable.mask;

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

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block, value_words) : state;
}

// Execute driver specific M-code
static void trinamic_MCodeExecute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled = true;
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
                if(hal_stepper_pulse_start == NULL) {
                    hal_stepper_pulse_start = hal.stepper.pulse_start;
                    hal.stepper.pulse_start = stepper_pulse_start;
                }
                stepper[report.sg_status_axis].coolconf.reg.sfilt = report.sfilt;
                TMC2130_WriteRegister(&stepper[report.sg_status_axis], (TMC2130_datagram_t *)&stepper[report.sg_status_axis].coolconf);
            } else if(hal_stepper_pulse_start != NULL) {
                hal.stepper.pulse_start = hal_stepper_pulse_start;
                hal_stepper_pulse_start = NULL;
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
                if(bit_istrue(trinamic.driver_enable.mask, bit(idx))) {
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
                    TMC2130_SetHybridThreshold(&stepper[idx], (uint32_t)gc_block->values.xyz[idx], settings.axis[idx].steps_per_mm);
            } while(idx);
            break;

        case Trinamic_HomingSensivity:
            do {
                idx--;
                if(!isnan(gc_block->values.xyz[idx])) {
                    trinamic.driver[idx].homing_sensitivity = (int8_t)gc_block->values.xyz[idx];
                    stepper[idx].coolconf.reg.sfilt = report.sfilt;
                    stepper[idx].coolconf.reg.sgt = trinamic.driver[idx].homing_sensitivity; // 7 bits signed
                    TMC2130_WriteRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].coolconf);
                }
            } while(idx);
            break;

        default:
            handled = false;
            break;
    }

    if(!handled && hal.user_mcode.execute)
        hal.user_mcode.execute(state, gc_block);
}

#if TRINAMIC_I2C
// Called by stepperEnable in driver.c
axes_signals_t trinamic_stepper_enable (axes_signals_t enable)
{
    dgr_enable.reg.enable.mask = enable.mask & trinamic.driver_enable.mask;

    TMC2130_WriteRegister(NULL, (TMC2130_datagram_t *)&dgr_enable);

    return trinamic.driver_enable;
}
#endif

// hal.limits.get_state is redirected here when homing
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

    homing.mask = trinamic.driver_enable.mask & trinamic.homing_enable.mask;

    is_homing = enable;
    enable = enable && homing.mask;

    do {
        if(bit_istrue(homing.mask, bit(--idx)))
            stallGuard_enable(idx, enable);
    } while(idx);

    if(enable) {
        if(limits_get_state == NULL) {
            limits_get_state = hal.limits.get_state;
            hal.limits.get_state = trinamic_limits;
        }
        diag1_poll = 0;
    } else if(limits_get_state != NULL) {
        hal.limits.get_state = limits_get_state;
        limits_get_state = NULL;
    }
}

// Write Marlin style driver debug report to output stream (M122)
// NOTE: this output is not in a parse friendly format for grbl senders
static void write_debug_report (void)
{
    uint_fast8_t idx = N_AXIS;

    hal.stream.write("[TRINAMIC]" ASCII_EOL);

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
                sprintf(append(sbuf), "%8ld", (uint32_t)((float)TMC2130_GetCurrent(&stepper[idx]) * sqrtf(2)));
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

        hal.stream.write("pwm" ASCII_EOL);

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
                    sprintf(append(sbuf), "%8ld", TMC2130_GetTPWMTHRS(&stepper[idx], settings.axis[idx].steps_per_mm));
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

        hal.stream.write("OT prewarn has" ASCII_EOL);
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

        hal.stream.write("hysteresis" ASCII_EOL);

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

        hal.stream.write("DRIVER STATUS:" ASCII_EOL);

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

        hal.stream.write("STATUS REGISTERS:" ASCII_EOL);
        for(idx = 0; idx < N_AXIS; idx++) {
            if(bit_istrue(report.axes.mask, bit(idx))) {
                uint32_t reg = stepper[idx].drv_status.reg.value;
                sprintf(sbuf, " %s = 0x%02X:%02X:%02X:%02X", axis_letter[idx], (uint8_t)(reg >> 24), (uint8_t)((reg >> 16) & 0xFF), (uint8_t)((reg >> 8) & 0xFF), (uint8_t)(reg & 0xFF));
                write_line(sbuf);
            }
        }
    }
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:TMC2130 v0.01]" ASCII_EOL);
    else if(trinamic.driver_enable.mask) {
        hal.stream.write(",TMC=");
        hal.stream.write(uitoa(trinamic.driver_enable.mask));
    }
}

bool trinamic_init (void)
{
    if((hal.driver_settings.nvs_address = nvs_alloc(sizeof(trinamic_settings_t)))) {

        memcpy(&driver_settings, &hal.driver_settings, sizeof(driver_setting_ptrs_t));

        hal.driver_settings.set = trinamic_setting;
        hal.driver_settings.report = trinamic_settings_report;
        hal.driver_settings.axis_report = trinamic_axis_settings_report;
        hal.driver_settings.load = trinamic_settings_load;
        hal.driver_settings.restore = trinamic_settings_restore;

        memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

        hal.user_mcode.check = trinamic_MCodeCheck;
        hal.user_mcode.validate = trinamic_MCodeValidate;
        hal.user_mcode.execute = trinamic_MCodeExecute;

        on_realtime_report = grbl.on_realtime_report;
        grbl.on_realtime_report = trinamic_realtime_report;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        details.on_report_settings = grbl.on_report_settings;
        grbl.on_report_settings = on_report_settings;
    }

    return driver_settings.nvs_address != 0;
}

void trinamic_start (bool allow_mixed)
{
    uint_fast8_t idx = N_AXIS;

    TMC_IOInit();

    do {

        if(bit_istrue(trinamic.driver_enable.mask, bit(--idx))) {

            TMC2130_SetDefaults(&stepper[idx]); // Init shadow registers to default values

            stepper[idx].axis = idx;
            stepper[idx].current = trinamic.driver[idx].current;
            stepper[idx].microsteps = trinamic.driver[idx].microsteps;
            stepper[idx].r_sense = trinamic.driver[idx].r_sense;

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

    trinamic_configure();
}

// Interrupt handler for DIAG1 signal(s)
void trinamic_fault_handler (void)
{
    if(is_homing)
        diag1_poll = 1;
    else
        hal.limits.interrupt_callback((axes_signals_t){AXES_BITMASK});
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
