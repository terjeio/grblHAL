/*
  trinamic.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Texas Instruments Tiva C (TM4C123GH6PM) ARM processor
   for Trinamic TMC2130 integration

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

#include <stdio.h>

#include "driver.h"
#include "i2c.h"

#define F_SPI 4000000
#define SPI_DELAY _delay_cycles(5);

#define ENABLE_AXIS_SETTING 90

static TMC2130_t stepper[N_AXIS];

void trinamic_init (void)
{
    uint_fast8_t idx = N_AXIS;

    do {
        if(bit_istrue(driver_settings.trinamic_enable.mask, bit(--idx))) {

            TMC2130_SetDefaults(&stepper[idx]);

            stepper[idx].cs_pin = (void *)idx;
            stepper[idx].current = driver_settings.motor_setting[idx].current;
            stepper[idx].microsteps = driver_settings.motor_setting[idx].microsteps;
            stepper[idx].r_sense = driver_settings.motor_setting[idx].r_sense;

            TMC2130_Init(&stepper[idx]);
        }
    } while(idx);
}

void trinamic_configure (void)
{
    uint_fast8_t idx = N_AXIS;

    do {
        if(bit_istrue(driver_settings.trinamic_enable.mask, bit(--idx))) {
            stepper[idx].r_sense = driver_settings.motor_setting[idx].r_sense;
            TMC2130_SetCurrent(&stepper[idx], driver_settings.motor_setting[idx].current, stepper[idx].hold_current_pct);
            TMC2130_SetMicrosteps(&stepper[idx], driver_settings.motor_setting[idx].microsteps);
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
                driver_settings.motor_setting[idx].current = (uint16_t)value;
                break;

            case AxisSetting_MicroSteps:
                if((ok = validate_microstepping((uint16_t)value)))
                    driver_settings.motor_setting[idx].microsteps = (tmc2130_microsteps_t)value;
                break;
        }
    } else if(setting == ENABLE_AXIS_SETTING) {
        ok = true;
        driver_settings.trinamic_enable.mask = (uint8_t)value & AXES_BITMASK;
    }

    return ok;
}

void trinamic_settings_restore (uint8_t restore_flag)
{
    if(restore_flag & SETTINGS_RESTORE_DRIVER_PARAMETERS) {

        uint_fast8_t idx = N_AXIS;

        driver_settings.trinamic_enable.mask = 0;

        do {
            idx--;
            driver_settings.motor_setting[idx].current = TMC2130_CURRENT;
            driver_settings.motor_setting[idx].microsteps = TMC2130_MICROSTEPS;
            driver_settings.motor_setting[idx].r_sense = TMC2130_R_SENSE;
        } while(idx);
    }
}

void trinamic_settings_report (bool axis_settings, axis_setting_type_t setting_type, uint8_t axis_idx)
{
    if(axis_settings) {

        setting_type_t basetype = (setting_type_t)(Setting_AxisSettingsBase + setting_type * AXIS_SETTINGS_INCREMENT);

        switch(setting_type) {

            case AxisSetting_StepperCurrent:
                report_uint_setting((setting_type_t)(basetype + axis_idx), driver_settings.motor_setting[axis_idx].current);
                break;

            case AxisSetting_MicroSteps:
                report_uint_setting((setting_type_t)(basetype + axis_idx), driver_settings.motor_setting[axis_idx].microsteps);
                break;
        }
    } else
        report_uint_setting((setting_type_t)ENABLE_AXIS_SETTING, driver_settings.trinamic_enable.mask);
}

void SPI_DriverInit (SPI_driver_t *driver)
{
    I2C_DriverInit(driver);
}

uint_fast16_t trimamic_MCodeCheck (uint_fast16_t mcode)
{
    return driver_settings.trinamic_enable.mask && (mcode == 121 || mcode == 122 || mcode == 906 || mcode == 911 || mcode == 912) ? mcode : 0;
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
        if(bit_istrue(*value_words, bit(wordmap[idx])) && bit_istrue(driver_settings.trinamic_enable.mask, bit(idx))) {
            ok = true;
            bit_false(*value_words, bit(wordmap[idx]));
        } else
            gc_block->values.xyz[idx] = NAN;
    } while(idx);

    return ok;
}

status_code_t trimamic_MCodeValidate (parser_block_t *gc_block, uint_fast16_t *value_words)
{
    status_code_t state = Status_GcodeValueWordMissing;

    switch(gc_block->driver_mcode) {

        case 121:
            state = Status_OK;
            gc_block->driver_mcode_sync = true;
            if(bit_istrue(*value_words, bit(Word_S))) {
                bit_false(*value_words, bit(Word_S));
            }
            break;

        case 122:
            if(check_params(gc_block, value_words)) {
                state = Status_OK;
                gc_block->driver_mcode_sync = true;
            }
            break;

        case 906:
            if(check_params(gc_block, value_words)) {
                state = Status_OK;
                gc_block->driver_mcode_sync = true;
            }
            break;

        case 911:
        case 912:
            state = Status_OK;
            break;
    }

    return state;
}

static char *append (char *s)
{
    while(*s) s++;

    return s;
}

static void write_line (char *s)
{
    strcat(s, "\r\n");
    hal.stream_write(s);
}

void trimamic_MCodeExecute (uint_fast16_t state, parser_block_t *gc_block)
{
    uint_fast8_t idx = N_AXIS;

    switch(gc_block->driver_mcode) {

        case 122:
            do {
                idx--;
                if(!isnan(gc_block->values.xyz[idx]))
                    TMC2130_SetMicrosteps(&stepper[idx], (tmc2130_microsteps_t)gc_block->values.xyz[idx]);
            } while(idx);
            break;

        case 906:
            do {
                idx--;
                if(!isnan(gc_block->values.xyz[idx]))
                    TMC2130_SetCurrent(&stepper[idx], (uint16_t)gc_block->values.xyz[idx], stepper[idx].hold_current_pct);
            } while(idx);
            break;

        case 911:;
            bool otpw[N_AXIS];
            do {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(--idx))) {
                    TMC2130_ReadRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].drv_status);
                    otpw[idx] = stepper[idx].drv_status.reg.otpw;
                }
            } while(idx);
         //   sprintf(buf, "ot warning %d %d %d\r\n", otpw[0], otpw[1], otpw[2]);
         //   hal.stream_write(buf);
            break;

        case 912: // clear prewarn flags
//            TMC2130_ReadRegister (&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].g);
            break;

        case 121:;
            char buf[50];
            hal.stream_write("Trinamic TMC2130:\r\n");
            do {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(--idx))) {
                    TMC2130_ReadRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].chopconf);
                    TMC2130_ReadRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].drv_status);
                    TMC2130_ReadRegister(&stepper[idx], (TMC2130_datagram_t *)&stepper[idx].pwm_scale);
                }
            } while(idx);

            sprintf(buf, "%-13s", "");
            sprintf(append(buf), "%8s", driver_settings.trinamic_enable.x ? "X" : "");
            sprintf(append(buf), "%8s", driver_settings.trinamic_enable.y ? "Y" : "");
            sprintf(append(buf), "%8s", driver_settings.trinamic_enable.z ? "Z" : "");
            write_line(buf);

            sprintf(buf, "%-13s", "run current");
            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(idx)))
                    sprintf(append(buf), "%5d/31", stepper[idx].ihold_irun.reg.irun);
            }
            write_line(buf);

            sprintf(buf, "%-13s", "hold current");
            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(idx)))
                    sprintf(append(buf), "%5d/31", stepper[idx].ihold_irun.reg.ihold);
            }
            write_line(buf);

            sprintf(buf, "%-13s", "CS actual");
            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(idx)))
                    sprintf(append(buf), "%5d/31", stepper[idx].drv_status.reg.cs_actual);
            }
            write_line(buf);

            sprintf(buf, "%-13s", "PWM scale");
            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(idx)))
                    sprintf(append(buf), "%8d", stepper[idx].pwm_scale.reg.pwm_scale);
            }
            write_line(buf);

            sprintf(buf, "%-13s", "vsense");
            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(idx)))
                    sprintf(append(buf), "    %d=%2d", stepper[idx].chopconf.reg.vsense, stepper[idx].ihold_irun.reg.irun);
            }
            write_line(buf);

            sprintf(buf, "%-13s", "mstep");
            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(idx)))
                    sprintf(append(buf), "%8d", 1 << (8 - stepper[idx].chopconf.reg.mres));
            }
            write_line(buf);

            hal.stream_write("pwm:\n\r");

            sprintf(buf, "%-13s", "pwm autoscale");

            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(idx)))
                    sprintf(append(buf), "%8d", stepper[idx].pwmconf.reg.pwm_autoscale);
            }
            write_line(buf);

            sprintf(buf, "%-13s", "pwm ampl");
            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(idx)))
                    sprintf(append(buf), "%8d", stepper[idx].pwmconf.reg.pwm_ampl);
            }
            write_line(buf);

            sprintf(buf, "%-13s", "pwm grad");
            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(idx)))
                    sprintf(append(buf), "%8d", stepper[idx].pwmconf.reg.pwm_grad);
            }
            write_line(buf);

            sprintf(buf, "%-13s", "off time");
            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(idx)))
                    sprintf(append(buf), "%8d", stepper[idx].chopconf.reg.toff);
            }
            write_line(buf);


            sprintf(buf, "%-13s", "blank time");
            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(idx)))
                    sprintf(append(buf), "%8d", stepper[idx].chopconf.reg.tbl);
            }
            write_line(buf);

            hal.stream_write("hysteresis:\r\n");

            sprintf(buf, "%-13s", "-end");
            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(idx)))
                    sprintf(append(buf), "%8d", stepper[idx].chopconf.reg.hend);
            }
            write_line(buf);

            sprintf(buf, "%-13s", "-start");
            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(idx)))
                    sprintf(append(buf), "%8d", stepper[idx].chopconf.reg.hstrt);
            }
            write_line(buf);

            hal.stream_write("DRV_STATUS:\r\n");

            sprintf(buf, "%-13s", "otpw");
            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(idx)))
                    sprintf(append(buf), "%8s", stepper[idx].drv_status.reg.otpw ? "*" : "");
            }
            write_line(buf);

            sprintf(buf, "%-13s", "ot");
            for(idx = 0; idx < N_AXIS; idx++) {
                if(bit_istrue(driver_settings.trinamic_enable.mask, bit(idx)))
                    sprintf(append(buf), "%8s", stepper[idx].drv_status.reg.ot ? "*" : "");
            }
            write_line(buf);

            break;
    }
}


/*
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
 * */

/*
TMC2130_status_t SPI_ReadRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    uint32_t data;
    TMC2130_status_t status;

    SPI_SELECT;
    SPI_DELAY

    // dummy write to prepare data
    SSIDataPut(SPI_BASE, reg->addr.value);
    SSIDataPut(SPI_BASE, 0);
    SSIDataPut(SPI_BASE, 0);
    SSIDataPut(SPI_BASE, 0);
    SSIDataPut(SPI_BASE, 0);
    while(SSIBusy(SPI_BASE));

    // Ditch data in FIFO
    while(SSIDataGetNonBlocking(SPI_BASE, &data));

    SPI_DESELECT;
    SPI_DELAY

    // get register data

    SPI_SELECT;
    SPI_DELAY

    SSIDataPut(SPI_BASE, reg->addr.value);
    SSIDataPut(SPI_BASE, 0);
    SSIDataPut(SPI_BASE, 0);
    SSIDataPut(SPI_BASE, 0);
    SSIDataPut(SPI_BASE, 0);
    while(SSIBusy(SPI_BASE));

    // Read values from FIFO
    SSIDataGetNonBlocking(SPI_BASE, &data);
    status.value = (uint8_t)data;
    SSIDataGetNonBlocking(SPI_BASE, &data);
    reg->payload.value = ((uint8_t)data << 24);
    SSIDataGetNonBlocking(SPI_BASE, &data);
    reg->payload.value |= ((uint8_t)data << 16);
    SSIDataGetNonBlocking(SPI_BASE, &data);
    reg->payload.value |= ((uint8_t)data << 8);
    SSIDataGetNonBlocking(SPI_BASE, &data);
    reg->payload.value |= (uint8_t)data;

    SPI_DESELECT;

    return status;
}

TMC2130_status_t SPI_WriteRegister (TMC2130_t *driver, TMC2130_datagram_t *reg)
{
    TMC2130_status_t status = {0};

    SPI_SELECT;

    // assume FIFO is empty?
    reg->addr.write = 1;
    SSIDataPut(SPI_BASE, reg->addr.value);
    reg->addr.write = 0;
    SSIDataPut(SPI_BASE, (reg->payload.value >> 24) & 0xFF);
    SSIDataPut(SPI_BASE, (reg->payload.value >> 16) & 0xFF);
    SSIDataPut(SPI_BASE, (reg->payload.value >> 8) & 0xFF);
    SSIDataPut(SPI_BASE, reg->payload.value & 0xFF);
    while(SSIBusy(SPI_BASE));

    SPI_DESELECT;

    return status;
}

void SPI_DriverInit (SPI_driver_t *driver)
{
    driver->WriteRegister = SPI_WriteRegister;
    driver->ReadRegister = SPI_ReadRegister;

    PREF(SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA));
    PREF(SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ));
    PREF(SysCtlPeripheralEnable(SPI_PERIPH));

    PREF(GPIOPinTypeGPIOOutput(SPI_CS_PORT, SPI_CS_PIN));

    PREF(GPIOPinConfigure(SPI_CLK));
    PREF(GPIOPinTypeSSI(SPI_PORT, SPI_SCLK_PIN));

    PREF(GPIOPinConfigure(SPI_TX));
    PREF(GPIOPinTypeSSI(SPI_PORT, SPI_MOSI_PIN));

    PREF(GPIOPinConfigure(SPI_RX));
    PREF(GPIOPinTypeSSI(SPI_PORT, SPI_MISO_PIN));

    SPI_DESELECT;

    SSIClockSourceSet(SPI_BASE, SSI_CLOCK_SYSTEM);
    PREF(SSIConfigSetExpClk(SPI_BASE, 120000000, SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, F_SPI, 8));
    PREF(SSIEnable(SPI_BASE));
}
*/
