/*
  trinamic.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Texas Instruments Tiva C (TM4C123GH6PM) ARM processor - Trinamic TMC2130 integration

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

#define F_SPI 4000000
#define SPI_DELAY _delay_cycles(5);

static TMC2130_t stepper[N_AXIS];

void trinamic_init (void)
{
    uint_fast8_t idx = N_AXIS;

    do {

        TMC2130_SetDefaults(&stepper[--idx]);

        stepper[idx].cs_pin = (void *)idx;
        stepper[idx].current = driver_settings.motor_setting[idx].current;
        stepper[idx].microsteps = driver_settings.motor_setting[idx].microsteps;
        stepper[idx].r_sense = driver_settings.motor_setting[idx].r_sense;

        TMC2130_Init(&stepper[idx]);

    } while(idx);
}

void trinamic_configure (void)
{
    uint_fast8_t idx = N_AXIS;

    do {
        idx--;
        stepper[idx].r_sense = driver_settings.motor_setting[idx].r_sense;
        TMC2130_SetCurrent(&stepper[idx], driver_settings.motor_setting[idx].current);
        TMC2130_SetMicrosteps(&stepper[idx], driver_settings.motor_setting[idx].microsteps);
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

        case 6:
            ok = true;
            driver_settings.motor_setting[idx].r_sense = (uint16_t)value;
            break;
    }

    return ok;
}

void trinamic_settings_restore (uint8_t restore_flag)
{
    if(restore_flag & SETTINGS_RESTORE_DRIVER_PARAMETERS) {

        uint_fast8_t idx = N_AXIS;

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

            case 6:
                report_uint_setting((setting_type_t)(basetype + axis_idx), driver_settings.motor_setting[axis_idx].r_sense);
                break;
        }
    }
}

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

