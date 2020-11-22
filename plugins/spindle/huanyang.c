/*

  huanyang.c - Huanyang VFD spindle support

  Part of grblHAL

  Copyright (c) 2020 Terje Io

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

#include "huanyang.h"

#if SPINDLE_HUANYANG

#ifdef ARDUINO
#include "../grbl/hal.h"
#include "../grbl/state_machine.h"
#include "../grbl/report.h"
#else
#include "grbl/hal.h"
#include "grbl/state_machine.h"
#include "grbl/report.h"
#endif

#ifdef SPINDLE_PWM_DIRECT
#error Not supported!
#endif

#ifndef VFD_ADDRESS
#define VFD_ADDRESS 0x01
#endif

typedef enum {
    VFD_Idle = 0,
    VFD_GetRPM,
    VFD_SetRPM,
    VFD_GetMaxRPM,
    VFD_GetStatus,
    VFD_SetStatus
} vfd_response_t;

static float rpm, rpm_programmed = -1.0f, rpm_low_limit = 0.0f, rpm_high_limit = 0.0f;
static spindle_state_t vfd_state = {0};
static on_report_options_ptr on_report_options;
#if SPINDLE_HUANYANG == 2
static uint32_t rpm_max = 0;
#endif

static void spindleSetRPM (float rpm, bool block)
{
    modbus_message_t rpm_cmd;

    if (rpm != rpm_programmed) {

        rpm_cmd.xx = (void *)VFD_SetRPM;
        rpm_cmd.adu[0] = VFD_ADDRESS;

#if SPINDLE_HUANYANG == 2

        uint16_t data = (uint32_t)(rpm) * 10000UL / rpm_max;

        rpm_cmd.adu[1] = ModBus_WriteRegister;
        rpm_cmd.adu[2] = 0x10;
        rpm_cmd.adu[3] = 0x00;
        rpm_cmd.adu[4] = data >> 8;
        rpm_cmd.adu[5] = data & 0xFF;
        rpm_cmd.tx_length = 8;
        rpm_cmd.rx_length = 8;

#else

        uint16_t data = (uint16_t)(rpm * 100 / 60); // send Hz * 10  (Ex:1500 RPM = 25Hz .... Send 2500)

        rpm_cmd.adu[1] = ModBus_WriteCoil;
        rpm_cmd.adu[2] = 0x02;
        rpm_cmd.adu[3] = data >> 8;
        rpm_cmd.adu[4] = data & 0xFF;
        rpm_cmd.tx_length = 7;
        rpm_cmd.rx_length = 6;

#endif

        vfd_state.at_speed = false;

        modbus_send(&rpm_cmd, block);

        if(settings.spindle.at_speed_tolerance > 0.0f) {
            rpm_low_limit = rpm / (1.0f + settings.spindle.at_speed_tolerance);
            rpm_high_limit = rpm * (1.0f + settings.spindle.at_speed_tolerance);
        }
        rpm_programmed = rpm;
    }
}

static void spindleUpdateRPM (float rpm)
{
    spindleSetRPM(rpm, false);
}

// Start or stop spindle
static void spindleSetState (spindle_state_t state, float rpm)
{
    modbus_message_t mode_cmd;

    mode_cmd.xx = (void *)VFD_SetStatus;
    mode_cmd.adu[0] = VFD_ADDRESS;

#if SPINDLE_HUANYANG == 2

    mode_cmd.adu[1] = ModBus_WriteRegister;
    mode_cmd.adu[2] = 0x20;
    mode_cmd.adu[3] = 0x00;
    mode_cmd.adu[4] = 0x00;
    mode_cmd.adu[5] = (!state.on || rpm == 0.0f) ? 6 : (state.ccw ? 2 : 1);

    mode_cmd.tx_length = 8;
    mode_cmd.rx_length = 8;

#else

    mode_cmd.adu[1] = ModBus_ReadHoldingRegisters;
    mode_cmd.adu[2] = 0x01;
    mode_cmd.adu[3] = (!state.on || rpm == 0.0f) ? 0x08 : (state.ccw ? 0x11 : 0x01);
    mode_cmd.tx_length = 6;
    mode_cmd.rx_length = 6;

#endif

    if(modbus_send(&mode_cmd, true))
        spindleSetRPM(rpm, true);
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    modbus_message_t mode_cmd;

    mode_cmd.xx = (void *)VFD_GetRPM;
    mode_cmd.adu[0] = VFD_ADDRESS;

#if SPINDLE_HUANYANG == 2

    // Get current RPM

    mode_cmd.adu[1] = ModBus_ReadHoldingRegisters;
    mode_cmd.adu[2] = 0x70;
    mode_cmd.adu[3] = 0x0C;
    mode_cmd.adu[4] = 0x00;
    mode_cmd.adu[5] = 0x02;
    mode_cmd.tx_length = 8;
    mode_cmd.rx_length = 8;

#else

    mode_cmd.adu[1] = ModBus_ReadInputRegisters;
    mode_cmd.adu[2] = 0x03;
    mode_cmd.adu[3] = 0x00;
    mode_cmd.adu[4] = 0x00;
    mode_cmd.adu[5] = 0x00;
    mode_cmd.tx_length = 8;
    mode_cmd.rx_length = 8;

#endif

    modbus_send(&mode_cmd, false);

    return vfd_state; // return previous state as we do not want to wait for the response
}

static void rx_packet (modbus_message_t *msg)
{
    if(!(msg->adu[0] & 0x80)) {

        switch((vfd_response_t)msg->xx) {

            case VFD_GetRPM:
#if SPINDLE_HUANYANG == 2
                rpm = (float)((msg->adu[4] << 8) | msg->adu[5]);
#else
                rpm = (float)((msg->adu[4] << 8) | msg->adu[5]) * 60.0f / 100.0f;
#endif
                vfd_state.at_speed = settings.spindle.at_speed_tolerance <= 0.0f || (rpm >= rpm_low_limit && rpm <= rpm_high_limit);
                break;
#if SPINDLE_HUANYANG == 2
            case VFD_GetMaxRPM:
                rpm_max = (msg->adu[4] << 8) | msg->adu[5];
                break;
#endif
            default:
                break;
        }
    }
}

static void rx_exception (uint8_t code)
{
    system_raise_alarm(Alarm_Spindle);
}

static void onReportOptions (void)
{
    on_report_options();
#if SPINDLE_HUANYANG == 2
    hal.stream.write("[PLUGIN:HUANYANG VFD P2A v0.01]" ASCII_EOL);
#else
    hal.stream.write("[PLUGIN:HUANYANG VFD v0.01]" ASCII_EOL);
#endif
}

void huanyang_init (modbus_stream_t *stream)
{
    hal.spindle.set_state = spindleSetState;
    hal.spindle.get_state = spindleGetState;
    hal.spindle.reset_data = NULL;
    hal.spindle.update_rpm = spindleUpdateRPM;

    hal.driver_cap.variable_spindle = On;
    hal.driver_cap.spindle_at_speed = On;
    hal.driver_cap.spindle_dir = On;

    stream->on_rx_packet = rx_packet;
    stream->on_rx_exception = rx_exception;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;

#if SPINDLE_HUANYANG == 2

    modbus_message_t cmd;

    cmd.xx = VFD_GetMaxRPM;
    cmd.adu[0] = VFD_ADDRESS;
    cmd.adu[1] = ModBus_ReadHoldingRegisters;
    cmd.adu[2] = 0xB0;
    cmd.adu[3] = 0x05;
    cmd.adu[4] = 0x00;
    cmd.adu[5] = 0x02;
    cmd.tx_length = 8;
    cmd.rx_length = 8;

    modbus_send(&cmd, true);

#endif
}

#endif
