/*

  coolant.c - plugin for for handling laser coolant

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

#include "driver.h"

#if LASER_COOLANT_ENABLE

#ifndef LASER_COOLANT_ON_PORT
#define LASER_COOLANT_ON_PORT 0
#endif
#ifndef LASER_COOLANT_OK_PORT
#define LASER_COOLANT_OK_PORT 0
#endif
#ifndef LASER_COOLANT_TEMPERATURE_PORT
#define LASER_COOLANT_TEMPERATURE_PORT 0
#endif

#include <string.h>
#include <math.h>

#include "grbl/hal.h"
#include "grbl/protocol.h"

static bool coolant_on = false, monitor_on = false, can_wait = false, can_monitor = false;
static float max_temp = 30.0f;
static user_mcode_ptrs_t user_mcode;
static on_report_options_ptr on_report_options;
static on_realtime_report_ptr on_realtime_report;
static on_program_completed_ptr on_program_completed;
static driver_reset_ptr driver_reset;

static user_mcode_t userMCodeCheck (user_mcode_t mcode)
{
    return mcode == Laser_Coolant
                     ? mcode
                     : (user_mcode.check ? user_mcode.check(mcode) : UserMCode_Ignore);
}

static status_code_t userMCodeValidate (parser_block_t *gc_block, parameter_words_t *value_words)
{
    status_code_t state = Status_GcodeValueWordMissing;

    switch(gc_block->user_mcode) {

        case Laser_Coolant:
            if((*value_words).p && isnan(gc_block->values.p))
                state = Status_BadNumberFormat;
            if((*value_words).q && isnan(gc_block->values.q))
                state = Status_BadNumberFormat;
            if((*value_words).r && isnan(gc_block->values.r))
                state = Status_BadNumberFormat;

            if(state != Status_BadNumberFormat) {
                if(((*value_words).q && gc_block->values.q > 0.0f && !can_wait) ||
                   ((*value_words).r && gc_block->values.r > 0.0f && !can_monitor))
                    state = Status_GcodeUnusedWords;
                else if((*value_words).q) {
                    state = Status_OK;
                    gc_block->user_mcode_sync = true;
                    if(!(*value_words).p)
                        gc_block->values.p = 0.0f;
                    if(!(*value_words).r)
                        gc_block->values.r = NAN;
                    (*value_words).p = (*value_words).q = (*value_words).r = Off;
                }
            }
            break;

        default:
            state = Status_Unhandled;
            break;
    }

    return state == Status_Unhandled && user_mcode.validate ? user_mcode.validate(gc_block, value_words) : state;
}

static void userMCodeExecute (uint_fast16_t state, parser_block_t *gc_block)
{
    bool handled = true;

    if (state != STATE_CHECK_MODE)
      switch(gc_block->user_mcode) {

        case Laser_Coolant:
            // TODO: cannot switch off unless state == STATE_IDLE?
            coolant_on = (uint16_t)gc_block->values.q != 0.0f;
            hal.port.digital_out(LASER_COOLANT_ON_PORT, coolant_on);
            if(coolant_on && can_wait && gc_block->values.p > 0.0f && hal.port.wait_on_input(true, LASER_COOLANT_OK_PORT, WaitMode_High, gc_block->values.p) != 1) {
                coolant_on = false;
                system_set_exec_alarm(Alarm_AbortCycle);
            }
            if(!isnan(gc_block->values.r)) {
                max_temp = gc_block->values.r;
                monitor_on = max_temp > 0.0f;
            }
            sys.report.coolant = On; // Set to report change immediately
            break;

        default:
            handled = false;
            break;
    }

    if(!handled && user_mcode.execute)
        user_mcode.execute(state, gc_block);
}

static void driverReset (void)
{
    driver_reset();

    if(coolant_on) {
        coolant_on = false;
        hal.port.digital_out(LASER_COOLANT_ON_PORT, false);
        sys.report.coolant = On; // Set to report change immediately
    }
}

static void onProgramCompleted (program_flow_t program_flow)
{
    // Keep coolant and exhaust (flood) on? Setting? Delayed task?
    if(coolant_on) {
        coolant_on = false;
        hal.port.digital_out(LASER_COOLANT_ON_PORT, false);
        sys.report.coolant = On; // Set to report change immediately
    }

    if(on_program_completed)
        on_program_completed(program_flow);
}

static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    static float coolant_temp_prev = 0.0f;

    char buf[20] = "";

    if(report.coolant) {
        strcpy(buf, "|Ex:");
        if(coolant_on)
            strcat(buf, "C");
    } else
        *buf = '\0';

    if(can_monitor) {

        float coolant_temp = (float)hal.port.wait_on_input(false, LASER_COOLANT_TEMPERATURE_PORT, WaitMode_Immediate, 0.0f) / 10.0f;

        if(coolant_temp_prev != coolant_temp || report.all) {
            strcat(buf, "|TCT:");
            strcat(buf, ftoa(coolant_temp, 1));
            coolant_temp_prev = coolant_temp;
        }

        if(monitor_on && coolant_temp > max_temp)
            system_set_exec_alarm(Alarm_AbortCycle);
    }

    if(*buf != '\0')
        stream_write(buf);

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:LASER COOLANT v0.01]" ASCII_EOL);
}

static void warning_msg (uint_fast16_t state)
{
    report_message("Laser coolant handler failed to initialize!", Message_Warning);
}

void laser_coolant_init (void)
{
    if(settings.mode == Mode_Laser && hal.port.num_digital_out > LASER_COOLANT_ON_PORT) {

        memcpy(&user_mcode, &hal.user_mcode, sizeof(user_mcode_ptrs_t));

        hal.user_mcode.check = userMCodeCheck;
        hal.user_mcode.validate = userMCodeValidate;
        hal.user_mcode.execute = userMCodeExecute;

        driver_reset = hal.driver_reset;
        hal.driver_reset = driverReset;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        on_realtime_report = grbl.on_realtime_report;
        grbl.on_realtime_report = onRealtimeReport;

        on_program_completed = grbl.on_program_completed;
        grbl.on_program_completed = onProgramCompleted;

        can_wait = hal.port.wait_on_input && hal.port.num_digital_in > LASER_COOLANT_OK_PORT;
        can_monitor = hal.port.wait_on_input && hal.port.num_analog_in > LASER_COOLANT_TEMPERATURE_PORT;
    } else
        protocol_enqueue_rt_command(warning_msg);
}

#endif
