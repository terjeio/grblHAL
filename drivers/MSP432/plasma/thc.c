/*

  thc.c - plasma cutter tool height control plugin

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

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

#if PLASMA_ENABLE

#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "grbl/config.h"
#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/report.h"
#include "grbl/pid.h"
#include "grbl/nvs_buffer.h"

#include "thc.h"

// Digital ports
#define PLASMA_CUTTER_DOWN_PORT   0
#define PLASMA_CUTTER_UP_PORT     1
#define PLASMA_THC_DISABLE_PORT   2 // output
#define PLASMA_TORCH_DISABLE_PORT 3 // output
#define PLASMA_ARC_OK_PORT        3
// Analog ports
#define PLASMA_VOLTAGE_PORT       0
#define PLASMA_FEED_OVERRIDE_PORT 3

typedef union {
    uint16_t value;
    struct {
        uint16_t arc_ok        :1,
                 torch_on      :1,
                 enabled       :1,
                 ohmic_probe   :1,
                 float_switch  :1,
                 breakaway     :1,
                 active        :1,
                 up            :1,
                 down          :1,
                 velocity_lock :1,
                 void_lock     :1,
                 unassigned    :5;
    };
} thc_signals_t;

static thc_signals_t thc = {0};
static float arc_vref = 0.0f, arc_voltage = 0.0f, arc_voltage_low, arc_voltage_high, vad_threshold;
static float fr_pgm, fr_actual, fr_thr_99, fr_thr_vad;
static uint_fast8_t feed_override, segment_id = 0;
static bool set_feed_override = false;

static void state_idle (void);
static void state_thc_delay (void);
static void state_thc_pid (void);
static void state_thc_adjust (void);
static void state_vad_lock (void);

static uint32_t thc_delay = 0;
static pidf_t pid;
static void (*volatile stateHandler)(void) = state_idle;
static driver_reset_ptr driver_reset = NULL;
static spindle_set_state_ptr spindle_set_state_ = NULL;
static on_execute_realtime_ptr on_execute_realtime = NULL;
static on_realtime_report_ptr on_realtime_report = NULL;
static control_signals_callback_ptr control_interrupt_callback = NULL;
static stepper_pulse_start_ptr stepper_pulse_start = NULL;
static nvs_address_t nvs_address;
static settings_changed_ptr settings_changed;
static plasma_settings_t plasma;
static on_report_options_ptr on_report_options;
static io_port_t port = {0};

static void plasma_settings_restore (void);
static void plasma_settings_load (void);

static void pause_on_error (void)
{
    system_set_exec_state_flag(EXEC_TOOL_CHANGE);   // Set up program pause for manual tool change
    protocol_execute_realtime();                    // Execute...
}

static void digital_out (uint8_t portnum, bool on)
{
    switch(portnum) {

        case PLASMA_THC_DISABLE_PORT:
            if(!(thc.enabled = !on))
                stateHandler = state_idle;
            else if(thc.arc_ok)
                stateHandler = plasma.mode == Plasma_mode2 ? state_thc_adjust : state_thc_pid;
            break;

        case PLASMA_TORCH_DISABLE_PORT:
            // TODO
            break;

        default:
            if(port.digital_out)
                port.digital_out(portnum, on);
            break;
    }
}

static bool analog_out (uint8_t portnum, float value)
{
    switch(portnum) {

        case PLASMA_FEED_OVERRIDE_PORT:
            // Let the foreground process handle this
            set_feed_override = true;
            feed_override = (uint_fast8_t)value;
            if(feed_override < 10 || feed_override > 100)
                feed_override = 100;
            break;

        default:
            return port.analog_out ? port.analog_out(portnum, value) : false;
    }

    return true;
}

static void set_target_voltage (float v)
{
    arc_vref = v;
    arc_voltage_low  = arc_vref - plasma.thc_threshold;
    arc_voltage_high = arc_vref + plasma.thc_threshold;
}

/* THC state machine */

static void state_idle (void)
{
    arc_voltage = (float)port.wait_on_input(false, PLASMA_VOLTAGE_PORT, WaitMode_Immediate, 0.0f) * plasma.arc_voltage_scale;
}

static void state_thc_delay (void)
{
    if(hal.get_elapsed_ticks() >= thc_delay) {
        thc.enabled = On;
        if(plasma.mode == Plasma_mode2)
            stateHandler = state_thc_adjust;
        else {
            pidf_reset(&pid);
            set_target_voltage((float)port.wait_on_input(false, PLASMA_VOLTAGE_PORT, WaitMode_Immediate, 0.0f) * plasma.arc_voltage_scale);
            stateHandler = state_vad_lock;
            stateHandler();
        }
    }
}

static void state_thc_adjust (void)
{
    if((thc.arc_ok = port.wait_on_input(true, PLASMA_ARC_OK_PORT, WaitMode_Immediate, 0.0f) == 1)) {

        if(port.wait_on_input(true, PLASMA_CUTTER_UP_PORT, WaitMode_Immediate, 0.0f))
            hal.stepper.output_step((axes_signals_t){Z_AXIS_BIT}, (axes_signals_t){Z_AXIS_BIT});
        else if(port.wait_on_input(true, PLASMA_CUTTER_DOWN_PORT, WaitMode_Immediate, 0.0f))
            hal.stepper.output_step((axes_signals_t){Z_AXIS_BIT}, (axes_signals_t){0});

    } else
        pause_on_error();
}

static void state_vad_lock (void)
{
    if((thc.active = fr_actual >= fr_thr_99))
        stateHandler = state_thc_pid;
}

static void state_thc_pid (void)
{
    if(!(thc.active = fr_actual >= fr_thr_vad)) {
        stateHandler = state_vad_lock;
        return;
    }

    if((thc.arc_ok = port.wait_on_input(true, PLASMA_ARC_OK_PORT, WaitMode_Immediate, 0.0f) == 1)) {

        arc_voltage = (float)port.wait_on_input(false, PLASMA_VOLTAGE_PORT, WaitMode_Immediate, 0.0f) * plasma.arc_voltage_scale;

        if(arc_voltage >= arc_voltage_high)
            hal.stepper.output_step((axes_signals_t){Z_AXIS_BIT}, (axes_signals_t){Z_AXIS_BIT});
        else if(arc_voltage <= arc_voltage_low)
            hal.stepper.output_step((axes_signals_t){Z_AXIS_BIT}, (axes_signals_t){0});

    } else
        pause_on_error();
/*
    if(arc_voltage < arc_voltage_low || arc_voltage > arc_voltage_high) {
        float err = pidf(&pid, arc_vref, arc_voltage, 1.0f);
        // Move Z
    }
    */
}

/* end THC state machine */

void onExecuteRealtime (uint_fast16_t state)
{
    static uint32_t last_ms;

    uint32_t ms = hal.get_elapsed_ticks();

    if(ms != last_ms) {
        last_ms = ms;
        stateHandler();
    }

    if(set_feed_override) {
        set_feed_override = false;
        plan_feed_override(feed_override, sys.override.rapid_rate);
    }
/*
    if(or) {
        or = false;
        hal.stream.write("[MSG:FR ");
        hal.stream.write(ftoa(fr_pgm, 1));
        hal.stream.write(" ");
        hal.stream.write(ftoa(fr_actual, 1));
        hal.stream.write("]" ASCII_EOL);
    }
*/
    on_execute_realtime(state);
}

// Optional function to be called on soft reset (ctrl-X)
static void reset (void)
{
    thc.value = 0;
    stateHandler = state_idle;

    driver_reset(); // If other plugins needs to be told about the reset call the next function in the chain here.
}

// Start or stop arc
static void arcSetState (spindle_state_t state, float rpm)
{
    if (!state.on) {
        if(plasma.pause_at_end > 0.0f)
            delay_sec(plasma.pause_at_end, DelayMode_Dwell);
        spindle_set_state_(state, rpm);
        thc.torch_on = thc.arc_ok = thc.enabled = Off;
        stateHandler = state_idle;
    } else {
        uint_fast8_t retries = plasma.arc_retries;
        do {
            spindle_set_state_(state, rpm);
            thc.torch_on = On;
            report_message("arc on", Message_Plain);
            if((thc.arc_ok = port.wait_on_input(true, PLASMA_ARC_OK_PORT, WaitMode_High, plasma.arc_fail_timeout) == 1)) {
                report_message("arc ok", Message_Plain);
                retries = 0;
                thc_delay = hal.get_elapsed_ticks() + (uint32_t)ceilf(1000.0f * plasma.thc_delay); // handle overflow!
                stateHandler = state_thc_delay;
            } else if(!(--retries)) {
                thc.torch_on = Off;
                report_message("arc failed", Message_Warning);
                spindle_set_state_((spindle_state_t){0}, 0.0f);
                pause_on_error(); // output message and enter similar state as tool change state (allow jogging before resume)
            } else {
                thc.torch_on = Off;
                report_message("arc delay", Message_Plain);
                spindle_set_state_((spindle_state_t){0}, 0.0f);
                delay_sec(plasma.arc_retry_delay, DelayMode_Dwell);
            }
        } while(retries);
    }
}

static void stepperPulseStart (stepper_t *stepper)
{
    static volatile bool get_rates = false;

    if(stepper->new_block) {
        get_rates = true;
        fr_pgm = stepper->exec_block->programmed_rate * 0.01f * sys.override.feed_rate;
        fr_thr_99 = fr_pgm * 0.99f;
        fr_thr_vad = fr_pgm * 0.01f * (float)plasma.vad_threshold;
        segment_id = 0;
    }

    if(stepper->exec_segment->id != segment_id) {
        segment_id = stepper->exec_segment->id;
        fr_actual = stepper->exec_segment->current_rate;
    }

    stepper_pulse_start(stepper);
}

// Reclaim entry points that may have been changed on settings change.
static void onSettingsChanged (settings_t *settings)
{
    settings_changed(settings);

    if(hal.spindle.set_state != arcSetState) {
        spindle_set_state_ = hal.spindle.set_state;
        hal.spindle.set_state = arcSetState;
    }

    if(hal.stepper.pulse_start != stepperPulseStart) {
        stepper_pulse_start = hal.stepper.pulse_start;
        hal.stepper.pulse_start = stepperPulseStart;
    }
}

// Trap cycle start commands and redirect to foreground process
// by temporarily claiming the HAL execute_realtime entry point
// in order to execute probing and spindle/coolant change.
// TODO: move to state machine with own EXEC_ bit?
ISR_CODE static void trap_control_interrupts (control_signals_t signals)
{
    if(signals.value)
        control_interrupt_callback(signals);
}

static void onRealtimeReport (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    static char buf[15];
    char *append = &buf[5];

    strcpy(buf, "|THC:");
    strcat(buf, ftoa(arc_voltage, 1));

    append = &buf[strlen(buf)];

    if (thc.value) {
        *append++ = ',';
        if (thc.arc_ok)
            *append++ = 'A';
        if (thc.enabled)
            *append++ = 'E';
        if (thc.active)
            *append++ = 'R';
        if (thc.torch_on)
            *append++ = 'T';
        if (thc.ohmic_probe)
            *append++ = 'O';
        if (thc.velocity_lock)
            *append++ = 'V';
        if (thc.void_lock)
            *append++ = 'H';
        if(thc.down)
            *append++ = 'D';
        if(thc.up)
            *append++ = 'U';
    }
    *append = '\0';
    stream_write(buf);

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}

static const setting_group_detail_t plasma_groups [] = {
    { Group_Root, Group_Plasma, "Plasma"},
};

static const setting_detail_t plasma_settings[] = {
    { Setting_THC_Mode, Group_Plasma, "Plasma mode", NULL, Format_RadioButtons, "Off,Up/down,Voltage", NULL, NULL, Setting_NonCore, &plasma.mode, NULL, NULL },
    { Setting_THC_Delay, Group_Plasma, "Plasma THC delay", "s", Format_Decimal, "#0.0", NULL, NULL, Setting_NonCore, &plasma.thc_delay, NULL, NULL },
    { Setting_THC_Threshold, Group_Plasma, "Plasma THC threshold", "V", Format_Decimal, "#0.00", NULL, NULL, Setting_NonCore, &plasma.thc_threshold, NULL, NULL },
    { Setting_THC_PGain, Group_Plasma, "Plasma THC P-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_NonCore, &plasma.pid.p_gain, NULL, NULL },
    { Setting_THC_IGain, Group_Plasma, "Plasma THC I-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_NonCore, &plasma.pid.i_gain, NULL, NULL },
    { Setting_THC_DGain, Group_Plasma, "Plasma THC D-gain", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_NonCore, &plasma.pid.d_gain, NULL, NULL },
    { Setting_THC_VADThreshold, Group_Plasma, "Plasma THC VAD threshold", "percent", Format_Integer, "##0", "0", "100", Setting_NonCore, &plasma.vad_threshold, NULL, NULL },
    { Setting_THC_VoidOverride, Group_Plasma, "Plasma THC Void override", "percent", Format_Integer, "##0", "0", "100", Setting_NonCore, &plasma.thc_override, NULL, NULL },
    { Setting_Arc_FailTimeout, Group_Plasma, "Plasma Arc fail timeout", "seconds", Format_Decimal, "#0.0", NULL, NULL, Setting_NonCore, &plasma.arc_fail_timeout, NULL, NULL },
    { Setting_Arc_RetryDelay, Group_Plasma, "Plasma Arc retry delay", "seconds", Format_Decimal, "#0.0", NULL, NULL, Setting_NonCore, &plasma.arc_retry_delay, NULL, NULL },
    { Setting_Arc_MaxRetries, Group_Plasma, "Plasma Arc max retries", NULL, Format_Int8, "#0", NULL, NULL, Setting_NonCore, &plasma.arc_retries, NULL, NULL },
    { Setting_Arc_VoltageScale, Group_Plasma, "Plasma Arc voltage scale", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_NonCore, &plasma.arc_voltage_scale, NULL, NULL },
    { Setting_Arc_VoltageOffset, Group_Plasma, "Plasma Arc voltage offset", NULL, Format_Decimal, "###0.000", NULL, NULL, Setting_NonCore, &plasma.arc_voltage_offset, NULL, NULL },
    { Setting_Arc_HeightPerVolt, Group_Plasma, "Plasma Arc height per volt", "mm", Format_Decimal, "###0.000", NULL, NULL, Setting_NonCore, &plasma.arc_height_per_volt, NULL, NULL },
    { Setting_Arc_OkHighVoltage, Group_Plasma, "Plasma Arc ok high volts", "V", Format_Decimal, "###0.000", NULL, NULL, Setting_NonCore, &plasma.arc_high_low_voltage, NULL, NULL },
    { Setting_Arc_OkLowVoltage, Group_Plasma, "Plasma Arc ok low volts", "V", Format_Decimal, "###0.000", NULL, NULL, Setting_NonCore, &plasma.arc_ok_low_voltage, NULL, NULL }
};

static void plasma_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plasma, sizeof(plasma_settings_t), true);
}

static setting_details_t details = {
    .groups = plasma_groups,
    .n_groups = sizeof(plasma_groups) / sizeof(setting_group_detail_t),
    .settings = plasma_settings,
    .n_settings = sizeof(plasma_settings) / sizeof(setting_detail_t),
    .save = plasma_settings_save,
    .load = plasma_settings_load,
    .restore = plasma_settings_restore
};

static setting_details_t *onReportSettings (void)
{
    return &details;
}

static void plasma_settings_restore (void)
{
    plasma.mode = Plasma_mode1;
    plasma.thc_delay = 3.0f;
    plasma.thc_threshold = 1.0f;
    plasma.thc_override = 100;
    plasma.vad_threshold = 90;
    plasma.pause_at_end = 0.0f;
    plasma.pierce_delay = 0.0f;
    plasma.pierce_height = 1.0f;
    plasma.arc_fail_timeout = 3.0f;
    plasma.arc_retries = 3;
    plasma.arc_retry_delay = 3.0f;
    plasma.arc_fail_timeout = 3.0f;
    plasma.arc_voltage_scale = 1.0f;
    plasma.arc_voltage_offset = 0.0f;
    plasma.arc_height_per_volt = 0.1f;
    plasma.arc_high_low_voltage = 150.0;
    plasma.arc_ok_low_voltage = 100.0f;
    plasma.pid.p_gain = 25.0f;
    plasma.pid.i_gain = 0.0f;
    plasma.pid.d_gain = 0.0f;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&plasma, sizeof(plasma_settings_t), true);
}

static void plasma_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&plasma, nvs_address, sizeof(plasma_settings_t), true) != NVS_TransferResult_OK)
        plasma_settings_restore();
}

static void plasma_warning (uint_fast16_t state)
{
    report_message("Plasma mode failed to initialize!", Message_Warning);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(newopt)
        hal.stream.write(",THC");
    else
        hal.stream.write("[PLUGIN:PLASMA v0.01]" ASCII_EOL);
}

bool plasma_init (void)
{
    if(hal.port.num_analog_in > 0 && hal.port.num_digital_in > 1 && hal.port.wait_on_input && hal.stepper.output_step) {

        if ((nvs_address = nvs_alloc(sizeof(plasma_settings_t)))) {

            if(port.wait_on_input == NULL) {

                memcpy(&port, &hal.port, sizeof(io_port_t));
                hal.port.digital_out = digital_out;
                hal.port.analog_out = analog_out;
                hal.port.num_digital_out = max(port.num_digital_out, 3);
                hal.port.num_analog_out = max(port.num_analog_out, 4);

                spindle_set_state_ = hal.spindle.set_state;
                hal.spindle.set_state = arcSetState;

                on_execute_realtime = grbl.on_execute_realtime;
                grbl.on_execute_realtime = onExecuteRealtime;

        /*
                control_interrupt_callback = hal.control_interrupt_callback;
                hal.control_interrupt_callback = trap_control_interrupts;
        */
                driver_reset = hal.driver_reset;
                hal.driver_reset = reset;

                stepper_pulse_start = hal.stepper.pulse_start;
                hal.stepper.pulse_start = stepperPulseStart;

                settings_changed = hal.settings_changed;
                hal.settings_changed = onSettingsChanged;

                on_report_options = grbl.on_report_options;
                grbl.on_report_options = onReportOptions;

                on_realtime_report = grbl.on_realtime_report;
                grbl.on_realtime_report = onRealtimeReport;

                details.on_get_settings = grbl.on_get_settings;
                grbl.on_get_settings = onReportSettings;

                hal.driver_cap.spindle_at_speed = Off;

                pidf_init(&pid, &plasma.pid);
            }
        }
    }

    if(nvs_address == 0)
        protocol_enqueue_rt_command(plasma_warning);

    return nvs_address != 0;
}

#endif
