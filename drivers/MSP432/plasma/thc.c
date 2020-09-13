/*

  thc.c - plasma cutter tool height control plugin

  Part of GrblHAL

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
static void (*spindle_set_state_)(spindle_state_t state, float rpm) = NULL;
static void (*on_execute_realtime)(uint_fast16_t state) = NULL;
static void (*on_realtime_report)(stream_write_ptr stream_write, report_tracking_flags_t report) = NULL;
static void (*control_interrupt_callback)(control_signals_t signals) = NULL;
static void (*stepper_pulse_start)(stepper_t *stepper) = NULL;
static io_port_t port = {0};

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
                stateHandler = driver_settings.plasma.mode == Plasma_mode2 ? state_thc_adjust : state_thc_pid;
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
    arc_voltage_low  = arc_vref - driver_settings.plasma.thc_threshold;
    arc_voltage_high = arc_vref + driver_settings.plasma.thc_threshold;
}

/* THC state machine */

static void state_idle (void)
{
    arc_voltage = (float)port.wait_on_input(false, PLASMA_VOLTAGE_PORT, WaitMode_Immediate, 0.0f) * driver_settings.plasma.arc_voltage_scale;
}

static void state_thc_delay (void)
{
    if(hal.get_elapsed_ticks() >= thc_delay) {
        thc.enabled = On;
        if(driver_settings.plasma.mode == Plasma_mode2)
            stateHandler = state_thc_adjust;
        else {
            pidf_reset(&pid);
            set_target_voltage((float)port.wait_on_input(false, PLASMA_VOLTAGE_PORT, WaitMode_Immediate, 0.0f) * driver_settings.plasma.arc_voltage_scale);
            stateHandler = state_vad_lock;
            stateHandler();
        }
    }
}

static void state_thc_adjust (void)
{
    if((thc.arc_ok = port.wait_on_input(true, PLASMA_ARC_OK_PORT, WaitMode_Immediate, 0.0f) == 1)) {

        if(port.wait_on_input(true, PLASMA_CUTTER_UP_PORT, WaitMode_Immediate, 0.0f))
            hal.stepper_output_step((axes_signals_t){Z_AXIS_BIT}, (axes_signals_t){Z_AXIS_BIT});
        else if(port.wait_on_input(true, PLASMA_CUTTER_DOWN_PORT, WaitMode_Immediate, 0.0f))
            hal.stepper_output_step((axes_signals_t){Z_AXIS_BIT}, (axes_signals_t){0});

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

        arc_voltage = (float)port.wait_on_input(false, PLASMA_VOLTAGE_PORT, WaitMode_Immediate, 0.0f) * driver_settings.plasma.arc_voltage_scale;

        if(arc_voltage >= arc_voltage_high)
            hal.stepper_output_step((axes_signals_t){Z_AXIS_BIT}, (axes_signals_t){Z_AXIS_BIT});
        else if(arc_voltage <= arc_voltage_low)
            hal.stepper_output_step((axes_signals_t){Z_AXIS_BIT}, (axes_signals_t){0});

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

void plasma_poll (uint_fast16_t state)
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
        if(driver_settings.plasma.pause_at_end > 0.0f)
            delay_sec(driver_settings.plasma.pause_at_end, DelayMode_Dwell);
        spindle_set_state_(state, rpm);
        thc.torch_on = thc.arc_ok = thc.enabled = Off;
        stateHandler = state_idle;
    } else {
        uint_fast8_t retries = driver_settings.plasma.arc_retries;
        do {
            spindle_set_state_(state, rpm);
            thc.torch_on = On;
            hal.stream.write("[MSG:arc on]" ASCII_EOL);
            if((thc.arc_ok = port.wait_on_input(true, PLASMA_ARC_OK_PORT, WaitMode_High, driver_settings.plasma.arc_fail_timeout) == 1)) {
                hal.stream.write("[MSG:arc ok]" ASCII_EOL);
                retries = 0;
                thc_delay = hal.get_elapsed_ticks() + (uint32_t)ceilf(1000.0f * driver_settings.plasma.thc_delay); // handle overflow!
                stateHandler = state_thc_delay;
            } else if(!(--retries)) {
                thc.torch_on = Off;
                hal.stream.write("[MSG:arc failed]" ASCII_EOL);
                spindle_set_state_((spindle_state_t){0}, 0.0f);
                pause_on_error(); // output message and enter similar state as tool change state (allow jogging before resume)
            } else {
                thc.torch_on = Off;
                hal.stream.write("[MSG:arc delay]" ASCII_EOL);
                spindle_set_state_((spindle_state_t){0}, 0.0f);
                delay_sec(driver_settings.plasma.arc_retry_delay, DelayMode_Dwell);
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
        fr_thr_vad = fr_pgm * 0.01f * (float)driver_settings.plasma.vad_threshold;
        segment_id = 0;
    }

    if(stepper->exec_segment->id != segment_id) {
        segment_id = stepper->exec_segment->id;
        fr_actual = stepper->exec_segment->current_rate;
    }

    stepper_pulse_start(stepper);
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

static void plasma_rt_report (stream_write_ptr stream_write, report_tracking_flags_t report)
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

status_code_t plasma_setting (setting_type_t param, float value, char *svalue)
{
    status_code_t status = svalue ? Status_OK : Status_Unhandled;

    if(svalue) switch(param) {

        case Setting_THC_Mode:
            if(isintf(value) && value != NAN && value > Plasma_mode0 && value <= Plasma_mode2)
                driver_settings.plasma.mode = (plasma_mode_t)value;
            else
                status = Status_InvalidStatement; //out of range...
            break;

        case Setting_THC_Delay:
            driver_settings.plasma.thc_delay = value;
            break;

        case Setting_THC_Threshold:
            driver_settings.plasma.thc_threshold = value;
            break;

        case Setting_THC_PGain:
            driver_settings.plasma.pid.p_gain = value;
            break;

        case Setting_THC_IGain:
            driver_settings.plasma.pid.i_gain = value;
            break;

        case Setting_THC_DGain:
            driver_settings.plasma.pid.d_gain = value;
            break;

        case Setting_THC_VADThreshold:
            if(isintf(value) && value != NAN && value > 0.0f && value <= 100.0f)
                driver_settings.plasma.vad_threshold = (uint8_t)value;
            else
                status = Status_InvalidStatement; //out of range...
            break;

        case Setting_THC_VoidOverride:
            if(isintf(value) && value != NAN && value > 0.0f && value <= 100.0f)
                driver_settings.plasma.thc_override = (uint8_t)value;
            else
                status = Status_InvalidStatement; //out of range...
            break;

        case Setting_Arc_FailTimeout:
            driver_settings.plasma.arc_fail_timeout = value;
            break;

        case Setting_Arc_RetryDelay:
            driver_settings.plasma.arc_retry_delay = value;
            break;

        case Setting_Arc_MaxRetries:
            if(isintf(value) && value != NAN && value > 0.0f)
                driver_settings.plasma.arc_retries = (uint8_t)value;
            else
                status = Status_InvalidStatement; //out of range...
            break;

        case Setting_Arc_VoltageScale:
            driver_settings.plasma.arc_voltage_scale = value;
            break;

        case Setting_Arc_VoltageOffset:
            driver_settings.plasma.arc_voltage_offset = value;
            break;

        case Setting_Arc_HeightPerVolt:
            driver_settings.plasma.arc_height_per_volt = value;
            break;

        case Setting_Arc_OkHighVoltage:
            driver_settings.plasma.arc_high_low_voltage = value;
            break;

        case Setting_Arc_OkLowVoltage:
            driver_settings.plasma.arc_ok_low_voltage = value;
            break;

        default:
            status = Status_Unhandled;
            break;
    }

    return status;
}

void plasma_settings_report (setting_type_t setting)
{
    switch(setting) {

        case Setting_THC_Mode:
            report_uint_setting(setting, (uint32_t)driver_settings.plasma.mode);
            break;

        case Setting_THC_Delay:
            report_float_setting(setting, driver_settings.plasma.thc_delay, 1);
            break;

        case Setting_THC_Threshold:
            report_float_setting(setting, driver_settings.plasma.thc_threshold, 1);
            break;
/*
        case Setting_THC_PGain:
            report_float_setting(setting, driver_settings.plasma.pid.p_gain, 1);
            break;

        case Setting_THC_IGain:
            report_float_setting(setting, driver_settings.plasma.pid.i_gain, 3);
            break;

        case Setting_THC_DGain:
            report_float_setting(setting, driver_settings.plasma.pid.d_gain, 3);
            break;
*/
        case Setting_THC_VADThreshold:
            report_uint_setting(setting, (uint32_t)driver_settings.plasma.vad_threshold);
            break;

        case Setting_THC_VoidOverride:
            report_uint_setting(setting, driver_settings.plasma.thc_override);
            break;

        case Setting_Arc_FailTimeout:
            report_float_setting(setting, driver_settings.plasma.arc_fail_timeout, 1);
            break;

        case Setting_Arc_RetryDelay:
            report_float_setting(setting, driver_settings.plasma.arc_retry_delay, 1);
            break;

        case Setting_Arc_MaxRetries:
            report_uint_setting(setting, driver_settings.plasma.arc_retries);
            break;

        case Setting_Arc_VoltageScale:
            report_float_setting(setting, driver_settings.plasma.arc_voltage_scale, 5);
            break;

        case Setting_Arc_VoltageOffset:
            report_float_setting(setting, driver_settings.plasma.arc_voltage_offset, 2);
            break;

        case Setting_Arc_HeightPerVolt:
            report_float_setting(setting, driver_settings.plasma.arc_height_per_volt, 3);
            break;
/*
        case Setting_Arc_OkHighVoltage:
            report_float_setting(setting, driver_settings.plasma.arc_high_low_voltage, 1);
            break;

        case Setting_Arc_OkLowVoltage:
            report_float_setting(setting, driver_settings.plasma.arc_ok_low_voltage, 1);
            break;
*/
        default:
            break;
    }
}

void plasma_settings_restore (void)
{
    driver_settings.plasma.mode = Plasma_mode1;
    driver_settings.plasma.thc_delay = 3.0f;
    driver_settings.plasma.thc_threshold = 1.0f;
    driver_settings.plasma.thc_override = 100;
    driver_settings.plasma.vad_threshold = 90;
    driver_settings.plasma.pause_at_end = 0.0f;
    driver_settings.plasma.pierce_delay = 0.0f;
    driver_settings.plasma.pierce_height = 1.0f;
    driver_settings.plasma.arc_fail_timeout = 3.0f;
    driver_settings.plasma.arc_retries = 3;
    driver_settings.plasma.arc_retry_delay = 3.0f;
    driver_settings.plasma.arc_fail_timeout = 3.0f;
    driver_settings.plasma.arc_voltage_scale = 1.0f;
    driver_settings.plasma.arc_voltage_offset = 0.0f;
    driver_settings.plasma.arc_height_per_volt = 0.1f;
    driver_settings.plasma.arc_high_low_voltage = 150.0;
    driver_settings.plasma.arc_ok_low_voltage = 100.0f;
    driver_settings.plasma.pid.p_gain = 25.0f;
    driver_settings.plasma.pid.i_gain = 0.0f;
    driver_settings.plasma.pid.d_gain = 0.0f;
}

void plasma_init (void)
{
    if(hal.port.num_analog_in > 0 && hal.port.num_digital_in > 1 && hal.port.wait_on_input && hal.stepper_output_step) {

        stepper_pulse_start = hal.stepper_pulse_start;
        hal.stepper_pulse_start = stepperPulseStart;

        if(port.wait_on_input == NULL) {

            memcpy(&port, &hal.port, sizeof(io_port_t));
            hal.port.digital_out = digital_out;
            hal.port.analog_out = analog_out;
            hal.port.num_digital_out = max(port.num_digital_out, 3);
            hal.port.num_analog_out = max(port.num_analog_out, 4);

            spindle_set_state_ = hal.spindle_set_state;
            hal.spindle_set_state = arcSetState;

            on_execute_realtime = grbl.on_execute_realtime;
            grbl.on_execute_realtime = plasma_poll;

    /*
            control_interrupt_callback = hal.control_interrupt_callback;
            hal.control_interrupt_callback = trap_control_interrupts;
    */
            on_realtime_report = grbl.on_realtime_report;
            grbl.on_realtime_report = plasma_rt_report;

            driver_reset = hal.driver_reset;        // Save pointer to previous reset handler and
            hal.driver_reset = reset;               // add our to top of the chain.

            hal.driver_cap.spindle_at_speed = Off;

            pidf_init(&pid, &driver_settings.plasma.pid);
        }

    } else
        hal.stream.write("[MSG:Plasma mode failed initialization]" ASCII_EOL);
}

#endif
