/*
  encoder.c - quadrature encoder plugin

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

#include <math.h>
#include <stdlib.h>

#include "encoder.h"

#if QEI_ENABLE

#ifdef ARDUINO
#include "../grbl/grbl.h"
#include "../grbl/report.h"
#include "../grbl/protocol.h"
#include "../grbl/nvs_buffer.h"
#else
#include "grbl/grbl.h"
#include "grbl/report.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#endif

#include <stdio.h>
#include <string.h>

#define MIN(a, b) (((a) > (b)) ? (b) : (a))

typedef bool (*mpg_algo_ptr)(sys_state_t state, axes_signals_t axes);

typedef union {
    uint8_t events;
    struct {
        uint8_t position_changed :1,
                zero             :1,
                lock             :1,
                reset            :1,
                scale            :1,
                stop             :1,
                unused           :2;
    };
} mpg_event_t;

typedef union {
    uint8_t all;
    struct {
        uint8_t moving  :1,
                zero    :1,
                lock    :1,
                reset   :1,
                unused  :4;
    };
} mpg_flags_t;

typedef struct {
    int32_t position;
    mpg_event_t event;
    mpg_flags_t flags;
    uint32_t next_event;
    float pos;
    float scale_factor;
    encoder_t *encoder;
    mpg_algo_ptr handler;
} mpg_t;

static char gcode[50];
static int32_t npos[QEI_ENABLE] = {0};
static mpg_t mpg[N_AXIS] = {0};
static mpg_event_t mpg_events[N_AXIS] = {0};
static encoder_t *override_encoder = NULL; // NULL when no Encoder_Universal available
static axes_signals_t mpg_event = {0};
static volatile bool mpg_spin_lock = false;
static on_realtime_report_ptr on_realtime_report = NULL;
static on_execute_realtime_ptr on_execute_realtime = NULL;
static on_report_options_ptr on_report_options;
static nvs_address_t nvs_address;
static encoder_settings_t encoders[QEI_ENABLE];
static uint_fast8_t n_encoder;

static void encoder_settings_load (void);
static void encoder_settings_restore (void);
static uint32_t encoder_get_value (setting_id_t setting);
static status_code_t encoder_set_value (setting_id_t setting, uint_fast16_t value);

static char *append (char *s)
{
    while(*s)
        s++;

    return s;
}

// MPG encoder movement algorithms
// Bind the one to use to the axis MPGs at end of encoder_init(), later this will be made configurable (per axis?)

static bool mpg_move_absolute (sys_state_t state, axes_signals_t axes)
{
    static bool is_moving = false;

    int32_t delta;
    uint32_t velocity = 0;
    uint_fast8_t idx = 0;

    strcpy(gcode, "G1");

    while(axes.mask) {

        if(axes.mask & 0x01) {
            if((delta = mpg[idx].position - npos[mpg[idx].encoder->id]) != 0) {
                float pos_delta = (float)delta * mpg[idx].scale_factor / 100.0f;
                mpg[idx].position = npos[idx];
                velocity = velocity == 0 ? mpg[idx].encoder->velocity : MIN(mpg[idx].encoder->velocity, velocity);
                if(!gc_state.modal.distance_incremental)
                    mpg[idx].pos += pos_delta;
                velocity = velocity == 0 ? mpg[idx].encoder->velocity : MIN(mpg[idx].encoder->velocity, velocity);
                sprintf(append(gcode), "%s%.3f", axis_letter[idx], gc_state.modal.distance_incremental ? pos_delta : mpg[idx].pos);
            }
        }

        idx++;
        axes.mask >>= 1;
    }

    if(strlen(gcode) > 2 && velocity > 0) {

        sprintf(append(gcode), "F%lu", velocity);

        is_moving = grbl.protocol_enqueue_gcode(gcode);
#ifdef UART_DEBUG
serialWriteS(gcode);
serialWriteS(" ");
serialWriteS(uitoa(is_moving));
serialWriteS(" ");
serialWriteS(uitoa(delta));
serialWriteS(ASCII_EOL);
#endif
    }

    return is_moving;
}

static bool mpg_jog_relative (sys_state_t state, axes_signals_t axes)
{
    static bool is_moving = false;

    int32_t delta;
    uint32_t velocity = 0;
    uint_fast8_t idx = 0;

    strcpy(gcode, "$J=G91");

//   serialWriteS(uitoa(mpg[idx].encoder->position));
//   serialWriteS(ASCII_EOL);

    while(axes.mask) {

        if(axes.mask & 0x01) {
            if((delta = mpg[idx].position - npos[mpg[idx].encoder->id]) != 0) {
                float pos_delta = (float)delta * mpg[idx].scale_factor / 100.0f;
                mpg[idx].position = npos[idx];
                velocity = velocity == 0 ? mpg[idx].encoder->velocity : MIN(mpg[idx].encoder->velocity, velocity);
                sprintf(append(gcode), "%s%.3f", axis_letter[idx], pos_delta);
            }
        }

        idx++;
        axes.mask >>= 1;
    }

    if(strlen(gcode) > 6 && velocity > 0) {

        sprintf(append(gcode), "F%lu", velocity);

        is_moving = grbl.protocol_enqueue_gcode(gcode);

#ifdef UART_DEBUG
serialWriteS(gcode);
serialWriteS(" ");
serialWriteS(uitoa(is_moving));
serialWriteS(ASCII_EOL);
#endif
/*
serialWriteS(itoa(npos[mpg[idx].encoder->id], gcode, 10));
serialWriteS(ASCII_EOL);
*/
    }

    return is_moving;
}

// End MPG encoder movement algorithms

static inline void reset_override (encoder_mode_t mode)
{
    switch(mode) {

        case Encoder_FeedRate:
            hal.stream.enqueue_realtime_command(CMD_OVERRIDE_FEED_RESET);
            break;

        case Encoder_RapidRate:
            hal.stream.enqueue_realtime_command(CMD_OVERRIDE_RAPID_RESET);
            break;

        case Encoder_Spindle_RPM:
            hal.stream.enqueue_realtime_command(CMD_OVERRIDE_SPINDLE_RESET);
            break;

        default:
            break;
    }
}

static void encoder_report_mode (uint_fast16_t state)
{
    if(override_encoder) {

        switch(override_encoder->mode) {

            case Encoder_FeedRate:
                hal.stream.write("[MSG:Encoder mode feed rate]" ASCII_EOL);
                break;

            case Encoder_RapidRate:
                hal.stream.write("[MSG:Encoder mode rapid rate]" ASCII_EOL);
                break;

            case Encoder_Spindle_RPM:
                hal.stream.write("[MSG:Encoder mode spindle RPM]" ASCII_EOL);
                break;

            default:
                break;
        }
    }
}

static void encoder_execute_realtime (sys_state_t state)
{
    static uint32_t elapsed = 0;

    uint32_t ms = hal.get_elapsed_ticks();

    if(ms != elapsed && mpg_event.mask && (state == STATE_IDLE || (state & STATE_JOG))) {

        bool move_action = false, stop_action = false;
        uint_fast8_t idx = 0;

        axes_signals_t event, axes;

#ifdef UART_DEBUG
        serialWriteS("+");
#endif

        while(mpg_spin_lock);

        event.mask = axes.mask = mpg_event.mask;

        mpg_event.mask = 0;
        for(idx = 0; idx < N_AXIS; idx++)
            mpg_events[idx].events = mpg[idx].event.events;

        idx = 0;

        while(event.mask) {

            if(event.mask & 0x01) {

                if(mpg_events[idx].zero) {
                    strcpy(gcode, "G90G10L20P0");
                    strcat(gcode, axis_letter[idx]);
                    strcat(gcode, "0");
                    if(grbl.protocol_enqueue_gcode(gcode)) {
                        mpg[idx].event.zero = Off;
                        mpg[idx].position = npos[mpg[idx].encoder->id] = mpg[idx].encoder->position = 0;
                        hal.encoder.reset(mpg[idx].encoder->id);
                    }
                }

                if(mpg_events[idx].scale) {
                    mpg[idx].scale_factor *= 10.0f;
                    if(mpg[idx].scale_factor > 100.0f)
                        mpg[idx].scale_factor = 1.0f;
#ifdef UART_DEBUG
serialWriteS("Distance scale: ");
serialWriteS(ftoa(mpg[idx].scale_factor, 0));
serialWriteS(ASCII_EOL);
#endif
                }

                if(mpg_events[idx].stop) {
                    if((stop_action = mpg[idx].flags.moving && (state & STATE_JOG))) {
                        hal.stream.enqueue_realtime_command(CMD_JOG_CANCEL);
#ifdef UART_DEBUG
serialWriteS("Jog cancel");
serialWriteS(ASCII_EOL);
#endif
                    }
                    mpg[idx].flags.moving = mpg_events[idx].position_changed = Off;
                }

                if(mpg_events[idx].position_changed) {

                    if(!mpg[idx].flags.moving) {
                        float target[N_AXIS];
                        system_convert_array_steps_to_mpos(target, sys.position);
                        mpg[idx].flags.moving = On;
                        mpg[idx].pos = target[idx] - gc_get_offset(idx);
                    }

                    move_action = true;

                    mpg[idx].flags.moving = On;
                    mpg[idx].next_event += 100;
                }
            }

            mpg[idx].event.events = 0;

            idx++;
            event.mask >>= 1;
        }

        if(move_action && !mpg[0].handler(state, axes))
            mpg_event.mask |= 0; //axes.mask; // gcode was rejected, restore events
    }

    elapsed = ms;

    on_execute_realtime(state);
}

static void encoder_event (encoder_t *encoder, int32_t position)
{
    bool update_position = false;

    if(encoder->event.click) {

        if(encoder->settings->mode == Encoder_Universal) {
            sys.report.encoder = On;
            encoder->event.click = Off;
            encoder->mode = encoder->mode == Encoder_FeedRate ? Encoder_RapidRate : (encoder->mode == Encoder_RapidRate ? Encoder_Spindle_RPM : Encoder_FeedRate);
            protocol_enqueue_rt_command(encoder_report_mode); // Output mode change message from foreground process.
        } else if(encoder->settings->mode == Encoder_MPG) {
            if(++encoder->axis == N_AXIS)
                encoder->axis = X_AXIS;
            mpg[encoder->axis].position = npos[encoder->id] = encoder->position = 0;
            mpg[encoder->axis].event.events = encoder->event.events = 0;
            hal.encoder.reset(encoder->id);
        }
    }

    if(encoder->event.position_changed) {

#ifdef UART_DEBUG
        itoa(position, gcode, 10);
        serialWriteS("Pos: ");
        serialWriteS(gcode);
        serialWriteS(ASCII_EOL);
#endif

        int32_t n_count = (position * 100L) / (int32_t)encoder->settings->cpr;

        encoder->event.position_changed = Off;

        if(n_count != npos[encoder->id] || encoder->velocity == 0) switch(encoder->mode) {

            case Encoder_FeedRate:
                update_position = true;
                if(n_count < npos[encoder->id]) {
                    while(npos[encoder->id]-- != n_count)
                        hal.stream.enqueue_realtime_command(CMD_OVERRIDE_FEED_FINE_MINUS);
                } else {
                    while(npos[encoder->id]++ != n_count)
                        hal.stream.enqueue_realtime_command(CMD_OVERRIDE_FEED_FINE_PLUS);
                }
                break;

            case Encoder_RapidRate:
                update_position = abs(position - encoder->position) >= encoder->settings->cpd;

                if(update_position) switch(sys.override.rapid_rate) {

                    case DEFAULT_RAPID_OVERRIDE:
                        if(position < encoder->position)
                            hal.stream.enqueue_realtime_command(CMD_OVERRIDE_RAPID_MEDIUM);
                        break;

                    case RAPID_OVERRIDE_MEDIUM:
                        if(position < encoder->position)
                            hal.stream.enqueue_realtime_command(CMD_OVERRIDE_RAPID_LOW);
                        else
                            hal.stream.enqueue_realtime_command(CMD_OVERRIDE_RAPID_RESET);
                        break;

                    case RAPID_OVERRIDE_LOW:
                        if(position > encoder->position)
                            hal.stream.enqueue_realtime_command(CMD_OVERRIDE_RAPID_MEDIUM);
                        break;

                    default:
                        break;
                }
                break;

            case Encoder_Spindle_RPM:
                update_position = true;
                if(n_count < npos[encoder->id]) {
                    while(npos[encoder->id]-- != n_count)
                        hal.stream.enqueue_realtime_command(CMD_OVERRIDE_SPINDLE_FINE_MINUS);
                } else {
                    while(npos[encoder->id]++ != n_count)
                        hal.stream.enqueue_realtime_command(CMD_OVERRIDE_SPINDLE_FINE_PLUS);
                }
                break;

            case Encoder_MPG:
            case Encoder_MPG_X:
            case Encoder_MPG_Y:
            case Encoder_MPG_Z:
#if N_AXIS > 3
            case Encoder_MPG_A:
#endif
#if N_AXIS > 4
            case Encoder_MPG_B:
#endif
#if N_AXIS > 5
            case Encoder_MPG_C:
#endif
                update_position = true;

                mpg_spin_lock = true;
                if(mpg[encoder->axis].encoder->velocity == 0) {
                    mpg[encoder->axis].event.stop = On; // mpg[encoder->axis].flags.moving;
                    mpg_event.mask |= (1 << encoder->axis);
                } else {
                    mpg[encoder->axis].event.position_changed = On;
                    mpg_event.mask |= (1 << encoder->axis);
                }
                mpg_spin_lock = false;
                break;

            default:
                break;
        }

        if(update_position) {
            encoder->position = position;
            npos[encoder->id] = n_count;
        }
    }

    if(encoder->event.events) switch(encoder->mode) {

        case Encoder_FeedRate:
        case Encoder_RapidRate:
        case Encoder_Spindle_RPM:
            npos[encoder->id] = encoder->position = 0;
            hal.encoder.reset(encoder->id);
            reset_override(encoder->mode);
            break;

        case Encoder_MPG:
        case Encoder_MPG_X:
        case Encoder_MPG_Y:
        case Encoder_MPG_Z:
#if N_AXIS > 3
        case Encoder_MPG_A:
#endif
#if N_AXIS > 4
        case Encoder_MPG_B:
#endif
#if N_AXIS > 5
        case Encoder_MPG_C:
#endif

            mpg_spin_lock = true;
            if(encoder->event.click) {;
                mpg[encoder->axis].event.scale = On;
                mpg_event.mask |= (1 << encoder->axis);
            }
            if(encoder->event.dbl_click) {
                mpg[encoder->axis].event.zero = On;
                mpg_event.mask |= (1 << encoder->axis);
            }
            mpg_spin_lock = false;
            break;

        default:
            break;
    }

    encoder->event.events = 0;
}

static void encoder_rt_report(stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if(override_encoder && report.encoder) {
        stream_write("|Enc:");
        stream_write(uitoa(override_encoder->mode));
    }

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}

static const setting_group_detail_t encoder_groups [] = {
    { Group_Root, Group_Encoders, "Encoders"},
    { Group_Encoders, Group_Encoder0, "Encoder 0"},
    { Group_Encoders, Group_Encoder1, "Encoder 1"},
    { Group_Encoders, Group_Encoder2, "Encoder 2"},
    { Group_Encoders, Group_Encoder3, "Encoder 3"},
    { Group_Encoders, Group_Encoder4, "Encoder 4"}
};

static const setting_detail_t encoder_settings[] = {
    { Setting_EncoderModeBase, Group_Encoder0, "Encoder mode", NULL, Format_RadioButtons, "Universal,Feed rate override,Rapid rate override,Spindle RPM override", NULL, NULL, Setting_NonCoreFn, encoder_set_value, encoder_get_value, NULL },
    { Setting_EncoderCPRBase, Group_Encoder0, "Encoder counts per revolution", NULL, Format_Integer, "###0", "1", NULL, Setting_NonCoreFn, encoder_set_value, encoder_get_value, NULL },
    { Setting_EncoderCPDBase, Group_Encoder0, "Encoder counts per detent", NULL, Format_Integer, "#0", "1", NULL, Setting_NonCoreFn, encoder_set_value, encoder_get_value, NULL },
    { Setting_EncoderDblClickWindowBase, Group_Encoder0, "Encoder double click sensitivity", "ms", Format_Integer, "##0", "100", "900", Setting_NonCoreFn, encoder_set_value, encoder_get_value, NULL }
};

static void encoder_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&encoders, sizeof(encoders), true);
}

static setting_details_t details = {
    .groups = encoder_groups,
    .n_groups = QEI_ENABLE + 1,
    .settings = encoder_settings,
    .n_settings = sizeof(encoder_settings) / sizeof(setting_detail_t),
    .save = encoder_settings_save,
    .load = encoder_settings_load,
    .restore = encoder_settings_restore
};

static setting_details_t *on_get_settings (void)
{
    return &details;
}

// Store encoder configuration. Encoder numbering sequence set by n_encoder define.
static status_code_t encoder_set_value (setting_id_t setting, uint_fast16_t value)
{
    status_code_t status = Status_OK;

    uint_fast16_t base_idx = (uint_fast16_t)setting - (uint_fast16_t)Setting_EncoderSettingsBase;
    uint_fast8_t setting_idx = base_idx % ENCODER_SETTINGS_INCREMENT;
    uint_fast8_t encoder_idx = (base_idx - setting_idx) / ENCODER_SETTINGS_INCREMENT;

    if(encoder_idx < n_encoder) switch(setting_idx) {

        case Setting_EncoderMode:
            if(isintf(value) && value != NAN && value >= (float)Encoder_Universal && value < (float)Encoder_Spindle_Position)
                encoders[encoder_idx].mode = (encoder_mode_t)value;
            else
                status = Status_InvalidStatement;
            break;

        case Setting_EncoderCPR:
            encoders[encoder_idx].cpr = (uint32_t)value;
            break;

        case Setting_EncoderCPD:
            encoders[encoder_idx].cpd = (uint32_t)value;
            break;

        case Setting_EncoderDblClickWindow:
            if(isintf(value) && value != NAN && value >= 100.0f && value <= 900.0f)
                encoders[encoder_idx].dbl_click_window = (uint32_t)value;
            else
                status = Status_InvalidStatement;
            break;

        default:
            status = Status_Unhandled;
            break;

    }

    return status;
}

// Report encoder configuration. Encoder numbering sequence set by n_encoder define.
static uint32_t encoder_get_value (setting_id_t setting)
{
    uint32_t value = 0;

    uint_fast16_t base_idx = (uint_fast16_t)setting - (uint_fast16_t)Setting_EncoderSettingsBase;
    uint_fast8_t setting_idx = base_idx % ENCODER_SETTINGS_INCREMENT;
    uint_fast8_t encoder_idx = (base_idx - setting_idx) / ENCODER_SETTINGS_INCREMENT;

    if(encoder_idx < n_encoder) switch(setting_idx) {

        case Setting_EncoderMode:
            value = (uint32_t)encoders[encoder_idx].mode;
            break;

        case Setting_EncoderCPR:
            value = encoders[encoder_idx].cpr;
            break;

        case Setting_EncoderCPD:
            value = encoders[encoder_idx].cpd;
            break;

        case Setting_EncoderDblClickWindow:
            value = encoders[encoder_idx].dbl_click_window;
            break;

        default:
            break;
    }

    return value;
}

static void encoder_settings_restore (void)
{
    uint_fast8_t idx;

    for(idx = 0; idx < n_encoder; idx++) {
        encoders[idx].mode = Encoder_Universal;
        encoders[idx].cpr = 400;
        encoders[idx].cpd = 4;
        encoders[idx].dbl_click_window = 500; // ms
    }

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&encoders, sizeof(encoders), true);
}

static void encoder_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&encoders, nvs_address, sizeof(encoders), true) != NVS_TransferResult_OK)
        encoder_settings_restore();
}

bool encoder_start (encoder_t *encoder)
{
    uint_fast8_t idx;
    bool has_mpg_encoder = false;

    if(n_encoder == 0)
        return false;

    override_encoder = NULL;

    for(idx = 0; idx < n_encoder; idx++) {

        encoder[idx].id = idx;
        encoder[idx].axis = 0xFF;
        encoder[idx].mode = encoders[idx].mode;
        encoder[idx].settings = &encoders[idx];

        switch(encoder[idx].settings->mode) {

            case Encoder_Universal:
                encoder[idx].mode = Encoder_FeedRate;
                override_encoder = &encoder[idx];
                break;

            case Encoder_MPG:
                {
                    uint_fast8_t i;
                    encoder[idx].axis = X_AXIS;
                    for(i = 0; i < N_AXIS; i++)
                        mpg[i].encoder = &encoder[idx];
                    has_mpg_encoder = true;
                }
                break;

            case Encoder_MPG_X:
                encoder[idx].axis   = X_AXIS;
                mpg[X_AXIS].encoder = &encoder[idx];
                has_mpg_encoder = true;
                break;

            case Encoder_MPG_Y:
                encoder[idx].axis   = Y_AXIS;
                mpg[Y_AXIS].encoder = &encoder[idx];
                has_mpg_encoder = true;
                break;

            case Encoder_MPG_Z:
                encoder[idx].axis   = Z_AXIS;
                mpg[Z_AXIS].encoder = &encoder[idx];
                has_mpg_encoder = true;
                break;
#if N_AXIS > 3
            case Encoder_MPG_A:
                encoder[idx].axis   = A_AXIS;
                mpg[A_AXIS].encoder = &encoder[idx];
                has_mpg_encoder = true;
                break;
#endif
#if N_AXIS > 4
            case Encoder_MPG_B:
                encoder[idx].axis   = B_AXIS;
                mpg[B_AXIS].encoder = &encoder[idx];
                has_mpg_encoder = true;
                break;
#endif
#if N_AXIS > 5
            case Encoder_MPG_C:
                encoder[idx].axis   = C_AXIS;
                mpg[C_AXIS].encoder = &encoder[idx];
                has_mpg_encoder = true;
                break;
#endif
            default:
                break;
        }

        hal.encoder.reset(idx);
    }

    for(idx = 0; idx < N_AXIS; idx++) {
        mpg[idx].scale_factor = 1.0f;
//        mpg[idx].handler = mpg_move_absolute;
        mpg[idx].handler = mpg_jog_relative;
    }

#if COMPATIBILITY_LEVEL <= 1
    if(override_encoder) {
        if(on_realtime_report == NULL) {
            on_realtime_report = grbl.on_realtime_report;
            grbl.on_realtime_report = encoder_rt_report;
        }
    } else if(on_realtime_report) {
        grbl.on_realtime_report = encoder_rt_report;
        on_realtime_report = grbl.on_realtime_report;
    }
#endif

    if(has_mpg_encoder) {
        if(!on_execute_realtime) {
            on_execute_realtime = grbl.on_execute_realtime;
            grbl.on_execute_realtime = encoder_execute_realtime;
        }
    } else if(on_execute_realtime) {
        grbl.on_execute_realtime = on_execute_realtime;
        on_execute_realtime = NULL;
    }

    return true;
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:ENCODER v0.01]" ASCII_EOL);
}

static uint8_t get_n_encoders (void)
{
    return QEI_ENABLE;
}

bool encoder_init (uint_fast8_t n_encoders)
{
    if((nvs_address = nvs_alloc(sizeof(encoder_settings_t) * n_encoders))) {
        n_encoder = n_encoders;

        hal.encoder.get_n_encoders = get_n_encoders;
        hal.encoder.on_event = encoder_event;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;

        details.on_get_settings = grbl.on_get_settings;
        grbl.on_get_settings = on_get_settings;
    }

    return nvs_address != 0;
}

#endif
