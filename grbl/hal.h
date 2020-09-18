/*
  hal.h - HAL (Hardware Abstraction Layer) entry points structure and capabilities type

  Part of GrblHAL

  Copyright (c) 2016-2020 Terje Io

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

#ifndef _HAL_H_
#define _HAL_H_

#include "grbl.h"
#include "gcode.h"
#include "system.h"
#include "coolant_control.h"
#include "spindle_control.h"
#include "stepper.h"
#include "nvs.h"
#include "stream.h"
#include "probe.h"
#include "plugins.h"
#include "settings.h"

#define HAL_VERSION 6

// driver capabilities, to be set by driver in driver_init(), flags may be cleared after to switch off option
typedef union {
    uint32_t value;
    struct {
        uint32_t mist_control              :1,
                 variable_spindle          :1,
                 safety_door               :1,
                 spindle_dir               :1,
                 software_debounce         :1,
                 step_pulse_delay          :1,
                 limits_pull_up            :1,
                 control_pull_up           :1,
                 probe_pull_up             :1,
                 amass_level               :2, // 0...3
                 program_stop              :1,
                 block_delete              :1,
                 e_stop                    :1,
                 spindle_at_speed          :1,
                 laser_ppi_mode            :1,
                 spindle_sync              :1,
                 sd_card                   :1,
                 bluetooth                 :1,
                 ethernet                  :1,
                 wifi                      :1,
                 spindle_pwm_invert        :1,
                 spindle_pid               :1,
                 axis_ganged_x             :1,
                 axis_ganged_y             :1,
                 axis_ganged_z             :1,
                 mpg_mode                  :1,
                 spindle_pwm_linearization :1,
                 probe_connected           :1,
                 atc                       :1,
                 unassigned                :2;
    };
} driver_cap_t;

typedef void (*stream_write_ptr)(const char *s);
typedef axes_signals_t (*limits_get_state_ptr)(void);
typedef void (*driver_reset_ptr)(void);

typedef struct {
    stream_type_t type;
    uint16_t (*get_rx_buffer_available)(void);
//    bool (*stream_write)(char c);
    stream_write_ptr write; // write to current I/O stream only
    stream_write_ptr write_all; // write to all active output streams
    int16_t (*read)(void);
    void (*reset_read_buffer)(void);
    void (*cancel_read_buffer)(void);
    bool (*suspend_read)(bool await);
    bool (*enqueue_realtime_command)(char data); // NOTE: set by grbl at startup
} io_stream_t;

typedef struct {
    uint8_t num_digital_in;
    uint8_t num_digital_out;
    uint8_t num_analog_in;
    uint8_t num_analog_out;
    void (*digital_out)(uint8_t port, bool on);
    bool (*analog_out)(uint8_t port, float value);
    int32_t (*wait_on_input)(bool digital, uint8_t port, wait_mode_t wait_mode, float timeout);
} io_port_t;

typedef struct {
    uint32_t version;
    char *info;
    char *driver_version;
    char *driver_options;
    char *board;
    uint32_t f_step_timer;
    uint32_t rx_buffer_size;

    bool (*driver_setup)(settings_t *settings);

    void (*limits_enable)(bool on, bool homing);
    limits_get_state_ptr limits_get_state;
    void (*coolant_set_state)(coolant_state_t mode);
    coolant_state_t (*coolant_get_state)(void);
    void (*delay_ms)(uint32_t ms, void (*callback)(void));

    void (*spindle_set_state)(spindle_state_t state, float rpm);
    spindle_state_t (*spindle_get_state)(void);
#ifdef SPINDLE_PWM_DIRECT
    uint_fast16_t (*spindle_get_pwm)(float rpm);
    void (*spindle_update_pwm)(uint_fast16_t pwm);
#else
    void (*spindle_update_rpm)(float rpm);
#endif
    control_signals_t (*system_control_get_state)(void);

    void (*stepper_wake_up)(void);
    void (*stepper_go_idle)(bool clear_signals);
    void (*stepper_enable)(axes_signals_t enable);
    void (*stepper_disable_motors)(axes_signals_t axes, squaring_mode_t mode);
    void (*stepper_cycles_per_tick)(uint32_t cycles_per_tick);
    void (*stepper_pulse_start)(stepper_t *stepper);

    io_stream_t stream; // pointers to current I/O stream handlers

    void (*set_bits_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits);
    uint_fast16_t (*clear_bits_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits);
    uint_fast16_t (*set_value_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits);

    void (*irq_enable)(void);
    void (*irq_disable)(void);

    void (*settings_changed)(settings_t *settings);

    // optional entry points, may be unassigned (null)
    bool (*driver_release)(void);
    probe_state_t (*probe_get_state)(void);
    void (*probe_configure_invert_mask)(bool is_probe_away, bool probing);
    user_mcode_t (*user_mcode_check)(user_mcode_t mcode);
    status_code_t (*user_mcode_validate)(parser_block_t *gc_block, uint32_t *value_words);
    void (*user_mcode_execute)(uint_fast16_t state, parser_block_t *gc_block);
    bool (*get_position)(int32_t (*position)[N_AXIS]);
    void (*tool_select)(tool_data_t *tool, bool next);
    status_code_t (*tool_change)(parser_state_t *gc_state);
    void (*show_message)(const char *msg);
    driver_reset_ptr driver_reset;
    status_code_t (*driver_setting)(setting_type_t setting, float value, char *svalue);
    void (*driver_settings_restore)(void);
    void (*driver_settings_report)(setting_type_t setting_type);
    void (*driver_axis_settings_report)(axis_setting_type_t setting_type, uint8_t axis_idx);
    spindle_data_t (*spindle_get_data)(spindle_data_request_t request);
    void (*spindle_reset_data)(void);

    void (*encoder_event_handler)(encoder_t *encoder, int32_t position);
    void (*encoder_reset)(uint_fast8_t id);
    uint32_t (*get_elapsed_ticks)(void);
    void (*pallet_shuttle)(void);
    void (*stepper_output_step)(axes_signals_t step_outbits, axes_signals_t dir_outbits);
    void (*reboot)(void);
#ifdef DEBUGOUT
    void (*debug_out)(bool on);
#endif

    nvs_io_t nvs;

    io_port_t port;

    // callbacks - up by core before driver_init() is called
    bool (*stream_blocking_callback)(void);
    void (*stepper_interrupt_callback)(void);
    void (*limit_interrupt_callback)(axes_signals_t state);
    void (*control_interrupt_callback)(control_signals_t signals);
//    void (*spindle_index_callback)(spindle_data_t *rpm);
    // driver capabilities flags
    driver_cap_t driver_cap;
} grbl_hal_t;

// TODO: move the following structs to grbl.h?

/* TODO: add to grbl pointers so that a different formatting (xml, json etc) of reports may be implemented by driver?
typedef struct {
    status_code_t (*report_status_message)(status_code_t status_code);
    alarm_code_t (*report_alarm_message)(alarm_code_t alarm_code);
    message_code_t (*report_feedback_message)(message_code_t message_code);
    void (*report_init_message)(void);
    void (*report_grbl_help)(void);
    void (*report_grbl_settings)(void);
    void (*report_echo_line_received)(char *line);
    void (*report_realtime_status)(void);
    void (*report_probe_parameters)(void);
    void (*report_ngc_parameters)(void);
    void (*report_gcode_modes)(void);
    void (*report_startup_line)(uint8_t n, char *line);
    void (*report_execute_startup_message)(char *line, status_code_t status_code);
} grbl_report_t;
*/

typedef struct {
    status_code_t (*status_message)(status_code_t status_code);
    message_code_t (*feedback_message)(message_code_t message_code);
} report_t;

typedef void (*on_execute_realtime_ptr)(uint_fast16_t state);

typedef struct {
    // report entry points set by core at reset
    report_t report;
    // grbl core events - may be subscribed to by drivers or by the core
    void (*on_state_change)(uint_fast16_t state);
    void (*on_probe_completed)(void);
    on_execute_realtime_ptr on_execute_realtime;
    void (*on_unknown_accessory_override)(uint8_t cmd);
    void (*on_report_options)(void);
    void (*on_realtime_report)(stream_write_ptr stream_write, report_tracking_flags_t report);
    void (*on_unknown_feedback_message)(stream_write_ptr stream_write);
    status_code_t (*on_unknown_sys_command)(uint_fast16_t state, char *line, char *lcline); // return Status_Unhandled
    status_code_t (*on_user_command)(char *line);
    // core entry points - set up by core before driver_init() is called
    bool (*protocol_enqueue_gcode)(char *data);
} grbl_t;

extern grbl_t grbl;
extern grbl_hal_t hal;
extern bool driver_init (void);

#endif
