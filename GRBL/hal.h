/*
  hal.h - HAL (Hardware Abstraction Layer) entry points structure and capabilities type

  Part of Grbl

  Copyright (c) 2016-2018 Terje Io

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

#ifndef __HAL__
#define __HAL__

#include <stdint.h>
#include <stdbool.h>

#include "gcode.h"
#include "system.h"
#include "coolant_control.h"
#include "spindle_control.h"
#include "stepper.h"
#include "eeprom.h"

#define HAL_VERSION 4

// driver capabilities, to be set by driver in driver_init(), flags may be cleared after to switch off option
typedef union {
    uint32_t value;
    struct {
        uint32_t mist_control            :1,
                 variable_spindle        :1,
                 safety_door             :1,
                 spindle_dir             :1,
                 software_debounce       :1,
                 step_pulse_delay        :1,
                 limits_pull_up          :1,
                 control_pull_up         :1,
                 probe_pull_up           :1,
                 amass_level             :2, // 0...3
                 program_stop            :1,
                 spindle_at_speed        :1,
                 laser_ppi_mode          :1,
                 constant_surface_speed  :1,
                 spindle_sync            :1,
                 sd_card                 :1,
                 bluetooth               :1,
                 ethernet                :1,
                 wifi                    :1,
                 unassigned              :12;
    };
} driver_cap_t;

typedef void (*stream_write_ptr)(const char *s);
typedef axes_signals_t (*limits_get_state_ptr)(void);

/* TODO: add to HAL so that a different formatting (xml, json etc) of reports may be implemented by driver? */
typedef struct {
    void (*report_status_message)(status_code_t status_code);
    void (*report_alarm_message)(alarm_code_t alarm_code);
    void (*report_feedback_message)(message_code_t message_code);
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
} HAL_report_t;

typedef struct HAL {
    uint32_t version;
    char *info;
    uint32_t f_step_timer;
    uint32_t rx_buffer_size;
    uint_fast16_t spindle_pwm_off;

    bool (*driver_setup)(settings_t *settings);

    void (*limits_enable)(bool on, bool homing);
    limits_get_state_ptr limits_get_state;
    void (*coolant_set_state)(coolant_state_t mode);
    coolant_state_t (*coolant_get_state)(void);
    void (*delay_ms)(uint32_t ms, void (*callback)(void));

    void (*spindle_set_state)(spindle_state_t state, float rpm, uint8_t rpm_override);
    spindle_state_t (*spindle_get_state)(void);
    uint_fast16_t (*spindle_set_speed)(uint_fast16_t pwm_value);
    uint_fast16_t (*spindle_compute_pwm_value)(float rpm, uint8_t rpm_override);
    control_signals_t (*system_control_get_state)(void);

    void (*stepper_wake_up)(void);
    void (*stepper_go_idle)(void);
    void (*stepper_enable)(axes_signals_t enable);
    void (*stepper_set_outputs)(axes_signals_t step_outbits);
    void (*stepper_set_directions)(axes_signals_t dir_outbits);
    void (*stepper_cycles_per_tick)(uint32_t cycles_per_tick);
    void (*stepper_pulse_start)(stepper_t *stepper);

    uint16_t (*stream_get_rx_buffer_available)(void);
//    bool (*stream_write)(char c);
    stream_write_ptr stream_write; // write to current IO stream only
    stream_write_ptr stream_write_all; // write to all ative output streams
    int16_t (*stream_read)(void);
    void (*stream_reset_read_buffer)(void);
    void (*stream_cancel_read_buffer)(void);
    bool (*stream_suspend_read)(bool await);

    void (*set_bits_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits);
    uint_fast16_t (*clear_bits_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits);
    uint_fast16_t (*set_value_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits);

    void (*settings_changed)(settings_t *settings);

    // optional entry points, may be unassigned (null)
    bool (*driver_release)(void);
    bool (*probe_get_state)(void);
    void (*probe_configure_invert_mask)(bool is_probe_away);
    void (*execute_realtime)(uint_fast16_t state);
    uint_fast16_t (*driver_mcode_check)(uint_fast16_t mcode);
    status_code_t (*driver_mcode_validate)(parser_block_t *gc_block, uint32_t *value_words);
    void (*driver_mcode_execute)(uint_fast16_t state, parser_block_t *gc_block);
    void (*driver_rt_command_execute)(uint8_t cmd);
    void (*driver_rt_report)(stream_write_ptr stream_write);
    void (*driver_feedback_message)(stream_write_ptr stream_write);
    status_code_t (*driver_sys_command_execute)(uint_fast16_t state, char *line, char *lcline); // return Status_Unhandled
    bool (*get_position)(int32_t (*position)[N_AXIS]);
    void (*tool_select)(tool_data_t *tool);
    void (*tool_change)(parser_state_t *gc_state);
    void (*show_message)(const char *msg);
    void (*driver_reset)(void);
    bool (*driver_setting)(uint_fast16_t setting, float value, char *svalue);
    void (*driver_settings_restore)(uint8_t restore_flag);
    void (*driver_settings_report)(bool axis_settings, axis_setting_type_t setting_type, uint8_t axis_idx);
    spindle_data_t (*spindle_get_data)(spindle_data_request_t request);
    void (*spindle_reset_data)(void);
#ifdef DEBUGOUT
    void (*debug_out)(bool on);
#endif
    eeprom_io_t eeprom;

    // entry points set by grbl at reset

    void (*report_status_message)(status_code_t status_code);

    // callbacks - set up by grbl before MCU init
    bool (*protocol_enqueue_gcode)(char *data);
    bool (*protocol_process_realtime)(char data);
    bool (*stream_blocking_callback)(void);
    void (*stepper_interrupt_callback)(void);
    void (*limit_interrupt_callback)(axes_signals_t state);
    void (*control_interrupt_callback)(control_signals_t signals);
    void (*spindle_index_callback)(spindle_data_t *rpm);

    driver_cap_t driver_cap;
} HAL;

extern HAL hal;
extern bool driver_init (void);

#endif
