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
                 unassigned              :16;
    };
} driver_cap_t;

typedef struct HAL {
    uint32_t version;
    char *info;
    uint32_t f_step_timer;
    uint32_t rx_buffer_size;
    uint_fast16_t spindle_pwm_off;

    bool (*driver_setup)(settings_t *settings);

    void (*limits_enable)(bool on);
    axes_signals_t (*limits_get_state)(void);
    void (*coolant_set_state)(coolant_state_t mode);
    coolant_state_t (*coolant_get_state)(void);
    void (*delay_ms)(uint32_t ms, void (*callback)(void));

    bool (*probe_get_state)(void);
    void (*probe_configure_invert_mask)(bool is_probe_away);

    void (*spindle_set_state)(spindle_state_t state, float rpm, uint8_t spindle_rpm_ovr);
    spindle_state_t (*spindle_get_state)(void);
    uint_fast16_t (*spindle_set_speed)(uint_fast16_t pwm_value);
    uint_fast16_t (*spindle_compute_pwm_value)(float rpm, uint8_t spindle_rpm_ovr);
    control_signals_t (*system_control_get_state)(void);

    void (*stepper_wake_up)(void);
    void (*stepper_go_idle)(void);
    void (*stepper_enable)(axes_signals_t enable);
    void (*stepper_set_outputs)(axes_signals_t step_outbits);
    void (*stepper_set_directions)(axes_signals_t dir_outbits);
    void (*stepper_cycles_per_tick)(uint32_t cycles_per_tick);
    void (*stepper_pulse_start)(stepper_t *stepper);

    uint16_t (*serial_get_rx_buffer_available)(void);
    bool (*serial_write)(char c);
    void (*serial_write_string)(const char *s);
    int16_t (*serial_read)(void);
    void (*serial_reset_read_buffer)(void);
    void (*serial_cancel_read_buffer)(void);

    void (*set_bits_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits);
    uint_fast16_t (*clear_bits_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits);
    uint_fast16_t (*set_value_atomic)(volatile uint_fast16_t *value, uint_fast16_t bits);

    void (*settings_changed)(settings_t *settings);

    // optional entry points, may be unassigned (null)
    bool (*driver_release)(void);
    void (*execute_realtime)(uint8_t state);
    uint8_t (*userdefined_mcode_check)(uint8_t mcode);
    status_code_t (*userdefined_mcode_validate)(parser_block_t *gc_block, uint32_t *value_words);
    void (*userdefined_mcode_execute)(uint_fast16_t state, parser_block_t *gc_block);
    void (*userdefined_rt_command_execute)(uint8_t cmd);
    void (*userdefined_rt_report)(void);
    status_code_t (*userdefined_sys_command_execute)(uint_fast16_t state, char *line); // return Status_Unhandled
    bool (*get_position)(int32_t (*position)[N_AXIS]);
    void (*tool_select)(tool_data_t *tool);
    void (*tool_change)(parser_state_t *gc_state);
    void (*show_message)(const char *msg);
    bool (*driver_setting)(uint_fast16_t setting, float value);
    void (*driver_settings_restore)(uint8_t restore_flag);
    void (*driver_settings_report)(bool axis_settings);
    spindle_data_t (*spindle_get_data)(spindle_data_request_t request);
    void (*spindle_reset_data)(void);

    eeprom_io_t eeprom;

    // callbacks - set up by grbl before MCU init
    bool (*protocol_enqueue_gcode)(char *data);
    bool (*protocol_process_realtime)(char data);
    bool (*serial_blocking_callback)(void);
    void (*stepper_interrupt_callback)(void);
    void (*limit_interrupt_callback)(axes_signals_t state);
    void (*control_interrupt_callback)(control_signals_t signals);
    void (*spindle_index_callback)(spindle_data_t *rpm);

    driver_cap_t driver_cap;
} HAL;

extern HAL hal;
extern bool driver_init (void);

#endif
