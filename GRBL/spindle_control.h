/*
  spindle_control.h - spindle control methods
  Part of Grbl

  Copyright (c) 2012-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#ifndef spindle_control_h
#define spindle_control_h

typedef union {
    uint8_t value;
    uint8_t mask;
    struct {
        uint8_t on           :1,
                ccw          :1,
                at_speed     :1,
                synchronized :1,
                reserved4    :1,
                reserved5    :1,
                reserved6    :1,
                reserved7    :1;
    };
} spindle_state_t;

// Precalculated values that may be set/used by HAL driver to speed up RPM to PWM conversions if variable spindle is supported
typedef struct {
    uint_fast16_t period;
    uint_fast16_t off_value;
    uint_fast16_t min_value;
    uint_fast16_t max_value;
    float pwm_gradient;
} spindle_pwm_t;

// Used when HAL driver supports spindle synchronization
typedef struct {
    volatile uint32_t index_count;
    volatile uint32_t pulse_count;
    float rpm;
    float rpm_programmed;
    float rpm_low_limit;
    float rpm_high_limit;
    float angular_position; // Number of revolutions since last reset
} spindle_data_t;

typedef enum {
    SpindleData_Counters,
    SpindleData_RPM,
    SpindleData_AngularPosition
} spindle_data_request_t;

// Disables the spindle and sets spindle RPM to zero when variable spindle speed is enabled.
// Called by various main program and ISR routines. Keep routine small, fast, and efficient.
// Called by spindle_init(), spindle_set_speed(), spindle_set_state(), and mc_reset().
#define spindle_stop() hal.spindle_set_state((spindle_state_t){0}, 0.0f, 0)

void spindle_set_override (uint_fast8_t speed_ovr);

// Called by g-code parser when setting spindle state and requires a buffer sync.
// Immediately sets spindle running state with direction and spindle RPM, if enabled.
// Called by spindle_sync() after sync and parking motion/spindle stop override during restore.

// Called by g-code parser when setting spindle state and requires a buffer sync.
void spindle_sync (spindle_state_t state, float rpm);

// Sets spindle running state with direction, enable, and spindle RPM.
bool spindle_set_state (spindle_state_t state, float rpm);

#endif
