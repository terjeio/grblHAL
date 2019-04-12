/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2017-2019 Terje Io
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

#include "grbl.h"

// Set spindle speed override
// NOTE: Unlike motion overrides, spindle overrides do not require a planner reinitialization.
void spindle_set_override (uint_fast8_t speed_override)
{
    if(sys.override.control.spindle_rpm_disable)
        return;

    speed_override = max(min(speed_override, MAX_SPINDLE_RPM_OVERRIDE), MIN_SPINDLE_RPM_OVERRIDE);

    if ((uint8_t)speed_override != sys.override.spindle_rpm) {
        sys.step_control.update_spindle_rpm = On;
        sys.override.spindle_rpm = (uint8_t)speed_override;
        sys.report.override_counter = 0; // Set to report change immediately
    }
}

// Immediately sets spindle running state with direction and spindle rpm, if enabled.
// Called by g-code parser spindle_sync(), parking retract and restore, g-code program end,
// sleep, and spindle stop override.
bool spindle_set_state (spindle_state_t state, float rpm)
{
    if (!sys.abort) { // Block during abort.

        if (!state.on) { // Halt or set spindle direction and rpm.
            sys.spindle_rpm = 0.0f;
            spindle_stop();
        } else {
            // NOTE: Assumes all calls to this function is when Grbl is not moving or must remain off.
        	// TODO: alarm/interlock if going from CW to CCW directly in non-laser mode?
            if (settings.flags.laser_mode && state.ccw)
                rpm = 0.0f; // TODO: May need to be rpm_min*(100/MAX_SPINDLE_RPM_OVERRIDE);

            hal.spindle_set_state(state, spindle_set_rpm(rpm, sys.override.spindle_rpm));
        }
        sys.report.override_counter = -1; // Set to report change immediately
    }

    return !sys.abort;
}

// G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails
// if an abort or check-mode is active.
bool spindle_sync (spindle_state_t state, float rpm)
{
    bool at_speed = sys.state == STATE_CHECK_MODE || !hal.driver_cap.spindle_at_speed;

    if (sys.state != STATE_CHECK_MODE) {
        protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
        if(spindle_set_state(state, rpm) && !at_speed) {
            float delay = 0.0f;
            while(!sys.abort && !(at_speed = hal.spindle_get_state().at_speed)) {
                delay_sec(0.1f, DelayMode_Dwell);
                delay += 0.1f;
                if(delay > 3.0f) // TODO: add configurable timeout?
                    break;
            }
        }
    }

    return at_speed;
}

// Calculate and set programmed RPM according to override and max/min limits
float spindle_set_rpm (float rpm, uint8_t override_pct)
{
    if(override_pct != 100)
        rpm *= 0.01f * (float)override_pct; // Scale RPM by override value.

    // Apply RPM limits
    if (rpm <= 0.0f)
        rpm = 0.0f;
    else if (rpm > settings.spindle.rpm_max)
        rpm = settings.spindle.rpm_max;
    else if (rpm < settings.spindle.rpm_min)
        rpm = settings.spindle.rpm_min;

    sys.spindle_rpm = rpm;

    return rpm;
}

//
// The following functions are not called by the core, may be called by driver code.
//

// Precompute PWM values for faster conversion.
// Returns false if no PWM range possible, driver should revert to simple on/off spindle control if so.
bool spindle_precompute_pwm_values (spindle_pwm_t *pwm_data, uint32_t clock_hz)
{
    if(settings.spindle.rpm_max > settings.spindle.rpm_min) {
        pwm_data->period = (uint_fast16_t)((float)clock_hz / settings.spindle.pwm_freq);
        pwm_data->off_value = settings.spindle.invert.pwm ? pwm_data->period : 0;
        /* TODO: a use case for the calculation below is needed in order to reimplement it (and reenable related setting)
        pwm_data->off_value = (uint_fast16_t)(pwm_data->period * settings.spindle.pwm_off_value / 100.0f);
        */
        pwm_data->min_value = (uint_fast16_t)(pwm_data->period * settings.spindle.pwm_min_value / 100.0f);
        pwm_data->max_value = (uint_fast16_t)(pwm_data->period * settings.spindle.pwm_max_value / 100.0f) - 1;
        pwm_data->pwm_gradient = (float)(pwm_data->max_value - pwm_data->min_value) / (settings.spindle.rpm_max - settings.spindle.rpm_min);
    }

    return settings.spindle.rpm_max > settings.spindle.rpm_min;
}

// Spindle RPM to PWM conversion.
uint_fast16_t spindle_compute_pwm_value (spindle_pwm_t *pwm_data, float rpm, bool pid_limit)
{
    uint_fast16_t pwm_value;

#if SPINDLE_RPM_PIECES
    uint_fast8_t idx = pwm_data->n_pieces - 1;

    if(idx) {
        do {
            if(rpm > pwm_data->pieces[idx].rpm) {
                pwm_value = floorf(pwm_data->pieces[idx].start * rpm - pwm_data->pieces[idx].end);
                break;
            }
        } while(--idx);
    } else
#endif

    if(rpm > 0.0f) {
        // Compute intermediate PWM value with linear spindle speed model.
        pwm_value = (uint_fast16_t)floorf((rpm - settings.spindle.rpm_min) * pwm_data->pwm_gradient) + pwm_data->min_value;

        if(pwm_value >= (pid_limit ? pwm_data->period : pwm_data->max_value))
            pwm_value = pid_limit ? pwm_data->period - 1 : pwm_data->max_value;
        else if(pwm_value < pwm_data->min_value)
            pwm_value = pwm_data->min_value;

        pwm_value = settings.spindle.invert.pwm ? pwm_data->period - pwm_value - 1 : pwm_value;
    } else
        pwm_value = pwm_data->off_value;

    return pwm_value;
}
