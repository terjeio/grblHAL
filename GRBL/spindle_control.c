/*
  spindle_control.c - spindle control methods
  Part of Grbl

  Copyright (c) 2017-2018 Terje Io
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
void spindle_set_override (uint_fast8_t speed_ovr)
{
    if(sys.override_ctrl.spindle_rpm_disable)
        return;

    speed_ovr = max(min(speed_ovr, MAX_SPINDLE_RPM_OVERRIDE), MIN_SPINDLE_RPM_OVERRIDE);

    if ((uint8_t)speed_ovr != sys.spindle_rpm_ovr) {
        sys.step_control.update_spindle_rpm = On;
        sys.spindle_rpm_ovr = (uint8_t)speed_ovr;
        sys.report.ovr_counter = 0; // Set to report change immediately
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

            hal.spindle_set_state(state, rpm, sys.spindle_rpm_ovr);
        }
        sys.report.ovr_counter = -1; // Set to report change immediately
    }

    return !sys.abort;
}

// G-code parser entry-point for setting spindle state. Forces a planner buffer sync and bails
// if an abort or check-mode is active.
void spindle_sync (spindle_state_t state, float rpm)
{
    if (sys.state != STATE_CHECK_MODE) {
        protocol_buffer_synchronize(); // Empty planner buffer to ensure spindle is set when programmed.
        if(spindle_set_state(state, rpm) && hal.driver_cap.spindle_at_speed) {
            while(!hal.spindle_get_state().at_speed)
                delay_sec(0.1f, DelayMode_Dwell);
        }
    }
}

//
// The following functions are not called by the core, may be called by driver code.
//

// Precomute PWM values for faster conversion.
void spindle_precompute_pwm_values (spindle_pwm_t *pwm_data, uint32_t clock_hz)
{
    pwm_data->period = (uint32_t)((float)clock_hz / settings.spindle.pwm_freq);
    pwm_data->off_value = (uint32_t)(pwm_data->period * settings.spindle.pwm_off_value / 100.0f);
    pwm_data->min_value = (uint32_t)(pwm_data->period * settings.spindle.pwm_min_value / 100.0f);
    pwm_data->max_value = (uint32_t)(pwm_data->period * settings.spindle.pwm_max_value / 100.0f);
    pwm_data->pwm_gradient = (float)(pwm_data->max_value - pwm_data->min_value) / (settings.spindle.rpm_max - settings.spindle.rpm_min);
}

// Spindle speed to PWM conversion.
uint_fast16_t spindle_compute_pwm_value (spindle_pwm_t *pwm_data, float rpm, uint8_t speed_ovr)
{
    uint_fast16_t pwm_value;

    rpm *= 0.010f * speed_ovr; // Scale by spindle speed override value.

    // Calculate PWM register value based on rpm max/min settings and programmed rpm.
    if ((settings.spindle.rpm_min >= settings.spindle.rpm_max) || (rpm >= settings.spindle.rpm_max)) {
        // No PWM range possible. Set simple on/off spindle control pin state.
        sys.spindle_rpm = settings.spindle.rpm_max;
        pwm_value = pwm_data->max_value - 1;
    } else if (rpm <= settings.spindle.rpm_min) {
        if (rpm == 0.0f) { // S0 disables spindle
            sys.spindle_rpm = 0.0f;
            pwm_value = pwm_data->off_value;
        } else { // Set minimum PWM output
            sys.spindle_rpm = settings.spindle.rpm_min;
            pwm_value = pwm_data->min_value;
        }
    } else {
        // Compute intermediate PWM value with linear spindle speed model.
        sys.spindle_rpm = rpm;
        pwm_value = (uint_fast16_t)floorf((rpm - settings.spindle.rpm_min) * pwm_data->pwm_gradient) + pwm_data->min_value;
        if(pwm_value >= pwm_data->max_value)
            pwm_value = pwm_data->max_value - 1;
    }

    return pwm_value;
}
