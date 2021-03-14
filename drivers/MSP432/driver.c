/*

  driver.c - driver code for Texas Instruments MSP432P401R ARM processor

  Part of grblHAL

  Copyright (c) 2017-2021 Terje Io

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
#include <stdio.h>
#include <string.h>

#include "driver.h"
#include "serial.h"

#include "grbl/limits.h"
#include "grbl/spindle_sync.h"

#if I2C_ENABLE
#include "i2c.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#if KEYPAD_ENABLE
#include "keypad/keypad.h"
#endif

#if PLASMA_ENABLE
#include "plasma/thc.h"
#endif

#if ATC_ENABLE
#include "atc.h"
#endif

#if ODOMETER_ENABLE
#include "odometer/odometer.h"
#endif

static volatile bool spindleLock = false;
static bool IOInitDone = false;
// Inverts the probe pin state depending on user settings and probing cycle mode.
static uint16_t pulse_length;
static volatile uint32_t elapsed_tics = 0;
static axes_signals_t next_step_outbits;
static spindle_data_t spindle_data;
static spindle_encoder_t spindle_encoder = {
    .tics_per_irq = 4
};
static spindle_sync_t spindle_tracker;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup

#ifndef VFD_SPINDLE
static bool pwmEnabled = false;
static spindle_pwm_t spindle_pwm;

#ifdef SPINDLE_RPM_CONTROLLED

typedef enum {
    PIDState_Disabled = 0,
    PIDState_Pending,
    PIDState_Active,
} pid_state_t;

// PID data for closed loop spindle RPM control
typedef struct {
    pid_state_t pid_state;
    pidf_t pid;
    bool pid_enabled;
    float rpm;
} spindle_control_t;

static volatile uint32_t pid_count = 0;
static spindle_control_t spindle_control = { .pid_state = PIDState_Disabled, .pid = {0}};

#endif

static void spindle_set_speed (uint_fast16_t pwm_value);
#else
#undef SPINDLE_RPM_CONTROLLED
#endif

#if MODBUS_ENABLE
static modbus_stream_t modbus_stream = {0};
#endif

static probe_state_t probe = {
    .connected = On
};

static probeflags_t psettings =
{
    0 //.enable_protection = On
};

#if STEP_OUTMODE == GPIO_MAP

    static const uint8_t c_step_outmap[8] = {
        0,
        X_STEP_BIT,
        Y_STEP_BIT,
        X_STEP_BIT|Y_STEP_BIT,
        Z_STEP_BIT,
        X_STEP_BIT|Z_STEP_BIT,
        Y_STEP_BIT|Z_STEP_BIT,
        X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT
    };

    static uint8_t step_outmap[8];

#endif

#if DIRECTION_OUTMODE == GPIO_MAP

    static const uint8_t c_dir_outmap[8] = {
        0,
        X_DIRECTION_BIT,
        Y_DIRECTION_BIT,
        X_DIRECTION_BIT|Y_DIRECTION_BIT,
        Z_DIRECTION_BIT,
        X_DIRECTION_BIT|Z_DIRECTION_BIT,
        Y_DIRECTION_BIT|Z_DIRECTION_BIT,
        X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT
    };

    static uint8_t dir_outmap[8];

#endif

static void stepperPulseStartSynchronized (stepper_t *stepper);
static void spindleDataReset (void);
static spindle_data_t *spindleGetData (spindle_data_request_t request);

// LinuxCNC example settings
// MAX_OUTPUT = 300 DEADBAND = 0.0 P = 3 I = 1.0 D = 0.1 FF0 = 0.0 FF1 = 0.1 FF2 = 0.0 BIAS = 0.0 MAXI = 20.0 MAXD = 20.0 MAXERROR = 250.0
//
// You will always get oscillation on a PID system if you increase any P,I,D term too high
// I would try using less P (say 2) and then see how high an I term you can have and stay stable
// D term should not be needed

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
//    while(delay.callback);

    if((delay.ms = ms) > 0) {
        if(!(delay.callback = callback))
            while(delay.ms);
    } else if(callback)
        callback();
}

// Set stepper pulse output pins
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z...
// Mapping to registers can be done by
// 1. bitbanding. Pros: can assign pins to different ports, no RMW needed. Cons: overhead, pin changes not synchronous
// 2. bit shift. Pros: fast, Cons: bits must be consecutive
// 3. lookup table. Pros: signal inversions done at setup, Cons: slower than bit shift
inline __attribute__((always_inline)) static void set_step_outputs (axes_signals_t step_outbits)
{
#if STEP_OUTMODE == GPIO_BITBAND
    step_outbits.value ^= settings.steppers.step_invert.mask;
    BITBAND_PERI(STEP_PORT->OUT, X_STEP_PIN) = step_outbits.x;
    BITBAND_PERI(STEP_PORT->OUT, Y_STEP_PIN) = step_outbits.y;
    BITBAND_PERI(STEP_PORT->OUT, Z_STEP_PIN) = step_outbits.z;
#elif STEP_OUTMODE == GPIO_MAP
    STEP_PORT->OUT = (STEP_PORT->OUT & ~STEP_MASK) | step_outmap[step_outbits.value];
#else
    STEP_PORT->OUT = (STEP_PORT->OUT & ~STEP_MASK) | ((step_outbits.value << STEP_OUTMODE) ^ settings.steppers.step_invert.mask);
#endif
}

// Set stepper direction output pins
// NOTE: see note for set_step_outputs()
inline __attribute__((always_inline)) static void set_dir_outputs (axes_signals_t dir_outbits)
{
#if DIRECTION_OUTMODE == GPIO_BITBAND
    dir_outbits.value ^= settings.steppers.dir_invert.mask;
    BITBAND_PERI(DIRECTION_PORT->OUT, X_DIRECTION_PIN) = dir_outbits.x;
    BITBAND_PERI(DIRECTION_PORT->OUT, Y_DIRECTION_PIN) = dir_outbits.y;
    BITBAND_PERI(DIRECTION_PORT->OUT, Z_DIRECTION_PIN) = dir_outbits.z;
#elif DIRECTION_OUTMODE == GPIO_MAP
    DIRECTION_PORT->OUT = (DIRECTION_PORT->OUT & ~DIRECTION_MASK) | dir_outmap[dir_outbits.value];
#else
    DIRECTION_PORT->OUT = (DIRECTION_PORT->OUT & ~DIRECTION_MASK) | ((dir_outbits.value << DIRECTION_OUTMODE) ^ settings.steppers.dir_invert.mask);
#endif
}

// Enable/disable stepper motors
static void stepperEnable (axes_signals_t enable)
{
    enable.value ^= settings.steppers.enable_invert.mask;
#if TRINAMIC_ENABLE && TRINAMIC_I2C
    axes_signals_t tmc_enable = trinamic_stepper_enable(enable);
  #if !CNC_BOOSTERPACK // Trinamic BoosterPack does not support mixed drivers
    if(!tmc_enable.z)
        BITBAND_PERI(STEPPERS_DISABLE_Z_PORT->OUT, STEPPERS_DISABLE_Z_PIN) = enable.z;
    if(!tmc_enable.x)
        BITBAND_PERI(STEPPERS_DISABLE_XY_PORT->OUT, STEPPERS_DISABLE_X_PIN) = enable.x;
  #endif
#else
    BITBAND_PERI(STEPPERS_DISABLE_Z_PORT->OUT, STEPPERS_DISABLE_Z_PIN) = enable.z;
    BITBAND_PERI(STEPPERS_DISABLE_XY_PORT->OUT, STEPPERS_DISABLE_X_PIN) = enable.x;
#endif
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    stepperEnable((axes_signals_t){AXES_BITMASK});
    STEPPER_TIMER->LOAD = 0x000FFFFFUL;
    STEPPER_TIMER->CONTROL |= TIMER32_CONTROL_ENABLE|TIMER32_CONTROL_IE;
    spindle_tracker.segment_id = 0;
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    STEPPER_TIMER->CONTROL &= ~(TIMER32_CONTROL_ENABLE|TIMER32_CONTROL_IE);
    STEPPER_TIMER->INTCLR = 0;
    if(clear_signals) {
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}
static void stepperPulseStartDelayed (stepper_t *stepper);
// Sets up stepper driver interrupt timeout, limiting the slowest speed
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    STEPPER_TIMER->LOAD = cycles_per_tick < (1UL << 20) ? cycles_per_tick : 0x000FFFFFUL;
}

// "Normal" version: Sets stepper direction and pulse pins and starts a step pulse a few nanoseconds later.
// If spindle synchronized motion switch to PID version.
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->new_block) {

        if(stepper->exec_segment->spindle_sync) {
            spindle_tracker.stepper_pulse_start_normal = hal.stepper.pulse_start;
            hal.stepper.pulse_start = stepperPulseStartSynchronized;
            hal.stepper.pulse_start(stepper);
            return;
        }

        if(stepper->dir_change)
            set_dir_outputs(stepper->dir_outbits);
    }

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;
    }
}

// Delayed pulse version: sets stepper direction and pulse pins and starts a step pulse with an initial delay.
// If spindle synchronized motion switch to PID version.
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->new_block) {

        if(stepper->exec_segment->spindle_sync) {
            spindle_tracker.stepper_pulse_start_normal = hal.stepper.pulse_start;
            hal.stepper.pulse_start = stepperPulseStartSynchronized;
            hal.stepper.pulse_start(stepper);
            return;
        }

        if(stepper->dir_change) {

            set_dir_outputs(stepper->dir_outbits);

            if(stepper->step_outbits.value) {
                next_step_outbits = stepper->step_outbits;              // Store out_bits
                PULSE_TIMER->CCR[0] = 0;
                PULSE_TIMER->CCTL[1] &= ~TIMER_A_CCTLN_CCIFG;           // Clear and
                PULSE_TIMER->CCTL[1] |= TIMER_A_CCTLN_CCIE;             // enable CCR1 interrupt
                PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;

            }
            return;
        }
    }

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;
    }
}

// Spindle sync version: sets stepper direction and pulse pins and starts a step pulse.
// Switches back to "normal" version if spindle synchronized motion is finished.
// TODO: add delayed pulse handling...
static void stepperPulseStartSynchronized (stepper_t *stepper)
{
    static bool sync = false;
    static float block_start;

    if(stepper->new_block) {
        if(!stepper->exec_segment->spindle_sync) {
            hal.stepper.pulse_start = spindle_tracker.stepper_pulse_start_normal;
            hal.stepper.pulse_start(stepper);
            return;
        }
        sync = true;
        set_dir_outputs(stepper->dir_outbits);
        spindle_tracker.programmed_rate = stepper->exec_block->programmed_rate;
        spindle_tracker.steps_per_mm = stepper->exec_block->steps_per_mm;
        spindle_tracker.segment_id = 0;
        spindle_tracker.prev_pos = 0.0f;
        block_start = spindleGetData(SpindleData_AngularPosition)->angular_position * spindle_tracker.programmed_rate;
        pidf_reset(&spindle_tracker.pid);
#ifdef PID_LOG
        sys.pid_log.idx = 0;
        sys.pid_log.setpoint = 100.0f;
#endif
    }

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;
    }

    if(spindle_tracker.segment_id != stepper->exec_segment->id) {

        spindle_tracker.segment_id = stepper->exec_segment->id;

        if(!stepper->new_block) {  // adjust this segments total time for any positional error since last segment

            float actual_pos;

            if(stepper->exec_segment->cruising) {

                float dt = (float)hal.f_step_timer / (float)(stepper->exec_segment->cycles_per_tick * stepper->exec_segment->n_step);
                actual_pos = spindleGetData(SpindleData_AngularPosition)->angular_position * spindle_tracker.programmed_rate;

                if(sync) {
                    spindle_tracker.pid.sample_rate_prev = dt;
//                    block_start += (actual_pos - spindle_tracker.block_start) - spindle_tracker.prev_pos;
//                    block_start += spindle_tracker.prev_pos;
                    sync = false;
                }

                actual_pos -= block_start;
                int32_t step_delta = (int32_t)(pidf(&spindle_tracker.pid, spindle_tracker.prev_pos, actual_pos, dt) * spindle_tracker.steps_per_mm);


                int32_t ticks = (((int32_t)stepper->step_count + step_delta) * (int32_t)stepper->exec_segment->cycles_per_tick) / (int32_t)stepper->step_count;

                stepper->exec_segment->cycles_per_tick = (uint32_t)max(ticks, (int32_t)spindle_tracker.min_cycles_per_tick);

                stepperCyclesPerTick(stepper->exec_segment->cycles_per_tick);
           } else
               actual_pos = spindle_tracker.prev_pos;

#ifdef PID_LOG
            if(sys.pid_log.idx < PID_LOG) {

                sys.pid_log.target[sys.pid_log.idx] = spindle_tracker.prev_pos;
                sys.pid_log.actual[sys.pid_log.idx] = actual_pos; // - spindle_tracker.prev_pos;

                spindle_tracker.log[sys.pid_log.idx] = STEPPER_TIMER->BGLOAD << stepper->amass_level;
            //    spindle_tracker.pos[sys.pid_log.idx] = stepper->exec_segment->cycles_per_tick  stepper->amass_level;
                spindle_tracker.pos[sys.pid_log.idx] = stepper->exec_segment->cycles_per_tick * stepper->step_count;
                STEPPER_TIMER->BGLOAD = STEPPER_TIMER->LOAD;

             //   spindle_tracker.pos[sys.pid_log.idx] = spindle_tracker.prev_pos;

                sys.pid_log.idx++;
            }
#endif
        }

        spindle_tracker.prev_pos = stepper->exec_segment->target_position;
    }
}

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing)
{
    on = on && settings.limits.flags.hard_enabled;
#if CNC_BOOSTERPACK_SHORTS
  #if STEP_OUTMODE == GPIO_BITBAND
    BITBAND_PERI(LIMIT_PORT->IFG, X_LIMIT_PIN) = 0;
    BITBAND_PERI(LIMIT_PORT->IFG, Y_LIMIT_PIN) = 0;
    BITBAND_PERI(LIMIT_PORT->IFG, Z_LIMIT_PIN) = 0;
    BITBAND_PERI(LIMIT_PORT->IE, X_LIMIT_PIN) = on;
    BITBAND_PERI(LIMIT_PORT->IE, Y_LIMIT_PIN) = on;
    BITBAND_PERI(LIMIT_PORT->IE, Z_LIMIT_PIN) = on;
  #else
    LIMIT_PORT->IFG &= ~LIMIT_MASK;
    if(on)
        LIMIT_PORT->IE  |= LIMIT_MASK;
    else
        LIMIT_PORT->IE &= ~LIMIT_MASK;
  #endif
#else
    BITBAND_PERI(LIMIT_PORT_X->IFG, X_LIMIT_PIN) = 0;
    BITBAND_PERI(LIMIT_PORT_Y->IFG, Y_LIMIT_PIN) = 0;
    BITBAND_PERI(LIMIT_PORT_Z->IFG, Z_LIMIT_PIN) = 0;
    BITBAND_PERI(LIMIT_PORT_X->IE, X_LIMIT_PIN) = on;
    BITBAND_PERI(LIMIT_PORT_Y->IE, Y_LIMIT_PIN) = on;
    BITBAND_PERI(LIMIT_PORT_Z->IE, Z_LIMIT_PIN) = on;
#endif

#if TRINAMIC_ENABLE
    trinamic_homing(homing);
#endif
}


// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};
#if CNC_BOOSTERPACK_SHORTS
 #if defined(LIMIT_INMODE) &&LIMIT_INMODE == LIMIT_SHIFT
    signals.min.value = (uint8_t)(LIMIT_PORT->IN & LIMIT_MASK) >> LIMIT_SHIFT;
 #elif LIMIT_INMODE == GPIO_BITBAND
    signals.min.x = BITBAND_PERI(LIMIT_PORT, X_LIMIT_PIN);
    signals.min.y = BITBAND_PERI(LIMIT_PORT, Y_LIMIT_PIN);
    signals.min.z = BITBAND_PERI(LIMIT_PORT, Z_LIMIT_PIN);
 #else // masked
    uint8_t bits = LIMIT_PORT->IN;
    signals.min.x = (bits & X_LIMIT_BIT) != 0;
    signals.min.y = (bits & Y_LIMIT_BIT) != 0;
    signals.min.z = (bits & Z_LIMIT_BIT) != 0;
 #endif
#else
 #if LIMIT_INMODE == LIMIT_SHIFT
    signals.min.value = (uint8_t)(LIMIT_PORT->IN & LIMIT_MASK) >> LIMIT_SHIFT;
 #elif LIMIT_INMODE == GPIO_BITBAND
    signals.min.x = BITBAND_PERI(LIMIT_PORT_X->IN, X_LIMIT_PIN);
    signals.min.y = BITBAND_PERI(LIMIT_PORT_Y->IN, Y_LIMIT_PIN);
    signals.min.z = BITBAND_PERI(LIMIT_PORT_Z->IN, Z_LIMIT_PIN);
 #else
    uint8_t bits = LIMIT_PORT->IN;
    signals.min.x = (bits & X_LIMIT_PIN) != 0;
    signals.min.y = (bits & Y_LIMIT_PIN) != 0;
    signals.min.z = (bits & Z_LIMIT_PIN) != 0;
 #endif
#endif

    if (settings.limits.invert.mask)
        signals.min.value ^= settings.limits.invert.mask;

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.value = settings.control_invert.mask;

#if CNC_BOOSTERPACK_SHORTS
  #if CONTROL_INMODE == GPIO_BITBAND
   #if ESTOP_ENABLE
    signals.e_stop = BITBAND_PERI(CONTROL_PORT->IN, RESET_PIN);
   #else
    signals.reset = BITBAND_PERI(CONTROL_PORT->IN, RESET_PIN);
   #endif
    signals.safety_door_ajar = BITBAND_PERI(CONTROL_PORT->IN, SAFETY_DOOR_PIN);
    signals.feed_hold = BITBAND_PERI(CONTROL_PORT->IN, FEED_HOLD_PIN);
    signals.cycle_start = BITBAND_PERI(CONTROL_PORT->IN, CYCLE_START_PIN);
  #else
    uint8_t bits = CONTROL_PORT->IN;
   #if ESTOP_ENABLE
    signals.e_stop = (bits & RESET_BIT) != 0;
   #else
    signals.reset = (bits & RESET_BIT) != 0;
   #endif
    signals.feed_hold = (bits & FEED_HOLD_BIT) != 0;
    signals.cycle_start = (bits & CYCLE_START_BIT) != 0;
   #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    signals.safety_door_ajar = (bits & SAFETY_DOOR_BIT) != 0;
   #endif
  #endif
#else
#if CONTROL_INMODE == GPIO_BITBAND
  #if ESTOP_ENABLE
    signals.e_stop = BITBAND_PERI(CONTROL_PORT_RST->IN, RESET_PIN);
  #else
    signals.reset = BITBAND_PERI(CONTROL_PORT_RST->IN, RESET_PIN);
  #endif
    signals.feed_hold = BITBAND_PERI(CONTROL_PORT_FH->IN, FEED_HOLD_PIN);
    signals.cycle_start = BITBAND_PERI(CONTROL_PORT_CS->IN, CYCLE_START_PIN);
  #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    signals.safety_door_ajar = BITBAND_PERI(CONTROL_PORT_SD->IN, SAFETY_DOOR_PIN);
  #endif
#else
    uint8_t bits = CONTROL_PORT->IN;
  #if ESTOP_ENABLE
    signals.e_stop = (bits & RESET_BIT) != 0;
  #else
    signals.reset = (bits & RESET_BIT) != 0;
  #endif
    signals.feed_hold = (bits & FEED_HOLD_BIT) != 0;
    signals.cycle_start = (bits & CYCLE_START_BIT) != 0;
  #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    signals.safety_door_ajar = (bits & SAFETY_DOOR_BIT) != 0;
  #endif
#endif
#endif

    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

#if LIMITS_OVERRIDE_ENABLE
    signals.limits_override = BITBAND_PERI(LIMITS_OVERRIDE_PORT->IN, LIMITS_OVERRIDE_SWITCH_PIN) == 0;
#endif

    return signals;
}

// Toggle probe connected status. Used when no input pin is available.
static void probeConnectedToggle (void)
{
    probe.connected = !probe.connected;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
    probe.triggered = Off;
    probe.is_probing = probing;
    probe.inverted = is_probe_away ? !settings.probe.invert_probe_pin : settings.probe.invert_probe_pin;

    if(psettings.enable_protection) {
        BITBAND_PERI(PROBE_PORT->IE, PROBE_PIN) = 0;
        BITBAND_PERI(PROBE_PORT->IES, PROBE_PIN) = probing ? probing : probe.inverted;
        BITBAND_PERI(PROBE_PORT->IFG, PROBE_PIN) = 0;
        BITBAND_PERI(PROBE_PORT->IE, PROBE_PIN) = 1;
    }
}

// Returns the probe pin state. Triggered = true.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;

    if(psettings.enable_protection)
        state.triggered = probe.triggered || BITBAND_PERI(PROBE_PORT->IN, PROBE_PIN) ^ probe.inverted;
    else
        state.triggered = BITBAND_PERI(PROBE_PORT->IN, PROBE_PIN) ^ probe.inverted;

    return state;
}

inline static float spindle_calc_rpm (uint32_t tpp)
{
    return spindle_encoder.rpm_factor / (float)tpp;
}

#ifndef VFD_SPINDLE

// Static spindle (off, on cw & on ccw)

inline static void spindle_off (void)
{
    BITBAND_PERI(SPINDLE_ENABLE_PORT->OUT, SPINDLE_ENABLE_PIN) = settings.spindle.invert.on;
    if(hal.driver_cap.spindle_dir)
        BITBAND_PERI(SPINDLE_DIRECTION_PORT->OUT, SPINDLE_DIRECTION_PIN) = settings.spindle.invert.ccw;
}

inline static void spindle_on (void)
{
    BITBAND_PERI(SPINDLE_ENABLE_PORT->OUT, SPINDLE_ENABLE_PIN) = !settings.spindle.invert.on;
    spindleDataReset();
}

inline static void spindle_dir (bool ccw)
{
    if(hal.driver_cap.spindle_dir)
        BITBAND_PERI(SPINDLE_DIRECTION_PORT->OUT, SPINDLE_DIRECTION_PIN) = ccw ^ settings.spindle.invert.ccw;
}

// Start or stop spindle
static void spindleSetState (spindle_state_t state, float rpm)
{
    if (!state.on)
        spindle_off();
    else {
        spindle_dir(state.ccw);
        spindle_on();
    }
}

// Variable spindle control functions

// Sets spindle speed
static void spindle_set_speed (uint_fast16_t pwm_value)
{
    while(spindleLock); // wait for PID

    if (pwm_value == spindle_pwm.off_value) {
        pwmEnabled = false;
        if(settings.spindle.flags.pwm_action == SpindleAction_DisableWithZeroSPeed)
            spindle_off();
        if(spindle_pwm.always_on) {
            SPINDLE_PWM_TIMER->CCR[2] = spindle_pwm.off_value;
            SPINDLE_PWM_TIMER->CCTL[2] = settings.spindle.invert.pwm ? TIMER_A_CCTLN_OUTMOD_6 : TIMER_A_CCTLN_OUTMOD_2;
        } else
            SPINDLE_PWM_TIMER->CCTL[2] = settings.spindle.invert.pwm ? TIMER_A_CCTLN_OUT : 0; // Set PWM output according to invert setting
#ifdef SPINDLE_RPM_CONTROLLED
        spindle_control.pid.error = 0.0f;
#endif
    } else {
        if(!pwmEnabled)
            spindle_on();
        pwmEnabled = true;
        SPINDLE_PWM_TIMER->CCR[2] = pwm_value;
        SPINDLE_PWM_TIMER->CCTL[2] = settings.spindle.invert.pwm ? TIMER_A_CCTLN_OUTMOD_6 : TIMER_A_CCTLN_OUTMOD_2;
    }
}

#ifdef SPINDLE_PWM_DIRECT

static uint_fast16_t spindleGetPWM (float rpm)
{
    return spindle_compute_pwm_value(&spindle_pwm, rpm, false);
}

#else

static void spindleUpdateRPM (float rpm)
{
    while(spindleLock); // wait for PID

    spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm + spindle_control.pid.error, spindle_control.pid.error != 0.0f));
    if(settings.spindle.at_speed_tolerance > 0.0f) {
        spindle_data.rpm_low_limit = rpm / (1.0f + settings.spindle.at_speed_tolerance);
        spindle_data.rpm_high_limit = rpm * (1.0f + settings.spindle.at_speed_tolerance);
    }
    spindle_data.rpm_programmed = spindle_data.rpm = rpm;
}

#endif

// Start or stop spindle
static void spindleSetStateVariable (spindle_state_t state, float rpm)
{
    if (!state.on || rpm == 0.0f) {
        spindle_set_speed(spindle_pwm.off_value);
        spindle_off();
#ifdef SPINDLE_RPM_CONTROLLED
        spindle_control.rpm = 0.0f;
        spindle_control.pid_state = PIDState_Disabled;
        pidf_reset(&spindle_control.pid);
        spindle_control.pid.sample_rate_prev = 1.0f;
#endif
    } else {
        spindle_dir(state.ccw);
#ifdef SPINDLE_RPM_CONTROLLED
        if(spindle_data.rpm_programmed == 0.0f) {
            if(spindle_control.pid_enabled) {
                pid_count = 0;
                spindle_control.pid_state = PIDState_Pending;
            }
        }
  #ifdef sPID_LOG
        sys.pid_log.idx = 0;
  #endif
        spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm + spindle_control.pid.error, spindle_control.pid.error != 0.0f));
#else
        spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
#endif
    }

    if(settings.spindle.at_speed_tolerance > 0.0f) {
        spindle_data.rpm_low_limit = rpm / (1.0f + settings.spindle.at_speed_tolerance);
        spindle_data.rpm_high_limit = rpm * (1.0f + settings.spindle.at_speed_tolerance);
    }
    spindle_data.rpm_programmed = spindle_data.rpm = rpm;
}


// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    float rpm = spindleGetData(SpindleData_RPM)->rpm;
    spindle_state_t state = {0};

//    state.on = (SPINDLE_ENABLE_PORT->IN & SPINDLE_ENABLE_BIT) != 0;
    state.on = BITBAND_PERI(SPINDLE_ENABLE_PORT->IN, SPINDLE_ENABLE_PIN);
    if(hal.driver_cap.spindle_dir)
        state.ccw = BITBAND_PERI(SPINDLE_DIRECTION_PORT->IN, SPINDLE_DIRECTION_PIN);
    state.value ^= settings.spindle.invert.mask;
    if(pwmEnabled)
        state.on = On;

    state.at_speed = settings.spindle.at_speed_tolerance <= 0.0f || (rpm >= spindle_data.rpm_low_limit && rpm <= spindle_data.rpm_high_limit);
    state.encoder_error = spindle_encoder.error_count > 0;

    return state;
}

#ifdef SPINDLE_RPM_CONTROLLED

inline static void spindle_rpm_pid (uint32_t tpp)
{
    spindleLock = true;

    spindle_control.rpm = spindle_calc_rpm(tpp);
    float error = pidf(&spindle_control.pid, spindle_data.rpm_programmed, spindle_control.rpm, 1.0);

#ifdef sPID_LOG
    if(sys.pid_log.idx < PID_LOG) {
        sys.pid_log.target[sys.pid_log.idx] = error;
        sys.pid_log.actual[sys.pid_log.idx] = rpm;
        sys.pid_log.idx++;
    }
#endif

    SPINDLE_PWM_TIMER->CCR[2] = spindle_compute_pwm_value(&spindle_pwm, spindle_data.rpm_programmed + error, error != 0.0f);

    spindleLock = false;
}

#endif

#endif // VFD_SPINDLE

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    bool stopped;
    uint32_t pulse_length, rpm_timer_delta;
    spindle_encoder_counter_t encoder;

    while(spindle_encoder.spin_lock);

    __disable_irq();

    memcpy(&encoder, &spindle_encoder.counter, sizeof(spindle_encoder_counter_t));

    pulse_length = spindle_encoder.timer.pulse_length / spindle_encoder.tics_per_irq;
    rpm_timer_delta = spindle_encoder.timer.last_pulse - RPM_TIMER->VALUE; // NOTE: timer is counting down!

    __enable_irq();

    // If no (4) spindle pulses during last 250mS assume RPM is 0
    if((stopped = ((pulse_length == 0) || (rpm_timer_delta > spindle_encoder.maximum_tt)))) {
        spindle_data.rpm = 0.0f;
        rpm_timer_delta = (RPM_COUNTER->R - (uint16_t)spindle_encoder.counter.last_count) * pulse_length;
    }

    switch(request) {

        case SpindleData_Counters:
            spindle_data.index_count = encoder.index_count;
            spindle_data.pulse_count = encoder.pulse_count + (uint32_t)((uint16_t)RPM_COUNTER->R - (uint16_t)encoder.last_count);
            spindle_data.error_count = spindle_encoder.error_count;
            break;

        case SpindleData_RPM:
            if(!stopped)
#ifdef SPINDLE_RPM_CONTROLLED
                spindle_data.rpm = spindle_control.pid_enabled ? spindle_control.rpm : spindle_calc_rpm(pulse_length);
#else
                spindle_data.rpm = spindle_calc_rpm(pulse_length);
#endif
            break;

        case SpindleData_AngularPosition:;
            int32_t d = (uint16_t)((uint16_t)encoder.last_count - (uint16_t)encoder.last_index);
            if(d < 0)
                d++;
            spindle_data.angular_position = (float)encoder.index_count +
                    ((float)((uint16_t)encoder.last_count - (uint16_t)encoder.last_index) +
                              (pulse_length == 0 ? 0.0f : (float)rpm_timer_delta / (float)pulse_length)) *
                                spindle_encoder.pulse_distance;
            if(spindle_data.angular_position < (float)encoder.index_count)
                d++;
            break;
    }

    return &spindle_data;
}

static void spindleDataReset (void)
{
    while(spindleLock);

    uint32_t systick_state = SysTick->CTRL;

    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
/*
    uint32_t index_count = spindle_data.index_count + 2, timeout = elapsed_tics + 1000;
    if(spindleGetData(SpindleData_RPM).rpm > 0.0f) { // wait for index pulse if running

        while(index_count != spindle_data.index_count && elapsed_tics <= timeout);

//        if(uwTick > timeout)
//            alarm?
    }
*/

#ifdef SPINDLE_RPM_CONTROLLED
    if(spindle_control.pid_enabled)
        spindle_control.pid_state = PIDState_Pending;
#endif

    RPM_TIMER->LOAD = (uint32_t)-1L; // Reload RPM timer
    RPM_COUNTER->CTL = 0;

    spindle_encoder.timer.last_pulse =
    spindle_encoder.timer.last_index = RPM_TIMER->VALUE;

    spindle_encoder.timer.pulse_length =
    spindle_encoder.counter.last_count =
    spindle_encoder.counter.last_index =
    spindle_encoder.counter.pulse_count =
    spindle_encoder.counter.index_count =
    spindle_encoder.error_count = 0;

    RPM_COUNTER->CCR[0] = spindle_encoder.tics_per_irq;
    RPM_COUNTER->CTL = TIMER_A_CTL_MC__CONTINUOUS|TIMER_A_CTL_CLR;

    spindle_encoder.counter.last_count = RPM_COUNTER->R;

    if(systick_state & SysTick_CTRL_ENABLE_Msk)
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

// end spindle code

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;
    BITBAND_PERI(COOLANT_FLOOD_PORT->OUT, COOLANT_FLOOD_PIN) = mode.flood;
    BITBAND_PERI(COOLANT_MIST_PORT->OUT, COOLANT_MIST_PIN) = mode.mist;
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

    state.flood = (COOLANT_FLOOD_PORT->IN & COOLANT_FLOOD_BIT) != 0;
    state.mist  = (COOLANT_MIST_PORT->IN & COOLANT_MIST_BIT) != 0;
    state.value ^= settings.coolant_invert.mask;

    return state;
}

static void enable_irq (void)
{
    __enable_irq();
}

static void disable_irq (void)
{
    __disable_irq();
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    *ptr |= bits;
    __enable_irq();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    __enable_irq();
    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr = value;
    __enable_irq();
    return prev;
}

#if MPG_MODE_ENABLE

static void modeSelect (bool mpg_mode)
{
    BITBAND_PERI(MODE_PORT->IE, MODE_SWITCH_PIN) = 0;
    BITBAND_PERI(MODE_PORT->IES, MODE_SWITCH_PIN) = !mpg_mode;
    BITBAND_PERI(MODE_PORT->IFG, MODE_SWITCH_PIN) = 0;
    BITBAND_PERI(MODE_PORT->IE, MODE_SWITCH_PIN) = 1;

    // Deny entering MPG mode if busy
    if(mpg_mode == sys.mpg_mode || (mpg_mode && (gc_state.file_run || !(sys.state == STATE_IDLE || (sys.state & (STATE_ALARM|STATE_ESTOP)))))) {
        hal.stream.enqueue_realtime_command(CMD_STATUS_REPORT_ALL);
        return;
    }

    serialSelect(mpg_mode);

    if(mpg_mode) {
        hal.stream.read = serial2GetC;
        hal.stream.get_rx_buffer_available = serial2RxFree;
        hal.stream.cancel_read_buffer = serial2RxCancel;
        hal.stream.reset_read_buffer = serial2RxFlush;
    } else {
        hal.stream.read = serialGetC;
        hal.stream.get_rx_buffer_available = serialRxFree;
        hal.stream.cancel_read_buffer = serialRxCancel;
        hal.stream.reset_read_buffer = serialRxFlush;
    }

    hal.stream.reset_read_buffer();

    sys.mpg_mode = mpg_mode;
    sys.report.mpg_mode = On;

    // Force a realtime status report, all reports when MPG mode active
    hal.stream.enqueue_realtime_command(mpg_mode ? CMD_STATUS_REPORT_ALL : CMD_STATUS_REPORT);
}

static void modeChange (void)
{
    modeSelect((MODE_PORT->IN & MODE_SWITCH_BIT) == 0);
}

static void modeEnable (void)
{
    bool on = BITBAND_PERI(MODE_PORT->IN, MODE_SWITCH_PIN) == 0;

    if(sys.mpg_mode != on)
        modeSelect(true);
    BITBAND_PERI(MODE_PORT->IES, MODE_SWITCH_PIN) = !on;
    BITBAND_PERI(MODE_PORT->IFG, MODE_SWITCH_PIN) = 0;
    BITBAND_PERI(MODE_PORT->IE, MODE_SWITCH_PIN) = 1;
#if KEYPAD_ENABLE
    KEYPAD_PORT->IE |= KEYPAD_IRQ_BIT;
#endif
}

#endif

uint32_t getElapsedTicks (void)
{
    return elapsed_tics;
}

// Configure perhipherals when settings are initialized or changed
void settings_changed (settings_t *settings)
{
    uint_fast8_t idx;

#ifndef VFD_SPINDLE

    if((hal.driver_cap.variable_spindle = settings->spindle.rpm_min < settings->spindle.rpm_max)) {
        if(settings->spindle.pwm_freq > 200.0f)
            SPINDLE_PWM_TIMER->CTL &= ~TIMER_A_CTL_ID__8;
        else
            SPINDLE_PWM_TIMER->CTL |= TIMER_A_CTL_ID__8;

        spindle_precompute_pwm_values(&spindle_pwm, 12000000UL / (settings->spindle.pwm_freq > 200.0f ? 2 : 16));
    }

    hal.driver_cap.spindle_at_speed = hal.driver_cap.variable_spindle && settings->spindle.ppr > 0;
    hal.spindle.set_state = hal.driver_cap.variable_spindle ? spindleSetStateVariable : spindleSetState;

  #ifdef SPINDLE_RPM_CONTROLLED

    if((spindle_control.pid_enabled = hal.spindle.get_data && settings->spindle.pid.p_gain != 0.0f)) {
        if(memcmp(&spindle_control.pid.cfg, &settings->spindle.pid, sizeof(pid_values_t)) != 0) {
            spindle_set_state((spindle_state_t){0}, 0.0f);
            pidf_init(&spindle_control.pid, &settings->spindle.pid);
      //      spindle_encoder.pid.cfg.i_max_error = spindle_encoder.pid.cfg.i_max_error / settings->spindle.pid.i_gain; // Makes max value sensible?
        }
    } else
        spindle_control.pid_state = PIDState_Disabled;

  #endif

#endif

    if((hal.spindle.get_data = hal.driver_cap.spindle_at_speed ? spindleGetData : NULL) && spindle_encoder.ppr != settings->spindle.ppr) {

        hal.spindle.reset_data = spindleDataReset;

        pidf_init(&spindle_tracker.pid, &settings->position.pid);

        float timer_resolution = 1.0f / (float)(SystemCoreClock / 16);

        spindle_tracker.min_cycles_per_tick = hal.f_step_timer / 1000000UL * (uint32_t)ceilf(settings->steppers.pulse_microseconds * 2.0f + settings->steppers.pulse_delay_microseconds);
        spindle_set_state((spindle_state_t){0}, 0.0f);
        spindle_encoder.ppr = settings->spindle.ppr;
        spindle_encoder.tics_per_irq = 4;
        spindle_encoder.pulse_distance = 1.0f / spindle_encoder.ppr;
        spindle_encoder.maximum_tt = (uint32_t)(0.25f / timer_resolution) * spindle_encoder.tics_per_irq; // 250 mS
        spindle_encoder.rpm_factor = 60.0f / ((timer_resolution * (float)spindle_encoder.ppr));
        BITBAND_PERI(RPM_INDEX_PORT->IES, RPM_INDEX_PIN) = 1;
        BITBAND_PERI(RPM_INDEX_PORT->IE, RPM_INDEX_PIN) = 1;
        spindleDataReset();
    }

    if(!hal.spindle.get_data)
        BITBAND_PERI(RPM_INDEX_PORT->IE, RPM_INDEX_PIN) = 0;

#if STEP_OUTMODE == GPIO_MAP
    for(idx = 0; idx < sizeof(step_outmap); idx++)
        step_outmap[idx] = c_step_outmap[idx ^ settings->steppers.step_invert.mask];
#endif

#if DIRECTION_OUTMODE == GPIO_MAP
    for(idx = 0; idx < sizeof(dir_outmap); idx++)
        dir_outmap[idx] = c_dir_outmap[idx ^ settings->steppers.dir_invert.mask];
#endif

    if(IOInitDone) {

        stepperEnable(settings->steppers.deenergize);

#ifndef VFD_SPINDLE

        if(hal.driver_cap.variable_spindle) {
            SPINDLE_PWM_TIMER->CCR[0] = spindle_pwm.period;
            SPINDLE_PWM_TIMER->CCTL[2] = settings->spindle.invert.pwm ? TIMER_A_CCTLN_OUT : 0;  // Set PWM output according to invert setting and
            SPINDLE_PWM_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC0|TIMER_A_CTL_MC1;          // start PWM timer (with no pulse output)
        }
#endif

        pulse_length = (uint16_t)(12.0f * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY));

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            int16_t pulse_delay = (uint16_t)(12.0f * (settings->steppers.pulse_delay_microseconds - 1.2f));
            PULSE_TIMER->CCR[1] = pulse_delay < 2 ? 2 : pulse_delay;
            hal.stepper.pulse_start = stepperPulseStartDelayed;
        } else
            hal.stepper.pulse_start = stepperPulseStart;

        PULSE_TIMER->CCTL[1] &= ~TIMER_A_CCTLN_CCIE; // Disable CCR1 (step delay) interrupt
        PULSE_TIMER->CCR[0] = pulse_length;

        /*************************
         *  Control pins config  *
         *************************/

        control_signals_t control_ies;

        control_ies.mask = (settings->control_disable_pullup.mask ^ settings->control_invert.mask);

#if CNC_BOOSTERPACK_SHORTS
        CONTROL_PORT->IE &= ~CONTROL_MASK;

        BITBAND_PERI(CONTROL_PORT->OUT, CYCLE_START_PIN) = !settings->control_disable_pullup.cycle_start;
        BITBAND_PERI(CONTROL_PORT->IES, CYCLE_START_PIN) = control_ies.cycle_start;
        BITBAND_PERI(CONTROL_PORT->REN, CYCLE_START_PIN) = 1;

        BITBAND_PERI(CONTROL_PORT->OUT, FEED_HOLD_PIN) = !settings->control_disable_pullup.feed_hold;
        BITBAND_PERI(CONTROL_PORT->IES, FEED_HOLD_PIN) = control_ies.feed_hold;
        BITBAND_PERI(CONTROL_PORT->REN, FEED_HOLD_PIN) = 1;

        BITBAND_PERI(CONTROL_PORT->OUT, SAFETY_DOOR_PIN) = !settings->control_disable_pullup.safety_door_ajar;
        BITBAND_PERI(CONTROL_PORT->IES, SAFETY_DOOR_PIN) = control_ies.safety_door_ajar;
        BITBAND_PERI(CONTROL_PORT->REN, SAFETY_DOOR_PIN) = 1;

#if ESTOP_ENABLE
        BITBAND_PERI(CONTROL_PORT->OUT, RESET_PIN) = !settings->control_disable_pullup.e_stop;
        BITBAND_PERI(CONTROL_PORT->IES, RESET_PIN) = control_ies.reset;
        BITBAND_PERI(CONTROL_PORT->REN, RESET_PIN) = 1;
#else
        BITBAND_PERI(CONTROL_PORT->OUT, RESET_PIN) = !settings->control_disable_pullup.e_stop;
        BITBAND_PERI(CONTROL_PORT->IES, RESET_PIN) = control_ies.reset;
        BITBAND_PERI(CONTROL_PORT->REN, RESET_PIN) = 1;
#endif

        CONTROL_PORT->IFG &= ~CONTROL_MASK;
        CONTROL_PORT->IE |= CONTROL_MASK;
#else
        BITBAND_PERI(CONTROL_PORT_CS->IE, CYCLE_START_PIN) = 0;
        BITBAND_PERI(CONTROL_PORT_FH->IE, FEED_HOLD_PIN) = 0;
        BITBAND_PERI(CONTROL_PORT_SD->IE, SAFETY_DOOR_PIN) = 0;
        BITBAND_PERI(CONTROL_PORT_RST->IE, RESET_PIN) = 0;

        BITBAND_PERI(CONTROL_PORT_CS->OUT, CYCLE_START_PIN) = !settings->control_disable_pullup.cycle_start;
        BITBAND_PERI(CONTROL_PORT_CS->IES, CYCLE_START_PIN) = control_ies.cycle_start;
        BITBAND_PERI(CONTROL_PORT_CS->REN, CYCLE_START_PIN) = 1;

        BITBAND_PERI(CONTROL_PORT_FH->OUT, FEED_HOLD_PIN) = !settings->control_disable_pullup.feed_hold;
        BITBAND_PERI(CONTROL_PORT_FH->IES, FEED_HOLD_PIN) = control_ies.feed_hold;
        BITBAND_PERI(CONTROL_PORT_FH->REN, FEED_HOLD_PIN) = 1;

        BITBAND_PERI(CONTROL_PORT_SD->OUT, SAFETY_DOOR_PIN) = !settings->control_disable_pullup.safety_door_ajar;
        BITBAND_PERI(CONTROL_PORT_SD->IES, SAFETY_DOOR_PIN) = control_ies.safety_door_ajar;
        BITBAND_PERI(CONTROL_PORT_SD->REN, SAFETY_DOOR_PIN) = 1;
#if ESTOP_ENABLE
        BITBAND_PERI(CONTROL_PORT_RST->OUT, RESET_PIN) = !settings->control_disable_pullup.e_stop;
        BITBAND_PERI(CONTROL_PORT_RST->IES, RESET_PIN) = control_ies.e_stop;
        BITBAND_PERI(CONTROL_PORT_RST->REN, RESET_PIN) = 1;
#else
        BITBAND_PERI(CONTROL_PORT_RST->OUT, RESET_PIN) = !settings->control_disable_pullup.reset;
        BITBAND_PERI(CONTROL_PORT_RST->IES, RESET_PIN) = control_ies.reset;
        BITBAND_PERI(CONTROL_PORT_RST->REN, RESET_PIN) = 1;
#endif
        BITBAND_PERI(CONTROL_PORT_CS->IFG, CYCLE_START_PIN) = 0;
        BITBAND_PERI(CONTROL_PORT_FH->IFG, FEED_HOLD_PIN) = 0;
        BITBAND_PERI(CONTROL_PORT_SD->IFG, SAFETY_DOOR_PIN) = 0;
        BITBAND_PERI(CONTROL_PORT_RST->IFG, RESET_PIN) = 0;

        BITBAND_PERI(CONTROL_PORT_CS->IE, CYCLE_START_PIN) = 1;
        BITBAND_PERI(CONTROL_PORT_FH->IE, FEED_HOLD_PIN) = 1;
        BITBAND_PERI(CONTROL_PORT_SD->IE, SAFETY_DOOR_PIN) = 1;
        BITBAND_PERI(CONTROL_PORT_RST->IE, RESET_PIN) = 1;
#endif

#if LIMITS_OVERRIDE_ENABLE
        BITBAND_PERI(LIMITS_OVERRIDE_PORT->OUT, LIMITS_OVERRIDE_SWITCH_PIN) = 1;
        BITBAND_PERI(LIMITS_OVERRIDE_PORT->REN, LIMITS_OVERRIDE_SWITCH_PIN) = 1;
#endif

        /***********************
         *  Limit pins config  *
         ***********************/

        axes_signals_t limit_ies;

        limit_ies.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;
#if CNC_BOOSTERPACK_SHORTS
        BITBAND_PERI(LIMIT_PORT->OUT, X_LIMIT_PIN) = !settings->limits.disable_pullup.x;
        BITBAND_PERI(LIMIT_PORT->IES, X_LIMIT_PIN) = limit_ies.x;
        BITBAND_PERI(LIMIT_PORT->REN, X_LIMIT_PIN) = 1;

        BITBAND_PERI(LIMIT_PORT->OUT, Y_LIMIT_PIN) = !settings->limits.disable_pullup.y;
        BITBAND_PERI(LIMIT_PORT->IES, Y_LIMIT_PIN) = limit_ies.y;
        BITBAND_PERI(LIMIT_PORT->REN, Y_LIMIT_PIN) = 1;

        BITBAND_PERI(LIMIT_PORT->OUT, Z_LIMIT_PIN) = !settings->limits.disable_pullup.z;
        BITBAND_PERI(LIMIT_PORT->IES, Z_LIMIT_PIN) = limit_ies.z;
        BITBAND_PERI(LIMIT_PORT->REN, Z_LIMIT_PIN) = 1;
#else
        BITBAND_PERI(LIMIT_PORT_X->OUT, X_LIMIT_PIN) = !settings->limits.disable_pullup.x;
        BITBAND_PERI(LIMIT_PORT_X->IES, X_LIMIT_PIN) = limit_ies.x;
        BITBAND_PERI(LIMIT_PORT_X->REN, X_LIMIT_PIN) = 1;

        BITBAND_PERI(LIMIT_PORT_Y->OUT, Y_LIMIT_PIN) = !settings->limits.disable_pullup.y;
        BITBAND_PERI(LIMIT_PORT_Y->IES, Y_LIMIT_PIN) = limit_ies.y;
        BITBAND_PERI(LIMIT_PORT_Y->REN, Y_LIMIT_PIN) = 1;

        BITBAND_PERI(LIMIT_PORT_Z->OUT, Z_LIMIT_PIN) = !settings->limits.disable_pullup.z;
        BITBAND_PERI(LIMIT_PORT_Z->IES, Z_LIMIT_PIN) = limit_ies.z;
        BITBAND_PERI(LIMIT_PORT_Z->REN, Z_LIMIT_PIN) = 1;
#endif

        /**********************
         *  Probe pin config  *
         **********************/

        BITBAND_PERI(PROBE_PORT->OUT, PROBE_PIN) = hal.driver_cap.probe_pull_up;
        BITBAND_PERI(PROBE_PORT->REN, PROBE_PIN) = 1;
        BITBAND_PERI(PROBE_PORT->IE, PROBE_PIN) = 0;
        BITBAND_PERI(PROBE_PORT->IFG, PROBE_PIN) = 0;

        if(psettings.enable_protection) {
            BITBAND_PERI(PROBE_PORT->IES, PROBE_PIN) = 1;
            BITBAND_PERI(PROBE_PORT->IE, PROBE_PIN) = 1;
            NVIC_EnableIRQ(PROBE_INT);
        }

        /***************************
         *  MPG mode input enable  *
         ***************************/

#if MPG_MODE_ENABLE
        if(hal.driver_cap.mpg_mode) {
            // Enable pullup and switch to input
            BITBAND_PERI(MODE_PORT->OUT, MODE_SWITCH_PIN) = 1;
            BITBAND_PERI(MODE_PORT->REN, MODE_SWITCH_PIN) = 1;
            BITBAND_PERI(MODE_PORT->DIR, MODE_SWITCH_PIN) = 0;
            // Delay mode enable a bit so grbl can finish startup and MPG controller can check ready status
            hal.delay_ms(50, modeEnable);
        }
#endif
    }
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
 // Stepper init

    STEP_PORT->DIR |= STEP_MASK;
    DIRECTION_PORT->DIR |= DIRECTION_MASK;
#if !(TRINAMIC_ENABLE && TRINAMIC_I2C)
    STEPPERS_DISABLE_Z_PORT->DIR |= STEPPERS_DISABLE_Z_BIT;
    STEPPERS_DISABLE_XY_PORT->DIR |= STEPPERS_DISABLE_X_BIT;
#endif

#if CNC_BOOSTERPACK_A4998
    STEPPERS_VDD_PORT->DIR |= STEPPERS_VDD_BIT;
    STEPPERS_VDD_PORT->DS  |= STEPPERS_VDD_BIT;
    STEPPERS_VDD_PORT->OUT |= STEPPERS_VDD_BIT;
#endif

    STEPPER_TIMER->CONTROL = TIMER32_CONTROL_SIZE|TIMER32_CONTROL_MODE;

    PULSE_TIMER->CTL = TIMER_A_CTL_SSEL__SMCLK|TIMER_A_CTL_CLR; // CLK: 12Mhz = 833.33... ns
    PULSE_TIMER->CCTL[0] |= TIMER_A_CCTLN_CCIE;

    NVIC_EnableIRQ(STEPPER_TIMER_INT);  // Enable stepper interrupt and
    NVIC_EnableIRQ(PULSE_TIMER_INT0);   // step pulse interrupts
    NVIC_EnableIRQ(PULSE_TIMER_INTN);   // ...

    NVIC_SetPriority(PULSE_TIMER_INT0, 1);
    NVIC_SetPriority(PULSE_TIMER_INTN, 1);
    NVIC_SetPriority(STEPPER_TIMER_INT, 2);

 // Limit pins init
#if CNC_BOOSTERPACK_SHORTS
    NVIC_EnableIRQ(LIMIT_INT);  // Enable limit port interrupt
#else
    NVIC_EnableIRQ(LIMIT_INT_X);  // Enable limit port X interrupt
    NVIC_EnableIRQ(LIMIT_INT_YZ); // Enable limit port Y,Z interrupt
#endif

 // Control pins init
#if CNC_BOOSTERPACK_SHORTS
    NVIC_EnableIRQ(CONTROL_INT); // Enable limit port interrupt
#else
    // NOTE: CS is shared with limit isr
    NVIC_EnableIRQ(CONTROL_INT_SD_RST); // Enable limit port SD,RST interrupt
    NVIC_EnableIRQ(CONTROL_INT_FH);     // Enable limit port Y,Z interrupt
#endif

#if MPG_MODE_ENABLE
    NVIC_EnableIRQ(MODE_INT); // mode switch interrupt
#endif

    if(hal.driver_cap.software_debounce) {
        DEBOUNCE_TIMER->EX0 = TIMER_A_EX0_IDEX__6; // -> SMCLK (12MHz) / 6 = 2MHz
        DEBOUNCE_TIMER->CTL = TIMER_A_CTL_SSEL__SMCLK|TIMER_A_CTL_ID__2|TIMER_A_CTL_CLR; // CLK: 4Mhz / 4 = 1uS
        DEBOUNCE_TIMER->CCR[0] = 32000;  // 32ms
        DEBOUNCE_TIMER->CCTL[0] |= TIMER_A_CCTLN_CCIE;
        NVIC_EnableIRQ(DEBOUNCE_TIMER_INT0); // Enable limit port Y,Z interrupt
    }

 // Spindle init

#ifndef VFD_SPINDLE

    SPINDLE_ENABLE_PORT->DIR |= SPINDLE_ENABLE_BIT;
    SPINDLE_DIRECTION_PORT->DIR |= SPINDLE_DIRECTION_BIT; // Configure as output pin.

    SPINDLE_PWM_PORT->DIR |= SPINDLE_PWM_BIT;
    SPINDLE_PWM_PORT->SEL1 &= ~SPINDLE_PWM_BIT;
    SPINDLE_PWM_PORT->SEL0 |= SPINDLE_PWM_BIT;
    SPINDLE_PWM_TIMER->CTL = TIMER_A_CTL_SSEL__SMCLK;
    SPINDLE_PWM_TIMER->EX0 = 0;

#endif

//    if(hal.spindle.index_callback || true) {
        RPM_INDEX_PORT->OUT |= RPM_INDEX_BIT;
        RPM_INDEX_PORT->REN |= RPM_INDEX_BIT;
        RPM_INDEX_PORT->IES |= RPM_INDEX_BIT;
//    }

    NVIC_EnableIRQ(RPM_INDEX_INT);

    memset(&spindle_encoder, 0, sizeof(spindle_encoder_t));
    memset(&spindle_tracker, 0, sizeof(spindle_sync_t));
    memset(&spindle_data, 0, sizeof(spindle_data));

    RPM_COUNTER_PORT->SEL0 |= RPM_COUNTER_BIT; // Set as counter input
    RPM_COUNTER->CTL = TIMER_A_CTL_MC__CONTINUOUS|TIMER_A_CTL_CLR;
    RPM_COUNTER->CCTL[0] = TIMER_A_CCTLN_CCIE;
    RPM_COUNTER->CCR[0] = spindle_encoder.tics_per_irq;

    NVIC_EnableIRQ(RPM_COUNTER_INT0);   // Enable RPM timer interrupt

    RPM_TIMER->CONTROL = TIMER32_CONTROL_SIZE|TIMER32_CONTROL_ENABLE|TIMER32_CONTROL_PRESCALE_1; // rolls over after ~23 minutes

  // Coolant init

    COOLANT_FLOOD_PORT->DIR |= COOLANT_FLOOD_BIT;
    COOLANT_MIST_PORT->DIR |= COOLANT_MIST_BIT;

// Set defaults

#if KEYPAD_ENABLE
    BITBAND_PERI(KEYPAD_PORT->OUT, KEYPAD_IRQ_PIN) = 1;
    BITBAND_PERI(KEYPAD_PORT->REN, KEYPAD_IRQ_PIN) = 1;
    BITBAND_PERI(KEYPAD_PORT->IES, KEYPAD_IRQ_PIN) = (KEYPAD_PORT->IN & KEYPAD_IRQ_BIT) ? 1 : 0;
    KEYPAD_PORT->IFG &= ~KEYPAD_IRQ_BIT;
#if !MPG_MODE_ENABLE
    KEYPAD_PORT->IE |= KEYPAD_IRQ_BIT;
#endif
    NVIC_EnableIRQ(KEYPAD_INT);  // Enable keypad  port interrupt
#endif

#if TRINAMIC_ENABLE == 2130

    // Configure input pin for DIAG1 signal (with pullup) and enable interrupt
    BITBAND_PERI(TRINAMIC_DIAG_IRQ_PORT->OUT, TRINAMIC_DIAG_IRQ_PIN) = 1;
    BITBAND_PERI(TRINAMIC_DIAG_IRQ_PORT->REN, TRINAMIC_DIAG_IRQ_PIN) = 1;
    BITBAND_PERI(TRINAMIC_DIAG_IRQ_PORT->IES, TRINAMIC_DIAG_IRQ_PIN) = 1;
    BITBAND_PERI(TRINAMIC_DIAG_IRQ_PORT->IFG, TRINAMIC_DIAG_IRQ_PIN) = 0;
    BITBAND_PERI(TRINAMIC_DIAG_IRQ_PORT->IE, TRINAMIC_DIAG_IRQ_PIN) = 1;
    NVIC_EnableIRQ(TRINAMIC_DIAG_INT);
  #if TRINAMIC_I2C
    // Configure input pin for WARN signal (with pullup) and enable interrupt
    BITBAND_PERI(TRINAMIC_WARN_IRQ_PORT->OUT, TRINAMIC_WARN_IRQ_PIN) = 1;
    BITBAND_PERI(TRINAMIC_WARN_IRQ_PORT->REN, TRINAMIC_WARN_IRQ_PIN) = 1;
    BITBAND_PERI(TRINAMIC_WARN_IRQ_PORT->IES, TRINAMIC_WARN_IRQ_PIN) = 1;
    BITBAND_PERI(TRINAMIC_WARN_IRQ_PORT->IFG, TRINAMIC_WARN_IRQ_PIN) = 0;
    BITBAND_PERI(TRINAMIC_WARN_IRQ_PORT->IE, TRINAMIC_WARN_IRQ_PIN) = 1;
    NVIC_EnableIRQ(TRINAMIC_WARN_INT);
  #endif

#endif

#if ATC_ENABLE
    atc_init();
#endif

    IOInitDone = settings->version == 19;

    hal.settings_changed(settings);
    hal.stepper.go_idle(true);

    return IOInitDone;
}

#ifdef ENABLE_SPINDLE_LINEARIZATION
static void driver_rt_report (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if(report.pwm) {
        char sbuf[20];
        sprintf(sbuf, "|PWM:%d", settings.spindle.invert.pwm ? spindle_pwm.period - SPINDLE_PWM_TIMER->CCR[2] - 1 : SPINDLE_PWM_TIMER->CCR[2]);
        stream_write(sbuf);
    }
}
#endif

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done

bool driver_init (void)
{
    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

    SystemInit();

    CS->KEY = CS_KEY_VAL;   // Unlock CS module for register access

 /* Change clock source to high frequency crystal */

    PJ->SEL0 |= BIT2|BIT3;
    PJ->SEL1 &= ~(BIT2|BIT3);

    BITBAND_PERI(CS->CTL2, CS_CTL2_HFXTDRIVE_OFS) = 1;
    CS->CTL2 = (CS->CTL2 & (~CS_CTL2_HFXTFREQ_MASK)) | (CS_CTL2_HFXTFREQ_5);
    BITBAND_PERI(CS->CTL2, CS_CTL2_HFXTBYPASS_OFS) = 0;

    BITBAND_PERI(CS->CTL2, CS_CTL2_HFXT_EN_OFS) = 1;
    while (BITBAND_PERI(CS->IFG, CS_IFG_HFXTIFG_OFS))
        BITBAND_PERI(CS->CLRIFG,CS_CLRIFG_CLR_HFXTIFG_OFS) = 1;

    BITBAND_PERI(CS->CTL2, CS_CTL2_HFXTDRIVE_OFS) = 1;

 /* set MCLK to 48 MHz */

    while (!BITBAND_PERI(CS->STAT, CS_STAT_MCLK_READY_OFS));
    CS->CTL1 = CS_CTL1_DIVM_0 | CS_CTL1_SELM__HFXTCLK | (CS->CTL1 & ~(CS_CTL1_SELM_MASK | CS_CTL1_DIVM_MASK));
    while (!BITBAND_PERI(CS->STAT, CS_STAT_MCLK_READY_OFS));

 /* set SMCLK to 12 MHz */

    while (!BITBAND_PERI(CS->STAT, CS_STAT_SMCLK_READY_OFS));
    CS->CTL1 = CS_CTL1_DIVS_2 | CS_CTL1_SELS__HFXTCLK | (CS->CTL1 & ~(CS_CTL1_DIVS_MASK | CS_CTL1_SELS_MASK));
    while (!BITBAND_PERI(CS->STAT, CS_STAT_SMCLK_READY_OFS));

    CS->KEY = 0;    // Lock CS module for register access

//    Interrupt_disableSleepOnIsrExit();

 /* enable lazy stacking of FPU registers */

    FPU->FPCCR = (FPU->FPCCR & ~FPU_FPCCR_LSPEN_Msk) | FPU_FPCCR_ASPEN_Msk;

 /* Set SysTick IRQ to lowest priority */
    NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk;

#if MPG_MODE_ENABLE
    // Drive MPG mode input pin low until setup complete
    BITBAND_PERI(MODE_PORT->DIR, MODE_SWITCH_PIN) = 1;
    BITBAND_PERI(MODE_PORT->OUT, MODE_SWITCH_PIN) = 0;
#endif

    hal.info = "MSP432";
    hal.driver_version = "210206";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.driver_setup = driver_setup;
    hal.f_step_timer = SystemCoreClock;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.probe.configure = probeConfigure;
    hal.probe.get_state = probeGetState;
    hal.probe.connected_toggle = probeConnectedToggle;

#ifndef VFD_SPINDLE
    hal.spindle.set_state = spindleSetState;
    hal.spindle.get_state = spindleGetState;
  #ifdef SPINDLE_PWM_DIRECT
    hal.spindle.get_pwm = spindleGetPWM;
    hal.spindle.update_pwm = spindle_set_speed;
  #else
    hal.spindle.update_rpm = spindleUpdateRPM;
  #endif
#endif

    hal.control.get_state = systemGetState;

    hal.stream.read = serialGetC;
    hal.stream.get_rx_buffer_available = serialRxFree;
    hal.stream.reset_read_buffer = serialRxFlush;
    hal.stream.cancel_read_buffer = serialRxCancel;
    hal.stream.write = serialWriteS;
    hal.stream.write_all = serialWriteS;
    hal.stream.suspend_read = serialSuspendInput;

    hal.irq_enable = enable_irq;
    hal.irq_disable = disable_irq;
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = getElapsedTicks;

    serialInit();

#if I2C_ENABLE
    i2c_init();
#endif

#if EEPROM_ENABLE
    i2c_eeprom_init();
#else
    hal.nvs.type = NVS_None;
#endif

#ifdef ENABLE_SPINDLE_LINEARIZATION
    grbl.on_realtime_report = driver_rt_report;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    hal.signals_cap.safety_door_ajar = On;
#endif
#if ESTOP_ENABLE
    hal.signals_cap.e_stop = On;
#endif
#if LIMITS_OVERRIDE_ENABLE
    hal.signals_cap.limits_override = On;
#endif

    hal.driver_cap.spindle_sync = On;
#ifndef VFD_SPINDLE
    hal.driver_cap.spindle_at_speed = On;
    hal.driver_cap.spindle_dir = On;
  #ifdef SPINDLE_RPM_CONTROLLED
    hal.driver_cap.spindle_pid = On;
  #endif
    hal.driver_cap.variable_spindle = On;
    hal.driver_cap.spindle_pwm_invert = On;
    hal.driver_cap.spindle_pwm_linearization = On;
#endif
    hal.driver_cap.mist_control = On;
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;
#if MPG_MODE_ENABLE
    hal.driver_cap.mpg_mode = On;
    serial2Init(19200);
#endif

#if TRINAMIC_ENABLE
    trinamic_init();
#endif

#if KEYPAD_ENABLE
    keypad_init();
#endif

#if MODBUS_ENABLE

    modbus_stream.write = serial2Write;
    modbus_stream.read = serial2GetC;
    modbus_stream.flush_rx_buffer = serial2RxFlush;
    modbus_stream.flush_tx_buffer = serial2TxFlush;
    modbus_stream.get_rx_buffer_count = serial2RxCount;
    modbus_stream.get_tx_buffer_count = serial2TxCount;
    modbus_stream.set_baud_rate = serial2SetBaudRate;

    bool modbus = modbus_init(&modbus_stream);

#if SPINDLE_HUANYANG > 0
    if(modbus)
        huanyang_init(&modbus_stream);
#endif

#endif

#if PLASMA_ENABLE
    plasma_init();
#endif

	my_plugin_init();

#if ODOMETER_ENABLE
    odometer_init(); // NOTE: this *must* be last plugin to be initialized as it claims storage at the end of NVS.
#endif

    // no need to move version check before init - compiler will fail any signature mismatch for existing entries
    return hal.version == 8;
}

/* interrupt handlers */

// Main stepper driver
void STEPPER_IRQHandler (void)
{
    STEPPER_TIMER->INTCLR = 0;
//    spindle_encoder.timer_value_step = RPM_TIMER->VALUE; // to be used for spindle synchronized motion?
    hal.stepper.interrupt_callback();
}

/* The Stepper Port Reset Interrupt: This interrupt handles the falling edge of the step
   pulse. This should always trigger before the next general stepper driver interrupt and independently
   finish, if stepper driver interrupts is disabled after completing a move.
   NOTE: Interrupt collisions between the serial and stepper interrupts can cause delays by
   a few microseconds, if they execute right before one another. Not a big deal, but can
   cause issues at high step rates if another high frequency asynchronous interrupt is
   added to Grbl.
*/

// This interrupt is used only when STEP_PULSE_DELAY is enabled. Here, the step pulse is
// initiated after the STEP_PULSE_DELAY time period has elapsed.
void STEPPULSE_N_IRQHandler (void)
{
    if(PULSE_TIMER->IV == 0x02) {                               // CCR1 - IV read clears interrupt
        set_step_outputs(next_step_outbits);                    // Begin step pulse
        PULSE_TIMER->CCTL[1] &= ~TIMER_A_CCTLN_CCIE;            // Disable CCR1 interrupt
        PULSE_TIMER->CCR[0] = pulse_length;                     // Set pulse length
        PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;    // and restart timer
    }
}

// This interrupt is enabled when Grbl sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
void STEPPULSE_0_IRQHandler (void)
{
    set_step_outputs((axes_signals_t){0});
    PULSE_TIMER->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    PULSE_TIMER->CTL &= ~(TIMER_A_CTL_MC0|TIMER_A_CTL_MC1); // Disable Timer0 to prevent re-entering this interrupt when it's not needed.
}

void DEBOUNCE_IRQHandler (void)
{
    DEBOUNCE_TIMER->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;            // Clear interrupt flag and
    DEBOUNCE_TIMER->CTL &= ~(TIMER_A_CTL_MC0|TIMER_A_CTL_MC1);  // stop debounce timer

    limit_signals_t state = limitsGetState();

    if(limit_signals_merge(state).mask)
        hal.limits.interrupt_callback(state);
}

void RPMCOUNTER_IRQHandler (void)
{
    spindle_encoder.spin_lock = true;

    __disable_irq();
    uint32_t tval = RPM_TIMER->VALUE;
    uint16_t cval = RPM_COUNTER->R;
    __enable_irq();

    RPM_COUNTER->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;
    RPM_COUNTER->CCR[0] += spindle_encoder.tics_per_irq;

    spindle_encoder.counter.pulse_count += (uint16_t)(cval - (uint16_t)spindle_encoder.counter.last_count);
    spindle_encoder.counter.last_count = cval;
    spindle_encoder.timer.pulse_length = spindle_encoder.timer.last_pulse - tval;
    spindle_encoder.timer.last_pulse = tval;

    spindle_encoder.spin_lock = false;
}

#if CNC_BOOSTERPACK_SHORTS

// Shared with spindle encoder index interrupt
void CONTROL_IRQHandler (void)
{
    uint8_t iflags = CONTROL_PORT->IFG;

    CONTROL_PORT->IFG &= ~iflags;

    if(iflags & RPM_INDEX_BIT) {

        if(spindle_encoder.index_count && (uint16_t)(RPM_COUNTER->R - (uint16_t)spindle_encoder.counter.last_index) != spindle_encoder.ppr)
            spindle_encoder.error_count++;

        spindle_encoder.timer.last_index = RPM_TIMER->VALUE;
        spindle_encoder.counter.last_index = RPM_COUNTER->R;
        spindle_encoder.counter.index_count++;
    }

    if(iflags & CONTROL_MASK) {
        CONTROL_PORT->IFG &= ~iflags;
        hal.control.interrupt_callback(systemGetState());
    }
}

void LIMIT_IRQHandler (void)
{
    uint32_t iflags = LIMIT_PORT->IFG;

    LIMIT_PORT->IFG &= ~iflags;

    if(iflags & LIMIT_MASK) {
        if(hal.driver_cap.software_debounce)
            DEBOUNCE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC0;
        else
            hal.limits.interrupt_callback(limitsGetState());
    }
}
/*
void PROBE_IRQHandler (void)
{
    static uint32_t ms = 0;

    BITBAND_PERI(PROBE_PORT->IFG, PROBE_PIN) = 0;

    if(!probe.triggered) {
        probe.triggered = On;
        if(!probe.is_probing) {
            if(probe.connected && elapsed_tics > ms)
                hal.control_interrupt_callback((control_signals_t){ .probe_triggered = On });
        } else
            ms = elapsed_tics + 300;
    }
}
*/
#if MPG_MODE_ENABLE

void MODE_IRQHandler (void)
{
    uint8_t iflags = MODE_PORT->IFG & MODE_SWITCH_BIT;

    if(iflags) {
        MODE_PORT->IFG &= ~iflags;
        if(delay.ms == 0) // Ignore if delay is active
            driver_delay_ms(50, modeChange);
    }
}

#endif

#else

void LIMIT_X_IRQHandler (void)
{
    uint32_t iflags = LIMIT_PORT_X->IFG;

    LIMIT_PORT_X->IFG = 0;

    if(iflags & LIMIT_MASK_X) {
        if(hal.driver_cap.software_debounce)
            DEBOUNCE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC0;
        else
            hal.limits.interrupt_callback(limitsGetState());
    }
}

// NOTE: this also handles the control Reset irq
void LIMIT_YZ_RST_IRQHandler (void)
{
    uint32_t iflags = LIMIT_PORT_Y->IFG;

    LIMIT_PORT_Y->IFG = 0;

    if(iflags & LIMIT_MASK_YZ) {
        if(hal.driver_cap.software_debounce)
            DEBOUNCE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC0;
        else
            hal.limits.interrupt_callback(limitsGetState());
    }

    if(iflags & RESET_BIT)
        hal.control.interrupt_callback(systemGetState());
}

void CONTROL_FH_CS_IRQHandler (void)
{
    uint8_t iflags = CONTROL_PORT_FH->IFG;

    CONTROL_PORT_FH->IFG = 0;

    if(iflags & RPM_INDEX_BIT) {
        spindle_encoder.timer.last_index = RPM_TIMER->VALUE;
        spindle_encoder.counter.last_index = RPM_COUNTER->R;
        spindle_encoder.counter.index_count++;
    }

    if(iflags & (FEED_HOLD_BIT|CYCLE_START_BIT))
        hal.control.interrupt_callback(systemGetState());
}

void CONTROL_SD_MODE_Handler (void)
{
    uint8_t iflags = CONTROL_PORT_SD->IFG;

    CONTROL_PORT_SD->IFG = 0;

  #if MPG_MODE_ENABLE
    if(iflags & MODE_SWITCH_BIT) {
        if(delay.ms == 0) // Ignore if delay is active
            driver_delay_ms(50, modeChange);
    } else
  #endif
  #if TRINAMIC_I2C
    if(iflags & TRINAMIC_WARN_IRQ_BIT)
        trinamic_warn_handler();
    else
  #endif
    if(iflags & SAFETY_DOOR_BIT)
        hal.control.interrupt_callback(systemGetState());
}
#endif

#if KEYPAD_ENABLE

void KEYPAD_IRQHandler (void)
{
    uint8_t iflags = KEYPAD_PORT->IFG;

    KEYPAD_PORT->IFG &= ~iflags;

#elif TRINAMIC_ENABLE && TRINAMIC_I2C

void TRINAMIC_DIAG_IRQHandler (void)
{
    uint8_t iflags = TRINAMIC_DIAG_IRQ_PORT->IFG;

    TRINAMIC_DIAG_IRQ_PORT->IFG &= ~iflags;
#endif

#if TRINAMIC_ENABLE && TRINAMIC_I2C
    if(iflags & TRINAMIC_DIAG_IRQ_BIT)
        trinamic_fault_handler();
#endif

#if KEYPAD_ENABLE
    if(iflags & KEYPAD_IRQ_BIT) {
        iflags = (KEYPAD_PORT->IN & KEYPAD_IRQ_BIT) ? 1 : 0;
        BITBAND_PERI(KEYPAD_PORT->IES, KEYPAD_IRQ_PIN) = iflags;
        keypad_keyclick_handler(!iflags);
    }
#endif

#if KEYPAD_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)
}
#endif

// Interrupt handler for 1 ms interval timer
void SysTick_Handler (void)
{
    elapsed_tics++;

#if defined(SPINDLE_RPM_CONTROLLED)

    static uint32_t spid = SPINDLE_PID_SAMPLE_RATE, tpp = 0;

    switch(spindle_control.pid_state) {

        case PIDState_Pending:
            if(pid_count == 0) {
                tpp = 0;
                spid = SPINDLE_PID_SAMPLE_RATE;
            }

            if(pid_count < 500)
                pid_count++;
            else if(spindle_data.index_count > 2)
                spindle_control.pid_state = PIDState_Active;

            tpp += spindle_encoder.timer.pulse_length / spindle_encoder.counter.tics_per_irq;

            if(--spid == 0) {
                spindle_control.rpm = spindle_calc_rpm(tpp / SPINDLE_PID_SAMPLE_RATE);
                tpp = 0;
                spid = SPINDLE_PID_SAMPLE_RATE;
            }
            break;

        case PIDState_Active:
            tpp += spindle_encoder.timer.pulse_length / spindle_encoder.counter.tics_per_irq;
            if(--spid == 0) {
                spindle_rpm_pid(tpp / SPINDLE_PID_SAMPLE_RATE);
                tpp = 0;
                spid = SPINDLE_PID_SAMPLE_RATE;
            }
            break;
    }

#endif

    if(delay.ms && !(--delay.ms) && delay.callback) {
        delay.callback();
        delay.callback = NULL;
    }
}
