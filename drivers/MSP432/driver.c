/*

  driver.c - driver code for Texas Instruments MSP432 ARM processor

  Part of Grbl

  Copyright (c) 2017-2019 Terje Io

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
#include "serial.h"

#if EEPROM_ENABLE
#include "eeprom.h"
#endif

#if KEYPAD_ENABLE
#include "keypad.h"
#endif

#if ATC_ENABLE
#include "atc.h"
#endif

typedef enum {
    PIDState_Disabled = 0,
    PIDState_Pending,
    PIDState_Active,
} pid_state_t;

typedef struct {
    pid_values_t cfg;
    float deadband;
    float i_error;
    float d_error;
    float sample_rate_prev;
    float error;
    float max_error;
    bool enabled;
} pid_t;

typedef struct {
    uint32_t ppr;                           // Encoder pulses per revolution
    float rpm_factor;
    float pulse_distance;                   // Encoder pulse distance in fraction of one revolution
    float timer_resolution;                 // Timer resolution (seconds per tick)
    volatile uint32_t timer_value_index;    // Timer value at last encoder index pulse
    volatile uint32_t timer_value_last;     // Timer value at last encoder pulse
    volatile uint32_t timer_value_step;     // Timer value at last stepper timeout TODO: not needed?
    volatile uint32_t tpp;                  // Last timer tics per spindle encoder pulse
    uint32_t maximum_tt;                    // Maximum timer tics since last spindle encoder pulse before RPM = 0 is returned
    uint32_t pulse_counter_trigger;         // Number of encoder pulses per interrupt generated
    volatile uint16_t pulse_counter_last;   // Encoder pulse counter at last trigger
    volatile uint16_t pulse_counter_index;  // Encoder pulse counter at last index pulse
    bool error;                             // Set when last encoder pulse count did not match at last index
    pid_state_t pid_state;
    pid_t pid;                              // PID data for RPM
} spindle_encoder_t;

typedef struct {
    float block_start;              // Spindle position at start of move (number of revolutions)
    float prev_pos;                 // Target position of previous segment
    float steps_per_mm;             // Steps per mm for current block
    float programmed_rate;          // Programmed feed in mm/rev for current block
    uint32_t min_cycles_per_tick;   // Minimum cycles per tick for PID loop
    void (*stepper_pulse_start_normal)(stepper_t *stepper);
    uint_fast8_t segment_id;        // Used for detecing start of new segment
    pid_t pid;                      // PID data for position
#ifdef PID_LOG
    int32_t log[PID_LOG];
    int32_t pos[PID_LOG];
#endif
} spindle_sync_t;

static volatile uint32_t pid_count = 0;
static volatile bool spindleLock = false;
static bool pwmEnabled = false, IOInitDone = false;
// Inverts the probe pin state depending on user settings and probing cycle mode.
static uint8_t probe_invert;
static axes_signals_t next_step_outbits;
static spindle_pwm_t spindle_pwm;
static spindle_data_t spindle_data;
static spindle_encoder_t spindle_encoder;
static spindle_sync_t spindle_tracker;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup

static void stepperPulseStartSynchronized (stepper_t *stepper);
static uint_fast16_t spindle_set_speed (uint_fast16_t pwm_value);
static void spindleDataReset (void);
static spindle_data_t spindleGetData (spindle_data_request_t request);

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

// LinuxCNC example settings
// MAX_OUTPUT = 300 DEADBAND = 0.0 P = 3 I = 1.0 D = 0.1 FF0 = 0.0 FF1 = 0.1 FF2 = 0.0 BIAS = 0.0 MAXI = 20.0 MAXD = 20.0 MAXERROR = 250.0
//
// You will always get oscillation on a PID system if you increase any P,I,D term too high
// I would try using less P (say 2) and then see how high an I term you can have and stay stable
// D term should not be needed

inline static float pid (pid_t *pid, float command, float actual, float sample_rate)
{
    float error = command - actual;
/*
    if(error > pid->deadband)
        error -= pid->deadband;
    else if (error < pid->deadband)
        error += pid->deadband;
    else
        error = 0.0f;
*/
    // calculate the proportional term
    float pidres = pid->cfg.d_gain * error;

    // calculate and add the integral term
    pid->i_error += error * (pid->sample_rate_prev / sample_rate);

    if(pid->cfg.i_max_error != 0.0f) {
        if (pid->i_error > pid->cfg.i_max_error)
            pid->i_error = pid->cfg.i_max_error;
        else if (pid->i_error < -pid->cfg.i_max_error)
            pid->i_error = -pid->cfg.i_max_error;
    }

    pidres += pid->cfg.i_gain * pid->i_error;

    // calculate and add the derivative term
    if(pid->cfg.d_gain != 0.0f) {
        float p_error = (error - pid->d_error) * (sample_rate / pid->sample_rate_prev);
        if(pid->cfg.d_max_error != 0.0f) {
            if (p_error > pid->cfg.d_max_error)
                p_error = pid->cfg.d_max_error;
            else if (p_error < -pid->cfg.d_max_error)
                p_error = -pid->cfg.d_max_error;
        }
        pidres += pid->cfg.d_gain * p_error;
        pid->d_error = error;
    }

    pid->sample_rate_prev = sample_rate;

    // limit error output
    if(pid->cfg.max_error != 0.0f) {
        if(pidres > pid->cfg.max_error)
            pidres = pid->cfg.max_error;
        else if(pidres < -pid->cfg.max_error)
            pidres = -pid->cfg.max_error;
    }

    pid->error = pidres;

    return pidres;
}

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if((delay.ms = ms) > 0) {
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        if(!(delay.callback = callback))
            while(delay.ms);
    } else if(delay.callback)
        delay.callback();
}

// Set stepper pulse output pins
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z...
// Mapping to registers can be done by
// 1. bitbanding. Pros: can assign pins to different ports, no RMW needed. Cons: overhead, pin changes not synchronous
// 2. bit shift. Pros: fast, Cons: bits must be consecutive
// 3. lookup table. Pros: signal inversions done at setup, Cons: slower than bit shift
inline static void set_step_outputs (axes_signals_t step_outbits)
{
#if STEP_OUTMODE == GPIO_BITBAND
    step_outbits.value ^= settings.steppers.step_invert.mask;
    BITBAND_PERI(STEP_PORT->OUT, X_STEP_PIN) = step_outbits.x;
    BITBAND_PERI(STEP_PORT->OUT, Y_STEP_PIN) = step_outbits.y;
    BITBAND_PERI(STEP_PORT->OUT, Z_STEP_PIN) = step_outbits.z;
#elif STEP_OUTMODE == GPIO_MAP
    STEP_PORT->OUT = (STEP_PORT->IN & ~STEP_MASK) | step_outmap[step_outbits.value];
#else
    STEP_PORT->OUT = (STEP_PORT->IN & ~STEP_MASK) | ((step_outbits.value << STEP_OUTMODE) ^ settings.steppers.step_invert.mask);
#endif
}

// Set stepper direction output pins
// NOTE: see note for set_step_outputs()
inline static void set_dir_outputs (axes_signals_t dir_outbits)
{
#if DIRECTION_OUTMODE == GPIO_BITBAND
    dir_outbits.value ^= settings.steppers.dir_invert.mask;
    BITBAND_PERI(DIRECTION_PORT->OUT, X_DIRECTION_PIN) = dir_outbits.x;
    BITBAND_PERI(DIRECTION_PORT->OUT, Y_DIRECTION_PIN) = dir_outbits.y;
    BITBAND_PERI(DIRECTION_PORT->OUT, Z_DIRECTION_PIN) = dir_outbits.z;
#elif DIRECTION_OUTMODE == GPIO_MAP
    DIRECTION_PORT->OUT = (DIRECTION_PORT->IN & ~DIRECTION_MASK) | dir_outmap[dir_outbits.value];
#else
    DIRECTION_PORT->OUT = (DIRECTION_PORT->IN & ~DIRECTION_MASK) | ((dir_outbits.value << DIRECTION_OUTMODE) ^ settings.steppers.dir_invert.mask);
#endif
}

// Enable/disable stepper motors
static void stepperEnable (axes_signals_t enable)
{
    enable.value ^= settings.steppers.enable_invert.mask;
    BITBAND_PERI(STEPPERS_DISABLE_Z_PORT->OUT, STEPPERS_DISABLE_Z_PIN) = enable.z;
    BITBAND_PERI(STEPPERS_DISABLE_XY_PORT->OUT, STEPPERS_DISABLE_X_PIN) = enable.x;
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    stepperEnable((axes_signals_t){AXES_BITMASK});
    STEPPER_TIMER->LOAD = 0x000FFFFFUL;
    STEPPER_TIMER->CONTROL |= TIMER32_CONTROL_ENABLE|TIMER32_CONTROL_IE;
    spindle_tracker.segment_id = 0;
    hal.stepper_interrupt_callback();   // start the show
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals) {
    STEPPER_TIMER->CONTROL &= ~(TIMER32_CONTROL_ENABLE|TIMER32_CONTROL_IE);
    if(clear_signals) {
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}

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
            spindle_tracker.stepper_pulse_start_normal = hal.stepper_pulse_start;
            hal.stepper_pulse_start = stepperPulseStartSynchronized;
            stepperPulseStartSynchronized(stepper);
            return;
        }
        stepper->new_block = false;
        set_dir_outputs(stepper->dir_outbits);
    }

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;
    } else spindle_tracker.segment_id++;
}

// Delayed pulse version: sets stepper direction and pulse pins and starts a step pulse with an initial delay.
// If spindle synchronized motion switch to PID version.
// TODO: only delay after setting dir outputs?
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->new_block) {
        if(stepper->exec_segment->spindle_sync) {
            spindle_tracker.stepper_pulse_start_normal = hal.stepper_pulse_start;
            hal.stepper_pulse_start = stepperPulseStartSynchronized;
            stepperPulseStartSynchronized(stepper);
            return;
        }
        stepper->new_block = false;
        set_dir_outputs(stepper->dir_outbits);
    }

    if(stepper->step_outbits.value) {
        next_step_outbits = stepper->step_outbits; // Store out_bits
        PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;
    }
}

// Spindle sync version: sets stepper direction and pulse pins and starts a step pulse.
// Switches back to "normal" version if spindle synchronized motion is finished.
// TODO: add delayed pulse handling...
static void stepperPulseStartSynchronized (stepper_t *stepper)
{
    if(stepper->new_block) {
        if(!stepper->exec_segment->spindle_sync) {
            hal.stepper_pulse_start = spindle_tracker.stepper_pulse_start_normal;
            hal.stepper_pulse_start(stepper);
            return;
        } else {
            spindle_tracker.block_start = spindleGetData(SpindleData_AngularPosition).angular_position;
            spindle_tracker.programmed_rate = stepper->exec_block->programmed_rate;
            spindle_tracker.steps_per_mm = stepper->exec_block->steps_per_mm;
            spindle_tracker.segment_id = 0;
            spindle_tracker.prev_pos = 0.0f;
            spindle_tracker.pid.i_error = 0.0f;
            spindle_tracker.pid.d_error = 0.0f;
            spindle_tracker.pid.sample_rate_prev = 0.0f;
#ifdef PID_LOG
            sys.pid_log.idx = 0;
            sys.pid_log.setpoint = 100.0f;
#endif
        }
        set_dir_outputs(stepper->dir_outbits);
    }

    if(stepper->step_outbits.value) {
        if(settings.steppers.pulse_delay_microseconds)
            next_step_outbits = stepper->step_outbits; // Store out_bits;
        else
            set_step_outputs(stepper->step_outbits);
        PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;
    }

    if(spindle_tracker.segment_id != stepper->exec_segment->id && stepper->step_count) {

        spindle_tracker.segment_id = stepper->exec_segment->id;

        if(stepper->new_block)
            stepper->new_block = false;

        else {  // adjust this segments total time for any positional error since last segment

            float dt = (float)hal.f_step_timer / (float)(stepper->exec_segment->cycles_per_tick * stepper->exec_segment->n_step);
            float actual_pos = (spindleGetData(SpindleData_AngularPosition).angular_position - spindle_tracker.block_start) * spindle_tracker.programmed_rate;
            int32_t step_delta = (int32_t)(pid(&spindle_tracker.pid, spindle_tracker.prev_pos, actual_pos, dt) * spindle_tracker.steps_per_mm);
#ifdef PID_LOG
            if(sys.pid_log.idx < PID_LOG) {

                sys.pid_log.target[sys.pid_log.idx] = spindle_tracker.prev_pos;
                sys.pid_log.actual[sys.pid_log.idx] = actual_pos;

                spindle_tracker.log[sys.pid_log.idx] = STEPPER_TIMER->LOAD;
             //   spindle_tracker.pos[sys.pid_log.idx] = spindle_tracker.prev_pos;

                sys.pid_log.idx++;
            }
#endif
//            if(stepper->exec_segment->cruising) {

                int32_t ticks = (((int32_t)stepper->step_count + step_delta) * (int32_t)stepper->exec_segment->cycles_per_tick) / (int32_t)stepper->step_count;

                stepper->exec_segment->cycles_per_tick = (uint32_t)max(ticks, (int32_t)spindle_tracker.min_cycles_per_tick);

                stepperCyclesPerTick(stepper->exec_segment->cycles_per_tick);
//            }
#ifdef PID_LOG
            spindle_tracker.pos[sys.pid_log.idx] = STEPPER_TIMER->LOAD;
#endif
        }

        spindle_tracker.prev_pos = stepper->exec_segment->target_position;
    }
}

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing) {
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
}


// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static axes_signals_t limitsGetState()
{
    axes_signals_t signals = {0};
#if CNC_BOOSTERPACK_SHORTS
 #if defined(LIMIT_INMODE) &&LIMIT_INMODE == LIMIT_SHIFT
    signals.value = (uint8_t)(LIMIT_PORT->IN & LIMIT_MASK) >> LIMIT_SHIFT;
 #elif LIMIT_INMODE == GPIO_BITBAND
    signals.x = BITBAND_PERI(LIMIT_PORT, X_LIMIT_PIN);
    signals.y = BITBAND_PERI(LIMIT_PORT, Y_LIMIT_PIN);
    signals.z = BITBAND_PERI(LIMIT_PORT, Z_LIMIT_PIN);
 #else // masked
    uint8_t bits = LIMIT_PORT->IN;
    signals.x = (bits & X_LIMIT_BIT) != 0;
    signals.y = (bits & Y_LIMIT_BIT) != 0;
    signals.z = (bits & Z_LIMIT_BIT) != 0;
 #endif
#else
 #if LIMIT_INMODE == LIMIT_SHIFT
    signals.value = (uint8_t)(LIMIT_PORT->IN & LIMIT_MASK) >> LIMIT_SHIFT;
 #elif LIMIT_INMODE == GPIO_BITBAND
    signals.x = BITBAND_PERI(LIMIT_PORT_X->IN, X_LIMIT_PIN);
    signals.y = BITBAND_PERI(LIMIT_PORT_Y->IN, Y_LIMIT_PIN);
    signals.z = BITBAND_PERI(LIMIT_PORT_Z->IN, Z_LIMIT_PIN);
 #else
    uint8_t bits = LIMIT_PORT->IN;
    signals.x = (bits & X_LIMIT_PIN) != 0;
    signals.y = (bits & Y_LIMIT_PIN) != 0;
    signals.z = (bits & Z_LIMIT_PIN) != 0;
 #endif
#endif

    if (settings.limits.invert.mask)
        signals.value ^= settings.limits.invert.mask;

#if LIMITS_OVERRIDE_ENABLE
    if(!BITBAND_PERI(LIMITS_OVERRIDE_PORT->IN, LIMITS_OVERRIDE_SWITCH_PIN))
        signals.value = 0;
#endif

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
static control_signals_t systemGetState (void)
{
    control_signals_t signals = {0};
#if CNC_BOOSTERPACK_SHORTS
  #if CONTROL_INMODE == GPIO_BITBAND
    signals.e_stop = BITBAND_PERI(CONTROL_PORT->IN, RESET_PIN);
    signals.safety_door_ajar = BITBAND_PERI(CONTROL_PORT->IN, SAFETY_DOOR_PIN);
    signals.feed_hold = BITBAND_PERI(CONTROL_PORT->IN, FEED_HOLD_PIN);
    signals.cycle_start = BITBAND_PERI(CONTROL_PORT->IN, CYCLE_START_PIN);
  #else
    uint8_t bits = CONTROL_PORT->IN;
    signals.e_stop = (bits & RESET_BIT) != 0;
    signals.safety_door_ajar = (bits & SAFETY_DOOR_BIT) != 0;
    signals.feed_hold = (bits & FEED_HOLD_BIT) != 0;
    signals.cycle_start = (bits & CYCLE_START_BIT) != 0;
  #endif
#else
#if CONTROL_INMODE == GPIO_BITBAND
    signals.e_stop = BITBAND_PERI(CONTROL_PORT_RST->IN, RESET_PIN);
    signals.safety_door_ajar = BITBAND_PERI(CONTROL_PORT_SD->IN, SAFETY_DOOR_PIN);
    signals.feed_hold = BITBAND_PERI(CONTROL_PORT_FH->IN, FEED_HOLD_PIN);
    signals.cycle_start = BITBAND_PERI(CONTROL_PORT_CS->IN, CYCLE_START_PIN);
#else
    uint8_t bits = CONTROL_PORT->IN;
    signals.e_stop = (bits & RESET_BIT) != 0;
    signals.safety_door_ajar = (bits & SAFETY_DOOR_BIT) != 0;
    signals.feed_hold = (bits & FEED_HOLD_BIT) != 0;
    signals.cycle_start = (bits & CYCLE_START_BIT) != 0;
#endif
#endif

    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

//    signals.safety_door_ajar = Off; // for now - annoying that this blocks config

    return signals;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigureInvertMask (bool is_probe_away)
{
  probe_invert = settings.flags.invert_probe_pin ? 0 : PROBE_BIT;

  if (is_probe_away)
      probe_invert ^= PROBE_BIT;
}

// Returns the probe pin state. Triggered = true.
bool probeGetState (void)
{
    return (PROBE_PORT->IN & PROBE_BIT) ^ probe_invert != 0;
}

// Static spindle (off, on cw & on ccw)

inline static bool spindle_is_ccw (void)
{
    return hal.driver_cap.spindle_dir && (BITBAND_PERI(SPINDLE_DIRECTION_PORT->OUT, SPINDLE_DIRECTION_PIN) ^ settings.spindle.invert.ccw);
}

inline static void spindle_dir (bool ccw)
{
    if(hal.driver_cap.spindle_dir)
        BITBAND_PERI(SPINDLE_DIRECTION_PORT->OUT, SPINDLE_DIRECTION_PIN) = ccw ^ settings.spindle.invert.ccw;
}

inline static void spindle_off (void)
{
    BITBAND_PERI(SPINDLE_ENABLE_PORT->OUT, SPINDLE_ENABLE_PIN) = settings.spindle.invert.on;
    spindle_dir(false);
}

inline static void spindle_on (void)
{
    if(settings.spindle.invert.mutex && !spindle_is_ccw())
        BITBAND_PERI(SPINDLE_ENABLE_PORT->OUT, SPINDLE_ENABLE_PIN) = !settings.spindle.invert.on;
    spindleDataReset();
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
static uint_fast16_t spindle_set_speed (uint_fast16_t pwm_value)
{
    while(spindleLock); // wait for PID

    if (pwm_value == spindle_pwm.off_value) {
        pwmEnabled = false;
        if(settings.spindle.disable_with_zero_speed)
            spindle_off();
        SPINDLE_PWM_TIMER->CCTL[2] = settings.spindle.invert.pwm ? TIMER_A_CCTLN_OUT : 0; // Set PWM output according to invert setting
        spindle_encoder.pid.error = 0.0f;
    } else {
        if(!pwmEnabled) {
            spindle_dir(spindle_data.state_programmed.ccw);
            spindle_on();
        }
        pwmEnabled = true;
        SPINDLE_PWM_TIMER->CCR[2] = pwm_value;
        SPINDLE_PWM_TIMER->CCTL[2] = TIMER_A_CCTLN_OUTMOD_2;
    }

    return pwm_value;
}

static void spindleUpdateRPM (float rpm)
{
    while(spindleLock); // wait for PID

    spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm + spindle_encoder.pid.error, spindle_encoder.pid.error != 0.0f));
    spindle_data.rpm_low_limit = rpm / 1.1f;
    spindle_data.rpm_high_limit = rpm * 1.1f;
    spindle_data.rpm_programmed = spindle_data.rpm = rpm;
}

// Start or stop spindle
static void spindleSetStateVariable (spindle_state_t state, float rpm)
{
    spindle_data.rpm_low_limit = rpm / 1.1f;
    spindle_data.rpm_high_limit = rpm * 1.1f;
    spindle_data.rpm_programmed = spindle_data.rpm = rpm;
    spindle_data.state_programmed = state;

    if (!state.on || rpm == 0.0f) {
        spindle_off();
        spindle_set_speed(spindle_pwm.off_value);
        if(delay.ms == 0)
            SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        spindle_encoder.pid_state = PIDState_Disabled;
        spindle_encoder.pid.error = 0.0f;
        spindle_encoder.pid.i_error = 0.0f;
        spindle_encoder.pid.d_error = 0.0f;
        spindle_encoder.pid.sample_rate_prev = 1.0f;
    } else {
        if(spindle_data.rpm_programmed == 0.0f) {
            if(spindle_encoder.pid.enabled) {
                pid_count = 0;
                spindle_encoder.pid_state = PIDState_Pending;
                SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
            }
        }
#ifdef sPID_LOG
        sys.pid_log.idx = 0;
#endif
        spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm + spindle_encoder.pid.error, spindle_encoder.pid.error != 0.0f));
    }
}

inline static float spindle_calc_rpm (void)
{
    return 1.0f / ((float)spindle_encoder.tpp * spindle_encoder.rpm_factor);
}

static spindle_data_t spindleGetData (spindle_data_request_t request)
{
    bool stopped;

    uint32_t rpm_timer_delta = spindle_encoder.timer_value_last - RPM_TIMER->VALUE; // NOTE: timer is counting down!

    // If no (4) spindle pulses during last 250mS assume RPM is 0
    if((stopped = ((spindle_encoder.tpp == 0) || (rpm_timer_delta > spindle_encoder.maximum_tt)))) {
        spindle_data.rpm = 0.0f;
        rpm_timer_delta = (RPM_COUNTER->R - spindle_encoder.pulse_counter_last) * spindle_encoder.tpp;
    }

    switch(request) {

        case SpindleData_Counters:
            spindle_data.pulse_count += (RPM_COUNTER->R - spindle_encoder.pulse_counter_last);
            break;

        case SpindleData_RPM:
            if(!stopped)
                spindle_data.rpm = spindle_calc_rpm();
            break;

        case SpindleData_AngularPosition:
            spindle_data.angular_position = (float)spindle_data.index_count +
                    ((float)(spindle_encoder.pulse_counter_last - spindle_encoder.pulse_counter_index) +
                              (spindle_encoder.tpp == 0 ? 0.0f : (float)rpm_timer_delta / (float)spindle_encoder.tpp)) *
                                spindle_encoder.pulse_distance;
            break;
    }

    return spindle_data;
}

inline static void spindle_rpm_pid (void)
{
    spindleLock = true;

    float rpm = spindle_calc_rpm();
    float error = pid(&spindle_encoder.pid, spindle_data.rpm_programmed, rpm, 1.0);

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

static void spindleDataReset (void)
{
    while(spindleLock);

    uint32_t systick_state = SysTick->CTRL;

    SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;

    uint32_t index_count = spindle_data.index_count + 2;

//    if(spindleGetData(SpindleData_RPM).rpm > 0.0f) // wait for index pulse if running
//        while(index_count != spindle_data.index_count);
    spindle_encoder.pid_state = PIDState_Pending;

    RPM_TIMER->LOAD = 0; // Reload RPM timer
    RPM_COUNTER->CTL = 0;

    spindle_encoder.timer_value_index = RPM_TIMER->VALUE;
    spindle_encoder.pulse_counter_index = 0;
    spindle_encoder.pulse_counter_last = 0;
    spindle_encoder.tpp = 0;
    spindle_data.pulse_count = 0;
    spindle_data.index_count = 0;
    RPM_COUNTER->CCR[0] = spindle_encoder.pulse_counter_trigger;
    RPM_COUNTER->CTL = TIMER_A_CTL_MC__CONTINUOUS|TIMER_A_CTL_CLR;

    if(systick_state & SysTick_CTRL_ENABLE_Msk)
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {0};

    state.on = BITBAND_PERI(SPINDLE_ENABLE_PORT->IN, SPINDLE_ENABLE_PIN) ^ settings.spindle.invert.on;
    state.ccw = spindle_is_ccw();

    if(settings.spindle.invert.mutex && hal.driver_cap.spindle_dir)
        state.on = state.ccw;

    if(hal.driver_cap.spindle_at_speed) {
        float rpm = spindleGetData(SpindleData_RPM).rpm;
        state.at_speed = rpm >= spindle_data.rpm_low_limit && rpm <= spindle_data.rpm_high_limit;
    }

    return state;
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

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_interrupts();
    *ptr |= bits;
    __enable_interrupts();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    __disable_interrupts();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    __enable_interrupts();
    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    __disable_interrupts();
    uint_fast16_t prev = *ptr;
    *ptr = value;
    __enable_interrupts();
    return prev;
}

static void showMessage (const char *msg)
{
//    if(sys.mpg_mode) {
        hal.stream.write("[MSG:");
        hal.stream.write(msg);
        hal.stream.write("]\r\n");
//    }
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

    BITBAND_PERI(MODE_PORT->OUT, MODE_LED_PIN) = mpg_mode;

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

static void modechange (void)
{
    modeSelect((MODE_PORT->IN & MODE_SWITCH_BIT) == 0);
}

#endif

// Configure perhipherals when settings are initialized or changed
void settings_changed (settings_t *settings)
{
    if((hal.driver_cap.variable_spindle = settings->spindle.rpm_min < settings->spindle.rpm_max)) {
        if(settings->spindle.pwm_freq > 200.0f)
            SPINDLE_PWM_TIMER->CTL &= ~TIMER_A_CTL_ID__8;
        else
            SPINDLE_PWM_TIMER->CTL |= TIMER_A_CTL_ID__8;

        spindle_precompute_pwm_values(&spindle_pwm, 12000000UL / (settings->spindle.pwm_freq > 200.0f ? 2 : 8));
    }

    hal.driver_cap.spindle_at_speed = hal.driver_cap.variable_spindle && settings->spindle.ppr > 0;
    hal.spindle_set_state = hal.driver_cap.variable_spindle ? spindleSetStateVariable : spindleSetState;

    if((hal.spindle_get_data = hal.driver_cap.spindle_at_speed ? spindleGetData : NULL)) {
        memcpy(&spindle_tracker.pid.cfg, &settings->position.pid, sizeof(pid_values_t));
        spindle_tracker.min_cycles_per_tick = hal.f_step_timer / 1000000UL * 20UL; // 20uS = 50KHz
    }

    if((spindle_encoder.pid.enabled = hal.spindle_get_data && settings->spindle.pid.p_gain != 0.0f)) {
        memcpy(&spindle_encoder.pid.cfg, &settings->spindle.pid, sizeof(pid_values_t));
        spindle_encoder.pid.cfg.i_max_error = spindle_encoder.pid.cfg.i_max_error / settings->spindle.pid.i_gain; // Makes max value sensible?
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
    }

    if(hal.spindle_get_data && spindle_encoder.ppr != settings->spindle.ppr) {
        spindle_encoder.ppr = settings->spindle.ppr;
        spindle_encoder.pulse_counter_trigger = 4;
        spindle_encoder.pulse_distance = 1.0f / spindle_encoder.ppr;
        spindle_encoder.tpp = 0;
        spindle_encoder.timer_resolution = 1.0f / (float)(SystemCoreClock / 16);
        spindle_encoder.maximum_tt = (uint32_t)(0.25f / spindle_encoder.timer_resolution) * spindle_encoder.pulse_counter_trigger; // 250 mS
        spindle_encoder.rpm_factor = (spindle_encoder.timer_resolution * (float)spindle_encoder.ppr) / 60.0f;
        NVIC_EnableIRQ(RPM_INDEX_INT);
        spindleDataReset();
        //        spindle_data.rpm = 60.0f / ((float)(spindle_encoder.tpp * spindle_encoder.ppr) * spindle_encoder.timer_resolution); // TODO: get rid of division
    }

    if(!hal.spindle_get_data)
        NVIC_DisableIRQ(RPM_INDEX_INT);

#if (STEP_OUTMODE == GPIO_MAP) || (DIRECTION_OUTMODE == GPIO_MAP)
    uint_fast8_t i;
#endif

#if STEP_OUTMODE == GPIO_MAP
    for(i = 0; i < sizeof(step_outmap); i++)
        step_outmap[i] = c_step_outmap[i] ^ c_step_outmap[settings->steppers.step_invert.mask];
#endif

#if DIRECTION_OUTMODE == GPIO_MAP
    for(i = 0; i < sizeof(dir_outmap); i++)
        dir_outmap[i] = c_dir_outmap[i] ^ c_dir_outmap[settings->steppers.dir_invert.mask];
#endif

    if(IOInitDone) {

        stepperEnable(settings->steppers.deenergize);

        if(hal.driver_cap.variable_spindle) {
            SPINDLE_PWM_TIMER->CCR[0] = spindle_pwm.period;
            SPINDLE_PWM_TIMER->CCTL[2] = 0;           // Set PWM output low and
            SPINDLE_PWM_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC0|TIMER_A_CTL_MC1;  // start PWM timer (with no pulse output)
        }

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds) {
            hal.stepper_pulse_start = stepperPulseStartDelayed;
            PULSE_TIMER->CCTL[1] |= TIMER_A_CCTLN_CCIE;                   // Enable CCR1 interrupt
        } else {
            hal.stepper_pulse_start = stepperPulseStart;
            PULSE_TIMER->CCTL[1] &= ~TIMER_A_CCTLN_CCIE;                  // Disable CCR1 interrupt
        }

        PULSE_TIMER->CCR[0] = settings->steppers.pulse_microseconds + settings->steppers.pulse_delay_microseconds;
        PULSE_TIMER->CCR[1] = settings->steppers.pulse_delay_microseconds;

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

        BITBAND_PERI(CONTROL_PORT->OUT, RESET_PIN) = !settings->control_disable_pullup.reset;
        BITBAND_PERI(CONTROL_PORT->IES, RESET_PIN) = control_ies.reset;
        BITBAND_PERI(CONTROL_PORT->REN, RESET_PIN) = 1;

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

        BITBAND_PERI(CONTROL_PORT_RST->OUT, RESET_PIN) = !settings->control_disable_pullup.reset;
        BITBAND_PERI(CONTROL_PORT_RST->IES, RESET_PIN) = control_ies.reset;
        BITBAND_PERI(CONTROL_PORT_RST->REN, RESET_PIN) = 1;

        BITBAND_PERI(CONTROL_PORT_CS->IFG, CYCLE_START_PIN) = 0;
        BITBAND_PERI(CONTROL_PORT_FH->IFG, FEED_HOLD_PIN) = 0;
        BITBAND_PERI(CONTROL_PORT_SD->IFG, SAFETY_DOOR_PIN) = 0;
        BITBAND_PERI(CONTROL_PORT_RST->IFG, RESET_PIN) = 0;

        BITBAND_PERI(CONTROL_PORT_CS->IE, CYCLE_START_PIN) = 1;
        BITBAND_PERI(CONTROL_PORT_FH->IE, FEED_HOLD_PIN) = 1;
        BITBAND_PERI(CONTROL_PORT_SD->IE, SAFETY_DOOR_PIN) = 1;
        BITBAND_PERI(CONTROL_PORT_RST->IE, RESET_PIN) = 1;
#endif

#if MPG_MODE_ENABLE
        if(hal.driver_cap.mpg_mode) {
            BITBAND_PERI(MODE_PORT->OUT, MODE_SWITCH_PIN) = 1;
            BITBAND_PERI(MODE_PORT->REN, MODE_SWITCH_PIN) = 1;
            BITBAND_PERI(MODE_PORT->DIR, MODE_LED_PIN) = 1;
            if(sys.mpg_mode != !BITBAND_PERI(MODE_PORT->IN, MODE_SWITCH_PIN))
                modeSelect(true);
            BITBAND_PERI(MODE_PORT->IFG, MODE_SWITCH_PIN) = 0;
            BITBAND_PERI(MODE_PORT->IE, MODE_SWITCH_PIN) = 1;
        }
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
    }
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
  //    Interrupt_disableSleepOnIsrExit();

    FPU->FPCCR = (FPU->FPCCR & ~FPU_FPCCR_LSPEN_Msk) | FPU_FPCCR_ASPEN_Msk;  // enable lazy stacking

 // Stepper init

    STEP_PORT->DIR |= STEP_MASK;
    DIRECTION_PORT->DIR |= DIRECTION_MASK;
    STEPPERS_DISABLE_Z_PORT->DIR |= STEPPERS_DISABLE_Z_BIT;
    STEPPERS_DISABLE_XY_PORT->DIR |= STEPPERS_DISABLE_X_BIT;

#if CNC_BOOSTERPACK_A4998
    STEPPERS_VDD_PORT->DIR |= STEPPERS_VDD_BIT;
    STEPPERS_VDD_PORT->DS  |= STEPPERS_VDD_BIT;
    STEPPERS_VDD_PORT->OUT |= STEPPERS_VDD_BIT;
#endif

    STEPPER_TIMER->CONTROL = TIMER32_CONTROL_SIZE|TIMER32_CONTROL_MODE;

    PULSE_TIMER->EX0 = TIMER_A_EX0_IDEX__6; // -> SMCLK (12MHz) / 6 = 2MHz
    PULSE_TIMER->CTL = TIMER_A_CTL_SSEL__SMCLK|TIMER_A_CTL_ID__2|TIMER_A_CTL_CLR; // CLK: 2Mhz / 2 = 1uS
    PULSE_TIMER->CCTL[0] |= TIMER_A_CCTLN_CCIE;

    NVIC_EnableIRQ(STEPPER_TIMER_INT);  // Enable stepper interrupt and
    NVIC_EnableIRQ(PULSE_TIMER_INT0);   // step pulse interrupts
    NVIC_EnableIRQ(PULSE_TIMER_INTN);   // ...

    NVIC_SetPriority(PULSE_TIMER_INT0, 0x00);
    NVIC_SetPriority(PULSE_TIMER_INTN, 0x00);
    NVIC_SetPriority(STEPPER_TIMER_INT, 0x20);

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

    SPINDLE_ENABLE_PORT->DIR |= SPINDLE_ENABLE_BIT;
    if(hal.driver_cap.spindle_dir)
        SPINDLE_DIRECTION_PORT->DIR |= SPINDLE_DIRECTION_BIT; // Configure as output pin.

    if(hal.driver_cap.variable_spindle) {
        SPINDLE_PWM_PORT->DIR |= SPINDLE_PWM_BIT;
        SPINDLE_PWM_PORT->SEL1 &= ~SPINDLE_PWM_BIT;
        SPINDLE_PWM_PORT->SEL0 |= SPINDLE_PWM_BIT;
        SPINDLE_PWM_TIMER->CTL = TIMER_A_CTL_SSEL__SMCLK;
        SPINDLE_PWM_TIMER->EX0 = 0;
    }

    if(hal.spindle_index_callback || true) {
        RPM_INDEX_PORT->OUT |= RPM_INDEX_BIT;
        RPM_INDEX_PORT->REN |= RPM_INDEX_BIT;
        RPM_INDEX_PORT->IES |= RPM_INDEX_BIT;
        RPM_INDEX_PORT->IE |= RPM_INDEX_BIT;
    }

    memset(&spindle_encoder, 0, sizeof(spindle_encoder_t));
    memset(&spindle_tracker, 0, sizeof(spindle_sync_t));
    memset(&spindle_data, 0, sizeof(spindle_data));

    spindle_encoder.pulse_counter_trigger = 4;

    RPM_COUNTER_PORT->SEL0 |= RPM_COUNTER_BIT; // Set as counter input
    RPM_COUNTER->CTL = TIMER_A_CTL_MC__CONTINUOUS|TIMER_A_CTL_CLR;
    RPM_COUNTER->CCTL[0] = TIMER_A_CCTLN_CCIE;
    RPM_COUNTER->CCR[0] = spindle_encoder.pulse_counter_trigger;
    NVIC_EnableIRQ(RPM_COUNTER_INT0);   // Enable RPM timer interrupt

    spindle_encoder.timer_value_index = 0;
    RPM_TIMER->CONTROL = TIMER32_CONTROL_SIZE|TIMER32_CONTROL_ENABLE|TIMER32_CONTROL_PRESCALE_1; // rolls over after ~23 minutes

  // Coolant init

    COOLANT_FLOOD_PORT->DIR |= COOLANT_FLOOD_BIT;
    COOLANT_MIST_PORT->DIR |= COOLANT_MIST_BIT;

// Set defaults

    IOInitDone = settings->version == 14;

    settings_changed(settings);

    hal.stepper_go_idle(true);
    hal.spindle_set_state((spindle_state_t){0}, 0.0f);
    hal.coolant_set_state((coolant_state_t){0});

#if KEYPAD_ENABLE
    keypad_setup();
#endif

    return IOInitDone;
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done

bool driver_init (void)
{
    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

    SystemInit();

    CS->KEY = CS_KEY_VAL;                                  // Unlock CS module for register access
    CS->CTL1 |= CS_CTL1_DIVS__4;                           // Set SMCLK divider - 48Mhz / 4 = 12MHz
    CS->KEY = 0;

    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk;

    serialInit();

#if EEPROM_ENABLE
    eepromInit();
#endif
    hal.info = "MSP432";
    hal.driver_setup = driver_setup;
    hal.f_step_timer = SystemCoreClock;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

    hal.stepper_wake_up = stepperWakeUp;
    hal.stepper_go_idle = stepperGoIdle;
    hal.stepper_enable = stepperEnable;
    hal.stepper_cycles_per_tick = stepperCyclesPerTick;
    hal.stepper_pulse_start = stepperPulseStart;

    hal.limits_enable = limitsEnable;
    hal.limits_get_state = limitsGetState;

    hal.coolant_set_state = coolantSetState;
    hal.coolant_get_state = coolantGetState;

    hal.probe_get_state = probeGetState;
    hal.probe_configure_invert_mask = probeConfigureInvertMask;

    hal.spindle_set_state = spindleSetState;
    hal.spindle_get_state = spindleGetState;
    hal.spindle_update_rpm = spindleUpdateRPM;
    hal.spindle_reset_data = spindleDataReset;

    hal.system_control_get_state = systemGetState;

    hal.show_message = showMessage;

    hal.stream.read = serialGetC;
    hal.stream.get_rx_buffer_available = serialRxFree;
    hal.stream.reset_read_buffer = serialRxFlush;
    hal.stream.cancel_read_buffer = serialRxCancel;
    hal.stream.write = serialWriteS;
    hal.stream.write_all = serialWriteS;
    hal.stream.suspend_read = serialSuspendInput;

#if EEPROM_ENABLE
    hal.eeprom.type = EEPROM_Physical;
    hal.eeprom.get_byte = eepromGetByte;
    hal.eeprom.put_byte = eepromPutByte;
    hal.eeprom.memcpy_to_with_checksum = eepromWriteBlockWithChecksum;
    hal.eeprom.memcpy_from_with_checksum = eepromReadBlockWithChecksum;
#else
    hal.eeprom.type = EEPROM_None;
#endif

    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;

#if KEYPAD_ENABLE
    hal.execute_realtime = process_keypress;
    hal.driver_setting = driver_setting;
    hal.driver_settings_restore = driver_settings_restore;
    hal.driver_settings_report = driver_settings_report;
#endif

#if ATC_ENABLE
    hal.driver_reset = atc_reset;
    hal.tool_select = atc_tool_select;
    hal.tool_change = atc_tool_change;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

    hal.driver_cap.spindle_sync = On;
    hal.driver_cap.spindle_at_speed = On;
    hal.driver_cap.spindle_dir = On;
    hal.driver_cap.spindle_pid = On;
    hal.driver_cap.variable_spindle = On;
    hal.driver_cap.spindle_pwm_invert = On;
    hal.driver_cap.mist_control = On;
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;
#if MPG_MODE_ENABLE
    hal.driver_cap.mpg_mode = On;
#endif

    // no need to move version check before init - compiler will fail any mismatch for existing entries
    return hal.version == 5;
}

/* interrupt handlers */

// Main stepper driver
void STEPPER_IRQHandler (void)
{
    STEPPER_TIMER->INTCLR = 0;
    spindle_encoder.timer_value_step = RPM_TIMER->VALUE; // to be used for spindle synchronized motion?
    hal.stepper_interrupt_callback();
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
// initiated after the STEP_PULSE_DELAY time period has elapsed. The ISR TIMER2_OVF interrupt
// will then trigger after the appropriate settings.pulse_microseconds, as in normal operation.
// The new timing between direction, step pulse, and step complete events are setup in the
// st_wake_up() routine.
void STEPPULSE_N_IRQHandler (void)
{
    if(PULSE_TIMER->IV == 0x02) // CCR1 - IV read clears interrupt
        set_step_outputs(next_step_outbits); // Begin step pulse.
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
    DEBOUNCE_TIMER->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG; // Clear interrupt flag and
    DEBOUNCE_TIMER->CTL &= ~(TIMER_A_CTL_MC0|TIMER_A_CTL_MC1); // stop debounce timer

    axes_signals_t state = (axes_signals_t)limitsGetState();

    if(state.mask) //TODO: add check for limit switches having same state as when limit_isr were invoked?
        hal.limit_interrupt_callback(state);
}

void RPMCOUNTER_IRQHandler (void)
{
    uint32_t tval = RPM_TIMER->VALUE;
    uint16_t cval = RPM_COUNTER->R;

    RPM_COUNTER->CCTL[0] &= ~TIMER_A_CCTLN_CCIFG;

    spindle_data.pulse_count += cval - spindle_encoder.pulse_counter_last;
    spindle_encoder.pulse_counter_last = cval;
    spindle_encoder.tpp = (spindle_encoder.timer_value_last - tval) >> 2; // / spindle_encoder.pulse_counter_trigger..
    spindle_encoder.timer_value_last = tval;
    RPM_COUNTER->CCR[0] += spindle_encoder.pulse_counter_trigger;
}

#if CNC_BOOSTERPACK_SHORTS

// Shared with spindle encoder index interrupt
void CONTROL_IRQHandler (void)
{
    uint8_t iflags = CONTROL_PORT->IFG;

    CONTROL_PORT->IFG &= ~iflags;

    if(iflags & RPM_INDEX_BIT) {
        spindle_encoder.timer_value_index = RPM_TIMER->VALUE;
        if((spindle_encoder.error = (RPM_COUNTER->R - spindle_encoder.pulse_counter_index) != spindle_encoder.ppr))
            RPM_COUNTER->CCR[0] = RPM_COUNTER->R + spindle_encoder.pulse_counter_trigger;
        spindle_encoder.pulse_counter_index = RPM_COUNTER->R;
        spindle_data.index_count++;
//        hal.spindle_index_callback(&spindle_data);
    }

    if(iflags & CONTROL_MASK) {
        CONTROL_PORT->IFG &= ~iflags;
        hal.control_interrupt_callback(systemGetState());
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
            hal.limit_interrupt_callback(limitsGetState());
    }
}

#if MPG_MODE_ENABLE

void MODE_IRQHandler (void)
{
    uint8_t iflags = MODE_PORT->IFG & MODE_SWITCH_BIT;

    if(iflags) {
        MODE_PORT->IFG &= ~iflags;
        if(delay.ms == 0) // Ignore if delay is active
            driver_delay_ms(50, modechange);
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
            hal.limit_interrupt_callback(limitsGetState());
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
            hal.limit_interrupt_callback(limitsGetState());
    }

    if(iflags & RESET_BIT)
        hal.control_interrupt_callback(systemGetState());
}

void CONTROL_FH_CS_IRQHandler (void)
{
    uint8_t iflags = CONTROL_PORT_FH->IFG;

    CONTROL_PORT_FH->IFG = 0;

    if(iflags & RPM_INDEX_BIT) {
        spindle_encoder.timer_value_index = RPM_TIMER->VALUE;
        if((spindle_encoder.error = (RPM_COUNTER->R - spindle_encoder.pulse_counter_index) != spindle_encoder.ppr))
            RPM_COUNTER->CCR[0] = RPM_COUNTER->R + spindle_encoder.pulse_counter_trigger;
        spindle_encoder.pulse_counter_index = RPM_COUNTER->R;
        spindle_data.index_count++;
//        hal.spindle_index_callback(&spindle_data);
    }

    if(iflags & (FEED_HOLD_BIT|CYCLE_START_BIT))
        hal.control_interrupt_callback(systemGetState());
}

void CONTROL_SD_MODE_Handler (void)
{
    uint8_t iflags = CONTROL_PORT_SD->IFG & (SAFETY_DOOR_BIT|MODE_SWITCH_BIT);

    CONTROL_PORT_SD->IFG = 0;

  #if MPG_MODE_ENABLE
    if(iflags & MODE_SWITCH_BIT) {
        if(delay.ms == 0) // Ignore if delay is active
            driver_delay_ms(50, modechange);
    } else
  #endif
        hal.control_interrupt_callback(systemGetState());
}
#endif

// Interrupt handler for 1 ms interval timer
void SysTick_Handler (void)
{
    static uint32_t spid = SPINDLE_PID_SAMPLE_RATE;

    switch(spindle_encoder.pid_state) {

        case PIDState_Pending:
            if(pid_count < 500)
                pid_count++;
            else if(spindle_data.index_count > 2)
                spindle_encoder.pid_state = PIDState_Active;
            break;

        case PIDState_Active:
            if(--spid == 0) {
                spindle_rpm_pid();
                spid = SPINDLE_PID_SAMPLE_RATE;
            }
            break;
    }

    if(delay.ms && !(--delay.ms)) {
        if(spindle_encoder.pid_state == PIDState_Disabled)
            SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        if(delay.callback) {
            delay.callback();
            delay.callback = NULL;
        }
    }
}
