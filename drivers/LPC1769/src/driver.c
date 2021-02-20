/*

  driver.c - driver code for NXP LPC176x ARM processors

  Part of grblHAL

  Copyright (c) 2018-2021 Terje Io

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

#include <stdint.h>
#include <string.h>

#include "driver.h"
#include "serial.h"
#include "grbl-lpc/pwm_driver.h"

#include "grbl/limits.h"

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "ff.h"
#include "diskio.h"
#endif

#if FLASH_ENABLE
#include "flash.h"
#endif

#if USB_SERIAL_CDC
#include "usb_serial.h"
#endif

#if I2C_ENABLE
#include "i2c.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#if defined(X_LIMIT_PORT_MAX) || defined(Z_LIMIT_PORT_MAX) || defined(Z_LIMIT_PORT_MAX)
#define HAS_MAX_LIMIT_INPUTS
#endif

#ifndef X_STEP_PN
#define X_STEP_PN STEP_PN
#endif
#ifndef Y_STEP_PN
#define Y_STEP_PN STEP_PN
#endif
#ifndef Z_STEP_PN
#define Z_STEP_PN STEP_PN
#endif

#ifndef X_DIRECTION_PN
#define X_DIRECTION_PN DIRECTION_PN
#endif
#ifndef Y_DIRECTION_PN
#define Y_DIRECTION_PN DIRECTION_PN
#endif
#ifndef Z_DIRECTION_PN
#define Z_DIRECTION_PN DIRECTION_PN
#endif

static bool pwmEnabled = false, IOInitDone = false;
static uint16_t pulse_length, pulse_delay;
// Inverts the probe pin state depending on user settings and probing cycle mode.
static probe_state_t probe = {
    .connected = On
};

static axes_signals_t next_step_outbits;
static spindle_pwm_t spindle_pwm;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup

#define DEBOUNCE_QUEUE 8 // Must be a power of 2

#define INPUT_GROUP_CONTROL 1
#define INPUT_GROUP_PROBE   2
#define INPUT_GROUP_LIMIT   4
#define INPUT_GROUP_KEYPAD  8

typedef enum {
    Input_Unassigned = 0,
    Input_Probe,
    Input_Reset,
    Input_FeedHold,
    Input_CycleStart,
    Input_SafetyDoor,
    Input_LimitX,
    Input_LimitX_Max,
    Input_LimitY,
    Input_LimitY_Max,
    Input_LimitZ,
    Input_LimitZ_Max,
    Input_LimitA,
    Input_LimitA_Max,
    Input_LimitB,
    Input_LimitB_Max,
    Input_LimitC,
    Input_LimitC_Max,
    Input_KeypadStrobe
} input_t;

typedef enum {
    GPIO_Intr_None = 0,
    GPIO_Intr_Falling,
    GPIO_Intr_Rising,
    GPIO_Intr_Both,
} gpio_intr_t;

typedef struct {
    LPC_GPIO_T *port;
    uint32_t pin;
    uint32_t bit;
    input_t id;
    uint8_t group;
    bool debounce;
    gpio_intr_t intr_type;
} input_signal_t;

typedef struct {
    volatile uint_fast8_t head;
    volatile uint_fast8_t tail;
    input_signal_t *signal[DEBOUNCE_QUEUE];
} debounce_queue_t;

static bool limits_debounce = false;
static uint32_t limits_invert;
static volatile uint32_t elapsed_tics = 0;
static debounce_queue_t debounce_queue = {0};
static input_signal_t gpio0_signals[10] = {0}, gpio1_signals[10] = {0}, gpio2_signals[10] = {0};

static input_signal_t inputpin[] = {
  #ifdef PROBE_PIN
    { .id = Input_Probe,        .port = PROBE_PORT,        .pin = PROBE_PIN,         .group = INPUT_GROUP_PROBE },
  #endif
  #ifdef RESET_PIN
    { .id = Input_Reset,        .port = RESET_PORT,        .pin = RESET_PIN,         .group = INPUT_GROUP_CONTROL },
  #endif
  #ifdef FEED_HOLD_PIN
    { .id = Input_FeedHold,     .port = FEED_HOLD_PORT,    .pin = FEED_HOLD_PIN,     .group = INPUT_GROUP_CONTROL },
  #endif
  #ifdef CYCLE_START_PIN
    { .id = Input_CycleStart,   .port = CYCLE_START_PORT,  .pin = CYCLE_START_PIN,   .group = INPUT_GROUP_CONTROL },
  #endif
  #ifdef SAFETY_DOOR_PIN
    { .id = Input_SafetyDoor,   .port = SAFETY_DOOR_PORT,  .pin = SAFETY_DOOR_PIN,   .group = INPUT_GROUP_CONTROL },
  #endif
    { .id = Input_LimitX,       .port = X_LIMIT_PORT,      .pin = X_LIMIT_PIN,       .group = INPUT_GROUP_LIMIT },
  #ifdef X_LIMIT_PIN_MAX
    { .id = Input_LimitX_Max,   .port = X_LIMIT_PORT_MAX,  .pin = X_LIMIT_PIN_MAX,   .group = INPUT_GROUP_LIMIT },
  #endif
    { .id = Input_LimitY,       .port = Y_LIMIT_PORT,      .pin = Y_LIMIT_PIN,       .group = INPUT_GROUP_LIMIT },
  #ifdef Y_LIMIT_PIN_MAX
    { .id = Input_LimitY_Max,   .port = Y_LIMIT_PORT_MAX,  .pin = Y_LIMIT_PIN_MAX,   .group = INPUT_GROUP_LIMIT },
  #endif
    { .id = Input_LimitZ,       .port = Z_LIMIT_PORT,      .pin = Z_LIMIT_PIN,       .group = INPUT_GROUP_LIMIT },
  #ifdef Z_LIMIT_PIN_MAX
    { .id = Input_LimitZ_Max,   .port = Z_LIMIT_PORT_MAX,  .pin = Z_LIMIT_PIN_MAX,   .group = INPUT_GROUP_LIMIT },
  #endif
  #ifdef A_LIMIT_PIN
    { .id = Input_LimitA,       .port = A_LIMIT_PORT,      .pin = A_LIMIT_PIN,       .group = INPUT_GROUP_LIMIT },
  #endif
  #ifdef A_LIMIT_PIN_MAX
    { .id = Input_LimitA_Max,   .port = A_LIMIT_PORT_MAX,  .pin = A_LIMIT_PIN_MAX,   .group = INPUT_GROUP_LIMIT },
  #endif
  #ifdef B_LIMIT_PIN
    { .id = Input_LimitB,       .port = B_LIMIT_PORT,      .pin = B_LIMIT_PIN,       .group = INPUT_GROUP_LIMIT },
  #endif
  #ifdef B_LIMIT_PIN_MAX
    { .id = Input_LimitB_Max,   .port = B_LIMIT_PORT_MAX,  .pin = B_LIMIT_PIN_MAX,   .group = INPUT_GROUP_LIMIT },
  #endif
  #ifdef C_LIMIT_PIN
    { .id = Input_LimitC,       .port = C_LIMIT_PORT,      .pin = C_LIMIT_PIN,       .group = INPUT_GROUP_LIMIT },
  #endif
  #ifdef C_LIMIT_PIN_MAX
    { .id = Input_LimitC_Max,   .port = C_LIMIT_PORT_MAX,  .pin = C_LIMIT_PIN_MAX,   .group = INPUT_GROUP_LIMIT },
  #endif
  #if KEYPAD_ENABLE
  , { .id = Input_KeypadStrobe, .port = KEYPAD_STROBE_PORT .pin = KEYPAD_STROBE_PIN, .group = INPUT_GROUP_KEYPAD }
  #endif
};

#if STEP_OUTMODE == GPIO_MAP

static const uint32_t c_step_outmap[] = {
    0,
    X_STEP_BIT,
    Y_STEP_BIT,
    Y_STEP_BIT | X_STEP_BIT,
    Z_STEP_BIT,
    Z_STEP_BIT | X_STEP_BIT,
    Z_STEP_BIT | Y_STEP_BIT,
    Z_STEP_BIT | Y_STEP_BIT | X_STEP_BIT,
#if N_AXIS > 3
    A_STEP_BIT,
    A_STEP_BIT | X_STEP_BIT,
    A_STEP_BIT | Y_STEP_BIT,
    A_STEP_BIT | Y_STEP_BIT | X_STEP_BIT,
    A_STEP_BIT | Z_STEP_BIT,
    A_STEP_BIT | Z_STEP_BIT | X_STEP_BIT,
    A_STEP_BIT | Z_STEP_BIT | Y_STEP_BIT,
    A_STEP_BIT | Z_STEP_BIT | Y_STEP_BIT | X_STEP_BIT,
#endif
#if N_AXIS > 4
    B_STEP_BIT,
    B_STEP_BIT | X_STEP_BIT,
    B_STEP_BIT | Y_STEP_BIT,
    B_STEP_BIT | X_STEP_BIT,
    B_STEP_BIT | Z_STEP_BIT,
    B_STEP_BIT | Z_STEP_BIT | X_STEP_BIT,
    B_STEP_BIT | Z_STEP_BIT | Y_STEP_BIT,
    B_STEP_BIT | Z_STEP_BIT | Y_STEP_BIT | X_STEP_BIT,
    B_STEP_BIT | A_STEP_BIT,
    B_STEP_BIT | A_STEP_BIT | X_STEP_BIT,
    B_STEP_BIT | A_STEP_BIT | Y_STEP_BIT,
    B_STEP_BIT | A_STEP_BIT | Y_STEP_BIT | X_STEP_BIT,
    B_STEP_BIT | A_STEP_BIT | Z_STEP_BIT,
    B_STEP_BIT | A_STEP_BIT | Z_STEP_BIT | X_STEP_BIT,
    B_STEP_BIT | A_STEP_BIT | Z_STEP_BIT | Y_STEP_BIT,
    B_STEP_BIT | A_STEP_BIT | Z_STEP_BIT | Y_STEP_BIT | X_STEP_BIT,
#endif
};

static uint32_t step_outmap[sizeof(c_step_outmap) / sizeof(uint32_t)];

#endif

#if DIRECTION_OUTMODE == GPIO_MAP

static const uint32_t c_dir_outmap[] = {
    0,
    X_DIRECTION_BIT,
    Y_DIRECTION_BIT,
    Y_DIRECTION_BIT | X_DIRECTION_BIT,
    Z_DIRECTION_BIT,
    Z_DIRECTION_BIT | X_DIRECTION_BIT,
    Z_DIRECTION_BIT | Y_DIRECTION_BIT,
    Z_DIRECTION_BIT | Y_DIRECTION_BIT | X_DIRECTION_BIT,
#if N_AXIS > 3
    A_DIRECTION_BIT,
    A_DIRECTION_BIT | X_DIRECTION_BIT,
    A_DIRECTION_BIT | Y_DIRECTION_BIT,
    A_DIRECTION_BIT | Y_DIRECTION_BIT | X_DIRECTION_BIT,
    A_DIRECTION_BIT | Z_DIRECTION_BIT,
    A_DIRECTION_BIT | Z_DIRECTION_BIT | X_DIRECTION_BIT,
    A_DIRECTION_BIT | Z_DIRECTION_BIT | Y_DIRECTION_BIT,
    A_DIRECTION_BIT | Z_DIRECTION_BIT | Y_DIRECTION_BIT | X_DIRECTION_BIT,
#endif
#if N_AXIS > 4
    B_DIRECTION_BIT,
    B_DIRECTION_BIT | X_DIRECTION_BIT,
    B_DIRECTION_BIT | Y_DIRECTION_BIT,
    B_DIRECTION_BIT | X_DIRECTION_BIT,
    B_DIRECTION_BIT | Z_DIRECTION_BIT,
    B_DIRECTION_BIT | Z_DIRECTION_BIT | X_DIRECTION_BIT,
    B_DIRECTION_BIT | Z_DIRECTION_BIT | Y_DIRECTION_BIT,
    B_DIRECTION_BIT | Z_DIRECTION_BIT | Y_DIRECTION_BIT | X_DIRECTION_BIT,
    B_DIRECTION_BIT | A_DIRECTION_BIT,
    B_DIRECTION_BIT | A_DIRECTION_BIT | X_DIRECTION_BIT,
    B_DIRECTION_BIT | A_DIRECTION_BIT | Y_DIRECTION_BIT,
    B_DIRECTION_BIT | A_DIRECTION_BIT | Y_DIRECTION_BIT | X_DIRECTION_BIT,
    B_DIRECTION_BIT | A_DIRECTION_BIT | Z_DIRECTION_BIT,
    B_DIRECTION_BIT | A_DIRECTION_BIT | Z_DIRECTION_BIT | X_DIRECTION_BIT,
    B_DIRECTION_BIT | A_DIRECTION_BIT | Z_DIRECTION_BIT | Y_DIRECTION_BIT,
    B_DIRECTION_BIT | A_DIRECTION_BIT | Z_DIRECTION_BIT | Y_DIRECTION_BIT | X_DIRECTION_BIT,
#endif
};

static uint32_t dir_outmap[sizeof(c_dir_outmap) / sizeof(uint32_t)];

#endif

static void spindle_set_speed (uint_fast16_t pwm_value);

// Interrupt handler prototypes

uint32_t cpt;

static void driver_delay (uint32_t ms, void (*callback)(void))
{
    if((delay.ms = ms) > 0) {
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        if(!(delay.callback = callback))
            while(delay.ms);
    } else if(callback)
        callback();
}

// Enable/disable stepper motors
static void stepperEnable (axes_signals_t enable)
{
    enable.mask ^= settings.steppers.enable_invert.mask;
#if DISABLE_OUTMODE == GPIO_BITBAND
    DIGITAL_OUT(X_DISABLE_PORT, X_DISABLE_BIT, enable.x);
    DIGITAL_OUT(Y_DISABLE_PORT, Y_DISABLE_BIT, enable.y);
    DIGITAL_OUT(Z_DISABLE_PORT, Z_DISABLE_BIT, enable.z);
  #ifdef A_AXIS
    DIGITAL_OUT(A_DISABLE_PORT, A_DISABLE_BIT, enable.a);
  #endif
  #ifdef B_AXIS
    DIGITAL_OUT(B_DISABLE_PORT, B_DISABLE_BIT, enable.z);
  #endif
#elif defined(DISABLE_MASK)
    DISABLE_PORT->PIN = (DISABLE_PORT->PIN & ~DISABLE_MASK) | enable.mask;
#else
    DIGITAL_OUT(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_BIT, enable.x);
#endif

}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    stepperEnable((axes_signals_t){AXES_BITMASK});

    STEPPER_TIMER->TCR = 0b10;          // reset
    STEPPER_TIMER->MR[0] = 0xFFFF;      // Set a long initial delay,
    STEPPER_TIMER->TCR = 0b01;          // start stepper ISR timer in up mode
//    hal.stepper.interrupt_callback();   // and start the show
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals) {
    STEPPER_TIMER->TCR = 0;   // Stop stepper timer
}

// Sets up stepper driver interrupt timeout, limiting the slowest speed
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    STEPPER_TIMER->MR[0] = cycles_per_tick < (1UL << 20) ? cycles_per_tick : 0x000FFFFFUL;
}

// Set stepper pulse output pins
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z...
// Mapping to registers can be done by
// 1. bitbanding. Pros: can assign pins to different ports, no RMW needed. Cons: overhead, pin changes not synchronous
// 2. bit shift. Pros: fast, Cons: bits must be consecutive
// 3. lookup table. Pros: signal inversions done at setup, Cons: slower than bit shift?
inline static __attribute__((always_inline)) void stepperSetStepOutputs (axes_signals_t step_outbits)
{
#if STEP_OUTMODE == GPIO_BITBAND
    step_outbits.value ^= settings.steppers.step_invert.value;
    DIGITAL_OUT(X_STEP_PORT, X_STEP_BIT, step_outbits.x);
    DIGITAL_OUT(Y_STEP_PORT, Y_STEP_BIT, step_outbits.y);
    DIGITAL_OUT(Z_STEP_PORT, Z_STEP_BIT, step_outbits.z);
  #ifdef A_AXIS
    DIGITAL_OUT(A_STEP_PORT, A_STEP_BIT, step_outbits.a);
  #endif
  #ifdef B_AXIS
    DIGITAL_OUT(B_STEP_PORT, B_STEP_BIT, step_outbits.b);
  #endif
#elif STEP_OUTMODE == GPIO_MAP
    STEP_PORT->PIN = (STEP_PORT->PIN & ~STEP_MASK) | step_outmap[step_outbits.value];
#else
    STEP_PORT->PIN = (STEP_PORT->PIN & ~STEP_MASK) | ((step_outbits.value << STEP_OUTMODE) ^ settings.steppers.step_invert.value);
#endif
}

// Set stepper direction output pins
// NOTE: see note for stepperSetStepOutputs()
inline static __attribute__((always_inline)) void stepperSetDirOutputs (axes_signals_t dir_outbits)
{
#if DIRECTION_OUTMODE == GPIO_BITBAND
    dir_outbits.value ^= settings.steppers.dir_invert.value;
    DIGITAL_OUT(X_DIRECTION_PORT, X_DIRECTION_BIT, dir_outbits.x);
    DIGITAL_OUT(Y_DIRECTION_PORT, Y_DIRECTION_BIT, dir_outbits.y);
    DIGITAL_OUT(Z_DIRECTION_PORT, Z_DIRECTION_BIT, dir_outbits.z);
  #ifdef A_AXIS
    DIGITAL_OUT(A_DIRECTION_PORT, A_DIRECTION_BIT, dir_outbits.a);
  #endif
  #ifdef B_AXIS
    DIGITAL_OUT(B_DIRECTION_PORT, B_DIRECTION_BIT, dir_outbits.b);
  #endif
#elif DIRECTION_OUTMODE == GPIO_MAP
    DIRECTION_PORT->PIN = (DIRECTION_PORT->PIN & ~DIRECTION_MASK) | dir_outmap[dir_outbits.value];
#else
    DIRECTION_PORT->PIN = (DIRECTION_PORT->PIN & ~DIRECTION_MASK) | ((dir_outbits.value << DIRECTION_OUTMODE) ^ settings.steppers.dir_invert.value);
#endif
}

// Sets stepper direction and pulse pins and starts a step pulse.
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_change)
        stepperSetDirOutputs(stepper->dir_outbits);

    if(stepper->step_outbits.value) {
        stepperSetStepOutputs(stepper->step_outbits);
        PULSE_TIMER->TCR = 0b01;
    }
}

// Start a stepper pulse, delay version.
// Note: delay is only added when there is a direction change and a pulse to be output.
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->dir_change) {

        stepperSetDirOutputs(stepper->dir_outbits);

        if(stepper->step_outbits.value) {
            PULSE_TIMER->TCR = 0b10;
            next_step_outbits = stepper->step_outbits; // Store out_bits
            PULSE_TIMER->MR[0] = pulse_delay;
            PULSE_TIMER->TCR = 0b01;
        }

        return;
    }

    if(stepper->step_outbits.value) {
        stepperSetStepOutputs(stepper->step_outbits);
        PULSE_TIMER->TCR = 1;
    }
}

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing)
{
    on = on && settings.limits.flags.hard_enabled;

#ifdef X_LIMIT_INTENR
    BITBAND_PERI(X_LIMIT_INTCLR, X_LIMIT_PIN) = 0;
    BITBAND_PERI(X_LIMIT_INTENR, X_LIMIT_PIN) = On;
#endif
#ifdef Y_LIMIT_INTENR
    BITBAND_PERI(Y_LIMIT_INTCLR, Y_LIMIT_PIN) = 0;
    BITBAND_PERI(Y_LIMIT_INTENR, Y_LIMIT_PIN) = On;
#endif
#ifdef Z_LIMIT_INTENR
    BITBAND_PERI(Z_LIMIT_INTCLR, Z_LIMIT_PIN) = 0;
    BITBAND_PERI(Z_LIMIT_INTENR, Z_LIMIT_PIN) = On;
#endif
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};

#if LIMIT_INMODE == LIMIT_SHIFT
    signals.value = (uint32_t)(LIMIT_PORT->PIN & LIMIT_MASK) >> LIMIT_SHIFT;
#elif LIMIT_INMODE == GPIO_BITBAND
    signals.min.x = DIGITAL_IN(X_LIMIT_PORT, X_LIMIT_BIT);
    signals.min.y = DIGITAL_IN(Y_LIMIT_PORT, Y_LIMIT_BIT);
    signals.min.z = DIGITAL_IN(Z_LIMIT_PORT, Z_LIMIT_BIT);
#else
    uint32_t bits = LIMIT_PORT->PIN;
    signals.min.x = (bits & X_LIMIT_BIT) != 0;
    signals.min.y = (bits & Y_LIMIT_BIT) != 0;
    signals.min.z = (bits & Z_LIMIT_BIT) != 0;
#endif

    if (settings.limits.invert.mask)
        signals.min.value ^= settings.limits.invert.mask;

#ifdef HAS_MAX_LIMIT_INPUTS

    signals.max.value = settings.limits.invert.mask;

#ifdef X_LIMIT_PORT_MAX
    signals.max.x = DIGITAL_IN(X_LIMIT_PORT_MAX, X_LIMIT_BIT_MAX);
#else
    signals.max.x = (bits & X_LIMIT_BIT_MAX) != 0;
#endif
#ifdef Y_LIMIT_PORT_MAX
    signals.max.y = DIGITAL_IN(Y_LIMIT_PORT_MAX, Y_LIMIT_BIT_MAX);
#else
    signals.max.y = (bits & Y_LIMIT_BIT_MAX) != 0;
#endif
#ifdef Z_LIMIT_PORT_MAX
    signals.max.z = DIGITAL_IN(Z_LIMIT_PORT_MAX, Z_LIMIT_BIT_MAX);
#else
    signals.max.z = (bits & Z_LIMIT_BIT_MAX) != 0;
#endif
    if (settings.limits.invert.mask)
        signals.max.value ^= settings.limits.invert.mask;
#endif

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.value = settings.control_invert.mask;

#if CONTROL_INMODE == GPIO_BITBAND
    signals.reset = DIGITAL_IN(RESET_PORT, RESET_BIT);
    signals.feed_hold = DIGITAL_IN(FEED_HOLD_PORT, FEED_HOLD_BIT);
    signals.cycle_start = DIGITAL_IN(CYCLE_START_PORT, CYCLE_START_BIT);
  #ifdef SAFETY_DOOR_PORT
    signals.safety_door_ajar = DIGITAL_IN(SAFETY_DOOR_PORT, SAFETY_DOOR_BIT);
  #endif
#else
    uint8_t bits = CONTROL_PORT->PIN;
    signals.reset = bits & RESET_BIT != 0;
    signals.safety_door_ajar = bits & SAFETY_DOOR_BIT != 0;
    signals.feed_hold = bits & FEED_HOLD_BIT != 0;
    signals.cycle_start = bits & CYCLE_START_BIT != 0;
#endif
    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

#ifndef SAFETY_DOOR_PORT
    signals.safety_door_ajar = 0;
#endif

    return signals;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure(bool is_probe_away, bool probing)
{
    probe.triggered = Off;
    probe.is_probing = probing;
    probe.inverted = settings.probe.invert_probe_pin;

    if (is_probe_away)
        probe.inverted = !probe.inverted;
}

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;
    state.triggered = !!(PROBE_PORT->PIN & PROBE_BIT) ^ probe.inverted;

    return state;
}

// Static spindle (off, on cw & on ccw)

inline static void spindle_off (void)
{
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT, settings.spindle.invert.on);
}

inline static void spindle_on (void)
{
    DIGITAL_OUT(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT, !settings.spindle.invert.on);
}

inline static void spindle_dir (bool ccw)
{
    if(hal.driver_cap.spindle_dir)
        DIGITAL_OUT(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_BIT, ccw ^ settings.spindle.invert.ccw);
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
    if (pwm_value == spindle_pwm.off_value) {
        pwmEnabled = false;
        if(settings.spindle.flags.pwm_action == SpindleAction_DisableWithZeroSPeed)
            spindle_off();
        if(spindle_pwm.always_on) {
            pwm_set_width(&SPINDLE_PWM_CHANNEL, spindle_pwm.off_value);
            pwm_enable(&SPINDLE_PWM_CHANNEL);
        } else {
            pwm_set_width(&SPINDLE_PWM_CHANNEL, 0);
//          pwm_disable(&SPINDLE_PWM_CHANNEL); // Set PWM output low
        }
    } else {
        if(!pwmEnabled)
            spindle_on();
        pwmEnabled = true;
        pwm_set_width(&SPINDLE_PWM_CHANNEL, pwm_value);
        pwm_enable(&SPINDLE_PWM_CHANNEL);
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
    spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
}

#endif

// Start or stop spindle
static void spindleSetStateVariable (spindle_state_t state, float rpm)
{
    if (!state.on || rpm == 0.0f) {
        spindle_set_speed(spindle_pwm.off_value);
        spindle_off();
    } else {
        if(hal.driver_cap.spindle_dir)
            spindle_dir(state.ccw);

        spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
    }
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {settings.spindle.invert.mask};

    state.on = (SPINDLE_ENABLE_PORT->PIN & SPINDLE_ENABLE_BIT) != 0;
    state.ccw = hal.driver_cap.spindle_dir && (SPINDLE_DIRECTION_PORT->PIN & SPINDLE_DIRECTION_BIT) != 0;
    state.value ^= settings.spindle.invert.mask;
    if(pwmEnabled)
        state.value = On;

    return state;
}

// end spindle code

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;

    DIGITAL_OUT(COOLANT_FLOOD_PORT, COOLANT_FLOOD_BIT, mode.flood);

#ifdef COOLANT_MIST_PORT
    DIGITAL_OUT(COOLANT_MIST_PORT, COOLANT_MIST_BIT, mode.mist);
#endif
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {settings.coolant_invert.mask};

    state.flood = DIGITAL_IN(COOLANT_FLOOD_PORT, COOLANT_FLOOD_BIT);
#ifdef COOLANT_MIST_PORT
    state.mist  = DIGITAL_IN(COOLANT_MIST_PORT, COOLANT_MIST_BIT);;
#endif
    state.value ^= settings.coolant_invert.mask;

    return state;
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

inline static uint8_t gpio_to_pn (LPC_GPIO_T *port)
{
    return ((uint32_t)port - LPC_GPIO0_BASE) / sizeof(LPC_GPIO_T);
}

static void gpio0_int_enable (uint32_t bit, gpio_intr_t intr_type)
{
    LPC_GPIOINT->IO0.CLR = bit;

    switch(intr_type) {
        case GPIO_Intr_Falling:
            LPC_GPIOINT->IO0.ENR &= ~bit;
            LPC_GPIOINT->IO0.ENF |= bit;
            break;
        case GPIO_Intr_Rising:
            LPC_GPIOINT->IO0.ENR |= bit;
            LPC_GPIOINT->IO0.ENF &= ~bit;
            break;
        case GPIO_Intr_Both:
            LPC_GPIOINT->IO0.ENR |= bit;
            LPC_GPIOINT->IO0.ENF |= bit;
            break;
        default:
            LPC_GPIOINT->IO0.ENR &= ~bit;
            LPC_GPIOINT->IO0.ENF &= ~bit;
            break;
    }
}

static void gpio2_int_enable (uint32_t bit, gpio_intr_t intr_type)
{
    LPC_GPIOINT->IO2.CLR = bit;

    switch(intr_type) {
        case GPIO_Intr_Falling:
            LPC_GPIOINT->IO2.ENR &= ~bit;
            LPC_GPIOINT->IO2.ENF |= bit;
            break;
        case GPIO_Intr_Rising:
            LPC_GPIOINT->IO2.ENR |= bit;
            LPC_GPIOINT->IO2.ENF &= ~bit;
            break;
        case GPIO_Intr_Both:
            LPC_GPIOINT->IO2.ENR |= bit;
            LPC_GPIOINT->IO2.ENF |= bit;
            break;
        default:
            LPC_GPIOINT->IO2.ENR &= ~bit;
            LPC_GPIOINT->IO2.ENF &= ~bit;
            break;
    }
}

uint32_t getElapsedTicks (void)
{
    return elapsed_tics;
}

// Configures peripherals when settings are initialized or changed
void settings_changed (settings_t *settings)
{
    hal.driver_cap.variable_spindle = spindle_precompute_pwm_values(&spindle_pwm, SystemCoreClock / Chip_Clock_GetPCLKDiv(SYSCTL_PCLK_PWM1));

#if (STEP_OUTMODE == GPIO_MAP) || (DIRECTION_OUTMODE == GPIO_MAP)
    uint8_t i;
#endif

#if STEP_OUTMODE == GPIO_MAP
    for(i = 0; i < sizeof(step_outmap) / sizeof(uint32_t); i++)
        step_outmap[i] = c_step_outmap[i ^ settings->steppers.step_invert.value];
#endif

#if DIRECTION_OUTMODE == GPIO_MAP
    for(i = 0; i < sizeof(dir_outmap) / sizeof(uint32_t); i++)
        dir_outmap[i] = c_dir_outmap[i ^ settings->steppers.dir_invert.value];
#endif

    if(IOInitDone) {

        stepperEnable(settings->steppers.deenergize);

        if(hal.driver_cap.variable_spindle) {
            pwm_init(&SPINDLE_PWM_CHANNEL, SPINDLE_PWM_USE_PRIMARY_PIN, SPINDLE_PWM_USE_SECONDARY_PIN, spindle_pwm.period, 0);
            hal.spindle.set_state = spindleSetStateVariable;
        } else
            hal.spindle.set_state = spindleSetState;

        int32_t t = (uint32_t)(12.0f * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY)) - 1;
        pulse_length = t < 2 ? 2 : t;

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            t = (uint32_t)(12.0f * (settings->steppers.pulse_delay_microseconds - 1.5f)) - 1;
            pulse_delay = t < 2 ? 2 : t;
            if(pulse_delay == pulse_length)
                pulse_delay++;
            hal.stepper.pulse_start = &stepperPulseStartDelayed;
        } else
            hal.stepper.pulse_start = &stepperPulseStart;

        PULSE_TIMER->TCR = 0b10;
        PULSE_TIMER->MR[0] = pulse_length;
        PULSE_TIMER->MCR |= (MR0I|MR0S|MR0R); // Enable interrupt for finish step pulse, reset and stop timer
        PULSE_TIMER->TCR = 0b00;

        stepperSetStepOutputs((axes_signals_t){0});

        NVIC_DisableIRQ(EINT3_IRQn);  // Disable GPIO interrupt

        /****************************************
         *  Control, limit & probe pins config  *
         ****************************************/

        bool pullup = true;

        control_signals_t control_fei;
        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        input_signal_t *pin;
        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t), a = 0, b = 0, c = 0;

        limits_invert = 0;

        do {

            LPC_GPIOINT->IO0.CLR = 0xFFFF;
            LPC_GPIOINT->IO2.CLR = 0xFFFF;

            pin = &inputpin[--i];

            if(pin->port != NULL) {

                pin->bit = 1 << pin->pin;

                switch(pin->id) {

                  #ifdef RESET_PIN
                    case Input_Reset:
                        pullup = !settings->control_disable_pullup.reset;
                        pin->intr_type = control_fei.reset ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef FEED_HOLD_PIN
                    case Input_FeedHold:
                        pullup = !settings->control_disable_pullup.feed_hold;
                        pin->intr_type = control_fei.feed_hold ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef CYCLE_START_PIN
                    case Input_CycleStart:
                        pullup = !settings->control_disable_pullup.cycle_start;
                        pin->intr_type = control_fei.cycle_start ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef SAFETY_DOOR_PIN
                    case Input_SafetyDoor:
                        pullup = !settings->control_disable_pullup.safety_door_ajar;
                        pin->intr_type = control_fei.safety_door_ajar ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif

                  #ifdef PROBE_PIN
                    case Input_Probe:
                        pullup = hal.driver_cap.probe_pull_up;
                        pin->intr_type = GPIO_Intr_None;
                        break;
                  #endif

                    case Input_LimitX:
                        if(settings->limits.invert.x)
                            limits_invert |= pin->bit;
                        pullup = !settings->limits.disable_pullup.x;
                        pin->intr_type = limit_fei.x ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;

                    case Input_LimitY:
                        if(settings->limits.invert.y)
                            limits_invert |= pin->bit;
                        pullup = !settings->limits.disable_pullup.y;
                        pin->intr_type = limit_fei.y ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;

                    case Input_LimitZ:
                        if(settings->limits.invert.z)
                            limits_invert |= pin->bit;
                        pullup = !settings->limits.disable_pullup.z;
                        pin->intr_type = limit_fei.z ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;

                  #ifdef X_LIMIT_PIN_MAX
                    case Input_LimitX_Max:
                        if(settings->limits.invert.x)
                            limits_invert |= pin->bit;
                       pullup = !settings->limits.disable_pullup.x;
                        pin->intr_type = limit_fei.x ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef Y_LIMIT_PIN_MAX
                    case Input_LimitY_Max:
                        if(settings->limits.invert.y)
                            limits_invert |= pin->bit;
                        pullup = !settings->limits.disable_pullup.y;
                        pin->intr_type = limit_fei.y ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef Z_LIMIT_PIN_MAX
                    case Input_LimitZ_Max:
                        if(settings->limits.invert.z)
                            limits_invert |= pin->bit;
                        pullup = !settings->limits.disable_pullup.z;
                        pin->intr_type = limit_fei.z ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef A_LIMIT_PIN
                    case Input_LimitA:
                        if(settings->limits.invert.a)
                            limits_invert |= pin->bit;
                        pullup = !settings->limits.disable_pullup.a;
                        pin->intr_type = limit_fei.a ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef A_LIMIT_PIN_MAX
                    case Input_LimitA_Max:
                        if(settings->limits.invert.a)
                            limits_invert |= pin->bit;
                        pullup = !settings->limits.disable_pullup.a;
                        pin->intr_type = limit_fei.a ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef B_LIMIT_PIN
                    case Input_LimitB:
                        if(settings->limits.invert.b)
                            limits_invert |= pin->bit;
                        pullup = !settings->limits.disable_pullup.b;
                        pin->intr_type = limit_fei.b ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef B_LIMIT_PIN_MAX
                    case Input_LimitB_Max:
                        if(settings->limits.invert.b)
                            limits_invert |= pin->bit;
                        pullup = !settings->limits.disable_pullup.b;
                        pin->intr_type = limit_fei.b ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef C_LIMIT_PIN
                    case Input_LimitC:
                        if(settings->limits.invert.c)
                            limits_invert |= pin->bit;
                        pullup = !settings->limits.disable_pullup.c;
                        pin->intr_type = limit_fei.c ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef C_LIMIT_PIN_MAX
                    case Input_LimitC_Max:
                        if(settings->limits.invert.c)
                            limits_invert |= pin->bit;
                        pullup = !settings->limits.disable_pullup.c;
                        pin->intr_type = limit_fei.c ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif

                  #if KEYPAD_ENABLE
                    case Input_KeypadStrobe:
                        pullup = true;
                        pin->intr_type = GPIO_Intr_Both; // -> any edge?
                        break;
                  #endif

                    default:
                        break;
                }

                Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, gpio_to_pn(pin->port), pin->pin, pullup ? IOCON_MODE_PULLUP : IOCON_MODE_PULLDOWN, IOCON_FUNC0);

                // GPIO1, GPIO3 and GPIO4 are not interrupt capable ports
                if(pin->port == LPC_GPIO3 || pin->port == LPC_GPIO4) {

                    if(pin->intr_type != GPIO_Intr_None) {
                        hal.stream.write("[MSG:Bad bin configuration]" ASCII_EOL);
                        while(true);
                    }

                    if(pin->group == INPUT_GROUP_LIMIT)
                        pin->intr_type = GPIO_Intr_None;
                }

                if(pin->intr_type != GPIO_Intr_None) {

                    pin->debounce = hal.driver_cap.software_debounce && !(pin->group == INPUT_GROUP_PROBE || pin->group == INPUT_GROUP_KEYPAD);

                    if(pin->port == LPC_GPIO0) {
                        gpio0_int_enable(pin->bit, pin->intr_type);
                        memcpy(&gpio0_signals[a++], &inputpin[i], sizeof(input_signal_t));
                    } else if(pin->port == LPC_GPIO1) {
                        memcpy(&gpio1_signals[b++], &inputpin[i], sizeof(input_signal_t));
                    } else if(pin->port == LPC_GPIO2) {
                        gpio2_int_enable(pin->bit, pin->intr_type);
                        memcpy(&gpio2_signals[c++], &inputpin[i], sizeof(input_signal_t));
                    }
                }
            }
        } while(i);

        NVIC_EnableIRQ(EINT3_IRQn);  // Enable GPIO interrupts
    }
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{

    // Cleanup after (potential) sloppy bootloader
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, X_STEP_PN, X_STEP_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, Y_STEP_PN, Y_STEP_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, Z_STEP_PN, Z_STEP_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, X_DIRECTION_PN, X_DIRECTION_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, Y_DIRECTION_PN, Y_DIRECTION_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, Z_DIRECTION_PN, Z_DIRECTION_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
#ifdef STEPPERS_DISABLE_PN
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, STEPPERS_DISABLE_PN, STEPPERS_DISABLE_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
#else
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, X_DISABLE_PN, X_DISABLE_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, Y_DISABLE_PN, Y_DISABLE_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, Z_DISABLE_PN, Z_DISABLE_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
#endif
#ifdef A_AXIS
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, A_STEP_PN, A_STEP_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, A_DIRECTION_PN, A_DIRECTION_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, A_DISABLE_PN, A_DISABLE_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
#endif
#ifdef B_AXIS
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, B_STEP_PN, B_STEP_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, B_DIRECTION_PN, B_DIRECTION_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, B_DISABLE_PN, B_DISABLE_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
#endif
#ifdef B_AXIS
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, C_STEP_PN, C_STEP_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, C_DIRECTION_PN, C_DIRECTION_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, C_DISABLE_PN, C_DISABLE_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
#endif

    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, COOLANT_FLOOD_PN, COOLANT_FLOOD_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
#ifdef COOLANT_MIST_PORT
    Chip_IOCON_PinMux((LPC_IOCON_T *)LPC_IOCON_BASE, COOLANT_MIST_PN, COOLANT_MIST_PIN, IOCON_MODE_INACT, IOCON_FUNC0);
#endif

    // Stepper init

    X_STEP_PORT->DIR |= X_STEP_BIT;
    Y_STEP_PORT->DIR |= Y_STEP_BIT;
    Z_STEP_PORT->DIR |= Z_STEP_BIT;
#ifdef A_AXIS
    A_STEP_PORT->DIR |= A_STEP_BIT;
#endif
#ifdef B_AXIS
    B_STEP_PORT->DIR |= B_STEP_BIT;
#endif

    X_DIRECTION_PORT->DIR |= X_DIRECTION_BIT;
    Y_DIRECTION_PORT->DIR |= Y_DIRECTION_BIT;
    Z_DIRECTION_PORT->DIR |= Z_DIRECTION_BIT;
#ifdef A_AXIS
    A_DIRECTION_PORT->DIR |= A_DIRECTION_BIT;
#endif
#ifdef B_AXIS
    B_DIRECTION_PORT->DIR |= B_DIRECTION_BIT;
#endif

#if DISABLE_OUTMODE == GPIO_BITBAND
    X_DISABLE_PORT->DIR |= X_DISABLE_BIT;
    Y_DISABLE_PORT->DIR |= Y_DISABLE_BIT;
    Z_DISABLE_PORT->DIR |= Z_DISABLE_BIT;
  #ifdef A_AXIS
    A_DISABLE_PORT->DIR |= A_DISABLE_BIT;
  #endif
  #ifdef B_AXIS
    B_DISABLE_PORT->DIR |= B_DISABLE_BIT;
  #endif
#elif defined(DISABLE_MASK)
    DISABLE_PORT->DIR |= DISABLE_MASK;
#else
    STEPPERS_DISABLE_PORT->DIR |= STEPPERS_DISABLE_PIN;
#endif

    Chip_TIMER_Init(STEPPER_TIMER);
    Chip_TIMER_Init(PULSE_TIMER);

    STEPPER_TIMER->TCR = 0;            // disable
    STEPPER_TIMER->CTCR = 0;           // timer mode
    STEPPER_TIMER->PR = 0;             // no prescale
    STEPPER_TIMER->MCR = MR0I|MR0R;    // MR0: !stop, reset, interrupt
    STEPPER_TIMER->CCR = 0;            // no capture
    STEPPER_TIMER->EMR = 0;            // no external match

    uint32_t xx = SystemCoreClock / 12000000UL / Chip_Clock_GetPCLKDiv(PULSE_TIMER_PCLK) - 1; // to 0.12 us

    PULSE_TIMER->TCR = 0b10;
    PULSE_TIMER->CTCR = 0;
    PULSE_TIMER->PR = xx; // to 0.1 us;
    PULSE_TIMER->TCR = 0;

    NVIC_EnableIRQ(STEPPER_TIMER_INT0);   // Enable stepper interrupt
    NVIC_EnableIRQ(PULSE_TIMER_INT0);     // Enable step pulse interrupt

    NVIC_SetPriority(PULSE_TIMER_INT0, 0);
    NVIC_SetPriority(STEPPER_TIMER_INT0, 2);

 // Limit pins init

    NVIC_EnableIRQ(EINT3_IRQn);  // Enable GPIO interrupt

 // Control pins init
 // NOTE: CS is shared with limit isr

//    NVIC_EnableIRQ(CONTROL_INT);  // Enable limit port interrupt

    if(hal.driver_cap.software_debounce) {
        Chip_TIMER_Init(DEBOUNCE_TIMER);
        DEBOUNCE_TIMER->TCR = 0b10;
        DEBOUNCE_TIMER->CTCR = 0;
        DEBOUNCE_TIMER->PR = SystemCoreClock / 1000000 / Chip_Clock_GetPCLKDiv(DEBOUNCE_TIMER_PCLK); // 1 us
        DEBOUNCE_TIMER->MCR |= (MR0I|MR0S);
        DEBOUNCE_TIMER->MR[0] = 4000; // 40 ms
        DEBOUNCE_TIMER->TCR = 0;
        NVIC_EnableIRQ(DEBOUNCE_TIMER_INT0); // Enable debounce interrupt
    }

 // Spindle init

    SPINDLE_ENABLE_PORT->DIR |= SPINDLE_ENABLE_BIT;
    SPINDLE_DIRECTION_PORT->DIR |= SPINDLE_DIRECTION_BIT; // Configure as output pin.

 // Coolant init

    COOLANT_FLOOD_PORT->DIR |= COOLANT_FLOOD_BIT;
#ifdef COOLANT_MIST_PORT
    COOLANT_MIST_PORT->DIR |= COOLANT_MIST_BIT;
#endif

#if SDCARD_ENABLE
    BITBAND_GPIO(SD_CS_PORT->DIR, SD_CS_PIN) = 1;
    BITBAND_GPIO(SD_CS_PORT->PIN, SD_CS_PIN) = 1;

    sdcard_init();
#endif

 // Set defaults

    IOInitDone = settings->version == 19;

    hal.settings_changed(settings);
    hal.spindle.set_state((spindle_state_t){0}, 0.0f);
    hal.coolant.set_state((coolant_state_t){0});
    stepperSetDirOutputs((axes_signals_t){0});

    return IOInitDone;
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done

bool driver_init (void) {

    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

    SystemCoreClockUpdate();

    Chip_SetupXtalClocking(); // Sets 96 MHz clock
    Chip_SYSCTL_SetFLASHAccess(FLASHTIM_120MHZ_CPU);

    SystemCoreClockUpdate();

    Chip_GPIO_Init(LPC_GPIO);
    Chip_IOCON_Init(LPC_IOCON);

    // Enable and set SysTick IRQ to lowest priority
    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk|SysTick_CTRL_ENABLE_Msk;
    NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

#if I2C_ENABLE
    i2c_init();
#endif

    hal.info = "LCP1769";
    hal.driver_version = "210219";
    hal.driver_setup = driver_setup;
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.f_step_timer = SystemCoreClock / Chip_Clock_GetPCLKDiv(STEPPER_TIMER_PCLK);
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = &driver_delay;
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

    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigure;

    hal.spindle.set_state = spindleSetState;
    hal.spindle.get_state = spindleGetState;
#ifdef SPINDLE_PWM_DIRECT
    hal.spindle.get_pwm = spindleGetPWM;
    hal.spindle.update_pwm = spindle_set_speed;
#else
    hal.spindle.update_rpm = spindleUpdateRPM;
#endif

    hal.control.get_state = systemGetState;

#if USB_SERIAL_CDC
    usbInit();
    hal.stream.read = usbGetC;
    hal.stream.write = usbWriteS;
    hal.stream.write_all = usbWriteS;
    hal.stream.get_rx_buffer_available = usbRxFree;
    hal.stream.reset_read_buffer = usbRxFlush;
    hal.stream.cancel_read_buffer = usbRxCancel;
    hal.stream.suspend_read = usbSuspendInput;
#else
    serialInit();
    hal.stream.read = serialGetC;
    hal.stream.write = serialWriteS;
    hal.stream.write_all = serialWriteS;
    hal.stream.get_rx_buffer_available = serialRxFree;
    hal.stream.reset_read_buffer = serialRxFlush;
    hal.stream.cancel_read_buffer = serialRxCancel;
    hal.stream.suspend_read = serialSuspendInput;
#endif

#if EEPROM_ENABLE
    i2c_eeprom_init();
#elif FLASH_ENABLE
    hal.nvs.type = NVS_Flash;
    hal.nvs.memcpy_from_flash = memcpy_from_flash;
    hal.nvs.memcpy_to_flash = memcpy_to_flash;
#else
    hal.nvs.type = NVS_None;
#endif

    hal.irq_enable = __enable_irq;
    hal.irq_disable = __disable_irq;
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = getElapsedTicks;

#ifdef HAS_KEYPAD
    hal.execute_realtime = process_keypress;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

#ifdef SAFETY_DOOR_PIN
    hal.signals_cap.safety_door_ajar = On;
#endif
    hal.driver_cap.spindle_dir = On;
    hal.driver_cap.variable_spindle = On;
#ifdef COOLANT_MIST_PORT
    hal.driver_cap.mist_control = On;
#endif
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;
#if SDCARD_ENABLE
    hal.driver_cap.sd_card = On;
#endif

#ifdef HAS_BOARD_INIT
    board_init();
#endif

#if TRINAMIC_ENABLE
    trinamic_init();
#endif

    my_plugin_init();

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 8;
}

/* interrupt handlers */

// Main stepper driver
void STEPPER_IRQHandler (void)
{
    STEPPER_TIMER->IR = STEPPER_TIMER->IR;
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
// initiated after the STEP_PULSE_DELAY time period has elapsed. The ISR TIMER2_OVF interrupt
// will then trigger after the appropriate settings.pulse_microseconds, as in normal operation.
// The new timing between direction, step pulse, and step complete events are setup in the
// st_wake_up() routine.

// This interrupt is enabled when Grbl sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
void STEPPULSE_IRQHandler (void)
{
    PULSE_TIMER->IR = PULSE_TIMER->IR;

    if(PULSE_TIMER->MR[0] == pulse_length)
        stepperSetStepOutputs((axes_signals_t){0}); // End step pulse.
    else {
        stepperSetStepOutputs(next_step_outbits);   // Begin step pulse.
        PULSE_TIMER->TCR = 0b10;
        PULSE_TIMER->MR[0] = pulse_length;
        PULSE_TIMER->TCR = 0b01;
    }
}

inline static bool enqueue_debounce (input_signal_t *signal)
{
    bool ok;
    uint_fast8_t bptr = (debounce_queue.head + 1) & (DEBOUNCE_QUEUE - 1);

    if((ok = bptr != debounce_queue.tail)) {
        debounce_queue.signal[debounce_queue.head] = signal;
        debounce_queue.head = bptr;
    }

    return ok;
}

// Returns NULL if no debounce checks enqueued
inline static input_signal_t *get_debounce (void)
{
    input_signal_t *signal = NULL;
    uint_fast8_t bptr = debounce_queue.tail;

    if(bptr != debounce_queue.head) {
        signal = debounce_queue.signal[bptr++];
        debounce_queue.tail = bptr & (DEBOUNCE_QUEUE - 1);
    }

    return signal;
}

void DEBOUNCE_IRQHandler (void)
{
    input_signal_t *signal;

    DEBOUNCE_TIMER->IR = MR0IFG;
    DEBOUNCE_TIMER->TCR = 0;

    while((signal = get_debounce())) {

        if(signal->port == LPC_GPIO0)
            gpio0_int_enable(signal->bit, signal->intr_type);
        else if(signal->port == LPC_GPIO2)
            gpio2_int_enable(signal->bit, signal->intr_type);

        if(DIGITAL_IN(signal->port, signal->bit) == (signal->intr_type == GPIO_Intr_Falling ? 0 : 1))
          switch(signal->group) {

            case INPUT_GROUP_LIMIT:
                {
                    limit_signals_t state = limitsGetState();
                    if(limit_signals_merge(state).value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
                        hal.limits.interrupt_callback(state);
                }
                break;

            case INPUT_GROUP_CONTROL:
                hal.control.interrupt_callback(systemGetState());
                break;
        }
    }
}

void GPIO_IRQHandler (void)
{
    bool debounce = false;
    uint32_t grp = 0, i = 0;
    uint32_t istat = LPC_GPIOINT->STATUS, iflags;

    if(istat & P0Int) {
        iflags = LPC_GPIOINT->IO0.STATR | LPC_GPIOINT->IO0.STATF;
        LPC_GPIOINT->IO0.CLR = iflags;

        while(gpio0_signals[i].port != NULL) {
            if(iflags & gpio0_signals[i].bit) {
                if(gpio0_signals[i].debounce && enqueue_debounce(&gpio0_signals[i])) {
                    gpio0_int_enable(gpio0_signals[i].bit, GPIO_Intr_None);
                    debounce = true;
                } else
                    grp |= gpio0_signals[i].group;
            }
            i++;
        }
    }

    if(istat & P2Int) {
        iflags = LPC_GPIOINT->IO2.STATR | LPC_GPIOINT->IO2.STATF;
        LPC_GPIOINT->IO2.CLR = iflags;

        while(gpio2_signals[i].port != NULL) {
            if(iflags & gpio2_signals[i].bit) {
                if(gpio2_signals[i].debounce && enqueue_debounce(&gpio2_signals[i])) {
                    gpio2_int_enable(gpio0_signals[i].bit, GPIO_Intr_None);
                    debounce = true;
                } else
                    grp |= gpio2_signals[i].group;
            }
            i++;
        }
    }

    if(debounce) {
        // Reset and start
        DEBOUNCE_TIMER->TCR = 0;
        DEBOUNCE_TIMER->TC = 1;
        DEBOUNCE_TIMER->TCR = 0b10;
        while(DEBOUNCE_TIMER->TC != 0);
        DEBOUNCE_TIMER->TCR = 0b01;
    }

    if(grp & INPUT_GROUP_LIMIT)
        hal.limits.interrupt_callback(limitsGetState());

    if(grp & INPUT_GROUP_CONTROL)
        hal.control.interrupt_callback(systemGetState());

#if KEYPAD_ENABLE
    if(grp & INPUT_GROUP_KEYPAD)
        keypad_keyclick_handler(BITBAND_PERI(KEYPAD_PORT->PIO_PDSR, KEYPAD_PIN));
#endif
}

// Interrupt handler for 1 ms interval timer
void SysTick_Handler (void)
{
    elapsed_tics++;

#ifdef LIMITS_POLL_PORT // Poll limit pins when hard limits enabled
    static uint32_t limits_state = 0, limits = 0;
    if(settings.limits.flags.hard_enabled) {
        limits = (LIMITS_POLL_PORT->PIN ^ limits_invert) & LIMIT_MASK;
        if(limits_state && limits == 0 && !limits_debounce)
            limits_state = 0;
        else if(limits_state != limits && limits) {

           uint32_t i = 0;
           while(gpio1_signals[i].port != NULL) {
                if(limits & gpio1_signals[i].bit && gpio1_signals[i].debounce && enqueue_debounce(&gpio1_signals[i]))
                    limits_debounce = true;
                i++;
            }

            if(limits_debounce) {
                // Reset and start
                DEBOUNCE_TIMER->TCR = 0;
                DEBOUNCE_TIMER->TC = 1;
                DEBOUNCE_TIMER->TCR = 0b10;
                while(DEBOUNCE_TIMER->TC != 0);
                DEBOUNCE_TIMER->TCR = 0b01;
            } else
                hal.limits.interrupt_callback(limitsGetState());

            limits_state = limits;
        }
    }
#endif

    if(delay.ms && !(--delay.ms) && delay.callback) {
        delay.callback();
        delay.callback = NULL;
    }
}
