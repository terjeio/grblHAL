/*

  driver.c - driver code for Atmel SAM3X8E ARM processor

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

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

#include "grbl/limits.h"

#if USB_SERIAL_CDC
#include "usb_serial.h"
#endif

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "diskio.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#if KEYPAD_ENABLE
#include "keypad/keypad.h"
#endif

#define DEBOUNCE_QUEUE 8 // Must be a power of 2

#if X_AUTO_SQUARE || Y_AUTO_SQUARE || Z_AUTO_SQUARE
#define SQUARING_ENABLED
#endif

#if defined(X_LIMIT_PIN_MAX) || defined(Y_LIMIT_PIN_MAX) || defined(Z_LIMIT_PIN_MAX) || defined(A_LIMIT_PIN_MAX) || defined(B_LIMIT_PIN_MAX) || defined(C_LIMIT_PIN_MAX)
#define DUAL_LIMIT_SWITCHES
#else
  #ifdef SQUARING_ENABLED
    #error "Squaring requires at least one axis with dual switch inputs!"
  #endif
#endif

#define INPUT_GROUP_CONTROL 1
#define INPUT_GROUP_PROBE   2
#define INPUT_GROUP_LIMIT   4
#define INPUT_GROUP_KEYPAD  8

#ifndef OUTPUT
#define OUTPUT true
#endif
#ifndef INPUT
#define INPUT false
#endif

typedef enum {
    Input_Probe = 0,
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
    Pio *port;
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

static bool IOInitDone = false;
static uint32_t pulse_length, pulse_delay;
static axes_signals_t next_step_outbits;
static delay_t delay_ms = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
static debounce_queue_t debounce_queue = {0};
static input_signal_t a_signals[10] = {0}, b_signals[10] = {0}, c_signals[10] = {0}, d_signals[10] = {0};
static probe_state_t probe = {
    .connected = On
};
#ifdef SQUARING_ENABLED
static axes_signals_t motors_1 = {AXES_BITMASK}, motors_2 = {AXES_BITMASK};
#endif

#ifndef VFD_SPINDLE
static bool pwmEnabled = false;
static spindle_pwm_t spindle_pwm = {0};

static void spindle_set_speed (uint_fast16_t pwm_value);
#endif

#if MODBUS_ENABLE
static modbus_stream_t modbus_stream = {0};
#endif

static input_signal_t inputpin[] = {
  #ifdef PROBE_PIN
    { .id = Input_Probe,        .port = PROBE_PORT,        .bit = PROBE_BIT,         .group = INPUT_GROUP_PROBE },
  #endif
  #ifdef RESET_PIN
    { .id = Input_Reset,        .port = RESET_PORT,        .bit = RESET_BIT,         .group = INPUT_GROUP_CONTROL },
  #endif
  #ifdef FEED_HOLD_PIN
    { .id = Input_FeedHold,     .port = FEED_HOLD_PORT,    .bit = FEED_HOLD_BIT,     .group = INPUT_GROUP_CONTROL },
  #endif
  #ifdef CYCLE_START_PIN
    { .id = Input_CycleStart,   .port = CYCLE_START_PORT,  .bit = CYCLE_START_BIT,   .group = INPUT_GROUP_CONTROL },
  #endif
  #ifdef SAFETY_DOOR_PIN
    { .id = Input_SafetyDoor,   .port = SAFETY_DOOR_PORT,  .bit = SAFETY_DOOR_BIT,   .group = INPUT_GROUP_CONTROL },
  #endif
    { .id = Input_LimitX,       .port = X_LIMIT_PORT,      .bit = X_LIMIT_BIT,       .group = INPUT_GROUP_LIMIT },
  #ifdef X_LIMIT_PIN_MAX
    { .id = Input_LimitX_Max,   .port = X_LIMIT_PORT_MAX,  .bit = X_LIMIT_BIT_MAX,   .group = INPUT_GROUP_LIMIT },
  #endif
    { .id = Input_LimitY,       .port = Y_LIMIT_PORT,      .bit = Y_LIMIT_BIT,       .group = INPUT_GROUP_LIMIT },
  #ifdef Y_LIMIT_PIN_MAX
    { .id = Input_LimitY_Max,   .port = Y_LIMIT_PORT_MAX,  .bit = Y_LIMIT_BIT_MAX,   .group = INPUT_GROUP_LIMIT },
  #endif
    { .id = Input_LimitZ,       .port = Z_LIMIT_PORT,      .bit = Z_LIMIT_BIT,       .group = INPUT_GROUP_LIMIT },
  #ifdef Z_LIMIT_PIN_MAX
    { .id = Input_LimitZ_Max,   .port = Z_LIMIT_PORT_MAX,  .bit = Z_LIMIT_BIT_MAX,   .group = INPUT_GROUP_LIMIT },
  #endif
  #ifdef A_LIMIT_PIN
    { .id = Input_LimitA,       .port = A_LIMIT_PORT,      .bit = A_LIMIT_BIT,       .group = INPUT_GROUP_LIMIT },
  #endif
  #ifdef A_LIMIT_PIN_MAX
    { .id = Input_LimitA_Max,   .port = A_LIMIT_PORT_MAX,  .bit = A_LIMIT_BIT_MAX,   .group = INPUT_GROUP_LIMIT },
  #endif
  #ifdef B_LIMIT_PIN
    { .id = Input_LimitB,       .port = B_LIMIT_PORT,      .bit = B_LIMIT_BIT,       .group = INPUT_GROUP_LIMIT },
  #endif
  #ifdef B_LIMIT_PIN_MAX
    { .id = Input_LimitB_Max,   .port = B_LIMIT_PORT_MAX,  .bit = B_LIMIT_BIT_MAX,   .group = INPUT_GROUP_LIMIT },
  #endif
  #ifdef C_LIMIT_PIN
    { .id = Input_LimitC,       .port = C_LIMIT_PORT,      .bit = C_LIMIT_BIT,       .group = INPUT_GROUP_LIMIT },
  #endif
  #ifdef C_LIMIT_PIN_MAX
    { .id = Input_LimitC_Max,   .port = C_LIMIT_PORT_MAX,  .bit = C_LIMIT_BIT_MAX,   .group = INPUT_GROUP_LIMIT },
  #endif
  #if KEYPAD_ENABLE
    { .id = Input_KeypadStrobe, .port = KEYPAD_PORT,       .bit = KEYPAD_BIT, .group = INPUT_GROUP_KEYPAD }
  #endif
};

static uint32_t vectorTable[sizeof(DeviceVectors) / sizeof(uint32_t)] __attribute__(( aligned (0x100ul) ));

static void spindle_set_speed (uint_fast16_t pwm_value);

static void SysTick_IRQHandler (void);
static void STEPPER_IRQHandler (void);
static void STEP_IRQHandler (void);
static void STEPDELAY_IRQHandler (void);
static void PIOA_IRQHandler (void);
static void PIOB_IRQHandler (void);
static void PIOC_IRQHandler (void);
static void PIOD_IRQHandler (void);
static void DEBOUNCE_IRQHandler (void);

extern void Dummy_Handler(void);

inline __attribute__((always_inline)) void IRQRegister(int32_t IRQnum, void (*IRQhandler)(void))
{
    vectorTable[IRQnum + 16] = (uint32_t)IRQhandler;
}

void IRQUnRegister(int32_t IRQnum)
{
    vectorTable[IRQnum + 16] = (uint32_t)Dummy_Handler;
}

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if((delay_ms.ms = ms) > 0) {
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        if(!(delay_ms.callback = callback))
            while(delay_ms.ms);
    } else if(callback)
        callback();
}

// Set stepper pulse output pins

#ifdef SQUARING_ENABLED

inline static __attribute__((always_inline)) void set_step_outputs (axes_signals_t step_outbits_1)
{
    axes_signals_t step_outbits_2;

    step_outbits_2.mask = (step_outbits_1.mask & motors_2.mask) ^ settings.steppers.step_invert.mask;
    step_outbits_1.mask = (step_outbits_1.mask & motors_1.mask) ^ settings.steppers.step_invert.mask;

    BITBAND_PERI(X_STEP_PORT->PIO_ODSR, X_STEP_PIN) = step_outbits_1.x;
  #ifdef X2_STEP_PIN
    BITBAND_PERI(X2_STEP_PORT->PIO_ODSR, X2_STEP_PIN) = step_outbits_2.x;
  #endif

    BITBAND_PERI(Y_STEP_PORT->PIO_ODSR, Y_STEP_PIN) = step_outbits_1.y;
  #ifdef Y2_STEP_PIN
    BITBAND_PERI(Y2_STEP_PORT->PIO_ODSR, Y2_STEP_PIN) = step_outbits_2.y;
  #endif
 
    BITBAND_PERI(Z_STEP_PORT->PIO_ODSR, Z_STEP_PIN) = step_outbits_1.z;
  #ifdef Z2_STEP_PIN
    BITBAND_PERI(Z2_STEP_PORT->PIO_ODSR, Z2_STEP_PIN) = step_outbits_2.z;
  #endif

  #ifdef A_STEP_PIN
    BITBAND_PERI(A_STEP_PORT->PIO_ODSR, A_STEP_PIN) = step_outbits_1.a;
  #endif
  #ifdef B_STEP_PIN
    BITBAND_PERI(B_STEP_PORT->PIO_ODSR, B_STEP_PIN) = step_outbits_1.b;
  #endif
  #ifdef C_STEP_PIN
    BITBAND_PERI(C_STEP_PORT->PIO_ODSR, C_STEP_PIN) = step_outbits_1.c;
  #endif
}

#else // SQUARING DISABLED

inline static void __attribute__((always_inline)) set_step_outputs (axes_signals_t step_outbits)
{
    step_outbits.value ^= settings.steppers.step_invert.mask;

    BITBAND_PERI(X_STEP_PORT->PIO_ODSR, X_STEP_PIN) = step_outbits.x;
  #ifdef X2_STEP_PIN
    BITBAND_PERI(X2_STEP_PORT->PIO_ODSR, X2_STEP_PIN) = step_outbits.x;
  #endif
    
    BITBAND_PERI(Y_STEP_PORT->PIO_ODSR, Y_STEP_PIN) = step_outbits.y;
  #ifdef Y2_STEP_PIN
    BITBAND_PERI(Y2_STEP_PORT->PIO_ODSR, Y2_STEP_PIN) = step_outbits.y;
  #endif

    BITBAND_PERI(Z_STEP_PORT->PIO_ODSR, Z_STEP_PIN) = step_outbits.z;
  #ifdef Z2_STEP_PIN
    BITBAND_PERI(Z2_STEP_PORT->PIO_ODSR, Z2_STEP_PIN) = step_outbits.z;
  #endif 

  #ifdef A_STEP_PIN
    BITBAND_PERI(A_STEP_PORT->PIO_ODSR, A_STEP_PIN) = step_outbits.a;
  #endif
  #ifdef B_STEP_PIN
    BITBAND_PERI(B_STEP_PORT->PIO_ODSR, B_STEP_PIN) = step_outbits.b;
  #endif
  #ifdef C_STEP_PIN
    BITBAND_PERI(C_STEP_PORT->PIO_ODSR, C_STEP_PIN) = step_outbits.c;
  #endif
}

#endif

// Set stepper direction output pins
inline static __attribute__((always_inline)) void set_dir_outputs (axes_signals_t dir_outbits)
{
    dir_outbits.value ^= settings.steppers.dir_invert.mask;

    BITBAND_PERI(X_DIRECTION_PORT->PIO_ODSR, X_DIRECTION_PIN) = dir_outbits.x;
  #ifdef X2_DIRECTION_PIN
    BITBAND_PERI(X2_DIRECTION_PORT->PIO_ODSR, X2_DIRECTION_PIN) = dir_outbits.x;
  #endif

    BITBAND_PERI(Y_DIRECTION_PORT->PIO_ODSR, Y_DIRECTION_PIN) = dir_outbits.y;
  #ifdef Y2_DIRECTION_PIN
    BITBAND_PERI(Y2_DIRECTION_PORT->PIO_ODSR, Y2_DIRECTION_PIN) = dir_outbits.y;
  #endif

    BITBAND_PERI(Z_DIRECTION_PORT->PIO_ODSR, Z_DIRECTION_PIN) = dir_outbits.z;
  #ifdef Z2_DIRECTION_PIN
    BITBAND_PERI(Z2_DIRECTION_PORT->PIO_ODSR, Z2_DIRECTION_PIN) = dir_outbits.z;
  #endif

  #ifdef A_STEP_PIN
    BITBAND_PERI(A_DIRECTION_PORT->PIO_ODSR, A_DIRECTION_PIN) = dir_outbits.a;
  #endif
  #ifdef B_STEP_PIN
    BITBAND_PERI(B_DIRECTION_PORT->PIO_ODSR, B_DIRECTION_PIN) = dir_outbits.b;
  #endif
  #ifdef C_STEP_PIN
    BITBAND_PERI(C_DIRECTION_PORT->PIO_ODSR, C_DIRECTION_PIN) = dir_outbits.c;
  #endif
}

// Enable/disable stepper motors
static void stepperEnable (axes_signals_t enable)
{
    enable.value ^= settings.steppers.enable_invert.mask;
#if TRINAMIC_ENABLE == 2130 && TRINAMIC_I2C
    trinamic_stepper_enable(enable);
#else
    BITBAND_PERI(X_DISABLE_PORT->PIO_ODSR, X_DISABLE_PIN) = enable.x;
  #ifdef X2_DISABLE_PIN
    BITBAND_PERI(X2_DISABLE_PORT->PIO_ODSR, X2_DISABLE_PIN) = enable.x;
  #endif
  #ifdef Y_DISABLE_PIN
    BITBAND_PERI(Y_DISABLE_PORT->PIO_ODSR, Y_DISABLE_PIN) = enable.y;
  #endif
  #ifdef Y2_DISABLE_PIN
    BITBAND_PERI(Y2_DISABLE_PORT->PIO_ODSR, Y2_DISABLE_PIN) = enable.y;
  #endif
  #ifdef Z_DISABLE_PIN
    BITBAND_PERI(Z_DISABLE_PORT->PIO_ODSR, Z_DISABLE_PIN) = enable.z;
  #endif
  #ifdef Z2_DISABLE_PIN
    BITBAND_PERI(Z2_DISABLE_PORT->PIO_ODSR, Z2_DISABLE_PIN) = enable.z;
  #endif
  #ifdef A_DISABLE_PIN
    BITBAND_PERI(A_DISABLE_PORT->PIO_ODSR, A_DISABLE_PIN) = enable.a;
  #endif
  #ifdef B_DISABLE_PIN
    BITBAND_PERI(B_DISABLE_PORT->PIO_ODSR, B_DISABLE_PIN) = enable.b;
  #endif
  #ifdef C_DISABLE_PIN
    BITBAND_PERI(C_DISABLE_PORT->PIO_ODSR, C_DISABLE_PIN) = enable.c;
  #endif
#endif
}

// Resets and enables stepper driver ISR timer
static void stepperWakeUp (void)
{
    stepperEnable((axes_signals_t){AXES_BITMASK});

    STEPPER_TIMER.TC_RC = 1000;
    STEPPER_TIMER.TC_CCR = TC_CCR_CLKEN|TC_CCR_SWTRG;
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    STEPPER_TIMER.TC_CCR = TC_CCR_CLKDIS;

    if(clear_signals) {
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}

// Sets up stepper driver interrupt timeout, AMASS version
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    STEPPER_TIMER.TC_CCR = TC_CCR_CLKDIS;
// Limit min steps/s to about 2 (hal.f_step_timer @ 20MHz)
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    STEPPER_TIMER.TC_RC = cycles_per_tick < (1UL << 18) ? cycles_per_tick : (1UL << 18) - 1UL;
#else
    STEPPER_TIMER.TC_RC = cycles_per_tick < (1UL << 23) ? cycles_per_tick : (1UL << 23) - 1UL;
#endif
    STEPPER_TIMER.TC_CCR = TC_CCR_CLKEN|TC_CCR_SWTRG;
}

// Sets stepper direction and pulse pins and starts a step pulse.
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_change)
        set_dir_outputs(stepper->dir_outbits);

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        STEP_TIMER.TC_CCR = TC_CCR_SWTRG;
    }
}

// Start a stepper pulse, delay version.
// Note: delay is only added when there is a direction change and a pulse to be output.
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->dir_change) {

        set_dir_outputs(stepper->dir_outbits);

        if(stepper->step_outbits.value) {

            next_step_outbits = stepper->step_outbits; // Store out_bits

            IRQRegister(STEP_TIMER_IRQn, STEPDELAY_IRQHandler);

            STEP_TIMER.TC_RC = pulse_delay;
            STEP_TIMER.TC_CCR = TC_CCR_SWTRG;
        }

        return;
    }

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        STEP_TIMER.TC_CCR = TC_CCR_SWTRG;
    }
}

#ifdef DUAL_LIMIT_SWITCHES

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
// Dual limit switch inputs per axis version. Only one needs to be dual input!
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};

    signals.min.mask = signals.max.mask = settings.limits.invert.mask;
#ifdef SQUARING_ENABLED
    signals.min2.mask = settings.limits.invert.mask;
#endif
    
    signals.min.x = BITBAND_PERI(X_LIMIT_PORT->PIO_PDSR, X_LIMIT_PIN);
    signals.min.y = BITBAND_PERI(Y_LIMIT_PORT->PIO_PDSR, Y_LIMIT_PIN);
    signals.min.z = BITBAND_PERI(Z_LIMIT_PORT->PIO_PDSR, Z_LIMIT_PIN);
#ifdef A_LIMIT_PIN
    signals.min.a = BITBAND_PERI(A_LIMIT_PORT->PIO_PDSR, A_LIMIT_PIN);
#endif
#ifdef B_LIMIT_PIN
    signals.min.b = BITBAND_PERI(B_LIMIT_PORT->PIO_PDSR, B_LIMIT_PIN);
#endif
#ifdef C_LIMIT_PIN
    signals.min.c = BITBAND_PERI(C_LIMIT_PORT->PIO_PDSR, C_LIMIT_PIN);
#endif

#ifdef X_LIMIT_PIN_MAX
  #if X_AUTO_SQUARE
    signals.min2.x = BITBAND_PERI(X_LIMIT_PORT_MAX->PIO_PDSR, X_LIMIT_PIN_MAX);
  #else
    signals.max.x = BITBAND_PERI(X_LIMIT_PORT_MAX->PIO_PDSR, X_LIMIT_PIN_MAX);
  #endif
#endif
#ifdef Y_LIMIT_PIN_MAX
  #if Y_AUTO_SQUARE
    signals.min2.y = BITBAND_PERI(Y_LIMIT_PORT_MAX->PIO_PDSR, Y_LIMIT_PIN_MAX);
  #else
    signals.max.y = BITBAND_PERI(Y_LIMIT_PORT_MAX->PIO_PDSR, Y_LIMIT_PIN_MAX);
  #endif
#endif
#ifdef Z_LIMIT_PIN_MAX
  #if Z_AUTO_SQUARE
    signals.min2.z = BITBAND_PERI(Z_LIMIT_PORT_MAX->PIO_PDSR, Z_LIMIT_PIN_MAX);
  #else
    signals.max.z = BITBAND_PERI(Z_LIMIT_PORT_MAX->PIO_PDSR, Z_LIMIT_PIN_MAX);
  #endif
#endif
#ifdef A_LIMIT_PIN_MAX
    signals.max.a = BITBAND_PERI(A_LIMIT_PORT_MAX->PIO_PDSR, A_LIMIT_PIN_MAX);
#endif
#ifdef B_LIMIT_PIN_MAX
    signals.max.b = BITBAND_PERI(B_LIMIT_PORT_MAX->PIO_PDSR, B_LIMIT_PIN_MAX);
#endif
#ifdef C_LIMIT_PIN_MAX
    signals.max.c = BITBAND_PERI(C_LIMIT_PORT_MAX->PIO_PDSR, C_LIMIT_PIN_MAX);
#endif

    if (settings.limits.invert.mask) {
        signals.min.value ^= settings.limits.invert.mask;
        signals.max.value ^= settings.limits.invert.mask;
#ifdef SQUARING_ENABLED
        signals.min2.mask ^= settings.limits.invert.mask;
#endif
    }

    return signals;
}

#else // SINGLE INPUT LIMIT SWITCHES

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
// Single limit switch input per axis version.
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};
    
    signals.min.x = BITBAND_PERI(X_LIMIT_PORT->PIO_PDSR, X_LIMIT_PIN);

    signals.min.y = BITBAND_PERI(Y_LIMIT_PORT->PIO_PDSR, Y_LIMIT_PIN);

    signals.min.z = BITBAND_PERI(Z_LIMIT_PORT->PIO_PDSR, Z_LIMIT_PIN);

#ifdef A_LIMIT_PIN
    signals.min.a = BITBAND_PERI(A_LIMIT_PORT->PIO_PDSR, A_LIMIT_PIN);
#endif
#ifdef B_LIMIT_PIN
    signals.min.b = BITBAND_PERI(B_LIMIT_PORT->PIO_PDSR, B_LIMIT_PIN);
#endif
#ifdef C_LIMIT_PIN
    signals.min.c = BITBAND_PERI(C_LIMIT_PORT->PIO_PDSR, C_LIMIT_PIN);
#endif

    if (settings.limits.invert.mask)
        signals.min.value ^= settings.limits.invert.mask;

    return signals;
}

#endif

#ifdef SQUARING_ENABLED

static axes_signals_t getAutoSquaredAxes (void)
{
    axes_signals_t ganged = {0};

    #if X_AUTO_SQUARE
    ganged.x = On;
#endif
#if Y_AUTO_SQUARE
    ganged.y = On;
#endif
#if Z_AUTO_SQUARE
    ganged.z = On;
#endif

    return ganged;
}

// Enable/disable motors for auto squaring of ganged axes
static void StepperDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    motors_1.mask = (mode == SquaringMode_A || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
    motors_2.mask = (mode == SquaringMode_B || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
}

#endif

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing)
{
    uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);

    on = on && settings.limits.flags.hard_enabled;

    do {
        if(inputpin[--i].group == INPUT_GROUP_LIMIT) {
            if(on)
                inputpin[i].port->PIO_IER = inputpin[i].bit;
            else
                inputpin[i].port->PIO_IDR = inputpin[i].bit;    
        }
    } while(i);

  #if TRINAMIC_ENABLE == 2130
    trinamic_homing(homing);
  #endif
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
static control_signals_t systemGetState (void)
{
    control_signals_t signals = {0};

    signals.value = settings.control_invert.mask;

    #ifdef RESET_PIN
    signals.reset = BITBAND_PERI(RESET_PORT->PIO_PDSR, RESET_PIN);
  #endif
  #ifdef FEED_HOLD_PIN
    signals.feed_hold = BITBAND_PERI(FEED_HOLD_PORT->PIO_PDSR, FEED_HOLD_PIN);
  #endif
  #ifdef CYCLE_START_PIN
    signals.cycle_start = BITBAND_PERI(CYCLE_START_PORT->PIO_PDSR, CYCLE_START_PIN);
  #endif
  #ifdef SAFETY_DOOR_PIN
    signals.safety_door_ajar = BITBAND_PERI(SAFETY_DOOR_PORT->PIO_PDSR, SAFETY_DOOR_PIN);
  #endif

    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

    return signals;
}

#ifdef PROBE_PIN

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
    probe.triggered = Off;
    probe.is_probing = probing;
    probe.inverted = is_probe_away ? !settings.probe.invert_probe_pin : settings.probe.invert_probe_pin;
}


// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;
    state.triggered = BITBAND_PERI(PROBE_PORT->PIO_PDSR, PROBE_PIN) ^ probe.inverted;

    return state;
}

#endif

#ifndef VFD_SPINDLE

// Static spindle (off, on cw & on ccw)

inline static void spindle_off (void)
{
    BITBAND_PERI(SPINDLE_ENABLE_PORT->PIO_ODSR, SPINDLE_ENABLE_PIN) = settings.spindle.invert.on;
}

inline static void spindle_on (void)
{
    BITBAND_PERI(SPINDLE_ENABLE_PORT->PIO_ODSR, SPINDLE_ENABLE_PIN) = !settings.spindle.invert.on;
}

inline static void spindle_dir (bool ccw)
{
  #ifdef SPINDLE_DIRECTION_PIN
    if(hal.driver_cap.spindle_dir)
        BITBAND_PERI(SPINDLE_DIRECTION_PORT->PIO_ODSR, SPINDLE_DIRECTION_PIN) = (ccw ^ settings.spindle.invert.ccw);
  #endif
}

// Start or stop spindle
static void spindleSetState (spindle_state_t state, float rpm)
{
    (void)rpm;

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
            SPINDLE_PWM_TIMER.TC_RA = spindle_pwm.period - spindle_pwm.off_value;
            SPINDLE_PWM_TIMER.TC_CMR &= ~TC_CMR_CPCSTOP;
            SPINDLE_PWM_TIMER.TC_CCR = TC_CCR_CLKEN|TC_CCR_SWTRG;
        } else
            SPINDLE_PWM_TIMER.TC_CMR |= TC_CMR_CPCSTOP; // Ensure output is low, by setting timer to stop at TCC match
    } else {
        SPINDLE_PWM_TIMER.TC_RA = spindle_pwm.period == pwm_value ? 1 : spindle_pwm.period - pwm_value;
        if(!pwmEnabled) {
            spindle_on();
            pwmEnabled = true;
            SPINDLE_PWM_TIMER.TC_CMR &= ~TC_CMR_CPCSTOP;
            SPINDLE_PWM_TIMER.TC_CCR = TC_CCR_CLKEN|TC_CCR_SWTRG;
        }
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
        spindle_dir(state.ccw);
        spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
    }
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {settings.spindle.invert.mask};

    state.on = BITBAND_PERI(SPINDLE_ENABLE_PORT->PIO_ODSR, SPINDLE_ENABLE_PIN) != 0;
  #ifdef SPINDLE_DIRECTION_PIN
    state.ccw = hal.driver_cap.spindle_dir && BITBAND_PERI(SPINDLE_DIRECTION_PORT->PIO_ODSR, SPINDLE_DIRECTION_PIN) != 0;
  #endif

    state.value ^= settings.spindle.invert.mask;
    if(pwmEnabled)
        state.on = On;

    return state;
}

// end spindle code

#endif

#ifdef DEBUGOUT
void debug_out (bool on)
{
    BITBAND_PERI(COOLANT_MIST_PIN, on);
}
#endif

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;

  #ifdef COOLANT_FLOOD_PIN
    BITBAND_PERI(COOLANT_FLOOD_PORT->PIO_ODSR, COOLANT_FLOOD_PIN) = mode.flood;
  #endif
  #ifdef COOLANT_MIST_PIN
    BITBAND_PERI(COOLANT_MIST_PORT->PIO_ODSR, COOLANT_MIST_PIN) = mode.mist;
  #endif
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

  #ifdef COOLANT_FLOOD_PIN
    state.flood = BITBAND_PERI(COOLANT_FLOOD_PORT->PIO_ODSR, COOLANT_FLOOD_PIN);
  #endif
  #ifdef COOLANT_MIST_PIN
    state.mist  = BITBAND_PERI(COOLANT_MIST_PORT->PIO_ODSR, COOLANT_MIST_PIN);
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

static void PIO_Mode (Pio *port, uint32_t bit, bool mode)
{
    port->PIO_WPMR = PIO_WPMR_WPKEY(0x50494F);

    port->PIO_ABSR &= ~bit;

    if(mode == OUTPUT)
        port->PIO_OER = bit;
    else
        port->PIO_ODR = bit;

    port->PIO_PER = bit;

    port->PIO_WPMR = PIO_WPMR_WPKEY(0x50494F)|PIO_WPMR_WPEN;
}

static void PIO_InputMode (Pio *port, uint32_t bit, bool no_pullup, gpio_intr_t intr_type, bool irq_enable)
{
    port->PIO_WPMR = PIO_WPMR_WPKEY(0x50494F);

    port->PIO_PER = bit;
    port->PIO_ODR = bit;

    if(no_pullup)
        port->PIO_PUDR = bit;
    else
        port->PIO_PUER = bit;

    switch(intr_type) {

        case GPIO_Intr_Falling:
            port->PIO_AIMER = bit;
            port->PIO_ESR = bit;
            port->PIO_FELLSR = bit;
            break;

        case GPIO_Intr_Rising:
            port->PIO_AIMER = bit;
            port->PIO_ESR = bit;
            port->PIO_REHLSR = bit;
            break;

        case GPIO_Intr_Both:
            port->PIO_AIMDR = bit;
            port->PIO_ESR = bit;
            break;

        default:
            break;
    }

    if(irq_enable && intr_type != GPIO_Intr_None)
        port->PIO_IER = bit;
    else
        port->PIO_IDR = bit;

    port->PIO_WPMR = PIO_WPMR_WPKEY(0x50494F)|PIO_WPMR_WPEN;
} 

// Configures perhipherals when settings are initialized or changed
void settings_changed (settings_t *settings)
{
    if(IOInitDone) {

        stepperEnable(settings->steppers.deenergize);

#ifdef SQUARING_ENABLED
        hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
#endif

      #ifndef VFD_SPINDLE
        if(hal.driver_cap.variable_spindle && spindle_precompute_pwm_values(&spindle_pwm, hal.f_step_timer)) {
            SPINDLE_PWM_TIMER.TC_RC = spindle_pwm.period;
            hal.spindle.set_state = spindleSetStateVariable;
        } else
            hal.spindle.set_state = spindleSetState;
      #endif

        pulse_length = (uint32_t)(42.0f * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY)) - 1;

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            int32_t delay = (uint32_t)(42.0f * (settings->steppers.pulse_delay_microseconds - 0.6f));
            pulse_delay = delay < 2 ? 2 : delay;
            hal.stepper.pulse_start = stepperPulseStartDelayed;
        } else
            hal.stepper.pulse_start = stepperPulseStart;

        IRQRegister(STEP_TIMER_IRQn, STEP_IRQHandler);
        STEP_TIMER.TC_RC = pulse_length;
        STEP_TIMER.TC_IER = TC_IER_CPCS; // Enable step end interrupt

        /****************************************
         *  Control, limit & probe pins config  *
         ****************************************/

        bool pullup = true, irq_enable = false;

        control_signals_t control_fei;
        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        NVIC_DisableIRQ(PIOA_IRQn);
        NVIC_DisableIRQ(PIOB_IRQn);
        NVIC_DisableIRQ(PIOC_IRQn);
        NVIC_DisableIRQ(PIOD_IRQn);

        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t), a = 0, b = 0, c = 0, d = 0;

        do {

            irq_enable = false;

            if(inputpin[--i].port != NULL) {

                switch(inputpin[i].id) {

                  #ifdef RESET_PIN
                    case Input_Reset:
                        irq_enable = true;
                        pullup = !settings->control_disable_pullup.reset;
                        inputpin[i].intr_type = control_fei.reset ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef FEED_HOLD_PIN
                    case Input_FeedHold:
                        irq_enable = true;
                        pullup = !settings->control_disable_pullup.feed_hold;
                        inputpin[i].intr_type = control_fei.feed_hold ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef CYCLE_START_PIN
                    case Input_CycleStart:
                        irq_enable = true;
                        pullup = !settings->control_disable_pullup.cycle_start;
                        inputpin[i].intr_type = control_fei.cycle_start ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef SAFETY_DOOR_PIN
                    case Input_SafetyDoor:     
                        irq_enable = true;
                        pullup = !settings->control_disable_pullup.safety_door_ajar;
                        inputpin[i].intr_type = control_fei.safety_door_ajar ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif

                  #ifdef PROBE_PIN
                    case Input_Probe:
                        pullup = hal.driver_cap.probe_pull_up;
                        break;
                  #endif

                    case Input_LimitX:
                        pullup = !settings->limits.disable_pullup.x;
                        inputpin[i].intr_type = limit_fei.x ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;

                    case Input_LimitY:
                        pullup = !settings->limits.disable_pullup.y;
                        inputpin[i].intr_type = limit_fei.y ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;

                    case Input_LimitZ:
                        pullup = !settings->limits.disable_pullup.z;
                        inputpin[i].intr_type = limit_fei.z ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;

                  #ifdef X_LIMIT_PIN_MAX
                    case Input_LimitX_Max:
                        pullup = !settings->limits.disable_pullup.x;
                        inputpin[i].intr_type = limit_fei.x ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef Y_LIMIT_PIN_MAX
                    case Input_LimitY_Max:
                        pullup = !settings->limits.disable_pullup.y;
                        inputpin[i].intr_type = limit_fei.y ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef Z_LIMIT_PIN_MAX
                    case Input_LimitZ_Max:
                        pullup = !settings->limits.disable_pullup.z;
                        inputpin[i].intr_type = limit_fei.z ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef A_LIMIT_PIN
                    case Input_LimitA:
                        pullup = !settings->limits.disable_pullup.a;
                        inputpin[i].intr_type = limit_fei.a ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef A_LIMIT_PIN_MAX
                    case Input_LimitA_Max:
                        pullup = !settings->limits.disable_pullup.a;
                        inputpin[i].intr_type = limit_fei.a ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef B_LIMIT_PIN
                    case Input_LimitB:
                        pullup = !settings->limits.disable_pullup.b;
                        inputpin[i].intr_type = limit_fei.b ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef B_LIMIT_PIN_MAX
                    case Input_LimitB_Max:
                        pullup = !settings->limits.disable_pullup.b;
                        inputpin[i].intr_type = limit_fei.b ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef C_LIMIT_PIN
                    case Input_LimitC:
                        pullup = !settings->limits.disable_pullup.c;
                        inputpin[i].intr_type = limit_fei.c ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif
                  #ifdef C_LIMIT_PIN_MAX
                    case Input_LimitC_Max:
                        pullup = !settings->limits.disable_pullup.c;
                        inputpin[i].intr_type = limit_fei.c ? GPIO_Intr_Falling : GPIO_Intr_Rising;
                        break;
                  #endif

                  #if KEYPAD_ENABLE
                    case Input_KeypadStrobe:
                        pullup = true;
                        irq_enable = true;
                        inputpin[i].intr_type = GPIO_Intr_Both; // -> any edge?
                        break;
                  #endif
                
                    default:
                        break;

                }

                PIO_InputMode(inputpin[i].port, inputpin[i].bit, !pullup, inputpin[i].intr_type, irq_enable);

                inputpin[i].debounce = hal.driver_cap.software_debounce && !(inputpin[i].group == INPUT_GROUP_PROBE || inputpin[i].group == INPUT_GROUP_KEYPAD);

                if(inputpin[i].port == PIOA)
                    memcpy(&a_signals[a++], &inputpin[i], sizeof(input_signal_t));
                else if(inputpin[i].port == PIOB)
                    memcpy(&b_signals[b++], &inputpin[i], sizeof(input_signal_t));
                else if(inputpin[i].port == PIOC)
                    memcpy(&c_signals[c++], &inputpin[i], sizeof(input_signal_t));
                else if(inputpin[i].port == PIOD)
                    memcpy(&d_signals[d++], &inputpin[i], sizeof(input_signal_t));
            }
        } while(i);

        // Clear and enable PIO interrupts
        if(a) {
            PIOA->PIO_ISR;
            IRQRegister(PIOA_IRQn, PIOA_IRQHandler);
            NVIC_ClearPendingIRQ(PIOA_IRQn);
            NVIC_EnableIRQ(PIOA_IRQn);
        }

        if(b) {
            PIOB->PIO_ISR;
            IRQRegister(PIOB_IRQn, PIOB_IRQHandler);
            NVIC_ClearPendingIRQ(PIOB_IRQn);
            NVIC_EnableIRQ(PIOB_IRQn);
        }

        if(c) {
            PIOC->PIO_ISR;
            IRQRegister(PIOC_IRQn, PIOC_IRQHandler);
            NVIC_ClearPendingIRQ(PIOC_IRQn);
            NVIC_EnableIRQ(PIOC_IRQn);
        }

        if(d) {
            PIOD->PIO_ISR;
            IRQRegister(PIOD_IRQn, PIOD_IRQHandler);
            NVIC_ClearPendingIRQ(PIOD_IRQn);
            NVIC_EnableIRQ(PIOD_IRQn);
        }

        if(settings->limits.flags.hard_enabled)
            hal.limits.enable(true, false);
    }
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
    pmc_enable_periph_clk(ID_PIOA);
    pmc_enable_periph_clk(ID_PIOB);
    pmc_enable_periph_clk(ID_PIOC);
    pmc_enable_periph_clk(ID_PIOD);
    pmc_enable_periph_clk(ID_TC0);
    pmc_enable_periph_clk(ID_TC1);
    pmc_enable_periph_clk(ID_TC2);
    pmc_enable_periph_clk(ID_TC3);
    pmc_enable_periph_clk(ID_TC6);
    pmc_enable_periph_clk(ID_TC7);
    pmc_enable_periph_clk(ID_TC8);

 // Stepper init

    TC0->TC_WPMR = TC_WPMR_WPKEY(0x54494D); //|TC_WPMR_WPEN;
    TC2->TC_WPMR = TC_WPMR_WPKEY(0x54494D); //|TC_WPMR_WPEN;

    STEPPER_TIMER.TC_CCR = TC_CCR_CLKDIS;
    STEPPER_TIMER.TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC;
    STEPPER_TIMER.TC_IER = TC_IER_CPCS;
    STEPPER_TIMER.TC_CCR = TC_CCR_CLKEN;

    IRQRegister(STEPPER_TIMER_IRQn, STEPPER_IRQHandler);
    NVIC_EnableIRQ(STEPPER_TIMER_IRQn); // Enable stepper interrupt
    NVIC_SetPriority(STEPPER_TIMER_IRQn, 2);

    STEP_TIMER.TC_CCR = TC_CCR_CLKDIS;
    STEP_TIMER.TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP|TC_CMR_CPCSTOP;
    STEP_TIMER.TC_IER = TC_IER_CPCS;
    STEP_TIMER.TC_CCR = TC_CCR_CLKEN;

    IRQRegister(STEP_TIMER_IRQn, STEP_IRQHandler);
    NVIC_EnableIRQ(STEP_TIMER_IRQn);    // Enable stepper interrupts
    NVIC_SetPriority(STEP_TIMER_IRQn, 0);

    PIO_Mode(X_STEP_PORT, X_STEP_BIT, OUTPUT);
    PIO_Mode(Y_STEP_PORT, Y_STEP_BIT, OUTPUT);
    PIO_Mode(Z_STEP_PORT, Z_STEP_BIT, OUTPUT);
  #ifdef A_STEP_PIN
    PIO_Mode(A_STEP_PORT, A_STEP_BIT, OUTPUT);
  #endif
  #ifdef B_STEP_PIN
    PIO_Mode(B_STEP_PORT, B_STEP_BIT, OUTPUT);
  #endif
  #ifdef C_STEP_PIN
    PIO_Mode(C_STEP_PORT, C_STEP_BIT, OUTPUT);
  #endif
  #ifdef X2_STEP_PIN
    PIO_Mode(X2_STEP_PORT, X2_STEP_BIT, OUTPUT);
  #endif
  #ifdef Y2_STEP_PIN
    PIO_Mode(Y2_STEP_PORT, Y2_STEP_BIT, OUTPUT);
  #endif
  #ifdef Z2_STEP_PIN
    PIO_Mode(Z2_STEP_PORT, Z2_STEP_BIT, OUTPUT);
  #endif

    PIO_Mode(X_DIRECTION_PORT, X_DIRECTION_BIT, OUTPUT);
    PIO_Mode(Y_DIRECTION_PORT, Y_DIRECTION_BIT, OUTPUT);
    PIO_Mode(Z_DIRECTION_PORT, Z_DIRECTION_BIT, OUTPUT);
  #ifdef A_DIRECTION_PIN
    PIO_Mode(A_DIRECTION_PORT, A_DIRECTION_BIT, OUTPUT);
  #endif
  #ifdef B_DIRECTION_PIN
    PIO_Mode(B_DIRECTION_PORT, B_DIRECTION_BIT, OUTPUT);
  #endif
  #ifdef C_DIRECTION_PIN
    PIO_Mode(C_DIRECTION_PORT, C_DIRECTION_BIT, OUTPUT);
  #endif
  #ifdef X2_DIRECTION_PIN
    PIO_Mode(X2_DIRECTION_PORT, X2_DIRECTION_BIT, OUTPUT);
  #endif
  #ifdef Y2_DIRECTION_PIN
    PIO_Mode(Y2_DIRECTION_PORT, Y2_DIRECTION_BIT, OUTPUT);
  #endif
  #ifdef Z2_DIRECTION_PIN
    PIO_Mode(Z2_DIRECTION_PORT, Z2_DIRECTION_BIT, OUTPUT);
  #endif

    PIO_Mode(X_DISABLE_PORT, X_DISABLE_BIT, OUTPUT);
  #ifdef Y_DISABLE_PIN
    PIO_Mode(Y_DISABLE_PORT, Y_DISABLE_BIT, OUTPUT);
  #endif
  #ifdef Z_DISABLE_PIN
    PIO_Mode(Z_DISABLE_PORT, Z_DISABLE_BIT, OUTPUT);
  #endif
  #ifdef A_DISABLE_PIN
    PIO_Mode(A_DISABLE_PORT, A_DISABLE_BIT, OUTPUT);
  #endif
  #ifdef B_DISABLE_PIN
    PIO_Mode(B_DISABLE_PORT, B_DISABLE_BIT, OUTPUT);
  #endif
  #ifdef C_DISABLE_PIN
    PIO_Mode(C_DISABLE_PORT, C_DISABLE_BIT, OUTPUT);
  #endif
  #ifdef X2_DISABLE_PIN
    PIO_Mode(X2_DISABLE_PORT, X2_DISABLE_BIT, OUTPUT);
  #endif
  #ifdef Y2_DISABLE_PIN
    PIO_Mode(Y2_DISABLE_PORT, Y2_DISABLE_BIT, OUTPUT);
  #endif
  #ifdef Z2_DISABLE_PIN
    PIO_Mode(Z2_DISABLE_PORT, Z2_DISABLE_BIT, OUTPUT);
  #endif

  // Software debounce init

    if(hal.driver_cap.software_debounce) {

        DEBOUNCE_TIMER.TC_CCR = TC_CCR_CLKDIS;
        DEBOUNCE_TIMER.TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP|TC_CMR_CPCSTOP;
        DEBOUNCE_TIMER.TC_IER = TC_IER_CPCS;
        DEBOUNCE_TIMER.TC_RC  = 40000 * 42; // 40ms
        DEBOUNCE_TIMER.TC_CCR = TC_CCR_CLKEN;

        IRQRegister(DEBOUNCE_TIMER_IRQn, DEBOUNCE_IRQHandler);
        NVIC_SetPriority(STEP_TIMER_IRQn, 4);
        NVIC_EnableIRQ(DEBOUNCE_TIMER_IRQn);    // Enable debounce interrupt
    }

#ifndef VFD_SPINDLE

 // Spindle init

    PIO_Mode(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_BIT, OUTPUT);
  #ifdef SPINDLE_DIRECTION_PIN
    PIO_Mode(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_BIT, OUTPUT);
  #endif

    SPINDLE_PWM_PORT->PIO_WPMR = PIO_WPMR_WPKEY(0x50494F);
    SPINDLE_PWM_PORT->PIO_ABSR |= SPINDLE_PWM_BIT;
    SPINDLE_PWM_PORT->PIO_PDR = SPINDLE_PWM_BIT;

#ifdef SPINDLE_PWM_CHANNEL
#error "Spindle PWM to be completed for this board!"
//    PWM->PWM_CLK = ;
//    PWM->PWM_ENA |= (1 << SPINDLE_PWM_CHANNEL);
//    PWM->PWM_CH_NUM[SPINDLE_PWM_CHANNEL].PWM_CPRD = ;
#else
    SPINDLE_PWM_TIMER.TC_CCR = TC_CCR_CLKDIS;
    SPINDLE_PWM_TIMER.TC_CMR = TC_CMR_WAVE|TC_CMR_WAVSEL_UP_RC|TC_CMR_ASWTRG_CLEAR|TC_CMR_ACPA_SET|TC_CMR_ACPC_CLEAR; //|TC_CMR_EEVT_XC0;
#endif

#endif

 // Coolant init

  #ifdef COOLANT_FLOOD_PIN
    PIO_Mode(COOLANT_FLOOD_PORT, COOLANT_FLOOD_BIT, OUTPUT);
  #endif
  #ifdef COOLANT_MIST_PIN
    PIO_Mode(COOLANT_MIST_PORT, COOLANT_MIST_BIT, OUTPUT);
  #endif

 // Limit signals init

    PIO_Mode(X_LIMIT_PORT, X_LIMIT_BIT, INPUT);
  #ifdef X_LIMIT_PIN_MAX
    PIO_Mode(X_LIMIT_PORT_MAX, X_LIMIT_BIT_MAX, INPUT);
  #endif

    PIO_Mode(Y_LIMIT_PORT, Y_LIMIT_BIT, false);
  #ifdef Y_LIMIT_PIN_MAX
    PIO_Mode(Y_LIMIT_PORT_MAX, Y_LIMIT_BIT_MAX, INPUT);
  #endif

    PIO_Mode(Z_LIMIT_PORT, Z_LIMIT_BIT, INPUT);
  #ifdef Z_LIMIT_PIN_MAX
    PIO_Mode(Z_LIMIT_PORT_MAX, Z_LIMIT_BIT_MAX, INPUT);
  #endif

  #ifdef A_LIMIT_PIN
    PIO_Mode(A_LIMIT_PORT, A_LIMIT_BIT, INPUT);
  #endif
  #ifdef A_LIMIT_PIN_MAX
    PIO_Mode(A_LIMIT_PORT_MAX, A_LIMIT_BIT_MAX, INPUT);
  #endif

  #ifdef B_LIMIT_PIN
    PIO_Mode(B_LIMIT_PORT, B_LIMIT_BIT, INPUT);
  #endif
  #ifdef B_LIMIT_PIN_MAX
    PIO_Mode(B_LIMIT_PORT_MAX, B_LIMIT_BIT_MAX, INPUT);
  #endif

  #ifdef C_STEP_PIN
    PIO_Mode(C_LIMIT_PORT, C_LIMIT_BIT, INPUT);
  #endif
  #ifdef C_LIMIT_PIN_MAX
    PIO_Mode(C_LIMIT_PORT_MAX, C_LIMIT_BIT_MAX, INPUT);
  #endif

 // Control signals init
  #ifdef RESET_PIN
    PIO_Mode(RESET_PORT, RESET_BIT, INPUT);
  #endif
  #ifdef FEED_HOLD_PIN
    PIO_Mode(FEED_HOLD_PORT, FEED_HOLD_BIT, INPUT);
  #endif
  #ifdef CYCLE_START_PIN
    PIO_Mode(CYCLE_START_PORT, CYCLE_START_BIT, INPUT);
  #endif
  #ifdef SAFETY_DOOR_PIN
    PIO_Mode(SAFETY_DOOR_PORT, SAFETY_DOOR_BIT, INPUT);
  #endif
  #ifdef PROBE_PIN
    PIO_Mode(PROBE_PORT, PROBE_BIT, INPUT);
  #endif

#if TRINAMIC_ENABLE == 2130
    trinamic_start(true);
#endif

 // Set defaults

    IOInitDone = settings->version == 19;

    hal.settings_changed(settings);
    hal.stepper.go_idle(true);
    hal.spindle.set_state((spindle_state_t){0}, 0.0f);
    hal.coolant.set_state((coolant_state_t){0});

#if SDCARD_ENABLE
    pinMode(SD_CD_PIN, INPUT_PULLUP);

// This does not work, the card detect pin is not interrupt capable(!) and inserting a card causes a hard reset...
// The bootloader needs modifying for it to work? Or perhaps the schematic is plain wrong?
// attachInterrupt(SD_CD_PIN, SD_IRQHandler, CHANGE);

    if(BITBAND_PERI(SD_CD_PIN) == 0)
        power_on();

    sdcard_init();
#endif

    return IOInitDone;
}

// EEPROM emulation - stores settings in flash
// Note: settings will not survive a reflash unless protected

typedef struct {
    void *addr;
    uint16_t page;
    uint16_t pages;
} nvs_storage_t;

static nvs_storage_t grblNVS;

bool nvsRead (uint8_t *dest)
{
    if(grblNVS.addr != NULL)
        memcpy(dest, grblNVS.addr, hal.nvs.size);

    return grblNVS.addr != NULL;
}

bool nvsWrite (uint8_t *source)
{
    uint16_t page = grblNVS.page;
    uint32_t size = grblNVS.pages;
    uint32_t *dest = (uint32_t *)grblNVS.addr, *src = (uint32_t *)source, words;

    while(!(EFC1->EEFC_FSR & EEFC_FSR_FRDY));

    while(size) {

        // Fill page buffer
        words = IFLASH1_PAGE_SIZE / sizeof(uint32_t);
        do {
            *dest++ = *src++;
        } while(--words);

        // Write page buffer to flash
        EFC1->EEFC_FCR = EEFC_FCR_FKEY(0x5A)|EEFC_FCR_FARG(page)|EEFC_FCR_FCMD(0x03);
        while(!(EFC1->EEFC_FSR & EEFC_FSR_FRDY));
        
        if(EFC1->EEFC_FSR & EEFC_FSR_FLOCKE)
            break;

        size--;
        page++;
    }

    return size == 0;
}

bool nvsInit (void)
{
    if(hal.nvs.size & ~(IFLASH1_PAGE_SIZE - 1))
        grblNVS.pages = (hal.nvs.size & ~(IFLASH1_PAGE_SIZE - 1)) / IFLASH1_PAGE_SIZE + 1;
    else
        grblNVS.pages = hal.nvs.size / IFLASH1_PAGE_SIZE;

//  grblNVS.row_size = IFLASH1_LOCK_REGION_SIZE; // 16K
    grblNVS.page = IFLASH1_NB_OF_PAGES - grblNVS.pages;
    grblNVS.addr = (void *)(IFLASH1_ADDR + IFLASH1_PAGE_SIZE * grblNVS.page);

    return true;
}

// End EEPROM emulation

#if USB_SERIAL_CDC
static void execute_realtime (uint_fast16_t state)
{
#if USB_SERIAL_CDC
    usb_execute_realtime(state);
#endif
}
#endif

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{
    WDT_Disable (WDT);

    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

    // Copy vector table to RAM so we can override the default Arduino IRQ assignments

    __disable_irq();

    memcpy(&vectorTable, (void *)SCB->VTOR, sizeof(vectorTable));

    SCB->VTOR = ((uint32_t)&vectorTable & SCB_VTOR_TBLOFF_Msk) | SCB_VTOR_TBLBASE_Msk;
    __DSB();
    __enable_irq();

    // End vector table copy

    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk;

    IRQRegister(SysTick_IRQn, SysTick_IRQHandler);

//    NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);
    NVIC_EnableIRQ(SysTick_IRQn);

    hal.info = "SAM3X8E";
	hal.driver_version = "210214";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.driver_setup = driver_setup;
    hal.f_step_timer = SystemCoreClock / 2; // 42 MHz
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
#ifdef SQUARING_ENABLED
    hal.stepper.get_auto_squared = getAutoSquaredAxes;
    hal.stepper.disable_motors = StepperDisableMotors;
#endif

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

#ifdef PROBE_PIN
    hal.probe.configure = probeConfigure;
    hal.probe.get_state = probeGetState;
#endif

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

#if USB_SERIAL_CDC
    usb_serialInit();
    hal.stream.read = usb_serialGetC;
    hal.stream.get_rx_buffer_available = usb_serialRxFree;
    hal.stream.reset_read_buffer = usb_serialRxFlush;
    hal.stream.cancel_read_buffer = usb_serialRxCancel;
    hal.stream.write = usb_serialWriteS;
    hal.stream.write_all = usb_serialWriteS;
    hal.stream.suspend_read = usb_serialSuspendInput;
#else
    serialInit();
    hal.stream.read = serialGetC;
    hal.stream.get_rx_buffer_available = serialRxFree;
    hal.stream.reset_read_buffer = serialRxFlush;
    hal.stream.cancel_read_buffer = serialRxCancel;
    hal.stream.write = serialWriteS;
    hal.stream.write_all = serialWriteS;
    hal.stream.suspend_read = serialSuspendInput;
#endif

#if EEPROM_ENABLE || KEYPAD_ENABLE || (TRINAMIC_ENABLE && TRINAMIC_I2C)
    i2c_init();
#endif

#if EEPROM_ENABLE
    i2c_eeprom_init();
#else
    if(nvsInit()) {
        hal.nvs.type = NVS_Flash;
        hal.nvs.memcpy_from_flash = nvsRead;
        hal.nvs.memcpy_to_flash = nvsWrite;
    } else
        hal.nvs.type = NVS_None;
#endif

    hal.irq_enable = __enable_irq;
    hal.irq_disable = __disable_irq;
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = millis;

#if USB_SERIAL_CDC
    hal.execute_realtime = execute_realtime;
#endif

#ifdef DEBUGOUT
    hal.debug_out = debug_out;
#endif

 // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

#ifdef SAFETY_DOOR_PIN
    hal.signals_cap.safety_door_ajar = On;
#endif

#ifndef VFD_SPINDLE
  #ifdef SPINDLE_DIRECTION_PIN
    hal.driver_cap.spindle_dir = On;
  #endif
    hal.driver_cap.variable_spindle = On;
#endif
#ifdef COOLANT_MIST_PIN
    hal.driver_cap.mist_control = On;
#endif
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;

#if TRINAMIC_ENABLE == 2130
    trinamic_init();
#endif

#if KEYPAD_ENABLE
    keypad_init();
#endif

#if MODBUS_ENABLE
    serial2Init(19200);

    modbus_stream.rx_timeout = 500;
    modbus_stream.write = serial2Write;
    modbus_stream.read = serial2GetC;
    modbus_stream.flush_rx_buffer = serial2RxFlush;
    modbus_stream.flush_tx_buffer = serial2TxFlush;
    modbus_stream.get_rx_buffer_count = serial2RxCount;
    modbus_stream.get_tx_buffer_count = serial2TxCount;

    modbus_init(&modbus_stream);
#endif

#if SPINDLE_HUANYANG
    huanyang_init(&modbus_stream);
#endif

    my_plugin_init();

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 8;
}

/* interrupt handlers */

// Main stepper driver
static void STEPPER_IRQHandler (void)
{
    if(STEPPER_TIMER.TC_SR & STEPPER_TIMER.TC_IMR)
        hal.stepper.interrupt_callback();
}

// Step output off
static void STEP_IRQHandler (void)
{
    if(STEP_TIMER.TC_SR & STEP_TIMER.TC_IMR)
        set_step_outputs((axes_signals_t){0});  
}

// Step output on/off
static void STEPDELAY_IRQHandler (void)
{
    if(STEP_TIMER.TC_SR & STEP_TIMER.TC_IMR) {

        set_step_outputs(next_step_outbits);

        IRQRegister(STEP_TIMER_IRQn, STEP_IRQHandler);

        STEP_TIMER.TC_RC = pulse_length;
        STEP_TIMER.TC_CCR = TC_CCR_SWTRG;
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

static void DEBOUNCE_IRQHandler (void)
{
    input_signal_t *signal;

    DEBOUNCE_TIMER.TC_SR;

    while((signal = get_debounce())) {

        signal->port->PIO_ISR;
        signal->port->PIO_IER = signal->bit;

        if((signal->port->PIO_PDSR & signal->bit) == (signal->intr_type == GPIO_Intr_Falling ? 0 : signal->bit))
          switch(signal->group) {

            case INPUT_GROUP_LIMIT:
                hal.limits.interrupt_callback(limitsGetState());
                break;

            case INPUT_GROUP_CONTROL:
                hal.control.interrupt_callback(systemGetState());
                break;
        }
    }
}

inline static void PIO_IRQHandler (input_signal_t *signals, uint32_t isr)
{
    bool debounce = false;
    uint32_t grp = 0, i = 0;

    while(signals[i].port != NULL) {
        if(isr & signals[i].bit) {
            if(signals[i].debounce && enqueue_debounce(&signals[i])) {
                signals[i].port->PIO_IDR = signals[i].bit;
                debounce = true;
            } else
                grp |= signals[i].group;
        }
        i++;
    }

    if(debounce)
        DEBOUNCE_TIMER.TC_CCR = TC_CCR_SWTRG;

    if(grp & INPUT_GROUP_LIMIT) {
        limit_signals_t state = limitsGetState();
        if(limit_signals_merge(state).value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
            hal.limits.interrupt_callback(state);
    }

    if(grp & INPUT_GROUP_CONTROL)
        hal.control.interrupt_callback(systemGetState());

#if KEYPAD_ENABLE
    if(grp & INPUT_GROUP_KEYPAD)
        keypad_keyclick_handler(!BITBAND_PERI(KEYPAD_PORT->PIO_PDSR, KEYPAD_PIN));
#endif
}

static void PIOA_IRQHandler (void)
{
    PIO_IRQHandler(a_signals, PIOA->PIO_ISR);
}

static void PIOB_IRQHandler (void)
{
    PIO_IRQHandler(b_signals, PIOB->PIO_ISR);
}

static void PIOC_IRQHandler (void)
{
    PIO_IRQHandler(c_signals, PIOC->PIO_ISR);
}

static void PIOD_IRQHandler (void)
{
    PIO_IRQHandler(d_signals, PIOD->PIO_ISR);
}

// Interrupt handler for 1 ms interval timer
static void SysTick_IRQHandler (void)
{

#if USB_SERIAL_CDC || SDCARD_ENABLE || MODBUS_ENABLE

#if USB_SERIAL_CDC
    SysTick_Handler(); // SerialUSB needs the Arduino SysTick handler running
#endif

#if SDCARD_ENABLE
    static uint32_t fatfs_ticks = 10;
    if(!(--fatfs_ticks)) {
        disk_timerproc();
        fatfs_ticks = 10;
    }
#endif

#if MODBUS_ENABLE
    modbus_poll();
#endif

    if(delay_ms.ms && !(--delay_ms.ms)) {
        if(delay_ms.callback) {
            delay_ms.callback();
            delay_ms.callback = NULL;
        }
    }
#else
    if(!(--delay_ms.ms)) {
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        if(delay_ms.callback) {
            delay_ms.callback();
            delay_ms.callback = NULL;
        }
    }
#endif
}
