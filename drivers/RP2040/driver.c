/*

  driver.c - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2021 Terje Io

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
#include <string.h>

#include "pico/stdlib.h"
#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/iobank0.h"

#include "driver.h"
#include "serial.h"
#include "driverPIO.pio.h"

#ifdef I2C_PORT
#include "i2c.h"
#endif

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "ff.h"
#include "diskio.h"
#endif

#if USB_SERIAL_CDC
#include "usb_serial.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#if KEYPAD_ENABLE
#include "keypad/keypad.h"
#endif

#if ODOMETER_ENABLE
#include "odometer/odometer.h"
#endif

#if PPI_ENABLE
#include "laser/ppi.h"
#endif

#if FLASH_ENABLE
#include "flash.h"
#endif

#if IOEXPAND_ENABLE
#include "ioexpand.h"
#endif

typedef union {
    uint8_t mask;
    struct {
        uint8_t limits :1,
                door   :1,
                unused :6;
    };
} debounce_t;

typedef union {
    uint32_t value;
    struct {
        uint32_t delay  :8,
                 length :8,
                 set    :6,
                 reset  :6;
    };
} pio_steps_t;

static pio_steps_t pio_steps = { .delay = 20, .length = 100 };
uint32_t pirq = 0;
static uint pulse, timer;
static uint16_t pulse_length, pulse_delay;
static bool pwmEnabled = false, IOInitDone = false;
static axes_signals_t next_step_outbits;
static spindle_pwm_t spindle_pwm;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
static status_code_t (*on_unknown_sys_command)(uint_fast16_t state, char *line, char *lcline);
static debounce_t debounce;
static probe_state_t probe; /* = {
    .connected = On
}; */

#if IOEXPAND_ENABLE
static ioexpand_t io_expander = {0};
#endif

#if MODBUS_ENABLE
static modbus_stream_t modbus_stream = {0};
#endif

#ifdef SPINDLE_SYNC_ENABLE

#include "grbl/spindle_sync.h"

static spindle_data_t spindle_data;
static spindle_encoder_t spindle_encoder = {
    .counter.tics_per_irq = 4
};
static spindle_sync_t spindle_tracker;
static volatile bool spindleLock = false;

static void stepperPulseStartSynchronized (stepper_t *stepper);
static void spindleDataReset (void);
static spindle_data_t spindleGetData (spindle_data_request_t request);

#endif

#define INPUT_GROUP_CONTROL       (1 << 0)
#define INPUT_GROUP_PROBE         (1 << 1)
#define INPUT_GROUP_LIMIT         (1 << 2)
#define INPUT_GROUP_KEYPAD        (1 << 3)
#define INPUT_GROUP_MPG           (1 << 4)
#define INPUT_GROUP_QEI           (1 << 5)
#define INPUT_GROUP_QEI_SELECT    (1 << 6)
#define INPUT_GROUP_SPINDLE_INDEX (1 << 7)

static input_signal_t inputpin[] = {
#if ESTOP_ENABLE
    { .id = Input_EStop,          .pin = CONTROL_RESET_PIN,           .group = INPUT_GROUP_CONTROL },
#else
    { .id = Input_Reset,          .pin = CONTROL_RESET_PIN,           .group = INPUT_GROUP_CONTROL },
#endif
    { .id = Input_FeedHold,       .pin = CONTROL_FEED_HOLD_PIN,       .group = INPUT_GROUP_CONTROL },
    { .id = Input_CycleStart,     .pin = CONTROL_CYCLE_START_PIN,     .group = INPUT_GROUP_CONTROL },
#if SAFETY_DOOR_ENABLE
    { .id = Input_SafetyDoor,     .pin = SAFETY_DOOR_PIN,             .group = INPUT_GROUP_CONTROL },
#endif
    { .id = Input_Probe,          .pin = PROBE_PIN,                   .group = INPUT_GROUP_PROBE },
    { .id = Input_LimitX,         .pin = X_LIMIT_PIN,                 .group = INPUT_GROUP_LIMIT },
    { .id = Input_LimitY,         .pin = Y_LIMIT_PIN,                 .group = INPUT_GROUP_LIMIT },
    { .id = Input_LimitZ,         .pin = Z_LIMIT_PIN,                 .group = INPUT_GROUP_LIMIT }
#ifdef A_LIMIT_PIN
  , { .id = Input_LimitA,         .pin = A_LIMIT_PIN,         .group = INPUT_GROUP_LIMIT }
#endif
#if KEYPAD_ENABLE && defined(KEYPAD_STROBE_PIN)
  , { .id = Input_KeypadStrobe,   .pin = KEYPAD_STROBE_PIN,   .group = INPUT_GROUP_KEYPAD }
#endif
};

#ifndef STEPPERS_DISABLE_MASK
#define STEPPERS_DISABLE_MASK 0
#endif

#ifndef X_LIMIT_PORT
  #define X_LIMIT_PORT LIMIT_PORT
#endif
#ifndef Y_LIMIT_PORT
  #define Y_LIMIT_PORT LIMIT_PORT
#endif
#ifndef Z_LIMIT_PORT
  #define Z_LIMIT_PORT LIMIT_PORT
#endif
#ifndef A_LIMIT_PORT
  #define A_LIMIT_PORT LIMIT_PORT
#endif

#if KEYPAD_ENABLE == 0
#define KEYPAD_STROBE_BIT 0
#endif

#ifndef SPINDLE_SYNC_ENABLE
#define SPINDLE_INDEX_BIT 0
#endif

#define NVIC_HIGH_LEVEL_PRIORITY 0xC0
#define NVIC_MEDIUM_LEVEL_PRIORITY 0x80
#define NVIC_LOW_LEVEL_PRIORITY 0x40

#define DRIVER_IRQMASK (LIMIT_MASK|CONTROL_MASK|KEYPAD_STROBE_BIT|SPINDLE_INDEX_BIT)

#define PICO_TIME_DEFAULT_ALARM_POOL_HARDWARE_ALARM_NUM 3

#define gpio_get_edge_event(x) x ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL

static void systick_handler (void);
static void spindle_set_speed (uint_fast16_t pwm_value);
static void STEPPER_TIMER_IRQHandler(void);
static void GPIO_IRQHandler(void);

static int64_t delay_callback(alarm_id_t id, void *callback)
{
    ((void (*)(void))callback)();

    return 0;
}

static void driver_delay (uint32_t ms, void (*callback)(void))
{
    if((delay.ms = ms) > 0) {
        if(!(delay.callback = callback)) {
            uint32_t delay = ms * 1000, start = timer_hw->timerawl;
            while(timer_hw->timerawl - start < delay);
        } else
            add_alarm_in_ms(ms, delay_callback, callback, false);
    } else if(callback)
        callback();
}

//*************************  STEPPER  *************************//

// Enable/disable stepper motors
static void stepperEnable (axes_signals_t enable)
{
    enable.mask ^= settings.steppers.enable_invert.mask;
#if TRINAMIC_ENABLE && TRINAMIC_I2C
    axes_signals_t tmc_enable = trinamic_stepper_enable(enable);
#else
  #if STEPPERS_DISABLE_OUTMODE == GPIO_IOEXPAND
    #ifdef STEPPERS_DISABLEX_PIN
    ioex_out(STEPPERS_DISABLEX_PIN) = enable.x;
    #endif
    #ifdef STEPPERS_DISABLEZ_PIN
    ioex_out(STEPPERS_DISABLEZ_PIN) = enable.z;
    #endif
    ioexpand_out(io_expander);
  #else
    gpio_put(STEPPERS_DISABLE_PIN, enable.x);
  #endif
#endif
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    stepperEnable((axes_signals_t){AXES_BITMASK});

    stepper_timer_set_period(pio1, 0, timer, 1000);
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    stepper_timer_stop(pio1, 0);
}

// Sets up stepper driver interrupt timeout, "Normal" version
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    stepper_timer_set_period(pio1, 0, timer, cycles_per_tick);
}

// Set stepper pulse output pins
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z...
inline static __attribute__((always_inline)) void stepperSetStepOutputs (axes_signals_t step_outbits)
{
    pio_steps.set = step_outbits.mask ^ settings.steppers.step_invert.mask;

    step_pulse_generate(pio0, 0, pio_steps.value);
}

// Set stepper direction output pins
// NOTE: see note for stepperSetStepOutputs()
inline static __attribute__((always_inline)) void stepperSetDirOutputs (axes_signals_t dir_outbits)
{
#if DIRECTION_OUTMODE == GPIO_MAP
    gpio_put_masked(DIRECTION_MASK, dir_outmap[dir_outbits.mask]);
#else
    gpio_put_masked(DIRECTION_MASK, (dir_outbits.mask ^ settings.steppers.dir_invert.mask) << DIRECTION_OUTMODE);
#endif
}

// Sets stepper direction and pulse pins and starts a step pulse.
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_change)
        stepperSetDirOutputs(stepper->dir_outbits);

    if(stepper->step_outbits.value)
        stepperSetStepOutputs(stepper->step_outbits);
}

//*************************  LIMIT  *************************//

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing)
{
    if(on && settings.limits.flags.hard_enabled) {

    //        EXTI->PR |= LIMIT_MASK;     // Clear any pending limit interrupts
//       EXTI->IMR |= LIMIT_MASK;    // and enable
    } //else
//        EXTI->IMR &= ~LIMIT_MASK;

#if TRINAMIC_ENABLE
    trinamic_homing(homing);
#endif
}

// Returns limit state as an limit_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};

#if LIMIT_INMODE == GPIO_BITBAND
//    signals.min.x = BITBAND_PERI(X_LIMIT_PORT->IDR, X_LIMIT_PIN);
//    signals.min.y = BITBAND_PERI(Y_LIMIT_PORT->IDR, Y_LIMIT_PIN);
//    signals.min.z = BITBAND_PERI(Z_LIMIT_PORT->IDR, Z_LIMIT_PIN);
  #ifdef A_LIMIT_PIN
//    signals.a = BITBAND_PERI(A_LIMIT_PORT->IDR, A_LIMIT_PIN);
  #endif
#elif LIMIT_INMODE == GPIO_MAP
//    uint32_t bits = LIMIT_PORT->IDR;
//    signals.min.x = (bits & X_LIMIT_BIT) != 0;
//    signals.min.y = (bits & Y_LIMIT_BIT) != 0;
//    signals.min.z = (bits & Z_LIMIT_BIT) != 0;
  #ifdef A_LIMIT_PIN
//    signals.min.a = (bits & A_LIMIT_BIT) != 0;
  #endif
#else
    signals.min.value = (uint8_t)((LIMIT_PORT->IDR & LIMIT_MASK) >> LIMIT_INMODE);
#endif

    if(settings.limits.invert.mask)
        signals.min.value ^= settings.limits.invert.mask;

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.value = settings.control_invert.mask;

#if CONTROL_INMODE == GPIO_BITBAND
//    signals.reset = BITBAND_PERI(CONTROL_PORT->IDR, CONTROL_RESET_PIN);
//    signals.feed_hold = BITBAND_PERI(CONTROL_PORT->IDR, CONTROL_FEED_HOLD_PIN);
//    signals.cycle_start = BITBAND_PERI(CONTROL_PORT->IDR, CONTROL_CYCLE_START_PIN);
  #ifdef CONTROL_SAFETY_DOOR_PIN
//    signals.safety_door_ajar = BITBAND_PERI(CONTROL_PORT->IDR, CONTROL_SAFETY_DOOR_PIN);
  #endif
#elif CONTROL_INMODE == GPIO_MAP
    uint32_t bits = CONTROL_PORT->IDR;
//    signals.reset = (bits & CONTROL_RESET_BIT) != 0;
//    signals.feed_hold = (bits & CONTROL_FEED_HOLD_BIT) != 0;
//    signals.cycle_start = (bits & CONTROL_CYCLE_START_BIT) != 0;
  #ifdef CONTROL_SAFETY_DOOR_PIN
    signals.safety_door_ajar = (bits & CONTROL_SAFETY_DOOR_BIT) != 0;
  #endif
#else
//   signals.value = (uint8_t)((CONTROL_PORT->IDR & CONTROL_MASK) >> CONTROL_INMODE);
  #ifndef ENABLE_SAFETY_DOOR_INPUT_PIN
    signals.safety_door_ajar = settings.control_invert.safety_door_ajar;
  #endif
#endif

    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

    return signals;
}

//*************************  PROBE  *************************//

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
//    state.triggered = !!(PROBE_PORT->IDR & PROBE_BIT) ^ probe.inverted;

    return state;
}

#endif

//*************************  SPINDLE  *************************//

// Static spindle (off, on cw & on ccw)
inline static void spindle_off (void)
{
#if SPINDLE_OUTMODE == GPIO_IOEXPAND
    ioex_out(SPINDLE_ENABLE_PIN) = settings.spindle.invert.on;
    ioexpand_out(io_expander);
#else
    gpio_put(SPINDLE_ENABLE_PIN, settings.spindle.invert.on);
#endif
}

inline static void spindle_on (void)
{
#if SPINDLE_OUTMODE == GPIO_IOEXPAND
    ioex_out(SPINDLE_ENABLE_PIN) = !settings.spindle.invert.on;
    ioexpand_out(io_expander);
#else
    gpio_put(SPINDLE_ENABLE_PIN, !settings.spindle.invert.on);
#endif
#ifdef SPINDLE_SYNC_ENABLE
    spindleDataReset();
#endif
}

inline static void spindle_dir (bool ccw)
{
#if SPINDLE_OUTMODE == GPIO_IOEXPAND
    if(hal.driver_cap.spindle_dir) {
        ioex_out(SPINDLE_DIRECTION_PIN) = settings.spindle.invert.ccw;
        ioexpand_out(io_expander);
    }
#else
    if(hal.driver_cap.spindle_dir)
        gpio_put(SPINDLE_DIRECTION_PIN, ccw ^ settings.spindle.invert.ccw);
#endif
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
        if(spindle_pwm.always_on) 
            pwm_set_gpio_level(SPINDLE_PWM_PIN, spindle_pwm.off_value);
        else
            pwm_set_gpio_level(SPINDLE_PWM_PIN, 0);
    } else {
        if(!pwmEnabled)
            spindle_on();
        pwmEnabled = true;
        pwm_set_gpio_level(SPINDLE_PWM_PIN, pwm_value);
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

#if SPINDLE_OUTMODE == GPIO_IOEXPAND
    state.on = ioex_out(SPINDLE_ENABLE_PIN);
    state.ccw = hal.driver_cap.spindle_dir && ioex_out(SPINDLE_DIRECTION_PIN);
#else
    state.on = gpio_get_out_state(SPINDLE_ENABLE_PIN);
    state.ccw = (hal.driver_cap.spindle_dir && (gpio_get_out_state(SPINDLE_DIRECTION_PIN))) != 0;
#endif

    state.value ^= settings.spindle.invert.mask;

    return state;
}

void driver_spindle_pwm_init (void) {

    if(hal.driver_cap.variable_spindle) {

        hal.spindle.set_state = spindleSetStateVariable;

        // Get the default config for 
        pwm_config config = pwm_get_default_config();
        
        uint32_t prescaler = settings.spindle.pwm_freq > 2000.0f ? 1 : (settings.spindle.pwm_freq > 200.0f ? 12 : 50);

        spindle_precompute_pwm_values(&spindle_pwm, clock_get_hz(clk_sys) / prescaler);

        // Set divider, not using the 4 fractional bit part of the clock divider, only the integer part
        pwm_config_set_clkdiv_int(&config, prescaler);
        // Set the top value of the PWM => the period
        pwm_config_set_wrap(&config, spindle_pwm.period);
        // Set the off value of the PWM => off duty cycle (either 0 or the off value)
        pwm_set_gpio_level(SPINDLE_PWM_PIN, spindle_pwm.off_value);

        // Set polarity of the channel
        uint channel = pwm_gpio_to_channel(SPINDLE_PWM_PIN);                                                                            // Get which is associated with the PWM pin
        pwm_config_set_output_polarity(&config, (!channel & settings.spindle.invert.pwm), (channel & settings.spindle.invert.pwm));   // Set the polarity of the pin's channel

        // Load the configuration into our PWM slice, and set it running.
        pwm_init(pwm_gpio_to_slice_num(SPINDLE_PWM_PIN), &config, true);
    } else
        hal.spindle.set_state = spindleSetState;
}

#if PPI_ENABLE

static void spindlePulseOn (uint_fast16_t pulse_length)
{
//    PPI_TIMER->ARR = pulse_length;
//    PPI_TIMER->EGR = TIM_EGR_UG;
//    PPI_TIMER->CR1 |= TIM_CR1_CEN;
    spindle_on();
}

#endif

// end spindle code

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;

  #if COOLANT_OUTMODE == GPIO_IOEXPAND
    ioex_out(COOLANT_FLOOD_PIN) = mode.flood;
    #ifdef COOLANT_MIST_PIN
    ioex_out(COOLANT_MIST_PIN) = mode.mist;
    #endif
    ioexpand_out(io_expander);
  #else
  //  gpio_put(STEPPERS_DISABLE_PIN, enable.x);
  #endif
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {settings.coolant_invert.mask};

  #if COOLANT_OUTMODE == GPIO_IOEXPAND
    state.flood = ioex_out(COOLANT_FLOOD_PIN);
    #ifdef COOLANT_MIST_PIN
    state.mist = ioex_out(COOLANT_MIST_PIN);
    #endif
  #else
//    state.flood = (COOLANT_FLOOD_PORT->IDR & COOLANT_FLOOD_BIT) != 0;
#ifdef COOLANT_MIST_PIN
//    state.mist  = (COOLANT_MIST_PORT->IDR & COOLANT_MIST_BIT) != 0;
#endif
  #endif

    state.value ^= settings.coolant_invert.mask;

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
//    __disable_irq();
    *ptr |= bits;
//    __enable_irq();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
//    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
//   __enable_irq();
    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
//    __disable_irq();
    uint_fast16_t prev = *ptr;
    *ptr = value;
//    __enable_irq();
    return prev;
}

static uint32_t getElapsedTicks (void)
{
   return 0; //uwTick;
}

// Configures peripherals when settings are initialized or changed
void settings_changed (settings_t *settings)
{

#ifdef SPINDLE_PWM_PIN
    hal.driver_cap.variable_spindle = settings->spindle.rpm_min < settings->spindle.rpm_max;
#endif

#if (DIRECTION_OUTMODE == GPIO_MAP)
    uint8_t i;
#endif

#if DIRECTION_OUTMODE == GPIO_MAP
    i = sizeof(dir_outmap) / sizeof(uint32_t);
    do {
        i--;
        dir_outmap[i] = c_dir_outmap[i ^ settings->steppers.dir_invert.value];
    } while(i);
#endif

    stepperSetStepOutputs((axes_signals_t){0});
    stepperSetDirOutputs((axes_signals_t){0});

    if(IOInitDone) {

        // Init of the spindle PWM
        driver_spindle_pwm_init();

        // PIO step parameters init
        pio_steps.length = (uint32_t)(10.0f * (settings->steppers.pulse_microseconds)) - 1;
        pio_steps.delay = (uint32_t)(10.0f * (settings->steppers.pulse_delay_microseconds)) - 1;
        pio_steps.reset = settings->steppers.step_invert.mask;

        // Disable GPIO IRQ while initializing the input pins
        irq_set_enabled(IO_IRQ_BANK0, false);

        //****** Input and output pins config ******//
        
        control_signals_t control_fei;
        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        bool pullup;
        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);
        input_signal_t *signal;

        do {

            pullup = true;
            signal = &inputpin[--i];
            signal->irq_mode = IRQ_Mode_None;

            switch(signal->id) {
#if ESTOP_ENABLE
                case Input_EStop:
                    pullup = !settings->control_disable_pullup.e_stop;
                    signal->debounce = hal.driver_cap.software_debounce;
                    signal->irq_mode = control_fei.e_stop ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#else
                case Input_Reset:
                    pullup = !settings->control_disable_pullup.reset;
                    signal->debounce = hal.driver_cap.software_debounce;
                    signal->irq_mode = control_fei.reset ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
                case Input_FeedHold:
                    pullup = !settings->control_disable_pullup.feed_hold;
                    signal->debounce = hal.driver_cap.software_debounce;
                    signal->irq_mode = control_fei.feed_hold ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_CycleStart:
                    pullup = !settings->control_disable_pullup.cycle_start;
                    signal->debounce = hal.driver_cap.software_debounce;
                    signal->irq_mode = control_fei.cycle_start ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

#if SAFETY_DOOR_ENABLE
                case Input_SafetyDoor:
                    pullup = !settings->control_disable_pullup.safety_door_ajar;
                    signal->debounce = hal.driver_cap.software_debounce;
                    signal->irq_mode = control_fei.safety_door_ajar ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#ifdef LIMITS_OVERRIDE_PIN
                case Input_LimitsOverride:
                    pullup = true;
                    signal->debounce = false;
                    break;
#endif
                case Input_Probe:
                    pullup = hal.driver_cap.probe_pull_up;
                    break;

                case Input_LimitX:
                case Input_LimitX_Max:
                    pullup = !settings->limits.disable_pullup.x;
                    signal->debounce = hal.driver_cap.software_debounce;
                    signal->irq_mode = limit_fei.x ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitY:
                case Input_LimitY_Max:
                    pullup = !settings->limits.disable_pullup.y;
                    signal->irq_mode = limit_fei.y ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitZ:
                case Input_LimitZ_Max:
                    pullup = !settings->limits.disable_pullup.z;
                    signal->irq_mode = limit_fei.z ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#ifdef A_LIMIT_PIN
                case Input_LimitA:
                case Input_LimitA_Max:
                    pullup = !settings->limits.disable_pullup.a;
                    signal->irq_mode = limit_fei.a ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#ifdef B_LIMIT_PIN
                case Input_LimitB:
                case Input_LimitB_Max:
                    pullup = !settings->limits.disable_pullup.b;
                    signal->irq_mode = limit_fei.b ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#ifdef C_LIMIT_PIN
                case Input_LimitC:
                case Input_LimitC_Max:
                    pullup = !settings->limits.disable_pullup.b;
                    signal->irq_mode = limit_fei.b ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#if MPG_MODE_ENABLE
                case Input_ModeSelect:
                    signal->irq_mode = IRQ_Mode_Change;
                    break;
#endif
#if KEYPAD_ENABLE
                case Input_KeypadStrobe:
                    pullup = true;
                    signal->irq_mode = IRQ_Mode_Change;
                    break;
#endif
#ifdef SPINDLE_INDEX_PIN
                case Input_SpindleIndex:
                    pullup = false;
                    signal->irq_mode = IRQ_Mode_Rising;
                    break;
#endif
#if QEI_ENABLE
                case Input_QEI_A:
                    if(qei_enable)
                        signal->irq_mode = IRQ_Mode_Change;
                    break;

                case Input_QEI_B:
                    if(qei_enable)
                        signal->irq_mode = IRQ_Mode_Change;
                    break;

  #if QEI_INDEX_ENABLED
                case Input_QEI_Index:
                    if(qei_enable)
                        signal->irq_mode = IRQ_Mode_None;
                    break;
  #endif

  #if QEI_SELECT_ENABLED
                case Input_QEI_Select:
                    signal->debounce = hal.driver_cap.software_debounce;
                    if(qei_enable)
                        signal->irq_mode = IRQ_Mode_Falling;
                    break;
  #endif
#endif
                default:
                    break;
            }

            pinMode(signal->pin, pullup ? INPUT_PULLUP : INPUT_PULLDOWN);
            signal->gpio.reg = (gpio_reg_t *)digital_pin_to_info_PGM[signal->pin].reg;
            signal->gpio.bit = digital_pin_to_info_PGM[signal->pin].mask;

            if(signal->port != NULL)
                memcpy(signal->port, &signal->gpio, sizeof(gpio_t));

            if(signal->irq_mode != IRQ_Mode_None) {

                if(signal->gpio.reg == (gpio_reg_t *)&GPIO6_DR)
                    signal->offset = 0;
                else if(signal->gpio.reg == (gpio_reg_t *)&GPIO7_DR)
                    signal->offset = 1;
                else if(signal->gpio.reg == (gpio_reg_t *)&GPIO8_DR)
                    signal->offset = 2;
                else
                    signal->offset = 3;

                if(signal->irq_mode == IRQ_Mode_Change)
                    signal->gpio.reg->EDGE_SEL |= signal->gpio.bit;
                else {
                    signal->gpio.reg->EDGE_SEL &= ~signal->gpio.bit;
                    uint32_t iopin = __builtin_ctz(signal->gpio.bit);
                    if(iopin < 16) {
                       uint32_t shift = iopin << 1;
                       signal->gpio.reg->ICR1 = (signal->gpio.reg->ICR1 & ~(0b11 << shift)) | (signal->irq_mode << shift);
                    } else {
                       uint32_t shift = (iopin - 16) << 1;
                       signal->gpio.reg->ICR2 = (signal->gpio.reg->ICR2 & ~(0b11 << shift)) | (signal->irq_mode << shift);
                    }
                }

                signal->gpio.reg->ISR = signal->gpio.bit;       // Clear interrupt.

                if(signal->group != INPUT_GROUP_LIMIT)          // If pin is not a limit pin
                    signal->gpio.reg->IMR |= signal->gpio.bit;  // enable interrupt

                signal->active = (signal->gpio.reg->DR & signal->gpio.bit) != 0;

                if(signal->irq_mode != IRQ_Mode_Change)
                    signal->active = signal->active ^ (signal->irq_mode == IRQ_Mode_Falling ? 0 : 1);
            }
        } while(i);

        /*************************
         *  Control pins config  *
         *************************/

        control_signals_t control_ire;

        control_ire.mask = ~(settings->control_disable_pullup.mask ^ settings->control_invert.mask);


        gpio_init(CONTROL_RESET_PIN);
        gpio_set_pulls(CONTROL_RESET_PIN, !settings->control_disable_pullup.reset, false);
        gpio_set_irq_enabled(CONTROL_RESET_PIN, gpio_get_edge_event(control_ire.reset), true);
        gpio_acknowledge_irq(CONTROL_RESET_PIN, gpio_get_edge_event(control_ire.reset));

        gpio_init(CONTROL_FEED_HOLD_PIN);
        gpio_set_pulls(CONTROL_FEED_HOLD_PIN, !settings->control_disable_pullup.feed_hold, false);
        gpio_set_irq_enabled(CONTROL_FEED_HOLD_PIN, gpio_get_edge_event(control_ire.feed_hold), true);
        gpio_acknowledge_irq(CONTROL_FEED_HOLD_PIN, gpio_get_edge_event(control_ire.feed_hold));

        gpio_init(CONTROL_CYCLE_START_PIN);
        gpio_set_pulls(CONTROL_CYCLE_START_PIN, !settings->control_disable_pullup.cycle_start, false);
        gpio_set_irq_enabled(CONTROL_CYCLE_START_PIN, gpio_get_edge_event(control_ire.cycle_start), true);
        gpio_acknowledge_irq(CONTROL_CYCLE_START_PIN, gpio_get_edge_event(control_ire.cycle_start));

#ifdef CONTROL_SAFETY_DOOR_PIN
        gpio_init(CONTROL_SAFETY_DOOR_PIN);
        gpio_set_pulls(CONTROL_SAFETY_DOOR_PIN, !settings->control_disable_pullup.safety_door_ajar, false);
        gpio_set_irq_enabled(CONTROL_SAFETY_DOOR_PIN, gpio_get_edge_event(control_ire.safety_door_ajar), true);
        gpio_acknowledge_irq(CONTROL_SAFETY_DOOR_PIN, gpio_get_edge_event(control_ire.safety_door_ajar));
#endif

        /***********************
         *  Limit pins config  *
         ***********************/
        axes_signals_t limit_ire;

        limit_ire.mask = ~(settings->limits.disable_pullup.mask ^ settings->limits.invert.mask);

        gpio_init(Z_LIMIT_PIN);
        gpio_set_pulls(Z_LIMIT_PIN, !settings->limits.disable_pullup.z, false);

 /*
            // NOTE: Z limit must be first. Do not change!
            GPIO_Init.Pin = Z_LIMIT_BIT;
            GPIO_Init.Mode = limit_ire.z ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING;
            GPIO_Init.Pull = settings->limits.disable_pullup.z ? GPIO_NOPULL : GPIO_PULLUP;
            HAL_GPIO_Init(Z_LIMIT_PORT, &GPIO_Init);

            GPIO_Init.Pin = X_LIMIT_BIT;
            GPIO_Init.Mode = limit_ire.x ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING;
            GPIO_Init.Pull = settings->limits.disable_pullup.x ? GPIO_NOPULL : GPIO_PULLUP;
            HAL_GPIO_Init(X_LIMIT_PORT, &GPIO_Init);

            GPIO_Init.Pin = Y_LIMIT_BIT;
            GPIO_Init.Mode = limit_ire.y ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING;
            GPIO_Init.Pull = settings->limits.disable_pullup.y ? GPIO_NOPULL : GPIO_PULLUP;
            HAL_GPIO_Init(Y_LIMIT_PORT, &GPIO_Init);

        gpio_init(X_LIMIT_PIN);
        gpio_set_pulls(X_LIMIT_PIN, !settings->limits.disable_pullup.x, false);

        gpio_init(Y_LIMIT_PIN);
        gpio_set_pulls(Y_LIMIT_PIN, !settings->limits.disable_pullup.y, false);
                
#ifdef A_LIMIT_BIT
        gpio_init(A_LIMIT_PIN);
        gpio_set_pulls(A_LIMIT_PIN, !settings->limits.disable_pullup.a, false);
#endif       

        if (settings->limits.flags.hard_enabled) {
            gpio_set_irq_enabled(Z_LIMIT_PIN, gpio_get_edge_event(limit_ire.z), true);
            gpio_acknowledge_irq(Z_LIMIT_PIN, gpio_get_edge_event(limit_ire.z));
            gpio_set_irq_enabled(X_LIMIT_PIN, gpio_get_edge_event(limit_ire.x), true);
            gpio_acknowledge_irq(X_LIMIT_PIN, gpio_get_edge_event(limit_ire.x));
            gpio_set_irq_enabled(Y_LIMIT_PIN, gpio_get_edge_event(limit_ire.y), true);
            gpio_acknowledge_irq(Y_LIMIT_PIN, gpio_get_edge_event(limit_ire.y));
#ifdef A_LIMIT_BIT
            gpio_set_irq_enabled(A_LIMIT_PIN, gpio_get_edge_event(limit_ire.a), true);
            gpio_acknowledge_irq(A_LIMIT_PIN, gpio_get_edge_event(limit_ire.a));
#endif
        }

#ifdef PROBE_PIN
        /**********************
         *  Probe pin config  *
         **********************/

        gpio_init(PROBE_PIN);
        gpio_set_pulls(PROBE_PIN, !probe.disable_probe_pullup, false);
        gpio_set_irq_enabled(PROBE_PIN, gpio_get_edge_event(!probe.invert_probe_pin), true);
        gpio_acknowledge_irq(PROBE_PIN, gpio_get_edge_event(!probe.invert_probe_pin));
#endif

#if KEYPAD_ENABLE
        gpio_init(KEYPAD_STROBE_PIN);
        gpio_set_pulls(PROBE_PIN, true, false);
        gpio_set_irq_enabled(PROBE_PIN, GPIO_IRQ_EDGE_RISE, true);
        gpio_set_irq_enabled(PROBE_PIN, GPIO_IRQ_EDGE_FALL, true);
        gpio_acknowledge_irq(PROBE_PIN, GPIO_IRQ_EDGE_RISE);
        gpio_acknowledge_irq(PROBE_PIN, GPIO_IRQ_EDGE_FALL);
#endif

        irq_set_priority(IO_IRQ_BANK0, NVIC_MEDIUM_LEVEL_PRIORITY); // By default all IRQ are medium priority but in case the GPIO IRQ would need high or low priority it can be done here 
        irq_set_enabled(IO_IRQ_BANK0, true);                        // Enable GPIO IRQ
    irq_set_exclusive_handler(IO_IRQ_BANK0, GPIO_IRQHandler);
    irq_set_enabled(IO_IRQ_BANK0, true);

//        __HAL_GPIO_EXTI_CLEAR_IT(DRIVER_IRQMASK);


//        HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 2);
//        HAL_NVIC_EnableIRQ(EXTI0_IRQn);


    }
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
 // Stepper init

    gpio_init_mask(DIRECTION_MASK|STEPPERS_DISABLE_MASK);
    gpio_set_dir_out_masked(DIRECTION_MASK|STEPPERS_DISABLE_MASK);

    pulse = pio_add_program(pio0, &step_pulse_program);
    timer = pio_add_program(pio1, &stepper_timer_program);

    step_pulse_program_init(pio0, 0, pulse, STEP_PINS_BASE, N_AXIS);
    stepper_timer_program_init(pio1, 0, timer, 12.5f); // 10MHz

    irq_set_exclusive_handler(PIO1_IRQ_0, STEPPER_TIMER_IRQHandler);
    irq_set_enabled(PIO1_IRQ_0, true);


// Limit pins init

    if (settings->limits.flags.hard_enabled)
        gpio_set_irq_enabled_with_callback()
        HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x02, 0x02);

 // Control pins init

 // Limit pins init

    gpio_init_mask(LIMIT_MASK);

/*
    if (settings->limits.flags.hard_enabled)
        HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x02, 0x02);
*/

 // Control pins init

    gpio_init_mask(CONTROL_MASK);

/*
    if(hal.driver_cap.software_debounce) {

        // Single-shot 0.1 ms per tick
        DEBOUNCE_TIMER->CR1 |= TIM_CR1_OPM|TIM_CR1_DIR|TIM_CR1_CKD_1|TIM_CR1_ARPE|TIM_CR1_URS;
        DEBOUNCE_TIMER->PSC = hal.f_step_timer / 10000UL - 1;
        DEBOUNCE_TIMER->SR &= ~TIM_SR_UIF;
        DEBOUNCE_TIMER->ARR = 400; // 40 ms timeout
        DEBOUNCE_TIMER->DIER |= TIM_DIER_UIE;

        HAL_NVIC_EnableIRQ(DEBOUNCE_TIMER_IRQn); // Enable debounce interrupt
    }
    


 // Spindle init

#if SPINDLE_OUTMODE != GPIO_IOEXPAND
    gpio_init_mask(SPINDLE_MASK);
    gpio_set_dir_out_masked(SPINDLE_MASK);
#endif

#ifdef SPINDLE_PWM_PIN
    if(hal.driver_cap.variable_spindle) {
        gpio_init(SPINDLE_PWM_PIN);
        gpio_set_function(SPINDLE_PWM_PIN, GPIO_FUNC_PWM);
    }
#endif

 // Coolant init

#if COOLANT_OUTMODE != GPIO_IOEXPAND
    gpio_init_mask(COOLANT_MASK);
    gpio_set_dir_out_masked(COOLANT_MASK);
#endif

#if SDCARD_ENABLE

    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Pin = SD_CS_BIT;
    HAL_GPIO_Init(SD_CS_PORT, &GPIO_Init);

    BITBAND_PERI(SD_CS_PORT->ODR, SD_CS_PIN) = 1;

    sdcard_init();

#endif

#if PPI_ENABLE

    // Single-shot 1 us per tick
    PPI_TIMER->CR1 |= TIM_CR1_OPM|TIM_CR1_DIR|TIM_CR1_CKD_1|TIM_CR1_ARPE|TIM_CR1_URS;
    PPI_TIMER->PSC = hal.f_step_timer / 1000000UL - 1;
    PPI_TIMER->SR &= ~(TIM_SR_UIF|TIM_SR_CC1IF);
    PPI_TIMER->CNT = 0;
    PPI_TIMER->DIER |= TIM_DIER_UIE;

    HAL_NVIC_EnableIRQ(PPI_TIMER_IRQn);

#endif

    IOInitDone = settings->version == 19;

    hal.settings_changed(settings);
    hal.spindle.set_state((spindle_state_t){0}, 0.0f);
    hal.coolant.set_state((coolant_state_t){0});
    stepperSetDirOutputs((axes_signals_t){0});

#if PPI_ENABLE
    ppi_init();
#endif

    return IOInitDone;
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done

bool driver_init (void)
{
    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

//    irq_set_exclusive_handler(-1, systick_handler);

//mpu_hw->rvr = 999;
//mpu_hw->csr = M0PLUS_SYST_CSR_TICKINT_BITS|M0PLUS_SYST_CSR_ENABLE_BITS;
// M0PLUS_SYST_CSR_CLKSOURCE_BITS - set to use processor clock

#if USB_SERIAL_CDC
    usbInit();
#else
    serialInit(115200);
#endif

#ifdef I2C_PORT
    I2C_Init();
#endif

    hal.info = "RP2040";
    hal.driver_version = "21xxxx";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.driver_setup = driver_setup;
    hal.f_step_timer = 10000000;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = driver_delay;
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

#ifdef PROBE_PIN
    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigure;
#endif

    hal.spindle.set_state = spindleSetState;
    hal.spindle.get_state = spindleGetState;
#ifdef SPINDLE_PWM_DIRECT
    hal.spindle.get_pwm = spindleGetPWM;
    hal.spindle.update_pwm = spindle_set_speed;
#else
    hal.spindle.update_rpm = spindleUpdateRPM;
#endif
#if PPI_ENABLE
    hal.spindle.pulse_on = spindlePulseOn;
#endif

    hal.control.get_state = systemGetState;

    hal.get_elapsed_ticks = getElapsedTicks;
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;

#if USB_SERIAL_CDC
    hal.stream.read = usbGetC;
    hal.stream.write = usbWriteS;
    hal.stream.write_all = usbWriteS;
    hal.stream.get_rx_buffer_available = usbRxFree;
    hal.stream.reset_read_buffer = usbRxFlush;
    hal.stream.cancel_read_buffer = usbRxCancel;
    hal.stream.suspend_read = usbSuspendInput;
#else
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

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

#ifdef CONTROL_SAFETY_DOOR_PIN
    hal.signals_cap.safety_door_ajar = On;
#endif
    hal.driver_cap.spindle_dir = On;
#ifdef SPINDLE_PWM_PIN
    hal.driver_cap.variable_spindle = On;
#endif
    hal.driver_cap.spindle_pwm_invert = On;
#ifdef COOLANT_MIST_PIN
    hal.driver_cap.mist_control = On;
#endif
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
#ifdef PROBE_PIN
    hal.driver_cap.probe_pull_up = On;
#endif

#ifdef HAS_BOARD_INIT
    board_init();
#endif

#if TRINAMIC_ENABLE
    trinamic_init();
#endif

#if KEYPAD_ENABLE
    keypad_init();
#endif

#if IOEXPAND_ENABLE
    ioexpand_init();
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

    my_plugin_init();

#if ODOMETER_ENABLE
    odometer_init(); // NOTE: this *must* be last plugin to be initialized as it claims storage at the end of NVS.
#endif

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 8;
}

/* interrupt handlers */

// Main stepper driver
void STEPPER_TIMER_IRQHandler (void)
{
    pirq = pio1->irq;
    stepper_timer_irq_clear(pio1);

    hal.stepper.interrupt_callback();
}

// Debounce timer interrupt handler
void DEBOUNCE_TIMER_IRQHandler (void)
{
    if(debounce.limits) {
        debounce.limits = Off;
        axes_signals_t state = (axes_signals_t)limitsGetState();
        if(state.value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
            hal.limits.interrupt_callback(state);
    }

    if(debounce.door) {
        debounce.door = Off;
        control_signals_t state = (control_signals_t)systemGetState();
        if(state.safety_door_ajar)
            hal.control.interrupt_callback(state);
    }
    
    uint8_t grp = 0;
    input_signal_t *signal;

    TMR3_CSCTRL0 &= ~TMR_CSCTRL_TCF1;

    while((signal = get_debounce())) {

        signal->gpio.reg->IMR |= signal->gpio.bit;

        if(((signal->gpio.reg->DR & signal->gpio.bit) != 0) == (signal->irq_mode == IRQ_Mode_Falling ? 0 : 1))
            grp |= signal->group;
    }

    if(grp & INPUT_GROUP_LIMIT)
        hal.limits.interrupt_callback(limitsGetState());

    if(grp & INPUT_GROUP_CONTROL)
        hal.control.interrupt_callback(systemGetState());

#if QEI_SELECT_ENABLED

    if(grp & INPUT_GROUP_QEI_SELECT) {
        if(!qei.dbl_click_timeout)
            qei.dbl_click_timeout = qei.encoder.settings->dbl_click_window;
        else if(qei.dbl_click_timeout < qei.encoder.settings->dbl_click_window - 40) {
            qei.dbl_click_timeout = 0;
            qei.encoder.event.dbl_click = On;
            hal.encoder.on_event(&qei.encoder, qei.count);
        }
    }

#endif


}

#if PPI_ENABLE

// PPI timer interrupt handler
void PPI_TIMER_IRQHandler (void)
{
    PPI_TIMER->SR = ~TIM_SR_UIF; // clear UIF flag;

    spindle_off();
}

#endif

static void GPIO_IRQHandler(void)
{
    uint32_t ifg = iobank0_hw->proc0_irq_ctrl.ints[1];

    iobank0_hw->intr[1] = 0b1100 << 16;

    if(ifg) {
  //      __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if CONTROL_MASK & (1<<4)
  #if ENABLE_SAFETY_DOOR_INPUT_PIN
        if(hal.driver_cap.software_debounce) {
            debounce.door = On;
 //           DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
//            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
  #endif
        hal.control.interrupt_callback(systemGetState());
#else
        if(hal.driver_cap.software_debounce) {
            debounce.limits = On;
            hardware_alarm_set_target()
        } else
            hal.limits.interrupt_callback(limitsGetState());
#endif
    }
}

// Interrupt handler for 1 ms interval timer
void isr_systick (void)
{
    static uint32_t cnt = 0;

    cnt++;
    /*

#if SDCARD_ENABLE
    static uint32_t fatfs_ticks = 10;
    if(!(--fatfs_ticks)) {
        disk_timerproc();
        fatfs_ticks = 10;
    }
#endif
    uwTick += uwTickFreq;

    if(delay.ms && !(--delay.ms)) {
        if(delay.callback) {
            delay.callback();
            delay.callback = NULL;
        }
    }
    */
}
