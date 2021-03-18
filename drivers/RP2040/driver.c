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

#include "hardware/timer.h"
#include "hardware/irq.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
#include "hardware/spi.h"
#include "hardware/structs/systick.h"
#include "hardware/structs/iobank0.h"
#include "hardware/structs/sio.h"

#include "driver.h"
#include "serial.h"
#include "driverPIO.pio.h"

#include "grbl/state_machine.h"

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
static void gpio_int_handler (uint gpio, uint32_t events);
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
    uint32_t value;
    struct {
        uint32_t delay  :8,
                 length :8,
                 set    :6,
                 reset  :6;
    };
} pio_steps_t;

static pio_steps_t pio_steps = { .delay = 20, .length = 100 };
static uint pulse, timer;
static uint16_t pulse_length, pulse_delay;
static bool pwmEnabled = false, IOInitDone = false;
static axes_signals_t next_step_outbits;
static spindle_pwm_t spindle_pwm;
static status_code_t (*on_unknown_sys_command)(uint_fast16_t state, char *line, char *lcline);
static volatile uint32_t elapsed_ticks = 0;
static volatile bool ms_event = false;
static probe_state_t probe;
static bool probeInputState = false;
static bool safetyDoorInputState = false;

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

static void gpio_int_handler (uint gpio, uint32_t events);

typedef struct {
    uint8_t pin;
    uint8_t bit;
} control_input_t;

static control_input_t control_input[] = {
#if ESTOP_ENABLE
    { .pin = CONTROL_RESET_PIN,         .bit = SIGNALS_ESTOP_BIT },
#else
    { .pin = CONTROL_RESET_PIN,         .bit = SIGNALS_RESET_BIT },
#endif
    { .pin = CONTROL_FEED_HOLD_PIN,     .bit = SIGNALS_FEEDHOLD_BIT },
    { .pin = CONTROL_CYCLE_START_PIN,   .bit = SIGNALS_CYCLESTART_BIT },
};

static uint8_t limit_input[] = {
    X_LIMIT_PIN,
    Y_LIMIT_PIN,
    Z_LIMIT_PIN,
#if N_AXIS > 3
    A_LIMIT_PIN,
#endif
#if N_AXIS > 4
    B_LIMIT_PIN,
#endif
#if N_AXIS > 5
    C_LIMIT_PIN,
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

// This should be a sdk function but it doesn't exist yet
#define gpio_set_irqover(gpio, value) hw_write_masked(&iobank0_hw->io[gpio].ctrl, value << IO_BANK0_GPIO0_CTRL_IRQOVER_LSB, IO_BANK0_GPIO0_CTRL_IRQOVER_BITS);

#define NVIC_HIGH_LEVEL_PRIORITY 0xC0
#define NVIC_MEDIUM_LEVEL_PRIORITY 0x80
#define NVIC_LOW_LEVEL_PRIORITY 0x40

#define DRIVER_IRQMASK (LIMIT_MASK|CONTROL_MASK|KEYPAD_STROBE_BIT|SPINDLE_INDEX_BIT)

#define GPIO_IRQ_ACK_ALL 0xFu

#define DEBOUNCE_ALARM_HW_TIMER 0   // Hardware alarm timer 0 used for the debounce alarm pool
#define DEBOUNCE_ALARM_MAX_TIMER 16 // Maximum number of alarm timer in the debounce alarm pool (based on SDK 'PICO_TIME_DEFAULT_ALARM_POOL_MAX_TIMERS 16' for default pool used for driver_delay in driver.c)
#define LIMIT_DEBOUNCE_TEMPO    40  // 40ms for Limit debounce
#define SR_LATCH_DEBOUNCE_TEMPO 40  // 40ms for SR LATCH 

typedef struct {
    alarm_id_t id;
    uint8_t pin;
    uint8_t level;
} debounce_pool_t;

static alarm_pool_t *  debounceAlarmPool;
static volatile debounce_pool_t debounceAlarmPoolArray[DEBOUNCE_ALARM_MAX_TIMER];

static void systick_handler (void);
static void spindle_set_speed (uint_fast16_t pwm_value);
static void STEPPER_TIMER_IRQHandler(void);
static void GPIO_IRQHandler(void);

static int64_t delay_callback(alarm_id_t id, void *callback)
{
    ((delay_callback_ptr)callback)();

    return 0;
}

static void driver_delay (uint32_t ms, delay_callback_ptr callback)
{
    if(ms > 0) {
        if(callback)
            add_alarm_in_ms(ms, delay_callback, callback, false);
        else {
            uint32_t delay = ms * 1000, start = timer_hw->timerawl;
            while(timer_hw->timerawl - start < delay);
        }
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
    bool active = on & settings.limits.flags.hard_enabled;

    for(int i=0; i<(sizeof(limit_input)/sizeof(uint8_t)); i++) {
        // Activate or de-activate the Limit GPIO IRQ
        gpio_set_irq_enabled(limit_input[i], GPIO_IRQ_EDGE_RISE, active);
    }

#if TRINAMIC_ENABLE
    trinamic_homing(homing);
#endif
}

// Returns limit state as an limit_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};

#if LIMIT_INMODE == GPIO_MAP
    signals.min.x = gpio_get(X_LIMIT_PIN);
    signals.min.y = gpio_get(Y_LIMIT_PIN);
    signals.min.z = gpio_get(Z_LIMIT_PIN);
  #ifdef A_LIMIT_PIN
    signals.min.a = gpio_get(A_LIMIT_PIN);
  #endif
  #ifdef B_LIMIT_PIN
    signals.min.b = gpio_get(B_LIMIT_PIN);
  #endif
  #ifdef C_LIMIT_PIN
    signals.min.c = gpio_get(C_LIMIT_PIN);
  #endif
#endif

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
static control_signals_t systemGetState (void)
{
    control_signals_t signals;

#if CONTROL_INMODE == GPIO_MAP
  #ifdef ESTOP_ENABLE
    signals.e_stop = gpio_get(CONTROL_RESET_PIN);
  #else                                   
    signals.reset = gpio_get(CONTROL_RESET_PIN);
  #endif
    signals.feed_hold = gpio_get(CONTROL_FEED_HOLD_PIN);
    signals.cycle_start = gpio_get(CONTROL_CYCLE_START_PIN);
  #ifdef CONTROL_SAFETY_DOOR_PIN
    signals.safety_door_ajar = safetyDoorInputState;
  #endif
#endif

    return signals;
}

//*************************  PROBE  *************************//

#ifdef PROBE_PIN

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
    probe.triggered = false;
    probe.is_probing = probing;
    probe.inverted = is_probe_away ? !settings.probe.invert_probe_pin : settings.probe.invert_probe_pin;
}

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;
    state.triggered = probeInputState;

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

#if COOLANT_OUTMODE == GPIO_IOEXPAND
    mode.value ^= settings.coolant_invert.mask;
    ioex_out(COOLANT_FLOOD_PIN) = mode.flood;
    #ifdef COOLANT_MIST_PIN
    ioex_out(COOLANT_MIST_PIN) = mode.mist;
    #endif
    ioexpand_out(io_expander);
#else
    gpio_put(COOLANT_FLOOD_PIN, mode.flood);
    #ifdef COOLANT_MIST_PIN
    gpio_put(COOLANT_MIST_PIN, mode.mist);
    #endif
#endif
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state;

#if COOLANT_OUTMODE == GPIO_IOEXPAND
//    state = {settings.coolant_invert.mask};  Not sure this is needed (debug??)
    state.flood = ioex_out(COOLANT_FLOOD_PIN);
    #ifdef COOLANT_MIST_PIN
    state.mist = ioex_out(COOLANT_MIST_PIN);
    state.value ^= settings.coolant_invert.mask;
    #endif
#else
    state.flood = !!(sio_hw->gpio_out & COOLANT_FLOOD_BIT);
    #ifdef COOLANT_MIST_PIN
    state.mist  = !!(sio_hw->gpio_out & COOLANT_MIST_BIT);
    #endif
#endif

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

static uint32_t getElapsedTicks (void)
{
   return elapsed_ticks;
}

#if MPG_MODE_ENABLE

static void modeSelect (bool mpg_mode)
{
    gpio_set_irq_enabled(MODE_SWITCH_PIN, !mpg_mode ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL, false);
    gpio_set_irq_enabled(MODE_SWITCH_PIN, mpg_mode ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL, true);

    // Deny entering MPG mode if busy
    sys_state_t state = state_get();
    if(mpg_mode == sys.mpg_mode || (mpg_mode && (gc_state.file_run || !(state == STATE_IDLE || (state & (STATE_ALARM|STATE_ESTOP)))))) {
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
    modeSelect(gpio_get(MODE_SWITCH_PIN) == 0);
}

// Save the gpio that has generated the IRQ together with the alarm pool id and the edge
static void debounce_alarm_pool_save_gpio (alarm_id_t id, uint pin, uint level)
{
    static volatile uint8_t index = 0;

    debounceAlarmPoolArray[index].id = id;
    debounceAlarmPoolArray[index].pin = pin;
    debounceAlarmPoolArray[index].level = level;
    index = (index + 1) % DEBOUNCE_ALARM_MAX_TIMER;
}

static void modeEnable (void)
{
    bool on = gpio_get(MODE_SWITCH_PIN) == 0;

    if(sys.mpg_mode != on)
        modeSelect(true);

    gpio_set_irq_enabled(MODE_SWITCH_PIN, !on ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL, false);
    gpio_set_irq_enabled(MODE_SWITCH_PIN, on ? GPIO_IRQ_EDGE_RISE : GPIO_IRQ_EDGE_FALL, true);
}

#endif

// Configures peripherals when settings are initialized or changed
void settings_changed (settings_t *settings)
{

#ifdef SPINDLE_PWM_PIN
    hal.driver_cap.variable_spindle = settings->spindle.rpm_min < settings->spindle.rpm_max;
#endif

#if (DIRECTION_OUTMODE == GPIO_MAP)
    uint8_t i = sizeof(dir_outmap) / sizeof(uint32_t);
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

        // Set the GPIO interrupt handler, the pin doesn't matter for now
        gpio_set_irq_enabled_with_callback(0, 0, false, gpio_int_handler); 

        // Disable GPIO IRQ while initializing the input pins
        irq_set_enabled(IO_IRQ_BANK0, false);
        
        /*************************
         *  Control pins config  *
         *************************/

        for(int i=0; i<(sizeof(control_input)/sizeof(control_input_t)); i++) {

            uint8_t pin = control_input[i].pin;

            gpio_init(pin);
            gpio_set_pulls(pin, !(!!(settings->control_disable_pullup.mask & control_input[i].bit)), false);
            gpio_set_irqover(pin, (!!(settings->control_invert.mask & control_input[i].bit) ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL));
            gpio_set_inover(pin, (!!(settings->control_invert.mask & control_input[i].bit) ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL));
            gpio_set_irq_enabled(pin, GPIO_IRQ_EDGE_RISE, true);
        }

#if CONTROL_SAFETY_DOOR_PIN
        gpio_init(CONTROL_SAFETY_DOOR_PIN);
        gpio_set_pulls(CONTROL_SAFETY_DOOR_PIN, !settings->control_disable_pullup.safety_door_ajar, false);
        gpio_set_irqover(CONTROL_SAFETY_DOOR_PIN, (settings->control_invert.safety_door_ajar ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL));
        gpio_set_irq_enabled(CONTROL_SAFETY_DOOR_PIN, GPIO_IRQ_LEVEL_HIGH, true);
#endif

        /***********************
         *  Limit pins config  *
         ***********************/

        for(int i=0; i<(sizeof(limit_input)/sizeof(uint8_t)); i++) {

            uint8_t pin = limit_input[i];

            gpio_init(pin);
            gpio_set_pulls(pin, !(!!(settings->limits.disable_pullup.mask & 0x1u<<i)), false);
            gpio_set_irqover(pin, (!!(settings->limits.invert.mask & 0x1u<<i) ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL));
            gpio_set_inover(pin, (!!(settings->limits.invert.mask & 0x1u<<i) ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL));
        }

#ifdef PROBE_PIN
        gpio_set_pulls(PROBE_PIN, !settings->probe.disable_probe_pullup, false);
        gpio_set_irqover(PROBE_PIN, (settings->probe.invert_probe_pin ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL));
        gpio_set_irq_enabled(PROBE_PIN, GPIO_IRQ_LEVEL_HIGH, true);
#endif

#if KEYPAD_ENABLE
        gpio_pull_up(KEYPAD_STROBE_PIN);
        gpio_set_irq_enabled(KEYPAD_STROBE_PIN, GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL, true);
#endif    

#if MPG_MODE_ENABLE
        if(hal.driver_cap.mpg_mode) {
            // Enable pullup
            gpio_set_pulls(MODE_SWITCH_PIN, true, false);
            // Delay mode enable a bit so grblHAL can finish startup and MPG controller can check ready status
            hal.delay_ms(50, modeEnable);
        }
#endif

#if COOLANT_OUTMODE != GPIO_IOEXPAND
        gpio_set_outover(COOLANT_FLOOD_PIN, (settings->coolant_invert.flood ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL));
    #ifdef COOLANT_MIST_PIN
        gpio_set_outover(COOLANT_FLOOD_PIN, (settings->coolant_invert.mist ? GPIO_OVERRIDE_INVERT : GPIO_OVERRIDE_NORMAL));
    #endif
#endif

        //Activate GPIO IRQ
        irq_set_priority(IO_IRQ_BANK0, NVIC_MEDIUM_LEVEL_PRIORITY); // By default all IRQ are medium priority but in case the GPIO IRQ would need high or low priority it can be done here 
        irq_set_enabled(IO_IRQ_BANK0, true);                        // Enable GPIO IRQ
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

    gpio_init_mask(LIMIT_MASK);

 // Control pins init

    gpio_init_mask(CONTROL_MASK);

 // Spindle init

#if SPINDLE_OUTMODE != GPIO_IOEXPAND
    gpio_init_mask(SPINDLE_MASK);
    gpio_set_dir_out_masked(SPINDLE_MASK);
#endif

    if(hal.driver_cap.variable_spindle) {
        gpio_init(SPINDLE_PWM_PIN);
        gpio_set_function(SPINDLE_PWM_PIN, GPIO_FUNC_PWM);
    }

#if PROBE_PIN
    gpio_init(PROBE_PIN);
#endif

 // Coolant init

#if COOLANT_OUTMODE != GPIO_IOEXPAND
    gpio_init_mask(COOLANT_MASK);
    gpio_set_dir_out_masked(COOLANT_MASK);
#endif

#if SDCARD_ENABLE
    sdcard_init();
#endif

#if PPI_ENABLE
/*
    // Single-shot 1 us per tick
    PPI_TIMER->CR1 |= TIM_CR1_OPM|TIM_CR1_DIR|TIM_CR1_CKD_1|TIM_CR1_ARPE|TIM_CR1_URS;
    PPI_TIMER->PSC = hal.f_step_timer / 1000000UL - 1;
    PPI_TIMER->SR &= ~(TIM_SR_UIF|TIM_SR_CC1IF);
    PPI_TIMER->CNT = 0;
    PPI_TIMER->DIER |= TIM_DIER_UIE;

    HAL_NVIC_EnableIRQ(PPI_TIMER_IRQn);
*/
#endif

#if MPG_MODE_ENABLE
    gpio_init(MODE_SWITCH_PIN);
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

#if USB_SERIAL_CDC
static void execute_realtime (uint_fast16_t state)
{
    if(ms_event) {
        ms_event = false;
        usb_execute_realtime(state);
    }
}
#endif

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{
    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

//    irq_set_exclusive_handler(-1, systick_handler);

    systick_hw->rvr = 999;
    systick_hw->cvr = 0;
    systick_hw->csr = M0PLUS_SYST_CSR_TICKINT_BITS|M0PLUS_SYST_CSR_ENABLE_BITS;

#if USB_SERIAL_CDC
    usb_serialInit();
#else
    serialInit(115200);
#endif

#ifdef SERIAL2_MOD
    serial2Init(115200);
#endif

#ifdef I2C_PORT
    I2C_Init();
#endif

    hal.info = "RP2040";
    hal.driver_version = "21xxxx";
    hal.driver_options = "SDK_" PICO_SDK_VERSION_STRING;
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
    hal.stream.read = usb_serialGetC;
    hal.stream.write = usb_serialWriteS;
    hal.stream.write_all = usb_serialWriteS;
    hal.stream.get_rx_buffer_available = usb_serialRxFree;
    hal.stream.reset_read_buffer = usb_serialRxFlush;
    hal.stream.cancel_read_buffer = usb_serialRxCancel;
    hal.stream.suspend_read = usb_serialSuspendInput;
    grbl.on_execute_realtime = execute_realtime;
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
#if MPG_MODE_ENABLE
    hal.driver_cap.mpg_mode = On;
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

    debounceAlarmPool = alarm_pool_create(DEBOUNCE_ALARM_HW_TIMER, DEBOUNCE_ALARM_MAX_TIMER);

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 8;
}

/* interrupt handlers */

// Main stepper driver
void STEPPER_TIMER_IRQHandler (void)
{
    stepper_timer_irq_clear(pio1);

    hal.stepper.interrupt_callback();
}

// Limit debounce callback
static int64_t limit_debounce_callback(alarm_id_t id, void *array)
{
    debounce_pool_t * pool = (debounce_pool_t *)array;

    // Find which pin set this callback and re-enable its IRQ
    for(int i=0; i<DEBOUNCE_ALARM_MAX_TIMER; i++) {
        if(pool[i].id == id) {
            gpio_set_irq_enabled(pool[i].pin, GPIO_IRQ_EDGE_RISE, true);
            break;
        }
    }
    
    hal.limits.interrupt_callback(limitsGetState());
    return 0;
}

// SR Latch callback - 
static int64_t srLatch_debounce_callback(alarm_id_t id, void *array)
{
    debounce_pool_t * pool = (debounce_pool_t *)array;

    // Find which pin set this callback and re-enable its IRQ
    for(int i=0; i<DEBOUNCE_ALARM_MAX_TIMER; i++) {
        if(pool[i].id == id) {
            gpio_set_irq_enabled(pool[i].pin, pool[i].level, true);
            return 0;;
        }
    }

    return 0;
}

#if PPI_ENABLE

// PPI timer interrupt handler
void PPI_TIMER_IRQHandler (void)
{
    PPI_TIMER->SR = ~TIM_SR_UIF; // clear UIF flag;

    spindle_off();
}

#endif

// GPIO Interrupt handler
void gpio_int_handler(uint gpio, uint32_t events)
{
    volatile alarm_id_t id;
    volatile uint8_t nextIRQ_Level;

    switch(gpio) {

#if CONTROL_SAFETY_DOOR_PIN
        case CONTROL_SAFETY_DOOR_PIN:
            if(events & (GPIO_IRQ_LEVEL_HIGH | GPIO_IRQ_LEVEL_LOW)) {
                safetyDoorInputState = !!(events & GPIO_IRQ_LEVEL_HIGH);                            // No use of the Setting invert because the signal has been inverted directly at the pad
                nextIRQ_Level = safetyDoorInputState ? GPIO_IRQ_LEVEL_LOW : GPIO_IRQ_LEVEL_HIGH;    // Determine the next IRQ level
                gpio_set_irq_enabled(gpio, GPIO_IRQ_LEVEL_HIGH | GPIO_IRQ_LEVEL_LOW, false);        // De-activate both IRQ level for that pin
                // Create an alarm to trigger an IT in "SR_LATCH_DEBOUNCE_TEMPO" ms to re-enable the IRQ
                id = alarm_pool_add_alarm_in_ms(debounceAlarmPool, SR_LATCH_DEBOUNCE_TEMPO, srLatch_debounce_callback, (void*)debounceAlarmPoolArray, false);
                if(id > 0)
                    debounce_alarm_pool_save_gpio(id, gpio, nextIRQ_Level);                          // Save the alarm id together with the pin that generated this IRQ and the next IRQ level
                else
                    gpio_set_irq_enabled(gpio, nextIRQ_Level, true);                                 // Re-activate the IRQ for that pin in case the alarm didn't trigger
            }
#endif
        case CONTROL_RESET_PIN:
        case CONTROL_FEED_HOLD_PIN:
        case CONTROL_CYCLE_START_PIN:
            hal.control.interrupt_callback(systemGetState());
            break;
            

#if PROBE_PIN
        case PROBE_PIN:
            if(events & (GPIO_IRQ_LEVEL_HIGH | GPIO_IRQ_LEVEL_LOW)) {
                probeInputState = !!(events & GPIO_IRQ_LEVEL_HIGH);                                 // No use of the Setting invert because the signal has been inverted directly at the pad
                nextIRQ_Level = probeInputState ? GPIO_IRQ_LEVEL_LOW : GPIO_IRQ_LEVEL_HIGH;         // Save which next IRQ level will trigger
                gpio_set_irq_enabled(gpio, GPIO_IRQ_LEVEL_HIGH | GPIO_IRQ_LEVEL_LOW, false);        // De-activate both IRQ level for that pin
                // Create an alarm to trigger an IT in SR_LATCH_DEBOUNCE_TEMPO ms
                id = alarm_pool_add_alarm_in_ms(debounceAlarmPool, SR_LATCH_DEBOUNCE_TEMPO, srLatch_debounce_callback, (void*)debounceAlarmPoolArray, false);
                if(id > 0)
                    debounce_alarm_pool_save_gpio(id, gpio, nextIRQ_Level);                          // Save the alarm id together with the pin that generated this IRQ and the next IRQ level
                else
                    gpio_set_irq_enabled(gpio, nextIRQ_Level, true);                                 // Re-activate the IRQ for that pin in case the alarm didn't trigger                   
            }
            break;
#endif

#if A_LIMIT_PIN
        case A_LIMIT_PIN:
#endif
#if B_LIMIT_PIN
        case B_LIMIT_PIN:
#endif
#if C_LIMIT_PIN
        case C_LIMIT_PIN:
#endif
        case X_LIMIT_PIN:
        case Y_LIMIT_PIN:
        case Z_LIMIT_PIN:
            // If debounce is activated
            if(hal.driver_cap.software_debounce)
            {
                // Create a new alarm in the debounce alarm pool to re-activate the IRQ when the delay ms will be done
                id = alarm_pool_add_alarm_in_ms(debounceAlarmPool, LIMIT_DEBOUNCE_TEMPO, limit_debounce_callback, (void*)debounceAlarmPoolArray, true);
                // if the alarm has been created
                if(id > 0) {
                    debounce_alarm_pool_save_gpio(id, gpio, false);             // Save the alarm id together with the pin that generated this IRQ
                    gpio_set_irq_enabled(gpio, GPIO_IRQ_EDGE_RISE, false);      // De-activate the IRQ for that pin
                }
            } else
                hal.limits.interrupt_callback(limitsGetState());
            break;

#if KEYPAD_ENABLE
        case KEYPAD_STROBE_PIN:
            keypad_keyclick_handler(gpio_get(gpio) == 0);
            break;
#endif
#if MPG_MODE_ENABLE
        case MODE_SWITCH_PIN:
            gpio_set_irq_enabled(MODE_SWITCH_PIN, GPIO_IRQ_EDGE_RISE|GPIO_IRQ_EDGE_FALL, false);
            hal.delay_ms(50, modeChange);
            break;
#endif
        default:
            break;
    }
}

// Interrupt handler for 1 ms interval timer
void isr_systick (void)
{
    ms_event = true;
    elapsed_ticks++;

#if SDCARD_ENABLE
    static uint32_t fatfs_ticks = 10;
    if(!(--fatfs_ticks)) {
        disk_timerproc();
        fatfs_ticks = 10;
    }
#endif
}
