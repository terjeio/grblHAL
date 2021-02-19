/*
  driver.c - driver code for simulator MCU

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

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

#include "mcu.h"
#include "driver.h"
#include "serial.h"
#include "eeprom.h"
#include "grbl_eeprom_extensions.h"
#include "platform.h"

#include "grbl/hal.h"

static bool probe_invert;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup

void SysTick_Handler (void);
void Stepper_IRQHandler (void);
void Limits0_IRQHandler (void);
void Control_IRQHandler (void);

#define SQUARING_ENABLED

#ifdef SQUARING_ENABLED
static axes_signals_t motors_0 = {AXES_BITMASK}, motors_1 = {AXES_BITMASK};

void Limits1_IRQHandler (void);
#endif

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if((delay.ms = ms) > 0) {
        systick_timer.enable = 1;
        if(!(delay.callback = callback))
            while(delay.ms);
    } else if(callback)
        callback();
}

inline static void set_step_outputs (axes_signals_t step_outbits_0)
{
    axes_signals_t step_outbits_1;

    step_outbits_1.mask = (step_outbits_0.mask & motors_1.mask) ^ settings.steppers.step_invert.mask;
    step_outbits_0.mask = (step_outbits_0.mask & motors_0.mask) ^ settings.steppers.step_invert.mask;

    mcu_gpio_set(&gpio[STEP_PORT0], step_outbits_0.mask, AXES_BITMASK);
    mcu_gpio_set(&gpio[STEP_PORT1], step_outbits_1.mask, AXES_BITMASK);
}

inline static void set_dir_outputs (axes_signals_t dir_outbits)
{
    mcu_gpio_set(&gpio[DIR_PORT], dir_outbits.value ^ settings.steppers.dir_invert.mask, AXES_BITMASK);
}

static void stepperEnable (axes_signals_t enable)
{
    mcu_gpio_set(&gpio[STEPPER_ENABLE_PORT], enable.value ^ settings.steppers.enable_invert.mask, AXES_BITMASK);
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    timer[STEPPER_TIMER].load = 5000;
    timer[STEPPER_TIMER].value = 0;
    timer[STEPPER_TIMER].enable = 1;

//    hal.stepper_interrupt_callback();   // start the show
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    timer[STEPPER_TIMER].value = 0;
    timer[STEPPER_TIMER].load = 0;
    timer[STEPPER_TIMER].enable = 0;

    if(clear_signals) {
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}

// Sets up stepper driver interrupt timeout, limiting the slowest speed
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    timer[STEPPER_TIMER].load = cycles_per_tick;
    timer[STEPPER_TIMER].value = 0;
    timer[STEPPER_TIMER].enable = 1;
}

// "Normal" version: Sets stepper direction and pulse pins and starts a step pulse a few nanoseconds later.
// If spindle synchronized motion switch to PID version.
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->new_block) {
        stepper->new_block = false;
        set_dir_outputs(stepper->dir_outbits);
    }

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
    }
}

// Delayed pulse version: sets stepper direction and pulse pins and starts a step pulse with an initial delay.
// If spindle synchronized motion switch to PID version.
// TODO: only delay after setting dir outputs?
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->new_block) {
        stepper->new_block = false;
        set_dir_outputs(stepper->dir_outbits);
    }

    if(stepper->step_outbits.value) {
//        next_step_outbits = stepper->step_outbits; // Store out_bits
//        PULSE_TIMER->CTL |= TIMER_A_CTL_CLR|TIMER_A_CTL_MC1;
    }
}

static axes_signals_t limitsGetState()
{
    axes_signals_t signals = {0};

    signals.value = gpio[LIMITS_PORT0].state.value;

    if (settings.limits.invert.mask)
        signals.value ^= settings.limits.invert.mask;

    return signals;
}

#ifdef SQUARING_ENABLED

// Enable/disable motors for auto squaring of ganged axes
static void StepperDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    motors_0.mask = (mode == SquaringMode_A || mode == SquaringMode_Both ? axes.mask : 0);
    motors_1.mask = (mode == SquaringMode_B || mode == SquaringMode_Both ? axes.mask : 0);
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
static axes_signals_t limitsGetHomeState()
{
    axes_signals_t signals_min = {0}, signals_max = {0};
    
    if(motors_0.mask) {

        signals_min.mask = gpio[LIMITS_PORT0].state.value;

        if (settings.limits.invert.mask)
            signals_min.mask ^= settings.limits.invert.mask;
    }

    if(motors_1.mask) {

       signals_max.mask = gpio[LIMITS_PORT1].state.value;

        if (settings.limits.invert.mask)
            signals_max.mask ^= settings.limits.invert.mask;
    }

    signals_min.mask |= signals_max.mask;

    return signals_min;
}

#endif

static void limitsEnable (bool on, bool homing)
{
    gpio[LIMITS_PORT0].irq_mask.mask = on ? AXES_BITMASK : 0;
    gpio[LIMITS_PORT0].irq_state.mask = 0;

  #ifdef SQUARING_ENABLED
    gpio[LIMITS_PORT1].irq_mask.mask = on ? AXES_BITMASK : 0;
    gpio[LIMITS_PORT1].irq_state.mask = 0;

    hal.limits.get_state = homing ? limitsGetHomeState : limitsGetState;
  #endif
}

static control_signals_t systemGetState (void)
{
    control_signals_t signals = {0};

    signals.value = gpio[CONTROL_PORT].state.value;

    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

    return signals;
}

static void probeConfigureInvertMask (bool is_probe_away, bool probing)
{
  probe_invert = settings.probe.invert_probe_pin;

  if (is_probe_away)
      probe_invert ^= is_probe_away;
}

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.value = mcu_gpio_get(&gpio[PROBE_PORT], PROBE_MASK);

    state.triggered ^= probe_invert;

    return state;
}

// Start or stop spindle
static void spindleSetState (spindle_state_t state, float rpm)
{
    mcu_gpio_set(&gpio[SPINDLE_PORT], state.value ^ settings.spindle.invert.mask, SPINDLE_MASK);
}

// Variable spindle control functions

// Sets spindle speed
static void spindle_set_speed (uint_fast16_t pwm_value)
{
}

#ifdef SPINDLE_PWM_DIRECT

static uint_fast16_t spindleGetPWM (float rpm)
{
    return 0; //spindle_compute_pwm_value(&spindle_pwm, rpm, false);
}

#else

static void spindleUpdateRPM (float rpm)
{
}

#endif

// Start or stop spindle
static void spindleSetStateVariable (spindle_state_t state, float rpm)
{ 
    mcu_gpio_set(&gpio[SPINDLE_PORT], state.value ^ settings.spindle.invert.mask, SPINDLE_MASK);
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {0};

    state.value = gpio[SPINDLE_PORT].state.value ^ settings.spindle.invert.mask;

    return state;
}

static void coolantSetState (coolant_state_t mode)
{
    mcu_gpio_set(&gpio[COOLANT_PORT], mode.value ^ settings.coolant_invert.mask, COOLANT_MASK);
}

static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

    state.value = gpio[COOLANT_PORT].state.value ^ settings.coolant_invert.mask;

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
//    __disable_interrupts();
    *ptr |= bits;
//    __enable_interrupts();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
//    __disable_interrupts();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
//    __enable_interrupts();
    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
//    __disable_interrupts();
    uint_fast16_t prev = *ptr;
    *ptr = value;
//    __enable_interrupts();
    return prev;
}

void settings_changed (settings_t *settings)
{

}

bool driver_setup (settings_t *settings)
{
    timer[STEPPER_TIMER].prescaler = 0;
    timer[STEPPER_TIMER].irq_enable = 1;
    mcu_register_irq_handler(Stepper_IRQHandler, Timer0_IRQ);

    gpio[STEPPER_ENABLE_PORT].dir.mask = AXES_BITMASK;
    gpio[STEP_PORT0].dir.mask = AXES_BITMASK;
    gpio[DIR_PORT].dir.mask = AXES_BITMASK;

    gpio[COOLANT_PORT].dir.mask = COOLANT_MASK;
    gpio[SPINDLE_PORT].dir.mask = SPINDLE_MASK;

    gpio[LIMITS_PORT0].dir.mask = AXES_BITMASK;
    gpio[LIMITS_PORT0].rising.mask = AXES_BITMASK;
    mcu_register_irq_handler(Limits0_IRQHandler, LIMITS_IRQ0);

#ifdef SQUARING_ENABLED
    gpio[STEP_PORT1].dir.mask = AXES_BITMASK;

    gpio[LIMITS_PORT1].dir.mask = AXES_BITMASK;
    gpio[LIMITS_PORT1].rising.mask = AXES_BITMASK;
    mcu_register_irq_handler(Limits1_IRQHandler, LIMITS_IRQ1);

#endif

    gpio[CONTROL_PORT].dir.mask = CONTROL_MASK;
    gpio[CONTROL_PORT].rising.mask = CONTROL_MASK;
    gpio[CONTROL_PORT].irq_mask.mask = CONTROL_MASK;
    mcu_register_irq_handler(Control_IRQHandler, CONTROL_IRQ);

    mcu_gpio_in(&gpio[PROBE_PORT], PROBE_CONNECTED_BIT, PROBE_CONNECTED_BIT); // default to connected

    hal.settings_changed(settings);
    hal.stepper.go_idle(true);
    hal.spindle.set_state((spindle_state_t){0}, 0.0f);
    hal.coolant.set_state((coolant_state_t){0});

    return settings->version == 19;
}

// used to inject a sleep in grbl main loop, 
// ensures hardware simulator gets some cycles in "parallel"
void sim_process_realtime (uint_fast16_t state)
{
    platform_sleep(0);
}

bool driver_init ()
{
    mcu_reset();

    mcu_register_irq_handler(SysTick_Handler, Systick_IRQ);

    systick_timer.load = F_CPU / 1000 - 1;
    systick_timer.irq_enable = 1;

    serialInit();

    hal.info = "Simulator";
    hal.driver_version = "210131";
    hal.driver_setup = driver_setup;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.f_step_timer = F_CPU;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

    grbl.on_execute_realtime = sim_process_realtime;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
#ifdef SQUARING_ENABLED
    hal.stepper.disable_motors = StepperDisableMotors;
#endif

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigureInvertMask;

    hal.spindle.set_state = spindleSetState;
    hal.spindle.get_state = spindleGetState;
#ifdef SPINDLE_PWM_DIRECT
    hal.spindle.get_pwm = spindleGetPWM;
    hal.spindle.update_pwm = spindle_set_speed;
#else
    hal.spindle.update_rpm = spindleUpdateRPM;
#endif

    hal.control.get_state = systemGetState;
/*
    hal.show_message = showMessage;
*/
    hal.stream.read = serialGetC;
    hal.stream.get_rx_buffer_available = serialRxFree;
    hal.stream.reset_read_buffer = serialRxFlush;
    hal.stream.cancel_read_buffer = serialRxCancel;
    hal.stream.write = serialWriteS;
    hal.stream.write_all = serialWriteS;
    hal.stream.suspend_read = serialSuspendInput;

    hal.nvs.type = NVS_EEPROM;
    hal.nvs.get_byte = eeprom_get_char;
    hal.nvs.put_byte = eeprom_put_char;
    hal.nvs.memcpy_to_nvs = memcpy_to_eeprom;
    hal.nvs.memcpy_from_nvs = memcpy_from_eeprom;

    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

    hal.driver_cap.amass_level = 3;
    hal.driver_cap.spindle_dir = On;

    hal.driver_cap.variable_spindle = On;
    hal.driver_cap.spindle_dir = On;

    hal.driver_cap.variable_spindle = On;
    hal.driver_cap.spindle_pwm_invert = On;
    hal.driver_cap.spindle_pwm_linearization = On;
    hal.driver_cap.mist_control = On;
//    hal.driver_cap.software_debounce = On;
//    hal.driver_cap.step_pulse_delay = On;

    hal.signals_cap.safety_door_ajar = On;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;
#ifdef SQUARING_ENABLED
 //   hal.driver_cap.axis_ganged_x = On;
#endif
    // no need to move version check before init - compiler will fail any signature mismatch for existing entries
    return hal.version == 8;
}

// Main stepper driver
void Stepper_IRQHandler (void)
{
    hal.stepper.interrupt_callback();
}

void Control_IRQHandler (void)
{
    gpio[CONTROL_PORT].irq_state.value = ~CONTROL_MASK;
    hal.control.interrupt_callback(hal.control.get_state());
}

void Limits0_IRQHandler (void)
{
    gpio[LIMITS_PORT0].irq_state.value = (uint8_t)~AXES_BITMASK;
    hal.limits.interrupt_callback(hal.limits.get_state());
}

#ifdef SQUARING_ENABLED

void Limits1_IRQHandler (void)
{
    gpio[LIMITS_PORT1].irq_state.value = (uint8_t)~AXES_BITMASK;
    hal.limits.interrupt_callback(hal.limits.get_state());
}

#endif

// Interrupt handler for 1 ms interval timer
void SysTick_Handler (void)
{
    if(!(--delay.ms)) {
        systick_timer.enable = 0;
        if(delay.callback) {
            delay.callback();
            delay.callback = NULL;
        }
    }
}
