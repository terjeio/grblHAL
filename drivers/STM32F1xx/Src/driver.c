/*

  driver.c - driver code for STM32F103C8 ARM processors

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

#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "main.h"

#include "driver.h"
#include "serial.h"

#include "grbl/limits.h"

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

#if FLASH_ENABLE
#include "flash.h"
#endif

typedef union {
    uint8_t mask;
    struct {
        uint8_t limits :1,
                door   :1,
                unused :6;
    };
} debounce_t;

extern __IO uint32_t uwTick;
static uint32_t pulse_length, pulse_delay;
static bool pwmEnabled = false, IOInitDone = false;
static axes_signals_t next_step_outbits;
static spindle_pwm_t spindle_pwm;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
static on_unknown_sys_command_ptr on_unknown_sys_command;
static debounce_t debounce;
static probe_state_t probe = {
    .connected = On
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

#if KEYPAD_ENABLE == 0
#define KEYPAD_STROBE_BIT 0
#endif

#define DRIVER_IRQMASK (LIMIT_MASK|CONTROL_MASK|KEYPAD_STROBE_BIT)

static void spindle_set_speed (uint_fast16_t pwm_value);

static void driver_delay (uint32_t ms, void (*callback)(void))
{
    if((delay.ms = ms) > 0) {
        // Restart systick...
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
#if TRINAMIC_ENABLE && TRINAMIC_I2C
    trinamic_stepper_enable(enable);
#else
    BITBAND_PERI(STEPPERS_DISABLE_PORT->ODR, STEPPERS_DISABLE_PIN) = enable.x;
#endif
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    stepperEnable((axes_signals_t){AXES_BITMASK});

    STEPPER_TIMER->ARR = 5000; // delay to allow drivers time to wake up
    STEPPER_TIMER->EGR = TIM_EGR_UG;
    STEPPER_TIMER->CR1 |= TIM_CR1_CEN;

//    hal.stepper.interrupt_callback();   // and start the show
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    STEPPER_TIMER->CR1 &= ~TIM_CR1_CEN;
    STEPPER_TIMER->CNT = 0;
}


// Sets up stepper driver interrupt timeout, "Normal" version
static void stepperCyclesPerTickPrescaled (uint32_t cycles_per_tick)
{
    // Set timer prescaling for normal step generation
    if (cycles_per_tick < (1UL << 16)) { // < 65536  (1.1ms @ 72MHz)
        STEPPER_TIMER->PSC = 0; // DIV 1
    } else if (cycles_per_tick < (1UL << 19)) { // < 524288 (8.8ms @ 72MHz)
        STEPPER_TIMER->PSC = 7; // DIV 8
        cycles_per_tick = cycles_per_tick >> 3;
    } else {
        STEPPER_TIMER->PSC = 63; // DIV64
        cycles_per_tick = cycles_per_tick >> 6;
    }
    STEPPER_TIMER->ARR = (uint16_t)(cycles_per_tick - 1);
}

// Set stepper pulse output pins
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z...
inline static __attribute__((always_inline)) void stepperSetStepOutputs (axes_signals_t step_outbits)
{
#if STEP_OUTMODE == GPIO_MAP
	STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | step_outmap[step_outbits.value];
#else
    STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | ((step_outbits.mask ^ settings.steppers.step_invert.mask) << STEP_OUTMODE);
#endif
}

// Set stepper direction output pins
// NOTE: see note for stepperSetStepOutputs()
inline static __attribute__((always_inline)) void stepperSetDirOutputs (axes_signals_t dir_outbits)
{
#if DIRECTION_OUTMODE == GPIO_MAP
    DIRECTION_PORT->ODR = (DIRECTION_PORT->ODR & ~DIRECTION_MASK) | dir_outmap[dir_outbits.value];
#else
    DIRECTION_PORT->ODR = (DIRECTION_PORT->ODR & ~DIRECTION_MASK) | ((dir_outbits.mask ^ settings.steppers.dir_invert.mask) << DIRECTION_OUTMODE);
#endif
}

// Sets stepper direction and pulse pins and starts a step pulse.
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_change)
        stepperSetDirOutputs(stepper->dir_outbits);

    if(stepper->step_outbits.value) {
        stepperSetStepOutputs(stepper->step_outbits);
        PULSE_TIMER->EGR = TIM_EGR_UG;
        PULSE_TIMER->CR1 |= TIM_CR1_CEN;
    }
}

// Start a stepper pulse, delay version.
// Note: delay is only added when there is a direction change and a pulse to be output.
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->dir_change) {

        stepperSetDirOutputs(stepper->dir_outbits);

        if(stepper->step_outbits.value) {
            next_step_outbits = stepper->step_outbits; // Store out_bits
            PULSE_TIMER->ARR = pulse_delay;
            PULSE_TIMER->EGR = TIM_EGR_UG;
            PULSE_TIMER->CR1 |= TIM_CR1_CEN;
        }

        return;
    }

    if(stepper->step_outbits.value) {
        stepperSetStepOutputs(stepper->step_outbits);
        PULSE_TIMER->EGR = TIM_EGR_UG;
        PULSE_TIMER->CR1 |= TIM_CR1_CEN;
    }
}

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing)
{
    if(on && settings.limits.flags.hard_enabled) {
        EXTI->PR |= LIMIT_MASK;     // Clear any pending limit interrupts
        EXTI->IMR |= LIMIT_MASK;    // and enable
    } else
        EXTI->IMR &= ~LIMIT_MASK;

#if TRINAMIC_ENABLE
    trinamic_homing(homing);
#endif
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};

#if LIMIT_INMODE == GPIO_BITBAND
    signals.min.x = BITBAND_PERI(LIMIT_PORT->IDR, X_LIMIT_PIN);
    signals.min.y = BITBAND_PERI(LIMIT_PORT->IDR, Y_LIMIT_PIN);
    signals.min.z = BITBAND_PERI(LIMIT_PORT->IDR, Z_LIMIT_PIN);
#elif LIMIT_INMODE == GPIO_MAP
    uint32_t bits = LIMIT_PORT->IDR;
    signals.min.x = (bits & X_LIMIT_BIT) != 0;
    signals.min.y = (bits & Y_LIMIT_BIT) != 0;
    signals.min.z = (bits & Z_LIMIT_BIT) != 0;
#else
    signals.min.value = (uint8_t)((LIMIT_PORT->IDR & LIMIT_MASK) >> LIMIT_INMODE);
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

#if CONTROL_INMODE == GPIO_BITBAND
    signals.reset = BITBAND_PERI(CONTROL_PORT->IDR, CONTROL_RESET_PIN);
    signals.feed_hold = BITBAND_PERI(CONTROL_PORT->IDR, CONTROL_FEED_HOLD_PIN);
    signals.cycle_start = BITBAND_PERI(CONTROL_PORT->IDR, CONTROL_CYCLE_START_PIN);
 #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    signals.safety_door_ajar = BITBAND_PERI(CONTROL_PORT->IDR, CONTROL_SAFETY_DOOR_PIN);
 #endif
#elif CONTROL_INMODE == GPIO_MAP
    uint32_t bits = CONTROL_PORT->IDR;
    signals.reset = (bits & CONTROL_RESET_BIT) != 0;
    signals.feed_hold = (bits & CONTROL_FEED_HOLD_BIT) != 0;
    signals.cycle_start = (bits & CONTROL_CYCLE_START_BIT) != 0;
 #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    signals.safety_door_ajar = (bits & CONTROL_SAFETY_DOOR_BIT) != 0;
 #endif
#else
    signals.value = (uint8_t)((CONTROL_PORT->IDR & CONTROL_MASK) >> CONTROL_INMODE);
 #ifndef ENABLE_SAFETY_DOOR_INPUT_PIN
 	signals.safety_door_ajar = settings.control_invert.safety_door_ajar;
 #endif
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
    state.triggered = !!(PROBE_PORT->IDR & PROBE_BIT) ^ probe.inverted;

    return state;
}

#endif

// Static spindle (off, on cw & on ccw)

inline static void spindle_off (void)
{
    BITBAND_PERI(SPINDLE_ENABLE_PORT->ODR, SPINDLE_ENABLE_PIN) = settings.spindle.invert.on;
}

inline static void spindle_on (void)
{
    BITBAND_PERI(SPINDLE_ENABLE_PORT->ODR, SPINDLE_ENABLE_PIN) = !settings.spindle.invert.on;
}

inline static void spindle_dir (bool ccw)
{
    if(hal.driver_cap.spindle_dir)
        BITBAND_PERI(SPINDLE_DIRECTION_PORT->ODR, SPINDLE_DIRECTION_PIN) = ccw ^ settings.spindle.invert.ccw;
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
            SPINDLE_PWM_TIMER->CCR1 = spindle_pwm.off_value;
            SPINDLE_PWM_TIMER->BDTR |= TIM_BDTR_MOE;
        } else
        	SPINDLE_PWM_TIMER->BDTR &= ~TIM_BDTR_MOE; // Set PWM output low
    } else {
        if(!pwmEnabled)
            spindle_on();
        pwmEnabled = true;
        SPINDLE_PWM_TIMER->CCR1 = pwm_value;
        SPINDLE_PWM_TIMER->BDTR |= TIM_BDTR_MOE;
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
    spindle_state_t state = {0};

    state.on = (SPINDLE_ENABLE_PORT->IDR & SPINDLE_ENABLE_BIT) != 0;
    state.ccw = hal.driver_cap.spindle_dir && (SPINDLE_DIRECTION_PORT->IDR & SPINDLE_DIRECTION_BIT) != 0;

    state.value ^= settings.spindle.invert.mask;

    return state;
}

// end spindle code

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;
    BITBAND_PERI(COOLANT_FLOOD_PORT->ODR, COOLANT_FLOOD_PIN) = mode.flood;
    BITBAND_PERI(COOLANT_MIST_PORT->ODR, COOLANT_MIST_PIN) = mode.mist;
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

    state.flood = (COOLANT_FLOOD_PORT->IDR & COOLANT_FLOOD_BIT) != 0;
    state.mist  = (COOLANT_MIST_PORT->IDR & COOLANT_MIST_BIT) != 0;
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

static uint32_t getElapsedTicks (void)
{
    return uwTick;
}

// Configures peripherals when settings are initialized or changed
void settings_changed (settings_t *settings)
{
    hal.driver_cap.variable_spindle = settings->spindle.rpm_min < settings->spindle.rpm_max;

#if (STEP_OUTMODE == GPIO_MAP) || (DIRECTION_OUTMODE == GPIO_MAP)
    uint8_t i;
#endif

#if STEP_OUTMODE == GPIO_MAP

    i = sizeof(step_outmap) / sizeof(uint32_t);
    do {
        i--;
        step_outmap[i] = c_step_outmap[i ^ settings->steppers.step_invert.value];
    } while(i);
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

        GPIO_InitTypeDef GPIO_Init;

        GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;

        stepperEnable(settings->steppers.deenergize);

        if(hal.driver_cap.variable_spindle) {

        	hal.spindle.set_state = spindleSetStateVariable;

            SPINDLE_PWM_TIMER->CR1 &= ~TIM_CR1_CEN;

        	spindle_precompute_pwm_values(&spindle_pwm, SystemCoreClock / (settings->spindle.pwm_freq > 200.0f ? 1 : 25));

            TIM_Base_InitTypeDef timerInitStructure = {
                .Prescaler = (settings->spindle.pwm_freq > 200.0f ? 1 : 25) - 1,
                .CounterMode = TIM_COUNTERMODE_UP,
                .Period = spindle_pwm.period - 1,
                .ClockDivision = TIM_CLOCKDIVISION_DIV1,
                .RepetitionCounter = 0
            };

            TIM_Base_SetConfig(SPINDLE_PWM_TIMER, &timerInitStructure);

            SPINDLE_PWM_TIMER->CCER &= ~TIM_CCER_CC1E;
            SPINDLE_PWM_TIMER->CCMR1 &= ~(TIM_CCMR1_OC1M|TIM_CCMR1_CC1S);
            SPINDLE_PWM_TIMER->CCMR1 |= TIM_CCMR1_OC1M_1|TIM_CCMR1_OC1M_2;
            SPINDLE_PWM_TIMER->CCR1 = 0;
            if(settings->spindle.invert.pwm) {
                SPINDLE_PWM_TIMER->CCER |= TIM_CCER_CC1P;
            	SPINDLE_PWM_TIMER->CR2 |= TIM_CR2_OIS1;
            } else {
                SPINDLE_PWM_TIMER->CCER &= ~TIM_CCER_CC1P;
            	SPINDLE_PWM_TIMER->CR2 &= ~TIM_CR2_OIS1;
            }
            SPINDLE_PWM_TIMER->BDTR |= TIM_BDTR_OSSR|TIM_BDTR_OSSI;
            SPINDLE_PWM_TIMER->CCER |= TIM_CCER_CC1E;
            SPINDLE_PWM_TIMER->CR1 |= TIM_CR1_CEN;

        } else
            hal.spindle.set_state = spindleSetState;

        pulse_length = (uint32_t)(10.0f * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY)) - 1;

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            pulse_delay = (uint32_t)(10.0f * (settings->steppers.pulse_delay_microseconds - 1.1f));
            if(pulse_delay < 2)
                pulse_delay = 2;
            else if(pulse_delay == pulse_length)
                pulse_delay++;
            hal.stepper.pulse_start = &stepperPulseStartDelayed;
        } else {
            pulse_delay = 0;
            hal.stepper.pulse_start = &stepperPulseStart;
        }

        PULSE_TIMER->ARR = pulse_length;
        PULSE_TIMER->EGR = TIM_EGR_UG;

#if DRIVER_IRQMASK & (1<<0)
        HAL_NVIC_DisableIRQ(EXTI0_IRQn);
#endif
#if DRIVER_IRQMASK & (1<<1)
        HAL_NVIC_DisableIRQ(EXTI1_IRQn);
#endif
#if DRIVER_IRQMASK & (1<<2)
        HAL_NVIC_DisableIRQ(EXTI2_IRQn);
#endif
#if DRIVER_IRQMASK & (1<<3)
        HAL_NVIC_DisableIRQ(EXTI3_IRQn);
#endif
#if DRIVER_IRQMASK & (1<<4)
        HAL_NVIC_DisableIRQ(EXTI4_IRQn);
#endif
#if DRIVER_IRQMASK & 0x03E0
        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
#endif
#if DRIVER_IRQMASK & 0xFC00
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
#endif

        /*************************
         *  Control pins config  *
         *************************/

        control_signals_t control_ire;

        control_ire.mask = ~(settings->control_disable_pullup.mask ^ settings->control_invert.mask);

        GPIO_Init.Pin = CONTROL_RESET_BIT;
        GPIO_Init.Mode = control_ire.reset ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING;
        GPIO_Init.Pull = settings->control_disable_pullup.reset ? GPIO_NOPULL : GPIO_PULLUP;
        HAL_GPIO_Init(CONTROL_PORT, &GPIO_Init);

        GPIO_Init.Pin = CONTROL_FEED_HOLD_BIT;
        GPIO_Init.Mode = control_ire.feed_hold ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING;
        GPIO_Init.Pull = settings->control_disable_pullup.feed_hold ? GPIO_NOPULL : GPIO_PULLUP;
        HAL_GPIO_Init(CONTROL_PORT, &GPIO_Init);

        GPIO_Init.Pin = CONTROL_CYCLE_START_BIT;
        GPIO_Init.Mode = control_ire.cycle_start ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING;
        GPIO_Init.Pull = settings->control_disable_pullup.cycle_start ? GPIO_NOPULL : GPIO_PULLUP;
        HAL_GPIO_Init(CONTROL_PORT, &GPIO_Init);

#ifdef CONTROL_SAFETY_DOOR_BIT
        GPIO_Init.Pin = CONTROL_SAFETY_DOOR_BIT;
        GPIO_Init.Mode = control_ire.safety_door_ajar ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING;
        GPIO_Init.Pull = settings->control_disable_pullup.safety_door_ajar ? GPIO_NOPULL : GPIO_PULLUP;
        HAL_GPIO_Init(CONTROL_PORT, &GPIO_Init);
#endif

        /***********************
         *  Limit pins config  *
         ***********************/

        if (settings->limits.flags.hard_enabled) {

            axes_signals_t limit_ire;

            limit_ire.mask = ~(settings->limits.disable_pullup.mask ^ settings->limits.invert.mask);

            GPIO_Init.Pin = X_LIMIT_BIT;
            GPIO_Init.Mode = limit_ire.x ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING;
            GPIO_Init.Pull = settings->limits.disable_pullup.x ? GPIO_NOPULL : GPIO_PULLUP;
            HAL_GPIO_Init(LIMIT_PORT, &GPIO_Init);

            GPIO_Init.Pin = Y_LIMIT_BIT;
            GPIO_Init.Mode = limit_ire.y ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING;
            GPIO_Init.Pull = settings->limits.disable_pullup.y ? GPIO_NOPULL : GPIO_PULLUP;
            HAL_GPIO_Init(LIMIT_PORT, &GPIO_Init);

            GPIO_Init.Pin = Z_LIMIT_BIT;
            GPIO_Init.Mode = limit_ire.z ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING;
            GPIO_Init.Pull = settings->limits.disable_pullup.z ? GPIO_NOPULL : GPIO_PULLUP;
            HAL_GPIO_Init(LIMIT_PORT, &GPIO_Init);

#ifdef A_LIMIT_BIT
            GPIO_Init.Pin = A_LIMIT_BIT;
            GPIO_Init.Mode = limit_ire.a ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING;
            GPIO_Init.Pull = settings->limits.disable_pullup.a ? GPIO_NOPULL : GPIO_PULLUP;
            HAL_GPIO_Init(LIMIT_PORT, &GPIO_Init);
#endif

        } else {

            GPIO_Init.Mode = GPIO_MODE_INPUT;

            GPIO_Init.Pin = X_LIMIT_BIT;
            GPIO_Init.Pull = settings->limits.disable_pullup.x ? GPIO_NOPULL : GPIO_PULLUP;
            HAL_GPIO_Init(LIMIT_PORT, &GPIO_Init);

            GPIO_Init.Pin = Y_LIMIT_BIT;
            GPIO_Init.Pull = settings->limits.disable_pullup.y ? GPIO_NOPULL : GPIO_PULLUP;
            HAL_GPIO_Init(LIMIT_PORT, &GPIO_Init);

            GPIO_Init.Pin = Z_LIMIT_BIT;
            GPIO_Init.Pull = settings->limits.disable_pullup.z ? GPIO_NOPULL : GPIO_PULLUP;
            HAL_GPIO_Init(LIMIT_PORT, &GPIO_Init);

#ifdef A_LIMIT_BIT
            GPIO_Init.Pin = A_LIMIT_BIT;
            GPIO_Init.Pull = settings->limits.disable_pullup.a ? GPIO_NOPULL : GPIO_PULLUP;
            HAL_GPIO_Init(LIMIT_PORT, &GPIO_Init);
#endif
        }

        /**********************
         *  Probe pin config  *
         **********************/

        GPIO_Init.Pin = PROBE_BIT;
        GPIO_Init.Mode = GPIO_MODE_INPUT;
        GPIO_Init.Pull = settings->probe.disable_probe_pullup ? GPIO_NOPULL : GPIO_PULLUP;
        HAL_GPIO_Init(PROBE_PORT, &GPIO_Init);

#if KEYPAD_ENABLE
        GPIO_Init.Pin = KEYPAD_STROBE_BIT;
        GPIO_Init.Mode = GPIO_MODE_IT_RISING_FALLING;
        GPIO_Init.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(KEYPAD_PORT, &GPIO_Init);
#endif

        __HAL_GPIO_EXTI_CLEAR_IT(DRIVER_IRQMASK);

#if DRIVER_IRQMASK & (1<<0)
        HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 2);
        HAL_NVIC_EnableIRQ(EXTI0_IRQn);
#endif
#if DRIVER_IRQMASK & (1<<1)
        HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 2);
        HAL_NVIC_EnableIRQ(EXTI1_IRQn);
#endif
#if DRIVER_IRQMASK & (1<<2)
        HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 2);
        HAL_NVIC_EnableIRQ(EXTI2_IRQn);
#endif
#if DRIVER_IRQMASK & (1<<3)
        HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 2);
        HAL_NVIC_EnableIRQ(EXTI3_IRQn);
#endif
#if DRIVER_IRQMASK & (1<<4)
        HAL_NVIC_SetPriority(EXTI4_IRQn, 0, 2);
        HAL_NVIC_EnableIRQ(EXTI4_IRQn);
#endif
#if DRIVER_IRQMASK & 0x03E0
        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 2);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
#endif
#if DRIVER_IRQMASK & 0xFC00
        HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 2);
        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
#endif
    }
}

static status_code_t jtag_enable (uint_fast16_t state, char *line)
{
    if(!strcmp(line, "$PGM")) {
        __HAL_AFIO_REMAP_SWJ_ENABLE();
        on_unknown_sys_command = NULL;
        return Status_OK;
    }

    return on_unknown_sys_command ? on_unknown_sys_command(state, line) : Status_Unhandled;
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
  //    Interrupt_disableSleepOnIsrExit();

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_Init;

    GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;

 // Stepper init

    GPIO_Init.Pin = STEPPERS_DISABLE_MASK;
    HAL_GPIO_Init(STEPPERS_DISABLE_PORT, &GPIO_Init);

    GPIO_Init.Pin = STEP_MASK;
    HAL_GPIO_Init(STEP_PORT, &GPIO_Init);

    GPIO_Init.Pin = DIRECTION_MASK;
    HAL_GPIO_Init(DIRECTION_PORT, &GPIO_Init);

    STEPPER_TIMER->CR1 &= ~TIM_CR1_CEN;
    STEPPER_TIMER->SR &= ~TIM_SR_UIF;
    STEPPER_TIMER->CNT = 0;
    STEPPER_TIMER->DIER |= TIM_DIER_UIE;

    // Single-shot 0.1 us per tick
    PULSE_TIMER->CR1 |= TIM_CR1_OPM|TIM_CR1_DIR|TIM_CR1_CKD_1|TIM_CR1_ARPE|TIM_CR1_URS;
    PULSE_TIMER->PSC = hal.f_step_timer / 10000000UL - 1;
    PULSE_TIMER->SR &= ~(TIM_SR_UIF|TIM_SR_CC1IF);
    PULSE_TIMER->CNT = 0;
    PULSE_TIMER->DIER |= TIM_DIER_UIE;

    //

    NVIC_SetPriority(TIM3_IRQn, 0);
    NVIC_SetPriority(TIM2_IRQn, 1);
    NVIC_EnableIRQ(TIM3_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);

 // Limit pins init

    if (settings->limits.flags.hard_enabled)
        HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0x02, 0x02);

 // Control pins init

    if(hal.driver_cap.software_debounce) {
        // Single-shot 0.1 ms per tick
        DEBOUNCE_TIMER->CR1 |= TIM_CR1_OPM|TIM_CR1_DIR|TIM_CR1_CKD_1|TIM_CR1_ARPE|TIM_CR1_URS;
        DEBOUNCE_TIMER->PSC = hal.f_step_timer / 10000UL - 1;
        DEBOUNCE_TIMER->SR &= ~TIM_SR_UIF;
        DEBOUNCE_TIMER->ARR = 400; // 40 ms timeout
        DEBOUNCE_TIMER->DIER |= TIM_DIER_UIE;

        HAL_NVIC_EnableIRQ(TIM4_IRQn); // Enable debounce interrupt
    }

 // Spindle init

    GPIO_Init.Pin = SPINDLE_DIRECTION_BIT;
    HAL_GPIO_Init(SPINDLE_DIRECTION_PORT, &GPIO_Init);

    GPIO_Init.Pin = SPINDLE_ENABLE_BIT;
    HAL_GPIO_Init(SPINDLE_ENABLE_PORT, &GPIO_Init);

    if(hal.driver_cap.variable_spindle) {
        GPIO_Init.Pin = SPINDLE_PWM_BIT;
        GPIO_Init.Mode = GPIO_MODE_AF_PP;
        HAL_GPIO_Init(SPINDLE_PWM_PORT, &GPIO_Init);
    }

 // Coolant init

    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;

    GPIO_Init.Pin = COOLANT_FLOOD_BIT;
    HAL_GPIO_Init(COOLANT_FLOOD_PORT, &GPIO_Init);

    GPIO_Init.Pin = COOLANT_MIST_BIT;
    HAL_GPIO_Init(COOLANT_MIST_PORT, &GPIO_Init);

    BITBAND_PERI(COOLANT_FLOOD_PORT->ODR, COOLANT_FLOOD_PIN) = 1;
    BITBAND_PERI(COOLANT_MIST_PORT->ODR, COOLANT_MIST_PIN) = 1;

    BITBAND_PERI(COOLANT_FLOOD_PORT->ODR, COOLANT_FLOOD_PIN) = 0;
    BITBAND_PERI(COOLANT_MIST_PORT->ODR, COOLANT_MIST_PIN) = 0;

#if SDCARD_ENABLE

    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_Init.Pin = SD_CS_BIT;
    HAL_GPIO_Init(SD_CS_PORT, &GPIO_Init);

    BITBAND_PERI(SD_CS_PORT->ODR, SD_CS_PIN) = 1;

    sdcard_init();

#endif

    on_unknown_sys_command = grbl.on_unknown_sys_command;
    grbl.on_unknown_sys_command = jtag_enable;

    IOInitDone = settings->version == 19;

    hal.settings_changed(settings);
    hal.spindle.set_state((spindle_state_t){0}, 0.0f);
    hal.coolant.set_state((coolant_state_t){0});
    stepperSetDirOutputs((axes_signals_t){0});

    return IOInitDone;
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done

bool driver_init (void)
{
    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

    // GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); // ??? Disable JTAG and SWD!?? Bug?

#if USB_SERIAL_CDC
    usbInit();
#else
    serialInit();
#endif

#ifdef I2C_PORT
    i2c_init();
#endif

    __HAL_AFIO_REMAP_SWJ_NOJTAG();

    hal.info = "STM32F103C8";
    hal.driver_version = "210206";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.driver_setup = driver_setup;
    hal.f_step_timer = SystemCoreClock;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = &driver_delay;
    hal.settings_changed = settings_changed;

    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTickPrescaled;
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

    hal.control.get_state = systemGetState;

    hal.irq_enable = __enable_irq;
    hal.irq_disable = __disable_irq;
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = getElapsedTicks;

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
    hal.driver_cap.variable_spindle = On;
    hal.driver_cap.spindle_pwm_invert = On;
    hal.driver_cap.mist_control = On;
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;

#if TRINAMIC_ENABLE
    trinamic_init();
#endif

#if KEYPAD_ENABLE
    keypad_init();
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
void TIM2_IRQHandler(void)
{
    if ((STEPPER_TIMER->SR & TIM_SR_UIF) != 0)                  // check interrupt source
    {
        STEPPER_TIMER->SR = ~TIM_SR_UIF; // clear UIF flag
        hal.stepper.interrupt_callback();
    }
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
void TIM3_IRQHandler(void)
{
    PULSE_TIMER->SR &= ~TIM_SR_UIF;                 // Clear UIF flag

    if (PULSE_TIMER->ARR == pulse_delay)            // Delayed step pulse?
    {
        PULSE_TIMER->ARR = pulse_length;
        stepperSetStepOutputs(next_step_outbits);   // begin step pulse
        PULSE_TIMER->EGR = TIM_EGR_UG;
        PULSE_TIMER->CR1 |= TIM_CR1_CEN;
    } else
        stepperSetStepOutputs((axes_signals_t){0}); // end step pulse
}

// Debounce timer interrupt handler
void TIM4_IRQHandler (void)
{
    DEBOUNCE_TIMER->SR = ~TIM_SR_UIF; // clear UIF flag;

    if(debounce.limits) {
        debounce.limits = Off;
        limit_signals_t state = limitsGetState();
        if(limit_signals_merge(state).value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
            hal.limits.interrupt_callback(state);
    }

    if(debounce.door) {
        debounce.door = Off;
        control_signals_t state = systemGetState();
        if(state.safety_door_ajar)
            hal.control.interrupt_callback(state);
    }
}

#if DRIVER_IRQMASK & (1<<0)

void EXTI0_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<0);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);

#if CONTROL_MASK & (1<<0)
  #if CONTROL_SAFETY_DOOR_BIT & (1<<0)
        if(hal.driver_cap.software_debounce) {
            debounce.door = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
  #endif
        hal.control.interrupt_callback(systemGetState());
#else
        if(hal.driver_cap.software_debounce) {
            debounce.limits = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
            hal.limits.interrupt_callback(limitsGetState());
#endif
    }
}

#endif

#if DRIVER_IRQMASK & (1<<1)

void EXTI1_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<1);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if CONTROL_MASK & (1<<1)
  #if CONTROL_SAFETY_DOOR_BIT & (1<<1)
        if(hal.driver_cap.software_debounce) {
            debounce.door = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
  #endif
        hal.control.interrupt_callback(systemGetState());
#else
        if(hal.driver_cap.software_debounce) {
            debounce.limits = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
            hal.limit.interrupt_callback(limitsGetState());
#endif
    }
}

#endif

#if DRIVER_IRQMASK & (1<<2)

void EXTI2_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<2);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if CONTROL_MASK & (1<<2)
  #if CONTROL_SAFETY_DOOR_BIT & (1<<2)
        if(hal.driver_cap.software_debounce) {
            debounce.door = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
  #endif
        hal.control.interrupt_callback(systemGetState());
#else
        if(hal.driver_cap.software_debounce) {
            debounce.limits = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
            hal.limits.interrupt_callback(limitsGetState());
#endif
    }
}

#endif

#if DRIVER_IRQMASK & (1<<3)

void EXTI3_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<3);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if CONTROL_MASK & (1<<3)
  #if CONTROL_SAFETY_DOOR_BIT & (1<<3)
        if(hal.driver_cap.software_debounce) {
            debounce.door = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
  #endif
        hal.control.interrupt_callback(systemGetState());
#else
        if(hal.driver_cap.software_debounce) {
            debounce.limits = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
            hal.limits.interrupt_callback(limitsGetState());
#endif
    }
}

#endif

#if DRIVER_IRQMASK & (1<<4)

void EXTI4_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(1<<4);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);
#if CONTROL_MASK & (1<<4)
  #if CONTROL_SAFETY_DOOR_BIT & (1<<2)
        if(hal.driver_cap.software_debounce) {
            debounce.door = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
  #endif
        hal.control.interrupt_callback(systemGetState());
#else
        if(hal.driver_cap.software_debounce) {
            debounce.limits = On;
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
            hal.limits.interrupt_callback(limitsGetState());
#endif
    }
}

#endif

#if DRIVER_IRQMASK & (0x03E0)

void EXTI9_5_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(0x03E0);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);

#if CONTROL_MASK & 0x03E0
        if(ifg & CONTROL_MASK) {
  #if CONTROL_SAFETY_DOOR_BIT & 0x03E0
            if((ifg & CONTROL_SAFETY_DOOR_BIT) && hal.driver_cap.software_debounce) {
                debounce.door = On;
                DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
                DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
            } else
  #endif
                hal.control.interrupt_callback(systemGetState());
        }
#endif
#if LIMIT_MASK & 0x03E0
        if(ifg & LIMIT_MASK) {
            if(hal.driver_cap.software_debounce) {
                debounce.limits = On;
                DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
                DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
            } else
                hal.limits.interrupt_callback(limitsGetState());
        }
#endif
    }
}

#endif

#if DRIVER_IRQMASK & (0xFC00)

void EXTI15_10_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(0xFC00);

    if(ifg) {
        __HAL_GPIO_EXTI_CLEAR_IT(ifg);

#if CONTROL_MASK & 0xFC00
        if(ifg & CONTROL_MASK) {
  #if CONTROL_SAFETY_DOOR_BIT & 0xFC00
            if((ifg & CONTROL_SAFETY_DOOR_BIT) && hal.driver_cap.software_debounce) {
                debounce.door = On;
                DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
                DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
            } else
  #endif
                hal.control.interrupt_callback(systemGetState());
        }
#endif
#if LIMIT_MASK & 0xFC00
        if(ifg & LIMIT_MASK) {
            if(hal.driver_cap.software_debounce) {
                debounce.limits = On;
                DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
                DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
            } else
                hal.limits.interrupt_callback(limitsGetState());
        }
#endif
#if KEYPAD_ENABLE
        if(ifg & KEYPAD_STROBE_BIT)
            keypad_keyclick_handler(BITBAND_PERI(KEYPAD_PORT->IDR, KEYPAD_STROBE_PIN));
#endif
    }
}

#endif

// Interrupt handler for 1 ms interval timer
void HAL_IncTick(void)
{
#if SDCARD_ENABLE
    static uint32_t fatfs_ticks = 10;
    if(!(--fatfs_ticks)) {
        disk_timerproc();
        fatfs_ticks = 10;
    }
#endif
    uwTick += uwTickFreq;

    if(delay.ms && !(--delay.ms) && delay.callback) {
        delay.callback();
        delay.callback = NULL;
    }
}
