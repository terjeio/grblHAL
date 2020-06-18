/*

  driver.c - driver code for STM32F103C8 ARM processors

  Part of GrblHAL

  Copyright (c) 2019-2020 Terje Io

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

#include "main.h"

#include "grbl.h"

#include "driver.h"
#include "serial.h"

#ifdef I2C_PORT
#include "i2c.h"
#endif

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "ff.h"
#include "diskio.h"
#endif

#if USB_ENABLE
#include "usb_serial.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#if KEYPAD_ENABLE
#include "keypad/keypad.h"
#endif

#if FLASH_ENABLE
#include "flash.h"
#endif

#ifdef DRIVER_SETTINGS
driver_settings_t driver_settings;
#endif

extern __IO uint32_t uwTick;

static bool pwmEnabled = false, IOInitDone = false;
// Inverts the probe pin state depending on user settings and probing cycle mode.
static bool probe_invert;
static axes_signals_t next_step_outbits;
static spindle_pwm_t spindle_pwm;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
static status_code_t (*syscmd)(uint_fast16_t state, char *line, char *lcline);

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
    axes_signals_t tmc_enable = trinamic_stepper_enable(enable);
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

//    hal.stepper_interrupt_callback();   // and start the show
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
inline static void stepperSetStepOutputs (axes_signals_t step_outbits)
{
#if STEP_OUTMODE == GPIO_MAP
	STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | step_outmap[step_outbits.value];
#else
    STEP_PORT->ODR = (STEP_PORT->ODR & ~STEP_MASK) | ((step_outbits.mask ^ settings.steppers.step_invert.mask) << STEP_OUTMODE);
#endif
}

// Set stepper direction output pins
// NOTE: see note for stepperSetStepOutputs()
inline static void stepperSetDirOutputs (axes_signals_t dir_outbits)
{
#if DIRECTION_OUTMODE == GPIO_MAP
    DIRECTION_PORT->ODR = (DIRECTION_PORT->ODR & ~DIRECTION_MASK) | dir_outmap[dir_outbits.value];
#else
    DIRECTION_PORT->ODR = (DIRECTION_PORT->ODR & ~DIRECTION_MASK) | ((dir_outbits.mask ^ settings.steppers.dir_invert.mask) << DIRECTION_OUTMODE);
#endif
}

// Sets stepper direction and pulse pins and starts a step pulse
// When delayed pulse the step register is written in the step delay interrupt handler
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->new_block) {
        stepper->new_block = false;
        stepperSetDirOutputs(stepper->dir_outbits);
    }

    if(stepper->step_outbits.value) {
        stepperSetStepOutputs(stepper->step_outbits);
        PULSE_TIMER->EGR = TIM_EGR_UG;
        PULSE_TIMER->CR1 |= TIM_CR1_CEN;
    }
}

// Sets stepper direction and pulse pins and starts a step pulse with and initial delay
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->new_block) {
        stepper->new_block = false;
        stepperSetDirOutputs(stepper->dir_outbits);
    }

    if(stepper->step_outbits.value) {
        next_step_outbits = stepper->step_outbits; // Store out_bits
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
inline static axes_signals_t limitsGetState()
{
    axes_signals_t signals;

#if LIMIT_INMODE == GPIO_BITBAND
    signals.x = BITBAND_PERI(LIMIT_PORT->IDR, X_LIMIT_PIN);
    signals.y = BITBAND_PERI(LIMIT_PORT->IDR, Y_LIMIT_PIN);
    signals.z = BITBAND_PERI(LIMIT_PORT->IDR, Z_LIMIT_PIN);
#elif LIMIT_INMODE == GPIO_MAP
    uint32_t bits = LIMIT_PORT->IDR;
    signals.x = (bits & X_LIMIT_BIT) != 0;
    signals.y = (bits & Y_LIMIT_BIT) != 0;
    signals.z = (bits & Z_LIMIT_BIT) != 0;
#else
    signals.value = (uint8_t)((LIMIT_PORT->IDR & LIMIT_MASK) >> LIMIT_INMODE);
#endif

    if (settings.limits.invert.mask)
        signals.value ^= settings.limits.invert.mask;

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
static control_signals_t systemGetState (void)
{
    control_signals_t signals = {0};

#if CONTROL_INMODE == GPIO_BITBAND
    signals.reset = BITBAND_PERI(CONTROL_PORT->IDR, CONTROL_RESET_PIN);
    signals.safety_door_ajar = BITBAND_PERI(CONTROL_PORT->IDR, CONTROL_SAFETY_DOOR_PIN);
    signals.feed_hold = BITBAND_PERI(CONTROL_PORT->IDR, CONTROL_FEED_HOLD_PIN);
    signals.cycle_start = BITBAND_PERI(CONTROL_PORT->IDR, CONTROL_CYCLE_START_PIN);
#elif CONTROL_INMODE == GPIO_MAP
    uint32_t bits = CONTROL_PORT->IDR;
    signals.reset = (bits & CONTROL_RESET_BIT) != 0;
    signals.safety_door_ajar = (bits & CONTROL_SAFETY_DOOR_BIT) != 0;
    signals.feed_hold = (bits & CONTROL_FEED_HOLD_BIT) != 0;
    signals.cycle_start = (bits & CONTROL_CYCLE_START_BIT) != 0;
#else
    signals.value = (uint8_t)((CONTROL_PORT->IDR & CONTROL_MASK) >> CONTROL_INMODE);
#endif

    if(settings.control_invert.mask)
        signals.value ^= settings.control_invert.mask;

    return signals;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigureInvertMask(bool is_probe_away)
{
    probe_invert = settings.flags.invert_probe_pin;

    if (is_probe_away)
        probe_invert ^= PROBE_BIT;
}

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {
        .connected = On
    };

    state.triggered = ((PROBE_PORT->IDR & PROBE_BIT) != 0)  ^ probe_invert;

    return state;
}

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
        if(settings.spindle.disable_with_zero_speed)
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
    if(pwmEnabled)
        state.value = On;

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

        GPIO_InitTypeDef GPIO_Init = {0};

        GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;


      #if TRINAMIC_ENABLE
        trinamic_configure();
      #endif

        stepperEnable(settings->steppers.deenergize);

        if(hal.driver_cap.variable_spindle) {

        	hal.spindle_set_state = spindleSetStateVariable;

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
            hal.spindle_set_state = spindleSetState;

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds) {
            hal.stepper_pulse_start = &stepperPulseStartDelayed;
            PULSE_TIMER->DIER |= TIM_DIER_CC1IE; // Enable CC1 interrupt
        } else {
            hal.stepper_pulse_start = &stepperPulseStart;
            PULSE_TIMER->DIER &= ~TIM_DIER_CC1IE; // Disable CC1 interrupt
        }

        PULSE_TIMER->ARR = settings->steppers.pulse_microseconds + settings->steppers.pulse_delay_microseconds - 1;
        PULSE_TIMER->CCR1 = settings->steppers.pulse_delay_microseconds;
        PULSE_TIMER->EGR = TIM_EGR_UG;

        /*************************
         *  Control pins config  *
         *************************/

        control_signals_t control_ire;

        control_ire.mask = ~(settings->control_disable_pullup.mask ^ settings->control_invert.mask);

        HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);

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

        GPIO_Init.Pin = CONTROL_SAFETY_DOOR_BIT;
        GPIO_Init.Mode = control_ire.safety_door_ajar ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING;
        GPIO_Init.Pull = settings->control_disable_pullup.safety_door_ajar ? GPIO_NOPULL : GPIO_PULLUP;
        HAL_GPIO_Init(CONTROL_PORT, &GPIO_Init);

        __HAL_GPIO_EXTI_CLEAR_IT(CONTROL_MASK);

        HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 2);
        HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

        /***********************
         *  Limit pins config  *
         ***********************/

        if (settings->limits.flags.hard_enabled) {

            axes_signals_t limit_ire;

            limit_ire.mask = ~(settings->limits.disable_pullup.mask ^ settings->limits.invert.mask);

            HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

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
#ifdef B_LIMIT_BIT
            GPIO_Init.Pin = B_LIMIT_BIT;
            GPIO_Init.Mode = limit_ire.b ? GPIO_MODE_IT_RISING : GPIO_MODE_IT_FALLING;
            GPIO_Init.Pull = settings->limits.disable_pullup.b ? GPIO_NOPULL : GPIO_PULLUP;
            HAL_GPIO_Init(LIMIT_PORT, &GPIO_Init);
#endif
            __HAL_GPIO_EXTI_CLEAR_IT(LIMIT_MASK);

            HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

        } else {

            GPIO_Init.Mode = GPIO_MODE_INPUT;
            GPIO_Init.Alternate = 0;

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
#ifdef B_LIMIT_BIT
            GPIO_Init.Pin = B_LIMIT_BIT;
            GPIO_Init.Pull = settings->limits.disable_pullup.b ? GPIO_NOPULL : GPIO_PULLUP;
            HAL_GPIO_Init(LIMIT_PORT, &GPIO_Init);
#endif
        }

        /**********************
         *  Probe pin config  *
         **********************/

        GPIO_Init.Pin = PROBE_BIT;
        GPIO_Init.Mode = GPIO_MODE_INPUT;
        GPIO_Init.Pull = settings->flags.disable_probe_pullup ? GPIO_NOPULL : GPIO_PULLUP;
        HAL_GPIO_Init(PROBE_PORT, &GPIO_Init);

#if KEYPAD_ENABLE
        HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);

        GPIO_Init.Pin = KEYPAD_STROBE_BIT;
        GPIO_Init.Mode = GPIO_MODE_IT_RISING_FALLING;
        GPIO_Init.Pull = GPIO_PULLUP;
        HAL_GPIO_Init(KEYPAD_PORT, &GPIO_Init);

        HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
#endif
    }
}

static status_code_t jtag_enable (uint_fast16_t state, char *line, char *lcline)
{
    if(!strcmp(line, "$PGM")) {
        //__HAL_AFIO_REMAP_SWJ_ENABLE();
        syscmd = NULL;
        return Status_OK;
    }

    return syscmd ? syscmd(state, line, lcline) : Status_Unhandled;
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
  //    Interrupt_disableSleepOnIsrExit();

    __HAL_RCC_TIM1_CLK_ENABLE();
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();

    GPIO_InitTypeDef GPIO_Init = {0};

    GPIO_Init.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_Init.Mode = GPIO_MODE_OUTPUT_PP;

#ifdef DRIVER_SETTINGS
    if(hal.eeprom.driver_area.address != 0) {
        if(!hal.eeprom.memcpy_from_with_checksum((uint8_t *)&driver_settings, hal.eeprom.driver_area.address, sizeof(driver_settings)))
            hal.driver_settings_restore();
      #if TRINAMIC_ENABLE && CNC_BOOSTERPACK // Trinamic BoosterPack does not support mixed drivers
        driver_settings.trinamic.driver_enable.mask = AXES_BITMASK;
      #endif
    }
#endif

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

    // Single-shot 1 us per tick
    PULSE_TIMER->CR1 |= TIM_CR1_OPM|TIM_CR1_DIR|TIM_CR1_CKD_1|TIM_CR1_ARPE|TIM_CR1_URS;
    PULSE_TIMER->PSC = hal.f_step_timer / 1000000UL - 1;
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
        GPIO_Init.Pull = GPIO_NOPULL;
        GPIO_Init.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(SPINDLE_PWM_PORT, &GPIO_Init);
        GPIO_Init.Alternate = 0;
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

    syscmd = hal.driver_sys_command_execute;
    hal.driver_sys_command_execute = jtag_enable;

#if TRINAMIC_ENABLE

    trinamic_init();

#endif

    IOInitDone = settings->version == 16;

    settings_changed(settings);

    hal.spindle_set_state((spindle_state_t){0}, 0.0f);
    hal.coolant_set_state((coolant_state_t){0});
    stepperSetDirOutputs((axes_signals_t){0});

    return IOInitDone;
}

#ifdef DRIVER_SETTINGS

static status_code_t driver_setting (setting_type_t param, float value, char *svalue)
{
    status_code_t status = Status_Unhandled;

#if KEYPAD_ENABLE
    status = keypad_setting(param, value, svalue);
#endif

#if TRINAMIC_ENABLE
    if(status == Status_Unhandled)
        status = trinamic_setting(param, value, svalue);
#endif

    if(status == Status_OK)
        hal.eeprom.memcpy_to_with_checksum(hal.eeprom.driver_area.address, (uint8_t *)&driver_settings, sizeof(driver_settings));

    return status;
}

static void driver_settings_report (setting_type_t setting)
{
#if KEYPAD_ENABLE
    keypad_settings_report(setting);
#endif
#if TRINAMIC_ENABLE
    trinamic_settings_report(setting);
#endif
}

static void driver_settings_restore ()
{
#if KEYPAD_ENABLE
    keypad_settings_restore();
#endif
#if TRINAMIC_ENABLE
    trinamic_settings_restore();
#endif
    hal.eeprom.memcpy_to_with_checksum(hal.eeprom.driver_area.address, (uint8_t *)&driver_settings, sizeof(driver_settings));
}

#endif

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done

bool driver_init (void)
{
    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

    // GPIO_PinRemapConfig(GPIO_Remap_SWJ_Disable, ENABLE); // ??? Disable JTAG and SWD!?? Bug?

#if USB_ENABLE
    usbInit();
#else
    serialInit();
#endif

#ifdef I2C_PORT
    i2c_init();
#endif

    //__HAL_AFIO_REMAP_SWJ_NOJTAG();

    hal.info = "STM32F401CC";
    hal.driver_version = "200528";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.driver_setup = driver_setup;
    hal.f_step_timer = SystemCoreClock;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = &driver_delay;
    hal.settings_changed = settings_changed;

    hal.stepper_wake_up = stepperWakeUp;
    hal.stepper_go_idle = stepperGoIdle;
    hal.stepper_enable = stepperEnable;
    hal.stepper_cycles_per_tick = stepperCyclesPerTickPrescaled;
    hal.stepper_pulse_start = stepperPulseStart;

    hal.limits_enable = limitsEnable;
    hal.limits_get_state = limitsGetState;

    hal.coolant_set_state = coolantSetState;
    hal.coolant_get_state = coolantGetState;

    hal.probe_get_state = probeGetState;
    hal.probe_configure_invert_mask = probeConfigureInvertMask;

    hal.spindle_set_state = spindleSetState;
    hal.spindle_get_state = spindleGetState;
#ifdef SPINDLE_PWM_DIRECT
    hal.spindle_get_pwm = spindleGetPWM;
    hal.spindle_update_pwm = spindle_set_speed;
#else
    hal.spindle_update_rpm = spindleUpdateRPM;
#endif

    hal.system_control_get_state = systemGetState;

    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;

#if USB_ENABLE
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
    hal.eeprom.type = EEPROM_Physical;
    hal.eeprom.get_byte = eepromGetByte;
    hal.eeprom.put_byte = eepromPutByte;
    hal.eeprom.memcpy_to_with_checksum = eepromWriteBlockWithChecksum;
    hal.eeprom.memcpy_from_with_checksum = eepromReadBlockWithChecksum;
#elif FLASH_ENABLE
    hal.eeprom.type = EEPROM_Emulated;
    hal.eeprom.memcpy_from_flash = memcpy_from_flash;
    hal.eeprom.memcpy_to_flash = memcpy_to_flash;
#else
    hal.eeprom.type = EEPROM_None;
#endif

#ifdef DRIVER_SETTINGS
  #if !TRINAMIC_ENABLE
    assert(EEPROM_ADDR_TOOL_TABLE - (sizeof(driver_settings_t) + 2) > EEPROM_ADDR_GLOBAL + sizeof(settings_t) + 1);
    hal.eeprom.driver_area.address = EEPROM_ADDR_TOOL_TABLE - (sizeof(driver_settings_t) + 2);
  #else
    hal.eeprom.driver_area.address = GRBL_EEPROM_SIZE;
  #endif
    hal.eeprom.driver_area.size = sizeof(driver_settings_t);
    hal.eeprom.size = GRBL_EEPROM_SIZE + sizeof(driver_settings_t) + 1;

    hal.driver_setting = driver_setting;
    hal.driver_settings_report = driver_settings_report;
    hal.driver_settings_restore = driver_settings_restore;

#endif

#if TRINAMIC_ENABLE
    hal.user_mcode_check = trinamic_MCodeCheck;
    hal.user_mcode_validate = trinamic_MCodeValidate;
    hal.user_mcode_execute = trinamic_MCodeExecute;
    hal.driver_rt_report = trinamic_RTReport;
    hal.driver_axis_settings_report = trinamic_axis_settings_report;
#endif

#if KEYPAD_ENABLE
    hal.execute_realtime = keypad_process_keypress;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

#ifdef CONTROL_SAFETY_DOOR_PIN
    hal.driver_cap.safety_door = On;
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
#if SDCARD_ENABLE
    hal.driver_cap.sd_card = On;
#endif

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 6;
}

/* interrupt handlers */

// Main stepper driver
void TIM2_IRQHandler(void)
{
    if ((STEPPER_TIMER->SR & TIM_SR_UIF) != 0)                  // check interrupt source
    {
        STEPPER_TIMER->SR = ~TIM_SR_UIF; // clear UIF flag
        STEPPER_TIMER->CNT = 0;
        hal.stepper_interrupt_callback();
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
    if ((PULSE_TIMER->SR & TIM_SR_CC1IF) != 0)          // Delayed step pulse?
    {
        PULSE_TIMER->SR &= ~(TIM_SR_UIF|TIM_SR_CC1IF);  // Clear UIF & CC1IF flags and
        stepperSetStepOutputs(next_step_outbits);       // begin step pulse
    } else {
        PULSE_TIMER->SR &= ~TIM_SR_UIF;                 // Clear UIF flag and
        stepperSetStepOutputs((axes_signals_t){0});     // end step pulse
    }
}

// Debounce timer interrupt handler
void TIM4_IRQHandler (void)
{
    DEBOUNCE_TIMER->SR = ~TIM_SR_UIF; // clear UIF flag;

    axes_signals_t state = (axes_signals_t)limitsGetState();

    if(state.value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
        hal.limit_interrupt_callback(state);
}

void EXTI15_10_IRQHandler(void)
{
#if KEYPAD_ENABLE
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(LIMIT_MASK|KEYPAD_STROBE_BIT);
#else
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(LIMIT_MASK);
#endif
    if(ifg) {

        __HAL_GPIO_EXTI_CLEAR_IT(ifg);

#if KEYPAD_ENABLE
        if(ifg & KEYPAD_STROBE_BIT)
            keypad_keyclick_handler(BITBAND_PERI(KEYPAD_PORT->IDR, KEYPAD_STROBE_PIN));
        if(ifg & LIMIT_MASK) {
#endif

        if(hal.driver_cap.software_debounce) {
            DEBOUNCE_TIMER->EGR = TIM_EGR_UG;
            DEBOUNCE_TIMER->CR1 |= TIM_CR1_CEN; // Start debounce timer (40ms)
        } else
            hal.limit_interrupt_callback(limitsGetState());

#if KEYPAD_ENABLE
        }
#endif
    }
}

void EXTI9_5_IRQHandler(void)
{
    uint32_t ifg = __HAL_GPIO_EXTI_GET_IT(CONTROL_MASK);

    if(ifg) {

        __HAL_GPIO_EXTI_CLEAR_IT(ifg);

        hal.control_interrupt_callback(systemGetState());
    }
}

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

    if(delay.ms && !(--delay.ms)) {
        if(delay.callback) {
            delay.callback();
            delay.callback = NULL;
        }
    }
}
