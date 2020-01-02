/*

  driver.c - driver code for NXP LPC176x ARM processors

  Part of GrblHAL

  Copyright (c) 2018-2019 Terje Io

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

#include "grbl/grbl.h"

#include "driver.h"
#include "serial.h"
#include "grbl-lpc/pwm_driver.h"

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "ff.h"
#include "diskio.h"
#endif

#if FLASH_ENABLE
#include "flash.h"
#endif

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#endif

#if USB_ENABLE
#include "usb_serial.h"
#endif

#if EEPROM_ENABLE
#include "eeprom.h"
#endif

static bool pwmEnabled = false, IOInitDone = false;
// Inverts the probe pin state depending on user settings and probing cycle mode.
static uint8_t probe_invert_mask;
static axes_signals_t next_step_outbits;
static spindle_pwm_t spindle_pwm;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup

#if STEP_OUTMODE == GPIO_MAP

    static const uint32_t c_step_outmap[8] = {
        0,
        X_STEP_BIT,
        Y_STEP_BIT,
        X_STEP_BIT|Y_STEP_BIT,
        Z_STEP_BIT,
        X_STEP_BIT|Z_STEP_BIT,
        Y_STEP_BIT|Z_STEP_BIT,
        X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT
    };

    static uint32_t step_outmap[8];

#endif

#if DIRECTION_OUTMODE == GPIO_MAP

    static const uint32_t c_dir_outmap[8] = {
        0,
        X_DIRECTION_BIT,
        Y_DIRECTION_BIT,
        X_DIRECTION_BIT|Y_DIRECTION_BIT,
        Z_DIRECTION_BIT,
        X_DIRECTION_BIT|Z_DIRECTION_BIT,
        Y_DIRECTION_BIT|Z_DIRECTION_BIT,
        X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT
    };

    static uint32_t dir_outmap[8];

#endif

static uint_fast16_t spindle_set_speed (uint_fast16_t pwm_value);

// Interrupt handler prototypes

uint32_t cpt;

static void driver_delay (uint32_t ms, void (*callback)(void))
{
    if((delay.ms = ms) > 0) {
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
    BITBAND_GPIO(X_DISABLE_PORT->PIN, X_DISABLE_PIN) = enable.x;
    BITBAND_GPIO(Y_DISABLE_PORT->PIN, Y_DISABLE_PIN) = enable.y;
    BITBAND_GPIO(Z_DISABLE_PORT->PIN, Z_DISABLE_PIN) = enable.z;
  #ifdef A_AXIS
    BITBAND_GPIO(A_DISABLE_PORT->PIN, A_DISABLE_PIN) = enable.a;
  #endif
  #ifdef B_AXIS
    BITBAND_GPIO(B_DISABLE_PORT->PIN, B_DISABLE_PIN) = enable.b;
  #endif
#elif defined(DISABLE_MASK)
    DISABLE_PORT->PIN = (DISABLE_PORT->PIN & ~DISABLE_MASK) | enable.mask;
#else
    BITBAND_GPIO(STEPPERS_DISABLE_PORT->PIN, STEPPERS_DISABLE_PIN) = enable.x;
#endif

}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    stepperEnable((axes_signals_t){AXES_BITMASK});

    STEPPER_TIMER->TCR = 0b10;          // reset
    STEPPER_TIMER->MR[0] = 0xFFFF;  	// Set a long initial delay,
    STEPPER_TIMER->TCR = 0b01;          // start stepper ISR timer in up mode
//    hal.stepper_interrupt_callback();   // and start the show
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
inline static void stepperSetStepOutputs (axes_signals_t step_outbits)
{
#if STEP_OUTMODE == GPIO_BITBAND
    step_outbits.value ^= settings.steppers.step_invert.value;
    BITBAND_GPIO(X_STEP_PORT->PIN, X_STEP_PIN) = step_outbits.x;
    BITBAND_GPIO(Y_STEP_PORT->PIN, Y_STEP_PIN) = step_outbits.y;
    BITBAND_GPIO(Z_STEP_PORT->PIN, Z_STEP_PIN) = step_outbits.z;
  #ifdef A_AXIS
    BITBAND_GPIO(A_STEP_PORT->PIN, A_STEP_PIN) = step_outbits.a;
  #endif
  #ifdef B_AXIS
    BITBAND_GPIO(B_STEP_PORT->PIN, B_STEP_PIN) = step_outbits.b;
  #endif
#elif STEP_OUTMODE == GPIO_MAP
    STEP_PORT->PIN = (STEP_PORT->PIN & ~STEP_MASK) | step_outmap[step_outbits.value];
#else
    STEP_PORT->PIN = (STEP_PORT->PIN & ~STEP_MASK) | ((step_outbits.value << STEP_OUTMODE) ^ settings.steppers.step_invert.value);
#endif
}

// Set stepper direction output pins
// NOTE: see note for stepperSetStepOutputs()
inline static void stepperSetDirOutputs (axes_signals_t dir_outbits)
{
#if DIRECTION_OUTMODE == GPIO_BITBAND
    dir_outbits.value ^= settings.steppers.dir_invert.value;
    BITBAND_GPIO(X_DIRECTION_PORT->PIN, X_DIRECTION_PIN) = dir_outbits.x;
    BITBAND_GPIO(Y_DIRECTION_PORT->PIN, Y_DIRECTION_PIN) = dir_outbits.y;
    BITBAND_GPIO(Z_DIRECTION_PORT->PIN, Z_DIRECTION_PIN) = dir_outbits.z;
  #ifdef A_AXIS
    BITBAND_GPIO(A_DIRECTION_PORT->PIN, A_DIRECTION_PIN) = dir_outbits.a;
  #endif
  #ifdef B_AXIS
    BITBAND_GPIO(B_DIRECTION_PORT->PIN, B_DIRECTION_PIN) = dir_outbits.b;
  #endif
#elif DIRECTION_OUTMODE == GPIO_MAP
    DIRECTION_PORT->PIN = (DIRECTION_PORT->PIN & ~DIRECTION_MASK) | dir_outmap[dir_outbits.value];
#else
    DIRECTION_PORT->PIN = (DIRECTION_PORT->PIN & ~DIRECTION_MASK) | ((dir_outbits.value << DIRECTION_OUTMODE) ^ settings.steppers.dir_invert.value);
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
		PULSE_TIMER->TCR = 1;
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
inline static axes_signals_t limitsGetState()
{
    axes_signals_t signals;

#if LIMIT_INMODE == LIMIT_SHIFT
    signals.value = (uint32_t)(LIMIT_PORT->PIN & LIMIT_MASK) >> LIMIT_SHIFT;
#elif LIMIT_INMODE == GPIO_BITBAND
    signals.x = BITBAND_GPIO(X_LIMIT_PORT->PIN, X_LIMIT_PIN)
  #ifdef X_LIMIT_PORT_MAX
    | BITBAND_GPIO(X_LIMIT_PORT_MAX->PIN, X_LIMIT_PIN_MAX);
  #else
    ;
  #endif
    signals.y = BITBAND_GPIO(Y_LIMIT_PORT->PIN, Y_LIMIT_PIN)
  #ifdef X_LIMIT_PORT_MAX
    | BITBAND_GPIO(Y_LIMIT_PORT_MAX->PIN, Y_LIMIT_PIN_MAX);
  #else
    ;
  #endif
    signals.z = BITBAND_GPIO(Z_LIMIT_PORT->PIN, Z_LIMIT_PIN)
  #ifdef X_LIMIT_PORT_MAX
    | BITBAND_GPIO(Y_LIMIT_PORT_MAX->PIN, Y_LIMIT_PIN_MAX);
  #else
    ;
#endif
#else
    uint32_t bits = LIMIT_PORT->PIN;
    signals.x = (bits & X_LIMIT_BIT) != 0
#ifdef X_LIMIT_PORT_MAX
  | (bits & X_LIMIT_BIT_MAX) != 0;
#else
  ;
#endif
    signals.y = (bits & Y_LIMIT_BIT) != 0;
    signals.z = (bits & Z_LIMIT_BIT) != 0;
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
    signals.reset = BITBAND_GPIO(RESET_PORT->PIN, RESET_PIN);
    signals.feed_hold = BITBAND_GPIO(FEED_HOLD_PORT->PIN, FEED_HOLD_PIN);
    signals.cycle_start = BITBAND_GPIO(CYCLE_START_PORT->PIN, CYCLE_START_PIN);
  #ifdef SAFETY_DOOR_PORT
    signals.safety_door_ajar = BITBAND_GPIO(SAFETY_DOOR_PORT->PIN, SAFETY_DOOR_PIN);
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
static void probeConfigureInvertMask(bool is_probe_away)
{
  probe_invert_mask = settings.flags.invert_probe_pin ? 0 : PROBE_BIT;

  if (is_probe_away)
      probe_invert_mask ^= PROBE_BIT;
}

// Returns the probe pin state. Triggered = true.
bool probeGetState (void)
{
    return ((PROBE_PORT->PIN & PROBE_BIT) ^ probe_invert_mask) != 0;
}

// Static spindle (off, on cw & on ccw)

inline static void spindle_off (void)
{
    BITBAND_GPIO(SPINDLE_ENABLE_PORT->PIN, SPINDLE_ENABLE_PIN) = settings.spindle.invert.on;
}

inline static void spindle_on (void)
{
    BITBAND_GPIO(SPINDLE_ENABLE_PORT->PIN, SPINDLE_ENABLE_PIN) = !settings.spindle.invert.on;
}

inline static void spindle_dir (bool ccw)
{
    if(hal.driver_cap.spindle_dir)
    	BITBAND_GPIO(SPINDLE_DIRECTION_PORT->PIN, SPINDLE_DIRECTION_PIN) = ccw ^ settings.spindle.invert.ccw;
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
    if (pwm_value == spindle_pwm.off_value) {
        pwmEnabled = false;
        if(settings.spindle.disable_with_zero_speed)
            spindle_off();
        if(settings.spindle.pwm_off_value == 0.0f) {
        	pwm_set_width(&SPINDLE_PWM_CHANNEL, 0);
//          pwm_disable(&SPINDLE_PWM_CHANNEL); // Set PWM output low
        } else {
            pwm_set_width(&SPINDLE_PWM_CHANNEL, spindle_pwm.off_value);
            pwm_enable(&SPINDLE_PWM_CHANNEL);
        }
    } else {
        if(!pwmEnabled)
            spindle_on();
        pwmEnabled = true;
        pwm_set_width(&SPINDLE_PWM_CHANNEL, pwm_value);
        pwm_enable(&SPINDLE_PWM_CHANNEL);
    }

    return pwm_value;
}

static void spindleUpdateRPM (float rpm)
{
    spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
}

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
    spindle_state_t state = {0};

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
    BITBAND_GPIO(COOLANT_FLOOD_PORT->PIN, COOLANT_FLOOD_PIN) = mode.flood;
#ifdef COOLANT_MIST_PORT
    BITBAND_GPIO(COOLANT_MIST_PORT->PIN, COOLANT_MIST_PIN) = mode.mist;
#endif
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

    state.flood = (COOLANT_FLOOD_PORT->PIN & COOLANT_FLOOD_BIT) != 0;
#ifdef COOLANT_MIST_PORT
    state.mist  = (COOLANT_MIST_PORT->PIN & COOLANT_MIST_BIT) != 0;
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

void gpio_pinmode (LPC_GPIO_T *port, uint8_t pin, uint8_t pinmode)
{
	uint32_t pn = ((uint32_t)port - LPC_GPIO0_BASE) / sizeof(LPC_GPIO_T);

	pn = LPC_IOCON_BASE + 0x40 + pn * 8;
	if(pin > 15) {
		pn += 4;
		pin &= 0xF;
	}
	pin <<= 1;
	port = (LPC_GPIO_T *)pn;
//	BITBAND_PERI(port, pin++) = pinmode & 0x1;
//    BITBAND_PERI(port, pin)   = (pinmode >> 1) & 0x1;

    *(uint32_t*)port = pinmode << pin;

    pn = *(uint32_t*)port;
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
        step_outmap[i] = c_step_outmap[i] ^ c_step_outmap[settings->steppers.step_invert.value];
#endif

#if DIRECTION_OUTMODE == GPIO_MAP
    for(i = 0; i < sizeof(dir_outmap) / sizeof(uint32_t); i++)
        dir_outmap[i] = c_dir_outmap[i] ^ c_dir_outmap[settings->steppers.dir_invert.value];
#endif

    if(IOInitDone) {

        stepperEnable(settings->steppers.deenergize);

        if(hal.driver_cap.variable_spindle) {
            pwm_init(&SPINDLE_PWM_CHANNEL, SPINDLE_PWM_USE_PRIMARY_PIN, SPINDLE_PWM_USE_SECONDARY_PIN, spindle_pwm.period, 0);
            hal.spindle_set_state = spindleSetStateVariable;
        } else
        	hal.spindle_set_state = spindleSetState;

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds) {
            hal.stepper_pulse_start = &stepperPulseStartDelayed;
            PULSE_TIMER->MCR |= MR0I;                   // Enable CCR1 interrupt
        } else {
            hal.stepper_pulse_start = &stepperPulseStart;
            PULSE_TIMER->MCR &= ~MR0I;                  // Disable CCR1 interrupt
        }
        PULSE_TIMER->MCR |= (MR1I|MR1S|MR1R); // Enable interrupt for finish step pulse, reset and stop timer
        PULSE_TIMER->MR[0] = settings->steppers.pulse_delay_microseconds;
        PULSE_TIMER->MR[1] = settings->steppers.pulse_microseconds + settings->steppers.pulse_delay_microseconds;

        NVIC_DisableIRQ(EINT3_IRQn);  // Disable GPIO interrupt

        /*************************
         *  Control pins config  *
         *************************/

        control_signals_t control_ire;

        control_ire.mask = ~(settings->control_disable_pullup.mask ^ settings->control_invert.mask);

        BITBAND_PERI(CYCLE_START_INTENR, CYCLE_START_PIN) = 0;
        BITBAND_PERI(CYCLE_START_INTENF, CYCLE_START_PIN) = 0;
        BITBAND_PERI(FEED_HOLD_INTENR, FEED_HOLD_PIN) = 0;
        BITBAND_PERI(FEED_HOLD_INTENF, FEED_HOLD_PIN) = 0;
        BITBAND_PERI(RESET_INTENR, RESET_PIN) = 0;
        BITBAND_PERI(RESET_INTENF, RESET_PIN) = 0;

        gpio_pinmode(CYCLE_START_PORT, CYCLE_START_PIN, settings->control_disable_pullup.cycle_start ? PINMODE_PULLDOWN : PINMODE_PULLUP);
        gpio_pinmode(FEED_HOLD_PORT, FEED_HOLD_PIN, settings->control_disable_pullup.feed_hold ? PINMODE_PULLDOWN : PINMODE_PULLUP);
        gpio_pinmode(RESET_PORT, RESET_PIN, settings->control_disable_pullup.reset ? PINMODE_PULLDOWN : PINMODE_PULLUP);

        if(control_ire.cycle_start)
        	BITBAND_PERI(CYCLE_START_INTENR, CYCLE_START_PIN) = control_ire.cycle_start;
        else
        	BITBAND_PERI(CYCLE_START_INTENF, CYCLE_START_PIN) = !control_ire.cycle_start;

        if(control_ire.feed_hold)
        	BITBAND_PERI(FEED_HOLD_INTENR, FEED_HOLD_PIN) = control_ire.feed_hold;
        else
        	BITBAND_PERI(FEED_HOLD_INTENF, FEED_HOLD_PIN) = !control_ire.feed_hold;

        if(control_ire.reset)
        	BITBAND_PERI(RESET_INTENR, RESET_PIN) = control_ire.reset;
        else
        	BITBAND_PERI(RESET_INTENF, RESET_PIN) = !control_ire.reset;

        BITBAND_PERI(CYCLE_START_INTCLR, RESET_PIN) = 0;
        BITBAND_PERI(FEED_HOLD_INTCLR, FEED_HOLD_PIN) = 0;
        BITBAND_PERI(RESET_INTCLR, RESET_PIN) = 0;

#ifdef SAFETY_DOOR_PORT
        BITBAND_PERI(CONTROL_INTENR, SAFETY_DOOR_PIN) = 0;
        BITBAND_PERI(CONTROL_INTENF, SAFETY_DOOR_PIN) = 0;
        gpio_pinmode(SAFETY_DOOR_PORT, SAFETY_DOOR_PIN, settings->control_disable_pullup.safety_door_ajar ? PINMODE_PULLDOWN : PINMODE_PULLUP);
        BITBAND_PERI(RESET_INTCLR, RESET_PIN) = 0;

        if(control_ire.safety_door_ajar)
        	BITBAND_PERI(CONTROL_INTENR, SAFETY_DOOR_PIN) = control_ire.safety_door_ajar;
        else
        	BITBAND_PERI(CONTROL_INTENF, SAFETY_DOOR_PIN) = !control_ire.safety_door_ajar;
#endif

        /***********************
         *  Limit pins config  *
         ***********************/

        axes_signals_t limit_ire;

        limit_ire.mask = ~(settings->limits.disable_pullup.mask ^ settings->limits.invert.mask);

        gpio_pinmode(X_LIMIT_PORT, X_LIMIT_PIN, settings->limits.disable_pullup.x ? PINMODE_PULLDOWN : PINMODE_PULLUP);
#ifdef X_LIMIT_PORT_MAX
        gpio_pinmode(X_LIMIT_PORT_MAX, X_LIMIT_PIN_MAX, settings->limits.disable_pullup.x ? PINMODE_PULLDOWN : PINMODE_PULLUP);
#endif
        gpio_pinmode(Y_LIMIT_PORT, Y_LIMIT_PIN, settings->limits.disable_pullup.y ? PINMODE_PULLDOWN : PINMODE_PULLUP);
#ifdef Y_LIMIT_PORT_MAX
        gpio_pinmode(Y_LIMIT_PORT_MAX, Y_LIMIT_PIN_MAX, settings->limits.disable_pullup.x ? PINMODE_PULLDOWN : PINMODE_PULLUP);
#endif
        gpio_pinmode(Z_LIMIT_PORT, Z_LIMIT_PIN, settings->limits.disable_pullup.z ? PINMODE_PULLDOWN : PINMODE_PULLUP);
#ifdef Z_LIMIT_PORT_MAX
        gpio_pinmode(Z_LIMIT_PORT_MAX, Z_LIMIT_PIN_MAX, settings->limits.disable_pullup.x ? PINMODE_PULLDOWN : PINMODE_PULLUP);
#endif

        // ---

        NVIC_EnableIRQ(EINT3_IRQn);  // Enable GPIO interrupt

		/**********************
	 	 *  Probe pin config  *
		 **********************/

        gpio_pinmode(PROBE_PORT, PROBE_PIN, hal.driver_cap.probe_pull_up ? PINMODE_PULLUP : PINMODE_PULLDOWN);

    }
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
 // Stepper init

    BITBAND_GPIO(X_STEP_PORT->DIR, X_STEP_PIN) = 1;
    BITBAND_GPIO(Y_STEP_PORT->DIR, Y_STEP_PIN) = 1;
    BITBAND_GPIO(Z_STEP_PORT->DIR, Z_STEP_PIN) = 1;
#ifdef A_AXIS
    BITBAND_GPIO(A_STEP_PORT->DIR, B_STEP_PIN) = 1;
#endif
#ifdef B_AXIS
    BITBAND_GPIO(B_STEP_PORT->DIR, B_STEP_PIN) = 1;
#endif

    BITBAND_GPIO(X_DIRECTION_PORT->DIR, X_DIRECTION_PIN) = 1;
    BITBAND_GPIO(Y_DIRECTION_PORT->DIR, Y_DIRECTION_PIN) = 1;
    BITBAND_GPIO(Z_DIRECTION_PORT->DIR, Z_DIRECTION_PIN) = 1;
#ifdef A_AXIS
    BITBAND_GPIO(A_DIRECTION_PORT->DIR, A_DIRECTION_PIN) = 1;
#endif
#ifdef B_AXIS
    BITBAND_GPIO(B_DIRECTION_PORT->DIR, B_DIRECTION_PIN) = 1;
#endif

#if DISABLE_OUTMODE == GPIO_BITBAND
    BITBAND_GPIO(X_DISABLE_PORT->DIR, X_DISABLE_PIN) = 1;
    BITBAND_GPIO(Y_DISABLE_PORT->DIR, Y_DISABLE_PIN) = 1;
    BITBAND_GPIO(Z_DISABLE_PORT->DIR, Z_DISABLE_PIN) = 1;
  #ifdef A_AXIS
    BITBAND_GPIO(A_DISABLE_PORT->DIR, A_DISABLE_PIN) = 1;
  #endif
  #ifdef B_AXIS
    BITBAND_GPIO(B_DISABLE_PORT->DIR, B_DISABLE_PIN) = 1;
  #endif
#elif defined(DISABLE_MASK)
    DISABLE_PORT->DIR |= DISABLE_MASK;
#else
    BITBAND_GPIO(STEPPERS_DISABLE_PORT->DIR, STEPPERS_DISABLE_PIN) = 1;
#endif

    STEPPER_TIMER->TCR = 0;            // disable
    STEPPER_TIMER->CTCR = 0;           // timer mode
    STEPPER_TIMER->PR = 0;             // no prescale
    STEPPER_TIMER->MCR = MR0I|MR0R;    // MR0: !stop, reset, interrupt
    STEPPER_TIMER->CCR = 0;            // no capture
    STEPPER_TIMER->EMR = 0;            // no external match

    PULSE_TIMER->TCR = 2;
    PULSE_TIMER->CTCR = 0;
    PULSE_TIMER->PR = SystemCoreClock / 1000000 / Chip_Clock_GetPCLKDiv(PULSE_TIMER_PCLK); // to 1 us
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
        DEBOUNCE_TIMER->TCR = 2;
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

    IOInitDone = settings->version == 15;

    settings_changed(settings);

    hal.spindle_set_state((spindle_state_t){0}, 0.0f);
    hal.coolant_set_state((coolant_state_t){0});
    stepperSetDirOutputs((axes_signals_t){0});

    return IOInitDone;
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done

bool driver_init (void) {

    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

	SystemCoreClockUpdate();

	Chip_SetupXtalClocking(); // Sets 96 MHz clock
	Chip_SYSCTL_SetFLASHAccess(FLASHTIM_100MHZ_CPU);

	SystemCoreClockUpdate();

	Chip_GPIO_Init(LPC_GPIO);
	Chip_IOCON_Init(LPC_IOCON);

	// Enable and set SysTick IRQ to lowest priority
	SysTick->LOAD = (SystemCoreClock / 1000) - 1;
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk;
	NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

#if USB_ENABLE
	usbInit();
#else
	serialInit();
#endif

#if EEPROM_ENABLE
	eepromInit();
#endif

    hal.info = "LCP1769";
    hal.driver_setup = driver_setup;
    hal.f_step_timer = SystemCoreClock / Chip_Clock_GetPCLKDiv(STEPPER_TIMER_PCLK);
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = &driver_delay;
    hal.settings_changed = settings_changed;

#if SDCARD_ENABLE
    hal.driver_reset = sdcard_reset;
#endif

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

    hal.system_control_get_state = systemGetState;

#if USB_ENABLE
    hal.stream.read = usbGetC;
    hal.stream.write = usbWriteS;
    hal.stream.write_all = usbWriteS;
    hal.stream.get_rx_buffer_available = usbRxFree;
    hal.stream.reset_read_buffer = usbRxFlush;
    hal.stream.cancel_read_buffer = usbRxCancel;
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

    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;

#ifdef HAS_KEYPAD
    hal.execute_realtime = process_keypress;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

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

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 6;
}

/* interrupt handlers */

// Main stepper driver
void STEPPER_IRQHandler (void)
{
    STEPPER_TIMER->IR = STEPPER_TIMER->IR;
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

// This interrupt is enabled when Grbl sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
void STEPPULSE_IRQHandler (void)
{
	uint32_t ifg = PULSE_TIMER->IR;
    if(ifg & MR0IFG)
        stepperSetStepOutputs(next_step_outbits); // Begin step pulse
    else if(ifg & MR1IFG)
        stepperSetStepOutputs((axes_signals_t){0}); // End step pulse
    PULSE_TIMER->IR = ifg;
}

void DEBOUNCE_IRQHandler (void)
{
	DEBOUNCE_TIMER->IR = MR0IFG;
	DEBOUNCE_TIMER->TCR = 0;

    axes_signals_t state = (axes_signals_t)limitsGetState();

    if(state.value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
        hal.limit_interrupt_callback(state);
}

void GPIO_IRQHandler (void)
{
    uint32_t istat = LPC_GPIOINT->STATUS, iflags;

    if(istat & P0Int) {
    	iflags = LPC_GPIOINT->IO0.STATR | LPC_GPIOINT->IO0.STATF;
    	LPC_GPIOINT->IO0.CLR = iflags;
#ifdef LIMIT_INTST0
		if(iflags & LIMIT_INTST2) {
			if(hal.driver_cap.software_debounce)
				DEBOUNCE_TIMER->TCR = 1;
			else
				hal.limit_interrupt_callback(limitsGetState());
		}
#endif
#ifdef CONTROL_INTST0
    	if(iflags & CONTROL_INTST0)
    		hal.control_interrupt_callback(systemGetState());
#endif
    }

    if(istat & P2Int) {
    	iflags = LPC_GPIOINT->IO2.STATR | LPC_GPIOINT->IO2.STATF;
    	LPC_GPIOINT->IO2.CLR = iflags;
#ifdef LIMIT_INTST2
		if(iflags & LIMIT_INTST2) {
			if(hal.driver_cap.software_debounce)
				DEBOUNCE_TIMER->TCR = 1;
			else
				hal.limit_interrupt_callback(limitsGetState());
		}
#endif
#ifdef CONTROL_INTST2
		if(iflags & CONTROL_INTST2)
			hal.control_interrupt_callback(systemGetState());
#endif
    }
}

// Interrupt handler for 1 ms interval timer
void SysTick_Handler (void)
{
#if SDCARD_ENABLE
	static uint32_t fatfs_ticks = 10;
	if(!(--fatfs_ticks)) {
		disk_timerproc();
		fatfs_ticks = 10;
	}
    if(delay.ms && !(--delay.ms)) {
        if(delay.callback) {
            delay.callback();
            delay.callback = NULL;
        }
    }
#else
    if(!(--delay.ms)) {
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        if(delay.callback) {
            delay.callback();
            delay.callback = NULL;
        }
    }
#endif
}
