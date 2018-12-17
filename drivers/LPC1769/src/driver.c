/*

  driver.c - driver code for NXP LPC176x ARM processors

  Part of Grbl

  Copyright (c) 2018 Terje Io

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

#include <LPC17xx.h>
#include <stdint.h>

#include "GRBL\grbl.h"

#include "driver.h"
#include "flash.h"
#include "serial.h"
#include "grbl-lpc/pwm_driver.h"

#if SDCARD_ENABLE
#include "sdcard.h"
#endif

static volatile uint32_t ms_count = 1; // NOTE: initial value 1 is for "resetting" systick timer
static bool pwmEnabled = false, IOInitDone = false;
// Inverts the probe pin state depending on user settings and probing cycle mode.
static uint8_t probe_invert_mask;
static axes_signals_t next_step_outbits;
static spindle_pwm_t spindle_pwm;
static void (*delayCallback)(void) = 0;

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

static uint_fast16_t spindleSetSpeed (uint_fast16_t pwm_value);

// Interrupt handler prototypes

uint32_t cpt;

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if((ms_count = ms) > 0) {
        SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
        if(!(delayCallback = callback))
            while(ms_count);
    } else if(callback)
        callback();
}

// Enable/disable stepper motors
static void stepperEnable (axes_signals_t enable)
{
    enable.mask ^= settings.steppers.enable_invert.mask;
    BITBAND_PERI(STEPPERS_DISABLE_PORT->FIOPIN, STEPPERS_DISABLE_PIN) = enable.x;
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    stepperEnable((axes_signals_t){AXES_BITMASK});

    STEPPER_TIMER->TCR = 0b10;          // reset
    STEPPER_TIMER->MR0 = 0xFFFF;        // Set a long initial delay,
    STEPPER_TIMER->TCR = 0b01;          // start stepper ISR timer in up mode
    hal.stepper_interrupt_callback();   // and start the show
}

// Disables stepper driver interrupts
static void stepperGoIdle (void) {
    STEPPER_TIMER->TCR = 0;   // Stop stepper timer
}

// Sets up stepper driver interrupt timeout, both "Normal" and AMASS version
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    STEPPER_TIMER->MR0 = cycles_per_tick;
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
    step_outbits.value ^= settings.step_invert.value;
    BITBAND_PERI(STEP_PORT->FIOPIN, X_STEP_PIN) = step_outbits.x;
    BITBAND_PERI(STEP_PORT->FIOPIN, Y_STEP_PIN) = step_outbits.y;
    BITBAND_PERI(STEP_PORT->FIOPIN, Z_STEP_PIN) = step_outbits.z;
#elif STEP_OUTMODE == GPIO_MAP
    STEP_PORT->FIOPIN = (STEP_PORT->FIOPIN & ~STEP_MASK) | step_outmap[step_outbits.value];
#else
    STEP_PORT->FIOPIN = (STEP_PORT->FIOPIN & ~STEP_MASK) | ((step_outbits.value << STEP_OUTMODE) ^ settings.step_invert.value);
#endif
}

// Set stepper direction output pins
// NOTE: see note for stepperSetStepOutputs()
inline static void stepperSetDirOutputs (axes_signals_t dir_outbits)
{
#if DIRECTION_OUTMODE == GPIO_BITBAND
    dir_outbits.value ^= settings.steppers.dir_invert.value;
    BITBAND_PERI(DIRECTION_PORT->FIOPIN, X_DIRECTION_PIN) = dir_outbits.x;
    BITBAND_PERI(DIRECTION_PORT->FIOPIN, Y_DIRECTION_PIN) = dir_outbits.y;
    BITBAND_PERI(DIRECTION_PORT->FIOPIN, Z_DIRECTION_PIN) = dir_outbits.z;
#elif DIRECTION_OUTMODE == GPIO_MAP
    DIRECTION_PORT->FIOPIN = (DIRECTION_PORT->FIOPIN & ~DIRECTION_MASK) | dir_outmap[dir_outbits.value];
#else
    DIRECTION_PORT->FIOPIN = (DIRECTION_PORT->FIOPIN & ~DIRECTION_MASK) | ((dir_outbits.value << DIRECTION_OUTMODE) ^ settings.dir_invert.value);
#endif
}

// Sets stepper direction and pulse pins and starts a step pulse
// When delayed pulse the step register is written in the step delay interrupt handler
static void stepperPulseStart (stepper_t *stepper)
{
    static uint_fast16_t current_pwm = 0;

    if(stepper->new_block) {
        stepper->new_block = false;
        stepperSetDirOutputs(stepper->dir_outbits);
    }

    if(stepper->step_outbits.value) {

    	if(stepper->spindle_pwm != current_pwm)
			current_pwm = spindleSetSpeed(stepper->spindle_pwm);

		stepperSetStepOutputs(stepper->step_outbits);
		PULSE_TIMER->TCR = 1;
    }
}

// Sets stepper direction and pulse pins and starts a step pulse with and initial delay
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    static uint_fast16_t current_pwm = 0;

    if(stepper->new_block) {
        stepper->new_block = false;
        stepperSetDirOutputs(stepper->dir_outbits);
    }

    if(stepper->step_outbits.value) {

		if(stepper->spindle_pwm != current_pwm)
			current_pwm = spindleSetSpeed(stepper->spindle_pwm);

		next_step_outbits = stepper->step_outbits; // Store out_bits
		PULSE_TIMER->TCR = 1;
    }
}

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing)
{
    on = on && settings.limits.flags.hard_enabled;
    BITBAND_PERI(LIMIT_INTCLR, X_LIMIT_PIN) = 0;
    BITBAND_PERI(LIMIT_INTCLR, Y_LIMIT_PIN) = 0;
    BITBAND_PERI(LIMIT_INTCLR, Z_LIMIT_PIN) = 0;
    BITBAND_PERI(LIMIT_INTENR, X_LIMIT_PIN) = on;
    BITBAND_PERI(LIMIT_INTENR, Y_LIMIT_PIN) = on;
    BITBAND_PERI(LIMIT_INTENR, Z_LIMIT_PIN) = on;
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static axes_signals_t limitsGetState()
{
    axes_signals_t signals;

#if LIMIT_INMODE == LIMIT_SHIFT
    signals.value = (uint8_t)(LIMIT_PORT->FIOPIN & LIMIT_MASK) >> LIMIT_SHIFT;
#elif LIMIT_INMODE == GPIO_BITBAND
    signals.x = BITBAND_PERI(LIMIT_PORT->FIOPIN, X_LIMIT_PIN);
    signals.y = BITBAND_PERI(LIMIT_PORT->FIOPIN, Y_LIMIT_PIN);
    signals.z = BITBAND_PERI(LIMIT_PORT->FIOPIN, Z_LIMIT_PIN);
#else
    uint8_t bits = LIMIT_PORT->FIOPIN;
    signals.x = (bits & X_LIMIT_PIN) != 0;
    signals.y = (bits & Y_LIMIT_PIN) != 0;
    signals.z = (bits & Z_LIMIT_PIN) != 0;
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
#if LIMIT_INMODE == GPIO_BITBAND
    signals.reset = BITBAND_PERI(CONTROL_PORT->FIOPIN, RESET_PIN);
    signals.safety_door_ajar = BITBAND_PERI(CONTROL_PORT->FIOPIN, SAFETY_DOOR_PIN);
    signals.feed_hold = BITBAND_PERI(CONTROL_PORT->FIOPIN, FEED_HOLD_PIN);
    signals.cycle_start = BITBAND_PERI(CONTROL_PORT->FIOPIN, CYCLE_START_PIN);
#else
    uint8_t bits = CONTROL_PORT->FIOPIN;
    signals.reset = bits & RESET_BIT != 0;
    signals.safety_door_ajar = bits & SAFETY_DOOR_BIT != 0;
    signals.feed_hold = bits & FEED_HOLD_BIT != 0;
    signals.cycle_start = bits & CYCLE_START_BIT != 0;
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
  probe_invert_mask = settings.flags.invert_probe_pin ? 0 : PROBE_BIT;

  if (is_probe_away)
      probe_invert_mask ^= PROBE_BIT;
}

// Returns the probe pin state. Triggered = true.
bool probeGetState (void)
{
    return ((PROBE_PORT->FIOPIN & PROBE_BIT) ^ probe_invert_mask) != 0;
}

// Static spindle (off, on cw & on ccw)

inline static void spindleOff (void)
{
    BITBAND_PERI(SPINDLE_ENABLE_PORT->FIOPIN, SPINDLE_ENABLE_PIN) = settings.spindle.invert.on;
}

inline static void spindleOn (void)
{
    BITBAND_PERI(SPINDLE_ENABLE_PORT->FIOPIN, SPINDLE_ENABLE_PIN) = !settings.spindle.invert.on;
}

inline static void spindleDir (bool ccw)
{
    if(hal.driver_cap.spindle_dir)
    	BITBAND_PERI(SPINDLE_DIRECTION_PORT->FIOPIN, SPINDLE_DIRECTION_PIN) = ccw ^ settings.spindle.invert.ccw;
}

// Start or stop spindle
static void spindleSetState (spindle_state_t state, float rpm, uint8_t speed_ovr)
{
    if (!state.on)
        spindleOff();
    else {
        spindleDir(state.ccw);
        spindleOn();
    }
}

// Variable spindle control functions

// Spindle speed to PWM conversion. Keep routine small and efficient.
static uint_fast16_t spindleComputePWMValue (float rpm, uint8_t speed_ovr)
{
	uint_fast16_t pwm_value;

    rpm *= (0.010f * speed_ovr); // Scale by spindle speed override value.
    // Calculate PWM register value based on rpm max/min settings and programmed rpm.
    if ((settings.spindle.rpm_min >= settings.spindle.rpm_max) || (rpm >= settings.spindle.rpm_max)) {
        // No PWM range possible. Set simple on/off spindle control pin state.
        sys.spindle_rpm = settings.spindle.rpm_max;
        pwm_value = spindle_pwm.max_value - 1;
    } else if (rpm <= settings.spindle.rpm_min) {
        if (rpm == 0.0f) { // S0 disables spindle
            sys.spindle_rpm = 0.0f;
            pwm_value = spindle_pwm.off_value;
        } else { // Set minimum PWM output
            sys.spindle_rpm = settings.spindle.rpm_min;
            pwm_value = spindle_pwm.min_value;
        }
    } else {
        // Compute intermediate PWM value with linear spindle speed model.
        // NOTE: A nonlinear model could be installed here, if required, but keep it VERY light-weight.
        sys.spindle_rpm = rpm;
        pwm_value = (uint_fast16_t)floorf((rpm - settings.spindle.rpm_min) * spindle_pwm.pwm_gradient) + spindle_pwm.min_value;
        if(pwm_value >= spindle_pwm.max_value)
            pwm_value = spindle_pwm.max_value - 1;
    }

    return pwm_value;
}

// Sets spindle speed
static uint_fast16_t spindleSetSpeed (uint_fast16_t pwm_value)
{
    if (pwm_value == hal.spindle_pwm_off) {
        pwmEnabled = false;
        if(settings.spindle.disable_with_zero_speed)
            spindleOff();
        pwm_disable(&SPINDLE_PWM_CHANNEL); // Set PWM output low
    } else {
        if(!pwmEnabled)
            spindleOn();
        pwmEnabled = true;
        pwm_set_period(pwm_value);
        pwm_enable(&SPINDLE_PWM_CHANNEL);
    }

    return pwm_value;
}

// Start or stop spindle
static void spindleSetStateVariable (spindle_state_t state, float rpm, uint8_t speed_ovr)
{
    if (!state.on || rpm == 0.0f) {
        spindleSetSpeed(hal.spindle_pwm_off);
        spindleOff();
    } else {

        if(!hal.driver_cap.spindle_dir)
            spindleDir(state.ccw);

        spindleSetSpeed(spindleComputePWMValue(rpm, speed_ovr));
    }
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {0};

    state.on = pwmEnabled || (SPINDLE_ENABLE_PORT->FIOPIN & SPINDLE_ENABLE_BIT) != 0;
    state.ccw = hal.driver_cap.spindle_dir && (SPINDLE_DIRECTION_PORT->FIOPIN & SPINDLE_DIRECTION_BIT) != 0;
    state.value ^= settings.spindle.invert.mask;

    return state;
}

// end spindle code

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;
    BITBAND_PERI(COOLANT_FLOOD_PORT->FIOPIN, COOLANT_FLOOD_PIN) = mode.flood;
    BITBAND_PERI(COOLANT_MIST_PORT->FIOPIN, COOLANT_MIST_PIN) = mode.mist;
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

    state.flood = (COOLANT_FLOOD_PORT->FIOPIN & COOLANT_FLOOD_BIT) != 0;
    state.mist  = (COOLANT_MIST_PORT->FIOPIN & COOLANT_MIST_BIT) != 0;
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

void gpio_pinmode (LPC_GPIO_TypeDef *port, uint8_t pin, uint8_t pinmode)
{
	port += LPC_PINCON_BASE - LPC_GPIO_BASE + 0x40;
	if(pin & 0x8) {
		port++;
		pin &= 0x7;
	}
	BITBAND_PERI(port, pin++) = pinmode & 0x1;
    BITBAND_PERI(port, pin)   = (pinmode >> 1) & 0x1;
}

// Configures peripherals when settings are initialized or changed
void settings_changed (settings_t *settings)
{
    spindle_pwm.period = (uint32_t)(12000000 / settings->spindle.pwm_freq);
    spindle_pwm.off_value = (uint32_t)(spindle_pwm.period * settings->spindle.pwm_off_value / 100.0f);
    spindle_pwm.min_value = (uint32_t)(spindle_pwm.period * settings->spindle.pwm_min_value / 100.0f);
    spindle_pwm.max_value = (uint32_t)(spindle_pwm.period * settings->spindle.pwm_max_value / 100.0f);
    spindle_pwm.pwm_gradient = (float)(spindle_pwm.max_value - spindle_pwm.min_value) / (settings->spindle.rpm_max - settings->spindle.rpm_min);

#if (STEP_OUTMODE == GPIO_MAP) || (DIRECTION_OUTMODE == GPIO_MAP)
    uint8_t i;
#endif

#if STEP_OUTMODE == GPIO_MAP
    for(i = 0; i < sizeof(step_outmap) / sizeof(uint32_t); i++)
        step_outmap[i] = c_step_outmap[i] ^ c_step_outmap[settings->steppers.step_invert.value];
#endif

#if DIRECTION_OUTMODE == GPIO_MAP
    for(i = 0; i < sizeof(dir_outmap) / sizeof(uint32_t); i++)
        dir_outmap[i] = c_dir_outmap[i] ^ c_dir_outmap[settings->dir_invert.value];
#endif

    hal.spindle_pwm_off = spindle_pwm.off_value;

    if(IOInitDone) {

        stepperEnable(settings->steppers.deenergize);

        if(hal.driver_cap.variable_spindle)
            pwm_init(&SPINDLE_PWM_CHANNEL, SPINDLE_PWM_USE_PRIMARY_PIN, SPINDLE_PWM_USE_SECONDARY_PIN, spindle_pwm.period, 0);

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds) {
            hal.stepper_pulse_start = &stepperPulseStartDelayed;
            PULSE_TIMER->MCR |= MR0I;                   // Enable CCR1 interrupt
        } else {
            hal.stepper_pulse_start = &stepperPulseStart;
            PULSE_TIMER->MCR &= ~MR0I;                  // Disable CCR1 interrupt
        }
        PULSE_TIMER->MCR |= (MR1I|MR1S);
        PULSE_TIMER->MR0 = settings->steppers.pulse_microseconds + settings->steppers.pulse_delay_microseconds;
        PULSE_TIMER->MR1 = settings->steppers.pulse_delay_microseconds;

        /*************************
         *  Control pins config  *
         *************************/

        control_signals_t control_ire;

        control_ire.mask = ~(settings->control_disable_pullup.mask ^ settings->control_invert.mask);

        BITBAND_PERI(CONTROL_INTENR, CYCLE_START_PIN) = 0;
        BITBAND_PERI(CONTROL_INTENR, FEED_HOLD_PIN) = 0;
        BITBAND_PERI(CONTROL_INTENR, SAFETY_DOOR_PIN) = 0;
        BITBAND_PERI(CONTROL_INTENR, RESET_PIN) = 0;
        BITBAND_PERI(CONTROL_INTENF, CYCLE_START_PIN) = 0;
        BITBAND_PERI(CONTROL_INTENF, FEED_HOLD_PIN) = 0;
        BITBAND_PERI(CONTROL_INTENF, SAFETY_DOOR_PIN) = 0;
        BITBAND_PERI(CONTROL_INTENF, RESET_PIN) = 0;

        gpio_pinmode(CONTROL_PORT, CYCLE_START_PIN, settings->control_disable_pullup.cycle_start ? PINMODE_PULLDOWN : PINMODE_PULLUP);
        gpio_pinmode(CONTROL_PORT, FEED_HOLD_PIN, settings->control_disable_pullup.feed_hold ? PINMODE_PULLDOWN : PINMODE_PULLUP);
        gpio_pinmode(CONTROL_PORT, SAFETY_DOOR_PIN, settings->control_disable_pullup.safety_door_ajar ? PINMODE_PULLDOWN : PINMODE_PULLUP);
        gpio_pinmode(CONTROL_PORT, RESET_PIN, settings->control_disable_pullup.reset ? PINMODE_PULLDOWN : PINMODE_PULLUP);

        CONTROL_INTCLR = CONTROL_MASK;

        if(control_ire.cycle_start)
        	BITBAND_PERI(CONTROL_INTENR, CYCLE_START_PIN) = control_ire.cycle_start;
        else
        	BITBAND_PERI(CONTROL_INTENF, CYCLE_START_PIN) = !control_ire.cycle_start;

        if(control_ire.feed_hold)
        	BITBAND_PERI(CONTROL_INTENR, FEED_HOLD_PIN) = control_ire.feed_hold;
        else
        	BITBAND_PERI(CONTROL_INTENF, FEED_HOLD_PIN) = !control_ire.feed_hold;

        if(control_ire.safety_door_ajar)
        	BITBAND_PERI(CONTROL_INTENR, SAFETY_DOOR_PIN) = control_ire.safety_door_ajar;
        else
        	BITBAND_PERI(CONTROL_INTENF, SAFETY_DOOR_PIN) = !control_ire.safety_door_ajar;

        if(control_ire.reset)
        	BITBAND_PERI(CONTROL_INTENR, RESET_PIN) = control_ire.reset;
        else
        	BITBAND_PERI(CONTROL_INTENF, RESET_PIN) = !control_ire.reset;

        /***********************
         *  Limit pins config  *
         ***********************/

        axes_signals_t limit_ire;

        limit_ire.mask = ~(settings->limits.disable_pullup.mask ^ settings->limits.invert.mask);

        gpio_pinmode(LIMIT_PORT, X_LIMIT_PIN, settings->limits.disable_pullup.x ? PINMODE_PULLDOWN : PINMODE_PULLUP);
        gpio_pinmode(LIMIT_PORT, X_LIMIT_PIN, settings->limits.disable_pullup.x ? PINMODE_PULLDOWN : PINMODE_PULLUP);
        gpio_pinmode(LIMIT_PORT, X_LIMIT_PIN, settings->limits.disable_pullup.x ? PINMODE_PULLDOWN : PINMODE_PULLUP);

		/**********************
	 	 *  Probe pin config  *
		 **********************/

        gpio_pinmode(PROBE_PORT, PROBE_PIN, hal.driver_cap.probe_pull_up ? PINMODE_PULLUP : PINMODE_PULLDOWN);

    }
}

void NVIC_IRQEnable (uint32_t irqNum)
{
    if(irqNum < 32)
        NVIC->ISER[0] = 1 << irqNum; // Enable limit port Y,Z interrupt
    else
        NVIC->ISER[1] = 1 << (irqNum - 32); // Enable limit port Y,Z interrupt
}

void NVIC_IRQPriority(uint32_t irqNum, uint8_t priority)
{
    uint32_t temp;

    temp = NVIC->IP[irqNum >> 2];
    temp &= ~(0xFF << (8 * (irqNum & 3)));
    temp |= priority << (8 * (irqNum & 3));
    NVIC->IP[irqNum >> 2] = temp;
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
  //    Interrupt_disableSleepOnIsrExit();

    SysTick->LOAD = 480000 - 1;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk;

 // Stepper init

    STEP_PORT->FIODIR |= STEP_MASK;
    DIRECTION_PORT->FIODIR |= DIRECTION_MASK;
    STEPPERS_DISABLE_PORT->FIODIR |= STEPPERS_DISABLE_BIT;

    STEPPER_TIMER->TCR = 0;            // disable
    STEPPER_TIMER->CTCR = 0;           // timer mode
    STEPPER_TIMER->PR = 0;             // no prescale
    STEPPER_TIMER->MCR = MR0I|MR0R;    // MR0: !stop, reset, interrupt
    STEPPER_TIMER->CCR = 0;            // no capture
    STEPPER_TIMER->EMR = 0;            // no external match

    PULSE_TIMER->TCR = 2;
    PULSE_TIMER->CTCR = 0;
    PULSE_TIMER->PR = SystemCoreClock / 1000000; // to 1 us
    PULSE_TIMER->TCR = 0;

    NVIC_IRQEnable(STEPPER_TIMER_INT0);   // Enable stepper interrupt
    NVIC_IRQEnable(PULSE_TIMER_INT0);     // Enable step pulse interrupt

    NVIC_IRQPriority(PULSE_TIMER_INT0, 0x00);
    NVIC_IRQPriority(STEPPER_TIMER_INT0, 0x20);

 // Limit pins init

    NVIC_IRQEnable(EINT3_IRQn);  // Enable limit & control ports interrupt

 // Control pins init
 // NOTE: CS is shared with limit isr

//    NVIC_IRQEnable(CONTROL_INT);  // Enable limit port interrupt

    if(hal.driver_cap.software_debounce) {
        DEBOUNCE_TIMER->TCR = 2;
        DEBOUNCE_TIMER->CTCR = 0;
        DEBOUNCE_TIMER->PR = SystemCoreClock / 1000000; // 1 us
        DEBOUNCE_TIMER->MCR |= (MR0I|MR0S);
        DEBOUNCE_TIMER->MR0 = 4000; // 40 ms
        DEBOUNCE_TIMER->TCR = 0;
        NVIC_IRQEnable(DEBOUNCE_TIMER_INT0); // Enable debounce interrupt
    }

 // Spindle init

    SPINDLE_ENABLE_PORT->FIODIR |= SPINDLE_ENABLE_BIT;
    SPINDLE_DIRECTION_PORT->FIODIR |= SPINDLE_DIRECTION_BIT; // Configure as output pin.

    if(!hal.driver_cap.variable_spindle)
        hal.spindle_set_state = &spindleSetState;

 // Coolant init

    COOLANT_FLOOD_PORT->FIODIR |= COOLANT_FLOOD_BIT;
    COOLANT_MIST_PORT->FIODIR |= COOLANT_MIST_BIT;

#if SDCARD_ENABLE
    sdcard_init();
#endif

 // Set defaults

    IOInitDone = settings->version == 14;

    settings_changed(settings);

    spindleSetState((spindle_state_t){0}, spindle_pwm.off_value, DEFAULT_SPINDLE_RPM_OVERRIDE);
    coolantSetState((coolant_state_t){0});
    stepperSetDirOutputs((axes_signals_t){0});

    return IOInitDone;
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done

bool driver_init (void) {

    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

    SystemInit();

    serialInit();

    hal.info = "LCP1769";
    hal.driver_setup = driver_setup;
    hal.f_step_timer = SystemCoreClock;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = &driver_delay_ms;
    hal.settings_changed = settings_changed;

#if SDCARD_ENABLE
    hal.driver_reset = sdcard_reset;
#endif

    hal.stepper_wake_up = stepperWakeUp;
    hal.stepper_go_idle = stepperGoIdle;
    hal.stepper_enable = stepperEnable;
    hal.stepper_set_outputs = stepperSetStepOutputs;
    hal.stepper_set_directions = stepperSetDirOutputs;
    hal.stepper_cycles_per_tick = stepperCyclesPerTick;
    hal.stepper_pulse_start = stepperPulseStart;

    hal.limits_enable = limitsEnable;
    hal.limits_get_state = limitsGetState;

    hal.coolant_set_state = coolantSetState;
    hal.coolant_get_state = coolantGetState;

    hal.probe_get_state = probeGetState;
    hal.probe_configure_invert_mask = probeConfigureInvertMask;

    hal.spindle_set_state = spindleSetStateVariable;
    hal.spindle_get_state = spindleGetState;
    hal.spindle_set_speed = spindleSetSpeed;
    hal.spindle_compute_pwm_value = spindleComputePWMValue;

    hal.system_control_get_state = systemGetState;

    hal.stream.read = serialGetC;
    hal.stream.write = serialWriteS;
    hal.stream.write_all = serialWriteS;
    hal.stream.get_rx_buffer_available = serialRxFree;
    hal.stream.reset_read_buffer = serialRxFlush;
    hal.stream.cancel_read_buffer = serialRxCancel;

    hal.eeprom.type = EEPROM_None;
//    hal.eeprom.get_byte = eepromGetByte;
//    hal.eeprom.put_byte = eepromPutByte;
//    hal.eeprom.memcpy_to_with_checksum = eepromWriteBlockWithChecksum;
//    hal.eeprom.memcpy_from_with_checksum = eepromReadBlockWithChecksum;

#ifdef _flash_h_
    hal.eeprom.memcpy_from_flash = memcpy_from_flash;
    hal.eeprom.memcpy_to_flash = memcpy_to_flash;
#endif

    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
/*
    hal.userdefined_mcode_check = userMCodeCheck;
    hal.userdefined_mcode_validate = userMCodeValidate;
    hal.userdefined_mcode_execute = userMCodeExecute;
*/
#ifdef HAS_KEYPAD
    hal.execute_realtime = process_keypress;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

    hal.driver_cap.spindle_dir = On;
    hal.driver_cap.variable_spindle = On;
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
    // no need to move version check before init - compiler will fail any mismatch for existing entries
    return hal.version == 4;
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
        stepperSetStepOutputs(next_step_outbits); // Begin step pulse.
    if(ifg & MR1IFG)
        stepperSetStepOutputs((axes_signals_t){0});
    PULSE_TIMER->IR = ifg;
}

void DEBOUNCE_IRQHandler (void)
{
	PULSE_TIMER->IR = MR0IFG;

    axes_signals_t state = (axes_signals_t)limitsGetState();

    if(state.value) //TODO: add check for limit swicthes having same state as when limit_isr were invoked?
        hal.limit_interrupt_callback(state);
}


void GPIO_IRQHandler (void)
{
    uint32_t istat = LPC_GPIOINT->IntStatus, iflags;

    if(istat & P0Int) {
    	iflags = LIMIT_INTSTR | LIMIT_INTSTF;
    	if(iflags & LIMIT_MASK) {
    		LIMIT_INTCLR = iflags;
			if(hal.driver_cap.software_debounce)
				DEBOUNCE_TIMER->TCR = 1;
			else
				hal.limit_interrupt_callback(limitsGetState());
    	}
    }

    if(istat & P2Int) {
    	iflags = CONTROL_INTSTR | CONTROL_INTSTF;
    	if(iflags & CONTROL_MASK) {
    		CONTROL_INTCLR = iflags;
			if(hal.driver_cap.software_debounce)
				DEBOUNCE_TIMER->TCR = 1;
			else
				hal.control_interrupt_callback(systemGetState());
    	}
    }

}

// Interrupt handler for 1 ms interval timer
void SYSTICK_IRQHandler (void)
{
    if(!(--ms_count)) {
        SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk;
        if(delayCallback) {
            delayCallback();
            delayCallback = 0;
        }
    }
}
