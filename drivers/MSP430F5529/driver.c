/*
  driver.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Texas Instruments MSP430F5529 16-bit processor

  Part of grblHAL

  Copyright (c) 2016-2021 Terje Io
  Copyright (c) 2011-2015 Sungeun K. Jeon
  Copyright (c) 2009-2011 Simen Svale Skogsrud

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

#include <msp430.h>

#include "driver.h"
#include "serial.h"

#include "grbl/hal.h"
#include "grbl/grbl.h"
#include "grbl/limits.h"
#include "grbl/nuts_bolts.h"

#ifdef EEPROM_ENABLE
#include "i2c.h"
#include "eeprom/eeprom.h"
#endif

static volatile uint16_t debounce_count = 0;
static bool pwmEnabled = false, IOInitDone = false;
static uint16_t pulse_length;
static axes_signals_t next_step_outbits;
static spindle_pwm_t spindle_pwm;
static delay_t delay = { .ms = 0, .callback = NULL };
static probe_state_t probe = {
    .connected = On
};

static void spindle_set_speed (uint_fast16_t pwm_value);

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if((delay.ms = ms > 0)) {
        SYSTICK_TIMER_CCR0 = ms;
        SYSTICK_TIMER_CTL |= TACLR|MC0;
        if(!(delay.callback = callback))
            while(delay.ms);
    } else if(callback)
        callback();
}

// Set stepper pulse output pins
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z, needs to be mapped to physical pins by bit shifting or other means
inline static __attribute__ ((always_inline)) void set_step_outputs (axes_signals_t step_outbits)
{
    STEP_PORT_OUT = (STEP_PORT_OUT & ~HWSTEP_MASK) | (step_outbits.mask ^ settings.steppers.step_invert.mask) << 1;
}

// Set stepper direction output pins
// NOTE1: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z, needs to be mapped to physical pins by bit shifting or other means
inline static __attribute__ ((always_inline)) void set_dir_outputs (axes_signals_t dir_outbits)
{
    DIRECTION_PORT_OUT = (DIRECTION_PORT_OUT & ~HWDIRECTION_MASK) | (dir_outbits.mask ^ settings.steppers.dir_invert.mask);
}

// Enable/disable stepper motors
static void stepperEnable (axes_signals_t enable)
{
    enable.mask ^= settings.steppers.enable_invert.mask;

    if(enable.x)
        STEPPERS_DISABLE_OUT_XY &= ~STEPPERS_DISABLE_PIN_XY;
    else
        STEPPERS_DISABLE_OUT_XY |= STEPPERS_DISABLE_PIN_XY;

    if(enable.z)
        STEPPERS_DISABLE_OUT_Z &= ~STEPPERS_DISABLE_PIN_Z;
    else
        STEPPERS_DISABLE_OUT_Z |= STEPPERS_DISABLE_PIN_Z;
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp ()
{
    stepperEnable((axes_signals_t){AXES_BITMASK});

    STEPPER_TIMER_CCR0 = 0xFFFF;        // Set a long initial delay,
    STEPPER_TIMER_CTL |= TACLR|MC0;     // start stepper ISR timer in up mode
    hal.stepper.interrupt_callback();   // and start the show
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    STEPPER_TIMER_CTL &= ~(MC0|MC1);
    if(clear_signals) {
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}

// Sets up stepper driver interrupt timeout, AMASS version
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    STEPPER_TIMER_CTL |= TACLR; // start in up mode
    STEPPER_TIMER_CCR0 = cycles_per_tick < (1UL << 16) ? (uint16_t)cycles_per_tick : 0xFFFF;
}

// Sets up stepper driver interrupt timeout, "Normal" version
static void stepperCyclesPerTickPrescaled (uint32_t cycles_per_tick)
{
    // Set timer prescaling for normal step generation
    if (cycles_per_tick < (1UL << 16)) { // < 65536  (2.6ms @ 25MHz)
        STEPPER_TIMER_EX0 = TAIDEX_0;  // DIV 1
        STEPPER_TIMER_CTL &= ~(ID0|ID1); // DIV 1
    } else if (cycles_per_tick < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
        STEPPER_TIMER_EX0 = TAIDEX_0; // DIV 1
        STEPPER_TIMER_CTL |= ID0|ID1; // DIV 8
        cycles_per_tick = cycles_per_tick >> 3;
    } else {
        STEPPER_TIMER_EX0 = TAIDEX_7; // DIV 8
        STEPPER_TIMER_CTL |= ID0|ID1; // DIV 8
        cycles_per_tick = cycles_per_tick >> 6;
    }
    STEPPER_TIMER_CCR0 = cycles_per_tick < (1UL << 16) ? (uint16_t)cycles_per_tick : 0xFFFF;
    STEPPER_TIMER_CTL |= TACLR|MC0;
}

// Sets stepper direction and pulse pins and starts a step pulse
// When delayed pulse the step register is written in the step delay interrupt handler
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_change)
        set_dir_outputs(stepper->dir_outbits);

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        PULSE_TIMER_CTL |= TACLR|MC0;
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
            PULSE_TIMER_CCR0 = 50000;
            PULSE_TIMER_CCTL1 &= ~CCIFG;               // Clear and
            PULSE_TIMER_CCTL1 |= CCIE;                 // enable CCR1 interrupt.
            PULSE_TIMER_CTL |= TACLR|MC0;
        }

        return;
    }

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        PULSE_TIMER_CTL |= TACLR|MC0;
    }
}

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing)
{
    if (on && settings.limits.flags.hard_enabled)
        LIMIT_PORT_IE |= HWLIMIT_MASK; // Enable Pin Change Interrupt
    else
        LIMIT_PORT_IE &= ~HWLIMIT_MASK; // Disable Pin Change Interrupt
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};
    uint8_t flags = LIMIT_PORT_IN;

    signals.min.x = (flags & X_LIMIT_PIN) == X_LIMIT_PIN;
    signals.min.y = (flags & Y_LIMIT_PIN) == Y_LIMIT_PIN;
    signals.min.z = (flags & Z_LIMIT_PIN) == Z_LIMIT_PIN;

    if (settings.limits.invert.value)
        signals.min.value ^= settings.limits.invert.value;

    return signals;
}

// Returns limit state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
inline static control_signals_t systemGetState (void)
{
    control_signals_t signals;
    uint8_t flags = CONTROL_PORT_IN;

    signals.value = settings.control_invert.value;

    signals.reset            = (flags & RESET_PIN) == RESET_PIN;
    signals.feed_hold        = (flags & FEED_HOLD_PIN) == FEED_HOLD_PIN;
    signals.cycle_start      = (flags & CYCLE_START_PIN) == CYCLE_START_PIN;
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    signals.safety_door_ajar = (flags & SAFETY_DOOR_PIN) == SAFETY_DOOR_PIN;
#endif

    if(settings.control_invert.value)
        signals.value ^= settings.control_invert.value;

    return signals;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure(bool is_probe_away, bool probing)
{
    probe.triggered = Off;
    probe.is_probing = probing;
    probe.inverted = is_probe_away ? !settings.probe.invert_probe_pin : settings.probe.invert_probe_pin;
}

// Returns the probe connected and pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;
    state.triggered = !!(PROBE_PORT_IN & PROBE_PIN) ^ probe.inverted;

    return state;
}

// Static spindle (off, on cw & on ccw)

inline static void spindle_off (void)
{
    if(settings.spindle.invert.on)
        SPINDLE_ENABLE_OUT |= SPINDLE_ENABLE_PIN;
    else
        SPINDLE_ENABLE_OUT &= ~SPINDLE_ENABLE_PIN;
}

inline static void spindle_on (void)
{
    if(settings.spindle.invert.on)
        SPINDLE_ENABLE_OUT &= ~SPINDLE_ENABLE_PIN;
    else
        SPINDLE_ENABLE_OUT |= SPINDLE_ENABLE_PIN;
}

inline static void spindle_dir (bool ccw)
{
    if(hal.driver_cap.spindle_dir) {
        if(ccw ^ settings.spindle.invert.ccw)
            SPINDLE_DIRECTION_OUT |= SPINDLE_DIRECTION_PIN;
        else
            SPINDLE_DIRECTION_OUT &= ~SPINDLE_DIRECTION_PIN;
    }
}

// Starts or stops spindle
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
            PWM_TIMER_CCR1 = spindle_pwm.off_value;
            PWM_TIMER_CCTL1 = settings.spindle.invert.pwm ? OUTMOD_6 : OUTMOD_2;
        } else
            PWM_TIMER_CCTL1 = settings.spindle.invert.pwm ? OUT : 0;
    } else {
        if(!pwmEnabled)
            spindle_on();
        pwmEnabled = true;
        PWM_TIMER_CCR1 = pwm_value;
        PWM_TIMER_CCTL1 = settings.spindle.invert.pwm ? OUTMOD_6 : OUTMOD_2;
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

// Starts or stops spindle
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

    state.on = (SPINDLE_ENABLE_In & SPINDLE_ENABLE_PIN) != 0;
    if(hal.driver_cap.spindle_dir)
        state.ccw = (SPINDLE_DIRECTION_IN & SPINDLE_DIRECTION_PIN) != 0;
    state.value ^= settings.spindle.invert.mask;
    if(pwmEnabled)
        state.on = On;

    return state;
}

// end spindle code

// Starts/stops coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;

    if(mode.flood)
        COOLANT_FLOOD_OUT |= COOLANT_FLOOD_PIN;
    else
        COOLANT_FLOOD_OUT &= ~COOLANT_FLOOD_PIN;

    if(mode.mist)
        COOLANT_MIST_OUT |= COOLANT_MIST_PIN;
    else
        COOLANT_MIST_OUT &= ~COOLANT_MIST_PIN;
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

    state.flood = (COOLANT_FLOOD_IN & COOLANT_FLOOD_PIN) != 0;
    state.mist  = (COOLANT_MIST_IN & COOLANT_MIST_PIN) != 0;

    if(settings.coolant_invert.mask)
        state.value ^= settings.coolant_invert.mask;

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    _DINT();
    *ptr |= bits;
    _EINT();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    _DINT();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    _EINT();
    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    _DINT();
    uint_fast16_t prev = *ptr;
    *ptr = value;
    _EINT();
    return prev;
}

static void enable_irq (void)
{
    _EINT();
}

static void disable_irq (void)
{
    _DINT();
}

// Configures perhipherals when settings are initialized or changed
static void settings_changed (settings_t *settings)
{
    PWM_TIMER_EX0 = settings->spindle.pwm_freq < 1000.0f ? TAIDEX_7 : TAIDEX_1;

    hal.driver_cap.variable_spindle = spindle_precompute_pwm_values(&spindle_pwm, 25000000UL / (PWM_TIMER_EX0 == TAIDEX_1 ? 4 : 16));

    if(IOInitDone) {

        stepperEnable(settings->steppers.deenergize);

        int16_t t = (uint16_t)(5.0f * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY)) - 1;
        pulse_length = t < 2 ? 2 : t;
        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            t = (uint16_t)(5.0f * (settings->steppers.pulse_delay_microseconds - 2.0f));
            PULSE_TIMER_CCR1 = t < 2 ? 2 : t;
            hal.stepper.pulse_start = stepperPulseStartDelayed;
        } else
            hal.stepper.pulse_start = stepperPulseStart;

        PULSE_TIMER_CCR0 = pulse_length;

        if(hal.driver_cap.variable_spindle) {
            PWM_TIMER_CCR0 = spindle_pwm.period;
            PWM_TIMER_CCTL1 = settings->spindle.invert.pwm ? OUT : 0;   // Set PWM output according to invert setting and
            PWM_TIMER_CTL |= TACLR|MC0|MC1;                             // start PWM timer (with no pulse output)
            hal.spindle.set_state = spindleSetStateVariable;
        } else
            hal.spindle.set_state = spindleSetState;

        /*************************
         *  Control pins config  *
         *************************/

        control_signals_t control_ies;
        control_ies.mask = ~(settings->control_disable_pullup.mask ^ settings->control_invert.mask);

        CONTROL_PORT_IE &= ~HWCONTROL_MASK;         // Disable control pins change interrupt
        CONTROL_PORT_DIR &= ~HWCONTROL_MASK;         // Disable control pins change interrupt

        if(settings->control_disable_pullup.cycle_start)
            CONTROL_PORT_OUT &= ~CYCLE_START_PIN;
        else
            CONTROL_PORT_OUT |= CYCLE_START_PIN;

        if(settings->control_disable_pullup.feed_hold)
            CONTROL_PORT_OUT &= ~FEED_HOLD_PIN;
        else
            CONTROL_PORT_OUT |= FEED_HOLD_PIN;

        if(settings->control_disable_pullup.reset)
            CONTROL_PORT_OUT &= ~RESET_PIN;
        else
            CONTROL_PORT_OUT |= RESET_PIN;

        if(settings->control_disable_pullup.safety_door_ajar)
            CONTROL_PORT_OUT &= ~SAFETY_DOOR_PIN;
        else
            CONTROL_PORT_OUT |= SAFETY_DOOR_PIN;

        if(control_ies.cycle_start)
            CONTROL_PORT_IES &= ~CYCLE_START_PIN;
        else
            CONTROL_PORT_IES |= CYCLE_START_PIN;

        if(control_ies.feed_hold)
            CONTROL_PORT_IES &= ~FEED_HOLD_PIN;
        else
            CONTROL_PORT_IES |= FEED_HOLD_PIN;

        if(control_ies.reset)
            CONTROL_PORT_IES &= ~RESET_PIN;
        else
            CONTROL_PORT_IES |= RESET_PIN;

        if(control_ies.safety_door_ajar)
            CONTROL_PORT_IES &= ~SAFETY_DOOR_PIN;
        else
            CONTROL_PORT_IES |= SAFETY_DOOR_PIN;

        CONTROL_PORT_REN |= HWCONTROL_MASK;     // Enable pull-ups/pull-down,
        CONTROL_PORT_IFG &= ~HWCONTROL_MASK;    // clear any pending interrupt
        CONTROL_PORT_IE |= HWCONTROL_MASK;      // and enable control pins change interrupt

        /***********************
         *  Limit pins config  *
         ***********************/

        axes_signals_t limit_ies;

        limit_ies.mask = ~(settings->limits.disable_pullup.mask ^ settings->limits.invert.mask);

         if(settings->limits.disable_pullup.x)
             LIMIT_PORT_OUT &= ~X_LIMIT_PIN;
         else
             LIMIT_PORT_OUT |= X_LIMIT_PIN;

         if(settings->limits.disable_pullup.y)
             LIMIT_PORT_OUT &= ~Y_LIMIT_PIN;
         else
             LIMIT_PORT_OUT |= Y_LIMIT_PIN;

         if(settings->limits.disable_pullup.z)
             LIMIT_PORT_OUT &= ~Z_LIMIT_PIN;
         else
             LIMIT_PORT_OUT |= Z_LIMIT_PIN;

         if(limit_ies.x)
             LIMIT_PORT_IES &= ~X_LIMIT_PIN;
         else
             LIMIT_PORT_IES |= X_LIMIT_PIN;

         if(limit_ies.y)
             LIMIT_PORT_IES &= ~Y_LIMIT_PIN;
         else
             LIMIT_PORT_IES |= Y_LIMIT_PIN;

         if(limit_ies.z)
             LIMIT_PORT_IES &= ~Z_LIMIT_PIN;
         else
             LIMIT_PORT_IES |= Z_LIMIT_PIN;

         LIMIT_PORT_REN |= HWLIMIT_MASK;

         /**********************
          *  Probe pin config  *
          **********************/

          if(hal.driver_cap.probe_pull_up)
              PROBE_PORT_OUT |= PROBE_PIN;
          else
              PROBE_PORT_OUT &= ~PROBE_PIN;
          PROBE_PORT_REN |= PROBE_PIN;
    }
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
    /******************
     *  Stepper init  *
     ******************/

    STEP_PORT_DIR |= HWSTEP_MASK;
    DIRECTION_PORT_DIR |= HWDIRECTION_MASK;
    STEPPERS_DISABLE_DIR_XY |= STEPPERS_DISABLE_PIN_XY;
    STEPPERS_DISABLE_DIR_Z |= STEPPERS_DISABLE_PIN_Z;

    // Configure stepper driver timer

    STEPPER_TIMER_EX0 = TAIDEX_0; //
    STEPPER_TIMER_CTL &= ~(ID0|ID1|TAIFG);
    STEPPER_TIMER_CTL |= TACLR|TASSEL1;
    STEPPER_TIMER_CCTL0 |= CCIE;

    // Configure step pulse timer

    PULSE_TIMER_EX0 |= TAIDEX_4; // DIV 5
    PULSE_TIMER_CTL |= TACLR|TASSEL1; // for 0.2uS per count
    PULSE_TIMER_CCTL0 |= CCIE;

#ifdef CNC_BOOSTERPACK_A4998
    STEPPERS_VDD_DIR |= STEPPERS_VDD_BIT;
    STEPPERS_VDD_OUT |= STEPPERS_VDD_BIT;
#endif

   /****************************
    *  Software debounce init  *
    ****************************/

    if(hal.driver_cap.software_debounce)
        WDTCTL = WDT_ADLY_16;   // Watchdog timeout ~16ms

   /***********************
    *  Control pins init  *
    ***********************/

    CONTROL_PORT_DIR &= ~HWCONTROL_MASK;

   /*********************
    *  Limit pins init  *
    *********************/

    LIMIT_PORT_DIR &= ~HWLIMIT_MASK;

   /********************
    *  Probe pin init  *
    ********************/

    PROBE_PORT_DIR &= ~PROBE_PIN;
    if(hal.driver_cap.probe_pull_up) {
        PROBE_PORT_OUT |= PROBE_PIN;
        PROBE_PORT_REN |= PROBE_PIN;
    }

    /***********************
    *  Coolant pins init  *
    ***********************/

    COOLANT_FLOOD_DIR |= COOLANT_FLOOD_PIN;
    COOLANT_MIST_DIR |= COOLANT_MIST_PIN;

    if(hal.driver_cap.amass_level == 0)
        hal.stepper.cycles_per_tick = &stepperCyclesPerTickPrescaled;

   /******************
    *  Spindle init  *
    ******************/

    SPINDLE_ENABLE_DIR |= SPINDLE_ENABLE_PIN;
    SPINDLE_DIRECTION_DIR |= SPINDLE_DIRECTION_PIN;

    if((hal.driver_cap.variable_spindle)) {
        PWM_PORT_DIR |= PWM_PIN;
        PWM_SEL = PWM_PIN;
        PWM_TIMER_CTL |= TASSEL1;
        PWM_TIMER_EX0 = TAIDEX_3;
    } else
        hal.spindle.set_state = spindleSetState;

#ifdef HAS_KEYPAD

   /*********************
    *  I2C KeyPad init  *
    *********************/

    keypad_setup();

#endif

  // Set defaults

    IOInitDone = settings->version == 19;

    hal.settings_changed(settings);
    hal.stepper.go_idle(true);
    hal.spindle.set_state((spindle_state_t){0}, 0.0f);
    hal.coolant.set_state((coolant_state_t){0});

    return IOInitDone;
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{
    // Systick timer setup, uses ACLK / 32

    SYSTICK_TIMER_EX0 |= TAIDEX_3;
    SYSTICK_TIMER_CTL |= TACLR|ID0|ID1|TASSEL__ACLK; // for 1mS per count
    SYSTICK_TIMER_CCR0 = 1;
    SYSTICK_TIMER_CCTL0 |= CCIE;

    serialInit();

    hal.info = "MSP430F5529";
    hal.driver_version = "210219";
    hal.driver_setup = driver_setup;
    hal.f_step_timer = 24000000;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = driver_delay_ms;
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

    hal.spindle.set_state = spindleSetStateVariable;
    hal.spindle.get_state = spindleGetState;
#ifdef SPINDLE_PWM_DIRECT
    hal.spindle.get_pwm = spindleGetPWM;
    hal.spindle.update_pwm = spindle_set_speed;
#else
    hal.spindle_update_rpm = spindleUpdateRPM;
#endif

    hal.control.get_state = systemGetState;

    hal.stream.read = serialGetC;
    hal.stream.write = serialWriteS;
    hal.stream.write_all = serialWriteS;
    hal.stream.get_rx_buffer_available = serialRxFree;
    hal.stream.reset_read_buffer = serialRxFlush;
    hal.stream.cancel_read_buffer = serialRxCancel;

#ifdef EEPROM_ENABLE
    i2c_init();
    i2c_eeprom_init();
#else
    hal.nvs.type = NVS_None;
#endif

    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.irq_enable = enable_irq;
    hal.irq_disable = disable_irq;

#ifdef HAS_KEYPAD
    hal.execute_realtime = process_keypress;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

#ifdef SAFETY_DOOR_PIN
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

    __bis_SR_register(GIE); // Enable interrupts

    my_plugin_init();

    // no need to move version check before init - compiler will fail any signature mismatch for existing entries
    return hal.version == 8;
}

/* interrupt handlers */

// Main stepper driver ISR
#pragma vector=STEPPER_TIMER0_VECTOR
__interrupt void stepper_driver_isr (void)
{
    static bool busy = false;

    if(!busy) {
        busy = true;
        _EINT();
        hal.stepper.interrupt_callback();
        busy = false;
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
// This interrupt is enabled when Grbl sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
// NOTE: MSP430 has a shared interrupt for match and timeout

#pragma vector=PULSE_TIMER0_VECTOR
__interrupt void stepper_pulse_isr (void)
{
    set_step_outputs(settings.steppers.step_invert);
    PULSE_TIMER_CTL &= ~(MC0|MC1);
}

#pragma vector=PULSE_TIMER1_VECTOR
__interrupt void stepper_pulse_isr_delayed (void)
{
    if(PULSE_TIMER_IV == TA0IV_TACCR1) {
        set_step_outputs(next_step_outbits);
        PULSE_TIMER_CCTL1 &= ~CCIE;                  // Disable CCR1 interrupt
        PULSE_TIMER_CCR0 = pulse_length;
        PULSE_TIMER_CTL |= TACLR|MC0;
    }
}

#pragma vector=WDT_VECTOR
__interrupt void software_debounce_isr (void)
{
    if(!--debounce_count) {
        SFRIE1 &= ~WDTIE;
        limit_signals_t state = limitsGetState();
        if(limit_signals_merge(state).value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
            hal.limits.interrupt_callback(state);
    }
}

#pragma vector=CONTROL_PORT_VECTOR
__interrupt void control_isr (void)
{
    uint8_t iflags = CONTROL_PORT_IFG & HWCONTROL_MASK;

    if(iflags) {
        CONTROL_PORT_IFG &= ~iflags;
        hal.control.interrupt_callback(systemGetState());
    }
}

#pragma vector=LIMIT_PORT_VECTOR
__interrupt void limit_isr (void)
{
    uint16_t iflags = LIMIT_PORT_IFG & HWLIMIT_MASK;

    if(iflags) {

        LIMIT_PORT_IFG &= ~iflags;

        if(hal.driver_cap.software_debounce) {
            WDTCTL = WDT_ADLY_16;   // Set watchdog timeout to ~16ms
            SFRIE1 |= WDTIE;        // and enable interrupt
            debounce_count = 3;     // Debounce = 3x watchdog timeout
        } else
            hal.limits.interrupt_callback(limitsGetState());
    }
}

// Interrupt handler for 1 ms interval timer
#pragma vector=SYSTICK_TIMER0_VECTOR
__interrupt void systick_isr (void)
{
    delay.ms = 0;
    SYSTICK_TIMER_CTL &= ~(MC0|MC1);
    if(delay.callback) {
        delay.callback();
        delay.callback = NULL;
    }
}
