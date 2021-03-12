/*

  driver.c - driver code for Atmel SAMD21 ARM processor

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

#include "Arduino.h"

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

#if IOEXPAND_ENABLE
#include "ioexpand.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#if KEYPAD_ENABLE
#include "keypad/keypad.h"
void KEYPAD_IRQHandler (void);
#endif

typedef struct {
    PortGroup *port;
    uint32_t bit;
} gpio_t;

static gpio_t stepX, stepY, stepZ, dirX, dirY, dirZ;
#if !IOEXPAND_ENABLE
static gpio_t spindleEnable, spindleDir, steppersEnable, Mist, Flood;
#endif
#ifdef DEBUGOUT
static gpio_t Led;
#endif

#define pinIn(p) ((PORT->Group[g_APinDescription[p].ulPort].IN.reg & (1 << g_APinDescription[p].ulPin)) != 0)
#define DIGITAL_OUT(gpio, on) { if(on) gpio.port->OUTSET.reg = gpio.bit; else gpio.port->OUTCLR.reg = gpio.bit; }

uint32_t vectorTable[sizeof(DeviceVectors) / sizeof(uint32_t)] __attribute__(( aligned (0x100ul) ));

//static uint32_t lim_IRQMask = 0;
static uint16_t pulse_length, pulse_delay;
static bool pwmEnabled = false, IOInitDone = false;
static bool sd_detect = false;
static axes_signals_t next_step_outbits;
static spindle_pwm_t spindle_pwm;
static delay_t delay_ms = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
static probe_state_t probe = {
    .connected = On
};

static axes_signals_t limit_ies; // declare here for now...

static void spindle_set_speed (uint_fast16_t pwm_value);

#if IOEXPAND_ENABLE
static ioexpand_t iopins = {0};
#endif

static void SysTick_IRQHandler (void);
static void STEPPER_IRQHandler (void);
static void STEPPULSE_IRQHandler (void);
static void STEPPULSE_Delayed_IRQHandler (void);
static void LIMIT_IRQHandler (void);
static void CONTROL_IRQHandler (void);
static void DEBOUNCE_IRQHandler (void);
static void SD_IRQHandler (void);

extern void Dummy_Handler(void);

void IRQRegister(int32_t IRQnum, void (*IRQhandler)(void))
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
static inline __attribute__((always_inline)) void set_step_outputs (axes_signals_t step_outbits)
{
    step_outbits.value ^= settings.steppers.step_invert.mask;

    DIGITAL_OUT(stepX, step_outbits.x);
    DIGITAL_OUT(stepY, step_outbits.y);
    DIGITAL_OUT(stepZ, step_outbits.z);
}

// Set stepper direction output pins
static inline __attribute__((always_inline)) void set_dir_outputs (axes_signals_t dir_outbits)
{
    dir_outbits.value ^= settings.steppers.dir_invert.mask;

    DIGITAL_OUT(dirX, dir_outbits.x);
    DIGITAL_OUT(dirY, dir_outbits.y);
    DIGITAL_OUT(dirZ, dir_outbits.z);
}

// Enable/disable stepper motors
static void stepperEnable (axes_signals_t enable)
{
    enable.value ^= settings.steppers.enable_invert.mask;
#if TRINAMIC_ENABLE == 2130 && TRINAMIC_I2C
    trinamic_stepper_enable(enable);
#elif IOEXPAND_ENABLE // TODO: read from expander?
    iopins.stepper_enable_xy = enable.x;
//    iopins.stepper_enable_y = enable.y;
    iopins.stepper_enable_z = enable.z;
    ioexpand_out(iopins);   
#else
    DIGITAL_OUT(steppersEnable, enable.x);
#endif
}

// Resets and enables stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    stepperEnable((axes_signals_t){AXES_BITMASK});

    STEPPER_TIMER->COUNT32.COUNT.reg = 0;
    while(STEPPER_TIMER->COUNT32.STATUS.bit.SYNCBUSY);
    
    STEPPER_TIMER->COUNT32.CTRLA.reg |= TC_CTRLA_ENABLE;
    while(STEPPER_TIMER->COUNT32.STATUS.bit.SYNCBUSY);

    STEP_TIMER->COUNT16.CTRLA.reg |= TC_CTRLA_ENABLE;
    while(STEP_TIMER->COUNT16.STATUS.bit.SYNCBUSY);

    hal.stepper.interrupt_callback();   // start the show
}

// Disables stepper driver interrupts
static void stepperGoIdle (bool clear_signals)
{
    STEPPER_TIMER->COUNT32.CTRLBSET.reg = TC_CTRLBCLR_CMD_STOP;
    while(STEPPER_TIMER->COUNT32.STATUS.bit.SYNCBUSY);

    if(clear_signals) {
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}

// Sets up stepper driver interrupt timeout, AMASS version
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
// Limit min steps/s to about 2 (hal.f_step_timer @ 20MHz)
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    STEPPER_TIMER->COUNT32.CC[0].reg = cycles_per_tick < (1UL << 18) ? cycles_per_tick : (1UL << 18) - 1UL;
#else
    STEPPER_TIMER->COUNT32.CC[0].reg = cycles_per_tick < (1UL << 23) ? cycles_per_tick : (1UL << 23) - 1UL;
#endif
    while(STEPPER_TIMER->COUNT32.STATUS.bit.SYNCBUSY);
}

// Sets stepper direction and pulse pins and starts a step pulse
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_change)
        set_dir_outputs(stepper->dir_outbits);

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        STEP_TIMER->COUNT16.CTRLBSET.reg = TC_CTRLBCLR_CMD_RETRIGGER|TCC_CTRLBSET_ONESHOT;
    }
}

// Start a stepper pulse, delay version.
// Note: delay is only added when there is a direction change and a pulse to be output.
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->dir_change) {

        set_dir_outputs(stepper->dir_outbits);

        if(stepper->step_outbits.value) {

            IRQRegister(STEP_TIMER_IRQn, STEPPULSE_Delayed_IRQHandler);

            next_step_outbits = stepper->step_outbits; // Store out_bits

            STEP_TIMER->COUNT16.CC[0].reg = pulse_delay;
            while(STEP_TIMER->COUNT16.STATUS.bit.SYNCBUSY);
            STEP_TIMER->COUNT16.CTRLBSET.reg = TC_CTRLBCLR_CMD_RETRIGGER|TCC_CTRLBSET_ONESHOT;
        }

        return;
    }

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        STEP_TIMER->COUNT16.CTRLBSET.reg = TC_CTRLBCLR_CMD_RETRIGGER|TCC_CTRLBSET_ONESHOT;
    }
}

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing)
{
    on = on && settings.limits.flags.hard_enabled;

    if(on) {
        attachInterrupt(X_LIMIT_PIN, LIMIT_IRQHandler, limit_ies.x ? FALLING : RISING);
        attachInterrupt(Y_LIMIT_PIN, LIMIT_IRQHandler, limit_ies.y ? FALLING : RISING);
        attachInterrupt(Z_LIMIT_PIN, LIMIT_IRQHandler, limit_ies.z ? FALLING : RISING);
    } else {
        detachInterrupt(X_LIMIT_PIN);
        detachInterrupt(Y_LIMIT_PIN);
        detachInterrupt(Z_LIMIT_PIN);
    }

/*
    if(on)
        EIC->INTENSET.reg = lim_IRQMask;
    else?
        EIC->INTENCLR.reg = lim_IRQMask;
*/
#if TRINAMIC_ENABLE == 2130
    trinamic_homing(homing);
#endif
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};
    
    signals.min.x = pinIn(X_LIMIT_PIN);
    signals.min.y = pinIn(Y_LIMIT_PIN);
    signals.min.z = pinIn(Z_LIMIT_PIN);

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

    signals.reset = pinIn(RESET_PIN);
    signals.feed_hold = pinIn(FEED_HOLD_PIN);
    signals.cycle_start = pinIn(CYCLE_START_PIN);
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    signals.safety_door_ajar = pinIn(SAFETY_DOOR_PIN);
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
#ifdef PROBE_PIN
    state.triggered = !!pinIn(PROBE_PIN) ^ probe.inverted;
#else
    state.triggered = false;
#endif

    return state;
}

#endif

// Static spindle (off, on cw & on ccw)

inline static void spindle_off (void)
{
#if IOEXPAND_ENABLE
    bool on = settings.spindle.invert.on ? On : Off;
    if(iopins.spindle_on != on) {
        iopins.spindle_on = on;
        ioexpand_out(iopins);
    }
#else
    DIGITAL_OUT(spindleEnable, settings.spindle.invert.on);
#endif
}

inline static void spindle_on (void)
{
#if IOEXPAND_ENABLE
    bool on = settings.spindle.invert.on ? Off : On;
    if(iopins.spindle_on != on) {
        iopins.spindle_on = on;
        ioexpand_out(iopins);
    }
#else
    DIGITAL_OUT(spindleEnable, !settings.spindle.invert.on);
#endif
}

inline static void spindle_dir (bool ccw)
{
#ifdef SPINDLE_DIRECTION_PIN
#if IOEXPAND_ENABLE
    if(hal.driver_cap.spindle_dir) {
        bool ccw = (ccw ^ settings.spindle.invert.ccw) ? On : Off;
        if(iopins.spindle_dir != ccw) {
            iopins.spindle_dir = ccw;
            ioexpand_out(iopins);
        }
    }
#else
    if(hal.driver_cap.spindle_dir)
        DIGITAL_OUT(spindleDir, (ccw ^ settings.spindle.invert.ccw));
#endif
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
        if(spindle_pwm.always_on) {
            SPINDLE_PWM_TIMER->CC[SPINDLE_PWM_CCREG].bit.CC = spindle_pwm.off_value;
            while(SPINDLE_PWM_TIMER->SYNCBUSY.bit.CC2);
            SPINDLE_PWM_TIMER->CTRLBSET.bit.CMD = TCC_CTRLBCLR_CMD_RETRIGGER_Val;
            while(SPINDLE_PWM_TIMER->SYNCBUSY.bit.CTRLB);
        } else {
            SPINDLE_PWM_TIMER->CTRLBSET.bit.CMD = TCC_CTRLBCLR_CMD_STOP_Val;
            while(SPINDLE_PWM_TIMER->SYNCBUSY.bit.CTRLB);
        }
    } else {
        if(!pwmEnabled)
            spindle_on();
        pwmEnabled = true;

        SPINDLE_PWM_TIMER->CC[SPINDLE_PWM_CCREG].bit.CC = pwm_value;
        while(SPINDLE_PWM_TIMER->SYNCBUSY.bit.CC2);
        SPINDLE_PWM_TIMER->CTRLBSET.bit.CMD = TCC_CTRLBCLR_CMD_RETRIGGER_Val;
        while(SPINDLE_PWM_TIMER->SYNCBUSY.bit.CTRLB);
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

#if IOEXPAND_ENABLE // TODO: read from expander?
    state.on = iopins.spindle_on;
    state.ccw = hal.driver_cap.spindle_dir && iopins.spindle_dir;
#else
    state.on = pinIn(SPINDLE_ENABLE_PIN) != 0;
  #ifdef SPINDLE_DIRECTION_PIN
    state.ccw = hal.driver_cap.spindle_dir && pinIn(SPINDLE_DIRECTION_PIN) != 0;
  #endif
#endif

    state.value ^= settings.spindle.invert.mask;
    if(pwmEnabled)
        state.on = On;

    return state;
}

// end spindle code

#ifdef DEBUGOUT
void debug_out (bool on)
{
//hal.stream.write(on ? "#" : "!");
    DIGITAL_OUT(Led, on);
}
#endif

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;
#if IOEXPAND_ENABLE
    if(!((iopins.flood_on == mode.flood) && (iopins.mist_on == mode.mist))) {
        iopins.flood_on = mode.flood;
        iopins.mist_on = mode.mist;
        ioexpand_out(iopins);
    }
#else
    DIGITAL_OUT(Flood, mode.flood);
    DIGITAL_OUT(Mist, mode.mist);
#endif
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

#if IOEXPAND_ENABLE // TODO: read from expander?
    state.flood = iopins.flood_on;
    state.mist = iopins.mist_on;
#else
    state.flood = pinIn(COOLANT_FLOOD_PIN);
    state.mist  = pinIn(COOLANT_MIST_PIN);
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

void pinModeOutput (gpio_t *gpio, uint8_t pin)
{
    pinMode(pin, OUTPUT);
    gpio->port = (PortGroup *)&PORT->Group[g_APinDescription[pin].ulPort].DIR.reg;
    gpio->bit = 1 << g_APinDescription[pin].ulPin;
}

// Configures perhipherals when settings are initialized or changed
void settings_changed (settings_t *settings)
{
    bool variable_spindle;

    if((variable_spindle = (hal.driver_cap.variable_spindle && settings->spindle.rpm_min < settings->spindle.rpm_max))) {

        SPINDLE_PWM_TIMER->CTRLA.bit.ENABLE = 0;
        while(SPINDLE_PWM_TIMER->SYNCBUSY.bit.ENABLE);

        if(settings->spindle.pwm_freq > 200.0f)
            SPINDLE_PWM_TIMER->CTRLA.bit.PRESCALER = TC_CTRLA_PRESCALER_DIV1_Val;
        else
            SPINDLE_PWM_TIMER->CTRLA.bit.PRESCALER = TC_CTRLA_PRESCALER_DIV8_Val;

//        while(SPINDLE_PWM_TIMER->SYNCBUSY.bit.PRESCALER);

        spindle_pwm.offset = 1;
        spindle_precompute_pwm_values(&spindle_pwm, hal.f_step_timer / (settings->spindle.pwm_freq > 200.0f ? 1 : 8));
    }

    if(IOInitDone) {

        stepperEnable(settings->steppers.deenergize);

        if(variable_spindle) {
            SPINDLE_PWM_TIMER->PER.bit.PER = spindle_pwm.period;
            while(SPINDLE_PWM_TIMER->SYNCBUSY.bit.PER);
            SPINDLE_PWM_TIMER->CC[SPINDLE_PWM_CCREG].bit.CC = 0;
            while(SPINDLE_PWM_TIMER->SYNCBUSY.bit.CC2);
            SPINDLE_PWM_TIMER->CTRLA.bit.ENABLE = 1;        
            while(SPINDLE_PWM_TIMER->SYNCBUSY.bit.ENABLE);
            hal.spindle.set_state = spindleSetStateVariable;
        } else
            hal.spindle.set_state = spindleSetState;

        int16_t t = (int16_t)(24.0f * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY)) - 1;
        pulse_length = t < 2 ? 2 : t;

        if(settings->steppers.pulse_delay_microseconds > 0.0f) {
            t = (int16_t)(24.0f * (settings->steppers.pulse_delay_microseconds - 1.7f)) - 1;
            pulse_delay = t < 2 ? 2 : t;
            hal.stepper.pulse_start = stepperPulseStartDelayed;
        } else
            hal.stepper.pulse_start = stepperPulseStart;

        next_step_outbits.value = 0;
        IRQRegister(STEP_TIMER_IRQn, STEPPULSE_IRQHandler);

        STEP_TIMER->COUNT16.CC[0].reg = pulse_length;
        STEP_TIMER->COUNT16.INTENSET.bit.MC0 = 1; // Enable CC0 interrupt

        /*************************
         *  Control pins config  *
         *************************/

        NVIC_DisableIRQ(EIC_IRQn);
        NVIC_SetPriority(EIC_IRQn, 3);

        control_signals_t control_ies;

        control_ies.mask = (settings->control_disable_pullup.mask ^ settings->control_invert.mask);

#ifdef SAFETY_DOOR_PIN
        detachInterrupt(SAFETY_DOOR_PIN);
        pinMode(SAFETY_DOOR_PIN, settings->control_disable_pullup.safety_door_ajar ? INPUT_PULLDOWN : INPUT_PULLUP);
        attachInterrupt(SAFETY_DOOR_PIN, CONTROL_IRQHandler, control_ies.safety_door_ajar ? FALLING : RISING);
#endif

        detachInterrupt(CYCLE_START_PIN);
        detachInterrupt(FEED_HOLD_PIN);
        detachInterrupt(RESET_PIN);
        
        pinMode(CYCLE_START_PIN, settings->control_disable_pullup.cycle_start ? INPUT_PULLDOWN : INPUT_PULLUP);
        pinMode(FEED_HOLD_PIN, settings->control_disable_pullup.feed_hold ? INPUT_PULLDOWN : INPUT_PULLUP);
        pinMode(RESET_PIN, settings->control_disable_pullup.reset ? INPUT_PULLDOWN : INPUT_PULLUP);

        attachInterrupt(CYCLE_START_PIN, CONTROL_IRQHandler, control_ies.cycle_start ? FALLING : RISING);
        attachInterrupt(FEED_HOLD_PIN, CONTROL_IRQHandler, control_ies.feed_hold ? FALLING : RISING);
        attachInterrupt(RESET_PIN, CONTROL_IRQHandler, control_ies.reset ? FALLING : RISING);
                
        /***********************
         *  Limit pins config  *
         ***********************/

//        axes_signals_t limit_ies;

        limit_ies.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;
        
        detachInterrupt(X_LIMIT_PIN);
        detachInterrupt(Y_LIMIT_PIN);
        detachInterrupt(Z_LIMIT_PIN);
        
        pinMode(X_LIMIT_PIN, settings->limits.disable_pullup.x ? INPUT_PULLDOWN : INPUT_PULLUP);
        pinMode(Y_LIMIT_PIN, settings->limits.disable_pullup.y ? INPUT_PULLDOWN : INPUT_PULLUP);
        pinMode(Z_LIMIT_PIN, settings->limits.disable_pullup.z ? INPUT_PULLDOWN : INPUT_PULLUP);

        attachInterrupt(X_LIMIT_PIN, LIMIT_IRQHandler, limit_ies.x ? FALLING : RISING);
        attachInterrupt(Y_LIMIT_PIN, LIMIT_IRQHandler, limit_ies.y ? FALLING : RISING);
        attachInterrupt(Z_LIMIT_PIN, LIMIT_IRQHandler, limit_ies.z ? FALLING : RISING);

#if KEYPAD_ENABLE
        pinMode(KEYPAD_PIN, INPUT_PULLUP);
        attachInterrupt(KEYPAD_PIN, KEYPAD_IRQHandler, CHANGE);
#endif

        // Bad code elsewhere requires this...
        hal.delay_ms(2, NULL);
        EIC->INTFLAG.reg = 0x0003FFFF;
        NVIC_ClearPendingIRQ(EIC_IRQn);
        NVIC_EnableIRQ(EIC_IRQn);
        // ...or we will enter ALARM!

/*      
        EExt_Interrupts irq;
    #if ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10606
        irq = g_APinDescription[X_LIMIT_PIN].ulExtInt;
    #else
        irq = digitalPinToInterrupt(X_LIMIT_PIN);
    #endif  
        lim_IRQMask = EIC_INTENSET_EXTINT(1 << irq);
        
    #if ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10606
        irq = g_APinDescription[Y_LIMIT_PIN].ulExtInt;
    #else
        irq = digitalPinToInterrupt(Y_LIMIT_PIN);
    #endif  
        lim_IRQMask |= EIC_INTENSET_EXTINT(1 << irq);
        
    #if ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10606
        irq = g_APinDescription[Z_LIMIT_PIN].ulExtInt;
    #else
        irq = digitalPinToInterrupt(Z_LIMIT_PIN);
    #endif  
        lim_IRQMask |= EIC_INTENSET_EXTINT(1 << irq);

        limitsEnable(settings->limits.flags.hard_enabled, false);
*/
        /**********************
         *  Probe pin config  *
         **********************/
#ifdef PROBE_PIN         
        pinMode(PROBE_PIN, hal.driver_cap.probe_pull_up ? INPUT_PULLUP : INPUT_PULLDOWN);
#endif
    }
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{   
    // Stepper timer clock - 16 MHz
    GCLK->GENDIV.reg = (uint32_t)(GCLK_GENDIV_ID(7)|GCLK_GENDIV_DIV(3));
    while(GCLK->STATUS.bit.SYNCBUSY);
    GCLK->GENCTRL.reg = (uint32_t)(GCLK_GENCTRL_ID(7)|GCLK_GENCTRL_SRC_DFLL48M|GCLK_GENCTRL_IDC|GCLK_GENCTRL_GENEN);
    while(GCLK->STATUS.bit.SYNCBUSY);
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK7 | GCLK_CLKCTRL_ID_TC4_TC5);
    while(GCLK->STATUS.bit.SYNCBUSY);

    // Step timer clock - 24 MHz
    GCLK->GENDIV.reg = (uint32_t)(GCLK_GENDIV_ID(6)|GCLK_GENDIV_DIV(2));
    while(GCLK->STATUS.bit.SYNCBUSY);
    GCLK->GENCTRL.reg = (uint32_t)(GCLK_GENCTRL_ID(6)|GCLK_GENCTRL_SRC_DFLL48M|GCLK_GENCTRL_IDC|GCLK_GENCTRL_GENEN);
    while(GCLK->STATUS.bit.SYNCBUSY);
    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK6 | GCLK_CLKCTRL_ID_TCC2_TC3);
    while(GCLK->STATUS.bit.SYNCBUSY);

 // Stepper init

    PM->APBCMASK.reg |= PM_APBCMASK_TC4;
    PM->APBCMASK.reg |= PM_APBCMASK_TC5;

    // Stepper driver timer - counts down
    STEPPER_TIMER->COUNT32.CTRLA.bit.ENABLE = 0;    // Disable and
    while(STEPPER_TIMER->COUNT32.STATUS.bit.SYNCBUSY);
    STEPPER_TIMER->COUNT32.CTRLA.bit.SWRST = 1;     // reset timer
    while(STEPPER_TIMER->COUNT32.CTRLA.bit.SWRST);
    STEPPER_TIMER->COUNT32.CTRLA.reg = TC_CTRLA_MODE_COUNT32|TC_CTRLA_WAVEGEN_MPWM;
    while(STEPPER_TIMER->COUNT32.STATUS.bit.SYNCBUSY);
//  STEPPER_TIMER->COUNT32.CTRLBSET.reg = (uint8_t)TC_CTRLBSET_ONESHOT;
//  while(STEPPER_TIMER->COUNT32.STATUS.bit.SYNCBUSY);
    STEPPER_TIMER->COUNT32.INTENSET.bit.MC0 = 1; // Enable overflow interrupt

    // Step pulse timer - counts up
    STEP_TIMER->COUNT16.CTRLA.bit.ENABLE = 0;   // Disable and
    while(STEP_TIMER->COUNT16.STATUS.bit.SYNCBUSY);
    STEP_TIMER->COUNT16.CTRLA.bit.SWRST = 1;    // reset timer
    while(STEP_TIMER->COUNT16.CTRLA.bit.SWRST);
    STEP_TIMER->COUNT16.CTRLBSET.reg = TC_CTRLBSET_ONESHOT;
//  STEP_TIMER->COUNT16.CTRLBSET.reg = (uint8_t)(TC_CTRLBSET_DIR|TC_CTRLBSET_ONESHOT);
//  while(STEP_TIMER->COUNT16.STATUS.bit.SYNCBUSY);
//  STEP_TIMER->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16|TC_CTRLA_PRESCALER_DIV2;
    STEP_TIMER->COUNT16.CTRLA.reg = TC_CTRLA_MODE_COUNT16|TC_CTRLA_WAVEGEN_MPWM;
    while(STEP_TIMER->COUNT16.STATUS.bit.SYNCBUSY);
    STEP_TIMER->COUNT16.INTENSET.bit.MC0 = 1; // Enable CC0 interrupt

    IRQRegister(STEPPER_TIMER_IRQn, STEPPER_IRQHandler);
    IRQRegister(STEP_TIMER_IRQn, STEPPULSE_IRQHandler);

    NVIC_EnableIRQ(STEPPER_TIMER_IRQn); // Enable stepper interrupt
    NVIC_EnableIRQ(STEP_TIMER_IRQn);    // Enable step pulse interrupt

    NVIC_SetPriority(STEPPER_TIMER_IRQn, 2);
    NVIC_SetPriority(STEP_TIMER_IRQn, 1);

    pinModeOutput(&stepX, X_STEP_PIN);
    pinModeOutput(&stepY, Y_STEP_PIN);
    pinModeOutput(&stepZ, Z_STEP_PIN);

    pinModeOutput(&dirX, X_DIRECTION_PIN);
    pinModeOutput(&dirY, Y_DIRECTION_PIN);
    pinModeOutput(&dirZ, Z_DIRECTION_PIN);

// Enable GPIO interrupt (done by Arduino library for now)
//  IRQRegister(EIC_IRQn, LIMIT_IRQHandler);
//  NVIC_EnableIRQ(EIC_IRQn);

    if(hal.driver_cap.software_debounce) {

        GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK7 | GCLK_CLKCTRL_ID_TCC0_TCC1); // 16 MHz
        while(GCLK->STATUS.bit.SYNCBUSY);

        DEBOUNCE_TIMER->CTRLA.bit.ENABLE = 0;   // Disable and
        while(DEBOUNCE_TIMER->SYNCBUSY.bit.ENABLE);
        DEBOUNCE_TIMER->CTRLA.bit.SWRST = 1;    // reset timer
        while(DEBOUNCE_TIMER->SYNCBUSY.bit.SWRST || DEBOUNCE_TIMER->CTRLA.bit.SWRST);
        DEBOUNCE_TIMER->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV16;
        DEBOUNCE_TIMER->CTRLBSET.reg = TCC_CTRLBSET_DIR|TCC_CTRLBSET_ONESHOT;
        while(DEBOUNCE_TIMER->SYNCBUSY.bit.CTRLB);
        DEBOUNCE_TIMER->PER.bit.PER = 48000; // 48 ms delay
        while(DEBOUNCE_TIMER->SYNCBUSY.bit.PER);

        DEBOUNCE_TIMER->CTRLA.bit.ENABLE = 1;       
        while(DEBOUNCE_TIMER->SYNCBUSY.bit.ENABLE);

        DEBOUNCE_TIMER->CTRLBSET.bit.CMD = TCC_CTRLBCLR_CMD_STOP_Val;
        while(DEBOUNCE_TIMER->SYNCBUSY.bit.CTRLB);

        DEBOUNCE_TIMER->INTENSET.bit.OVF = 1; // Enable overflow interrupt

        NVIC_SetPriority(DEBOUNCE_TIMER_IRQn, 3);
        IRQRegister(DEBOUNCE_TIMER_IRQn, DEBOUNCE_IRQHandler);
        NVIC_EnableIRQ(DEBOUNCE_TIMER_IRQn);    // Enable stepper interrupt
    }

 // Steppers disable init
#if !IOEXPAND_ENABLE
    pinModeOutput(&steppersEnable, STEPPERS_DISABLE_PIN);
#endif

 // Spindle init
#if !IOEXPAND_ENABLE
    pinModeOutput(&spindleEnable, SPINDLE_ENABLE_PIN);
  #ifdef SPINDLE_DIRECTION_PIN
    pinModeOutput(&spindleDir, SPINDLE_DIRECTION_PIN);
  #endif
#endif
    pinMode(SPINDLEPWMPIN, OUTPUT);

    GCLK->CLKCTRL.reg = (uint16_t)(GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK7 | GCLK_CLKCTRL_ID_TCC0_TCC1); // 16 MHz
    while(GCLK->STATUS.bit.SYNCBUSY);

    PORT->Group[g_APinDescription[SPINDLEPWMPIN].ulPort].PINCFG[g_APinDescription[SPINDLEPWMPIN].ulPin].bit.PMUXEN = 1;
    PORT->Group[g_APinDescription[SPINDLEPWMPIN].ulPort].PMUX[g_APinDescription[SPINDLEPWMPIN].ulPin >> 1].reg = PORT_PMUX_PMUXE_F;

    SPINDLE_PWM_TIMER->CTRLA.bit.ENABLE = 0;
    while(SPINDLE_PWM_TIMER->SYNCBUSY.bit.ENABLE);
    SPINDLE_PWM_TIMER->CTRLA.bit.SWRST = 1;
    while(SPINDLE_PWM_TIMER->SYNCBUSY.bit.SWRST || SPINDLE_PWM_TIMER->CTRLA.bit.SWRST);
    SPINDLE_PWM_TIMER->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
    while(SPINDLE_PWM_TIMER->SYNCBUSY.bit.WAVE);
    SPINDLE_PWM_TIMER->CTRLA.bit.RESOLUTION = TCC_CTRLA_RESOLUTION_NONE_Val;

 // Coolant init
#if !IOEXPAND_ENABLE
    pinModeOutput(&Flood, COOLANT_FLOOD_PIN);
    pinModeOutput(&Mist, COOLANT_MIST_PIN);
#endif

#if IOEXPAND_ENABLE
    ioexpand_init();
#endif

#ifdef DEBUGOUT
    pinModeOutput(&Led, LED_BUILTIN);
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

    if(pinIn(SD_CD_PIN) == 0)
        power_on();

    sdcard_init();
#endif

    return IOInitDone;
}

// EEPROM emulation - stores settings in flash
// Note: settings will not survive a reflash unless protected

typedef struct {
    void *addr;
    uint16_t row_size;
    uint16_t page_size;
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
    uint8_t *row = (uint8_t *)grblNVS.addr;
    uint32_t size = hal.nvs.size;

    // Erase flash pages
    do {
        NVMCTRL->ADDR.reg = ((uint32_t)row) / 2;
        NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY|NVMCTRL_CTRLA_CMD_ER;
        while(!NVMCTRL->INTFLAG.bit.READY);
        row += grblNVS.row_size;
        size -= grblNVS.row_size;
    } while(size);
    
    uint32_t *dest = (uint32_t *)grblNVS.addr, *src = (uint32_t *)source, words;

    size = GRBL_NVS_SIZE;
    NVMCTRL->CTRLB.bit.MANW = 1;

    // Clear page buffer
    NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY|NVMCTRL_CTRLA_CMD_PBC;
    while(!NVMCTRL->INTFLAG.bit.READY);

    while(size) {

        // Fill page buffer
        words = grblNVS.page_size / sizeof(uint32_t);
        do {
            *dest++ = *src++;
        } while(--words);

        // Write page buffer to flash
        NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY|NVMCTRL_CTRLA_CMD_WP;
        while(!NVMCTRL->INTFLAG.bit.READY);
        
        size -= grblNVS.page_size;
    }
        
    return true;
}

bool nvsInit (void)
{
    grblNVS.page_size = 8 << NVMCTRL->PARAM.bit.PSZ;
    grblNVS.row_size = grblNVS.page_size * 4;
    grblNVS.addr = (void *)(NVMCTRL->PARAM.bit.NVMP * grblNVS.page_size - GRBL_NVS_SIZE);

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
bool driver_init (void) {

    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

    init(); // system init (wiring.h)

    // Copy vector table to RAM so we can override the default Arduino IRQ assignments

    __disable_irq();

    memcpy(&vectorTable, (void *)SCB->VTOR, sizeof(vectorTable));

    SCB->VTOR = (uint32_t)&vectorTable & SCB_VTOR_TBLOFF_Msk;
    __DSB();
    __enable_irq();

    // End vector table copy

    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk;
    NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    IRQRegister(SysTick_IRQn, SysTick_IRQHandler);

    hal.info = "SAMD21";
    hal.driver_version = "210206";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.driver_setup = driver_setup;
    hal.f_step_timer = SystemCoreClock / 3;
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

#ifdef PROBE_PIN
    hal.probe.configure = probeConfigure;
    hal.probe.get_state = probeGetState;
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

#if I2C_ENABLE
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
    grbl.on_execute_realtime = execute_realtime;
#endif

#ifdef DEBUGOUT
    hal.debug_out = debug_out;
#endif

 // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality
#if defined(ENABLE_SAFETY_DOOR_INPUT_PIN) && defined(SAFETY_DOOR_PIN)
    hal.signals_cap.safety_door_ajar = On;
#endif

#ifdef SPINDLE_DIRECTION_PIN
    hal.driver_cap.spindle_dir = On;
#endif
    hal.driver_cap.variable_spindle = On;
    hal.driver_cap.mist_control = On;
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

    my_plugin_init();

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 8;
}

/* interrupt handlers */

// Main stepper driver
static void STEPPER_IRQHandler (void)
{
    STEPPER_TIMER->COUNT32.INTFLAG.bit.MC0 = 1;
    hal.stepper.interrupt_callback();
}

// Step pulse handler
static void STEPPULSE_IRQHandler (void)
{
    STEP_TIMER->COUNT16.INTFLAG.bit.MC0 = 1;
    set_step_outputs((axes_signals_t){0}); // End step pulse.
}

// Will only be called if AMASS is not used
static void STEPPULSE_Delayed_IRQHandler (void)
{
    STEP_TIMER->COUNT16.INTFLAG.bit.MC0 = 1;
    STEP_TIMER->COUNT16.CC[0].reg = pulse_length;

    set_step_outputs(next_step_outbits);

    STEP_TIMER->COUNT16.COUNT.reg = 0;
    while(STEP_TIMER->COUNT16.STATUS.bit.SYNCBUSY);

    IRQRegister(STEP_TIMER_IRQn, STEPPULSE_IRQHandler);

    STEP_TIMER->COUNT16.CTRLBSET.reg = TC_CTRLBCLR_CMD_RETRIGGER|TCC_CTRLBSET_ONESHOT;
}

static void DEBOUNCE_IRQHandler (void)
{
    DEBOUNCE_TIMER->INTFLAG.bit.OVF = 1;
#if SDCARD_ENABLE__NOT_WORKING // See comment above in driver_setup()
    if(sd_detect) {
        sd_detect = false;
        if(pinIn(SD_CD_PIN) == 0)
            power_on();
        else {
            BYTE pwr = 0;
            disk_ioctl(0, CTRL_POWER, &pwr);
        }
    } else {
        limit_signals_t state = limitsGetState();

        if(limit_signals_merge(state).value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
            hal.limit_interrupt_callback(state);
    }
#else
    limit_signals_t state = limitsGetState();
    if(limit_signals_merge(state).value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
        hal.limits.interrupt_callback(state);
#endif
}

static void CONTROL_IRQHandler (void)
{
    hal.control.interrupt_callback(systemGetState());
}

static void LIMIT_IRQHandler (void)
{
    if(hal.driver_cap.software_debounce) {
        DEBOUNCE_TIMER->CTRLBSET.bit.CMD = TCC_CTRLBCLR_CMD_RETRIGGER_Val;
        while(DEBOUNCE_TIMER->SYNCBUSY.bit.CTRLB);
    } else
        hal.limits.interrupt_callback(limitsGetState());
}

static void SD_IRQHandler (void)
{
    sd_detect = true;
    DEBOUNCE_TIMER->CTRLBSET.bit.CMD = TCC_CTRLBCLR_CMD_RETRIGGER_Val;
    while(DEBOUNCE_TIMER->SYNCBUSY.bit.CTRLB);
}

#if KEYPAD_ENABLE
void KEYPAD_IRQHandler (void)
{
    keypad_keyclick_handler(pinIn(KEYPAD_PIN) == 0);
}
#endif

// Interrupt handler for 1 ms interval timer
static void SysTick_IRQHandler (void)
{
#if SDCARD_ENABLE
    static uint32_t fatfs_ticks = 10;
    if(!(--fatfs_ticks)) {
        disk_timerproc();
        fatfs_ticks = 10;
    }

    if(delay_ms.ms && !(--delay_ms.ms) && delay_ms.callback) {
        delay_ms.callback();
        delay_ms.callback = NULL;
    }
#else
    if(delay_ms.ms && !(--delay_ms.ms) && delay_ms.callback) {
        delay_ms.callback();
        delay_ms.callback = NULL;
    }
#endif
}
