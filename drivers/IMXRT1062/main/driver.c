/*
  driver.h - driver code for IMXRT1062 processor (on Teensy 4.0 board)

  Part of GrblHAL

  Copyright (c) 2020 Terje Io

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

#include "uart.h"
#include "driver.h"
#include "src/grbl/grbl.h"

#if EEPROM_ENABLE
#include "src/eeprom/eeprom.h"
#else
#include "avr/eeprom.h"
#endif

#if IOPORTS_ENABLE
#include "ioports.h"
#endif

#if QEI_ENABLE
#include "src/encoder/encoder.h"
#endif

#if USB_SERIAL_GRBL == 1
#include "usb_serial_ard.h"
#elif USB_SERIAL_GRBL == 2
#include "usb_serial_pjrc.h"
#endif

#define DEBOUNCE_QUEUE 8 // Must be a power of 2

#define F_BUS_MHZ (F_BUS_ACTUAL / 1000000)

typedef struct {
    volatile uint32_t DR;
    volatile uint32_t GDIR;
    volatile uint32_t PSR;
    volatile uint32_t ICR1;
    volatile uint32_t ICR2;
    volatile uint32_t IMR;
    volatile uint32_t ISR;
    volatile uint32_t EDGE_SEL;
    uint32_t unused[25];
    volatile uint32_t DR_SET;
    volatile uint32_t DR_CLEAR;
    volatile uint32_t DR_TOGGLE;    
} gpio_reg_t; 

typedef struct {
    gpio_reg_t *reg;
    uint32_t bit;
} gpio_t;

typedef enum {
    Input_Probe = 0,
    Input_Reset,
    Input_FeedHold,
    Input_CycleStart,
    Input_SafetyDoor,
    Input_EStop,
    Input_ModeSelect,
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
    Input_KeypadStrobe,
    Input_QEI_A,
    Input_QEI_B,
    Input_QEI_Select,
    Input_QEI_Index,
} input_t;

#define INPUT_GROUP_CONTROL    (1 << 0)
#define INPUT_GROUP_PROBE      (1 << 1)
#define INPUT_GROUP_LIMIT      (1 << 2)
#define INPUT_GROUP_KEYPAD     (1 << 3)
#define INPUT_GROUP_MPG        (1 << 4)
#define INPUT_GROUP_QEI        (1 << 5)
#define INPUT_GROUP_QEI_SELECT (1 << 6)

typedef enum {
    IRQ_Mode_None    = 0b00,
    IRQ_Mode_Change  = 0b01,
    IRQ_Mode_Rising  = 0b10,    
    IRQ_Mode_Falling = 0b11
} irq_mode_t;

typedef struct {
    input_t id;
    uint8_t group;
    uint8_t pin;
    gpio_t *port;
    gpio_t gpio; // doubled up for now for speed...
    irq_mode_t irq_mode;
    uint8_t offset;
    volatile bool active;
    volatile bool debounce;
} input_signal_t;

typedef struct {
    volatile uint_fast8_t head;
    volatile uint_fast8_t tail;
    input_signal_t *signal[DEBOUNCE_QUEUE];
} debounce_queue_t;

typedef union {
    uint_fast8_t pins;
    struct {
        uint_fast8_t a :1,
                     b :1;
    };
} qei_state_t;

typedef struct {
    encoder_t encoder;
    qei_state_t state;
    qei_state_t iflags;
    bool initial_debounce;
    volatile uint32_t debounce;
} qei_t;

static qei_t qei = {0};
static debounce_queue_t debounce_queue = {0};

// Standard inputs
static gpio_t Reset, FeedHold, CycleStart, Probe, LimitX, LimitY, LimitZ;

// Standard outputs
static gpio_t spindleEnable, spindleDir, steppersEnable, Mist, Flood, stepX, stepY, stepZ, dirX, dirY, dirZ;

// Optional I/O
#ifdef SAFETY_DOOR_PIN
static gpio_t SafetyDoor;
#endif
#ifdef A_AXIS
static gpio_t stepA, dirA, LimitA;
#endif
#ifdef B_AXIS
static gpio_t stepB, dirB, LimitB;
#endif
#ifdef STEPPERS_ENABLE_Y_PIN
static gpio_t steppersEnableY;
#endif
#ifdef STEPPERS_ENABLE_Z_PIN
static gpio_t steppersEnableZ;
#endif
#ifdef STEPPERS_ENABLE_A_PIN
static gpio_t steppersEnableA;
#endif
#ifdef STEPPERS_ENABLE_B_PIN
static gpio_t steppersEnableB;
#endif
#if KEYPAD_ENABLE
static gpio_t KeypadStrobe;
#endif
#if MPG_MODE_ENABLE
static gpio_t ModeSelect;
#endif
#if QEI_ENABLE
static gpio_t QEI_A, QEI_B;
 #ifdef QEI_SELECT_PIN
  #define QEI_SELECT_ENABLED 1
  static gpio_t QEI_Select;
 #endif
 #ifdef QEI_INDEX_PIN
  #define QEI_INDEX_ENABLED 1
  static gpio_t QEI_Index;
 #endif
#endif

static input_signal_t inputpin[] = {
#if ESTOP_ENABLE
    { .id = Input_EStop,        .port = &Reset,        .pin = RESET_PIN,       .group = INPUT_GROUP_CONTROL },
#else
    { .id = Input_Reset,        .port = &Reset,        .pin = RESET_PIN,       .group = INPUT_GROUP_CONTROL },
#endif
    { .id = Input_FeedHold,     .port = &FeedHold,     .pin = FEED_HOLD_PIN,   .group = INPUT_GROUP_CONTROL },
    { .id = Input_CycleStart,   .port = &CycleStart,   .pin = CYCLE_START_PIN, .group = INPUT_GROUP_CONTROL },
#ifdef SAFETY_DOOR_PIN
    { .id = Input_SafetyDoor,   .port = &SafetyDoor ,  .pin = SAFETY_DOOR_PIN, .group = INPUT_GROUP_CONTROL },
#endif
    { .id = Input_Probe,        .port = &Probe,        .pin = PROBE_PIN,       .group = INPUT_GROUP_PROBE },
    { .id = Input_LimitX,       .port = &LimitX,       .pin = X_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT },
    { .id = Input_LimitY,       .port = &LimitY,       .pin = Y_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT },
    { .id = Input_LimitZ,       .port = &LimitZ,       .pin = Z_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT }
#ifdef A_LIMIT_PIN
  , { .id = Input_LimitA,       .port = &LimitA,       .pin = A_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT }
#endif
#ifdef B_LIMIT_PIN
  , { .id = Input_LimitB,       .port = &LimitB,       .pin = B_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT }
#endif
#if MPG_MODE_ENABLE
  ,  { .id = Input_ModeSelect,  .port = &ModeSelect,   .pin = MODE_PIN,        .group = INPUT_GROUP_MPG }
#endif
#if KEYPAD_ENABLE
  , { .id = Input_KeypadStrobe, .port = &KeypadStrobe, .pin = KEYPAD_PIN,      .group = INPUT_GROUP_KEYPAD }
#endif
#if QEI_ENABLE
  , { .id = Input_QEI_A,        .port = &QEI_A,        .pin = QEI_A_PIN,       .group = INPUT_GROUP_QEI }
  , { .id = Input_QEI_B,        .port = &QEI_B,        .pin = QEI_B_PIN,       .group = INPUT_GROUP_QEI }
  #if QEI_SELECT_ENABLED
  , { .id = Input_QEI_Select,   .port = &QEI_Select,   .pin = QEI_SELECT_PIN,  .group = INPUT_GROUP_QEI_SELECT }
  #endif
  #if QEI_INDEX_ENABLED
  , { .id = Input_QEI_Index,    .port = &QEI_Index,    .pin = QEI_INDEX_PIN,   .group = INPUT_GROUP_QEI }
  #endif
#endif
};

#define DIGITAL_OUT(gpio, on) { if(on) gpio.reg->DR_SET = gpio.bit; else gpio.reg->DR_CLEAR = gpio.bit; } 

static bool pwmEnabled = false, IOInitDone = false, probe_invert = false;
static axes_signals_t next_step_outbits;
static spindle_pwm_t spindle_pwm;
static delay_t grbl_delay = { .ms = 0, .callback = NULL };

static void spindle_set_speed (uint_fast16_t pwm_value);

// Interrupt handler prototypes
// Interrupt handlers needs to be registered, possibly by modifying a system specific startup file.
// It is possible to relocate the interrupt dispatch table from flash to RAM and programatically attach handlers.
// See the driver for SAMD21 for an example, relocation is done in the driver_init() function.
// Also, if a MCU specific driver library is used this might have functions to programatically attach handlers.

static void stepper_driver_isr (void);
static void stepper_pulse_isr (void);
static void stepper_pulse_isr_delayed (void);
static void gpio_isr (void);
static void debounce_isr (void);
static void systick_isr (void);

static void (*systick_isr_org)(void) = NULL;

// Millisecond resolution delay function
// Will return immediately if a callback function is provided
static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if(ms) {
        grbl_delay.ms = ms;
        if(!(grbl_delay.callback = callback))
            while(grbl_delay.ms);
    } else {
        if(grbl_delay.ms) {
            grbl_delay.callback = NULL;
            grbl_delay.ms = 1;
        }
        if(callback)
            callback();
    }
}

// Set stepper pulse output pins.
// step_outbits.value (or step_outbits.mask) are: bit0 -> X, bit1 -> Y...
// Individual step bits can be accessed by step_outbits.x, step_outbits.y, ...
inline static void set_step_outputs (axes_signals_t step_outbits)
{
    step_outbits.value ^= settings.steppers.step_invert.mask;

    DIGITAL_OUT(stepX, step_outbits.x);
    DIGITAL_OUT(stepY, step_outbits.y);
    DIGITAL_OUT(stepZ, step_outbits.z);
#ifdef A_AXIS
    DIGITAL_OUT(stepA, step_outbits.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(stepB, step_outbits.b);
#endif
}

// Set stepper direction ouput pins.
// dir_outbits.value (or dir_outbits.mask) are: bit0 -> X, bit1 -> Y...
// Individual direction bits can be accessed by dir_outbits.x, dir_outbits.y, ...
inline static void set_dir_outputs (axes_signals_t dir_outbits)
{
    dir_outbits.value ^= settings.steppers.dir_invert.mask;

    DIGITAL_OUT(dirX, dir_outbits.x);
    DIGITAL_OUT(dirY, dir_outbits.y);
    DIGITAL_OUT(dirZ, dir_outbits.z);
#ifdef A_AXIS
    DIGITAL_OUT(dirA, dir_outbits.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(dirB, dir_outbits.b);
#endif
}

// Enable steppers.
// enable.value (or enable.mask) are: bit0 -> X, bit1 -> Y...
// Individual enable bits can be accessed by enable.x, enable.y, ...
// NOTE: if a common signal is used to enable all drivers enable.x should be used to set the signal.
static void stepperEnable (axes_signals_t enable)
{
    enable.value ^= settings.steppers.enable_invert.mask;

    DIGITAL_OUT(steppersEnable, enable.x)
#ifdef STEPPERS_ENABLE_Y_PIN
    DIGITAL_OUT(steppersEnableY, enable.y)
#endif
#ifdef STEPPERS_ENABLE_Z_PIN
    DIGITAL_OUT(steppersEnableZ, enable.z)
#endif
#ifdef STEPPERS_ENABLE_A_PIN
    DIGITAL_OUT(steppersEnableA, enable.a)
#endif
#ifdef STEPPERS_ENABLE_B_PIN
    DIGITAL_OUT(steppersEnableB, enable.b)
#endif
}

// Starts stepper driver timer and forces a stepper driver interrupt callback.
static void stepperWakeUp (void)
{
    // Enable stepper drivers.
    stepperEnable((axes_signals_t){AXES_BITMASK});

    PIT_LDVAL0 = 5000;
    PIT_TFLG0 |= PIT_TFLG_TIF;
    PIT_TCTRL0 |= (PIT_TCTRL_TIE|PIT_TCTRL_TEN);
}

// Disables stepper driver interrupts and reset outputs.
static void stepperGoIdle (bool clear_signals)
{
    PIT_TCTRL0 &= ~(PIT_TCTRL_TIE|PIT_TCTRL_TEN);

    if(clear_signals) {
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}

// Sets up stepper driver interrupt timeout.
// Called at the start of each segment.
// NOTE: If a 32-bit timer is used it is advisable to limit max step time to about 2 seconds
//       in order to avoid excessive delays on completion of motions
// NOTE: If a 16 bit timer is used it may be neccesary to adjust the timer clock frequency (prescaler)
//       to cover the needed range. Refer to actual drivers for code examples.
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    PIT_TCTRL0 &= ~PIT_TCTRL_TEN;
    PIT_LDVAL0 = cycles_per_tick < (1UL << 20) ? cycles_per_tick : 0x000FFFFFUL;
    PIT_TFLG0 |= PIT_TFLG_TIF;
    PIT_TCTRL0 |= PIT_TCTRL_TEN;
}

// Start a stepper pulse, no delay version
// stepper_t struct is defined in grbl/stepper.h
static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->new_block) {
        stepper->new_block = false;
        set_dir_outputs(stepper->dir_outbits);
    }

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        TMR4_CTRL0 |= TMR_CTRL_CM(0b001);
    }
}

// Start a stepper pulse, delay version
// stepper_t struct is defined in grbl/stepper.h
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    if(stepper->new_block) {
        stepper->new_block = false;
        set_dir_outputs(stepper->dir_outbits);
    }

    if(stepper->step_outbits.value) {
        next_step_outbits = stepper->step_outbits; // Store out_bits
        TMR4_CTRL0 |= TMR_CTRL_CM(0b001);
    }
}

// Enable/disable limit pins interrupt.
// NOTE: the homing parameter is indended for configuring advanced
//        stepper drivers for sensorless homing.
static void limitsEnable (bool on, bool homing)
{
    uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);

    on &= settings.limits.flags.hard_enabled;

    do {
        if(inputpin[--i].group == INPUT_GROUP_LIMIT) {
            inputpin[i].gpio.reg->ISR = inputpin[i].gpio.bit;       // Clear interrupt.
            if(on)
                inputpin[i].gpio.reg->IMR |= inputpin[i].gpio.bit;  // Enable interrupt.
            else    
                inputpin[i].gpio.reg->IMR &= ~inputpin[i].gpio.bit; // Disable interrupt.
        } 
    } while(i);
}

// Returns limit state as an axes_signals_t bitmap variable.
// signals.value (or signals.mask) are: bit0 -> X, bit1 -> Y...
// Individual signals bits can be accessed by signals.x, signals.y, ...
// Each bit indicates a limit signal, where triggered is 1 and not triggered is 0.
// axes_signals_t is defined in grbl/nuts_bolts.h.
inline static axes_signals_t limitsGetState()
{
    axes_signals_t signals = {0};

    signals.x = (LimitX.reg->DR & LimitX.bit) != 0;
    signals.y = (LimitY.reg->DR & LimitY.bit) != 0;
    signals.z = (LimitZ.reg->DR & LimitZ.bit) != 0;
#ifdef A_LIMIT_PIN
    signals.a = (LimitA.reg->DR & LimitA.bit) != 0;
#endif
#ifdef B_LIMIT_PIN
    signals.b = (LimitB.reg->DR & LimitB.bit) != 0;
#endif

    if (settings.limits.invert.mask)
        signals.value ^= settings.limits.invert.mask;

    return signals;
}

// Returns system state as a control_signals_t bitmap variable.
// signals.value (or signals.mask) are: bit0 -> reset, bit1 -> feed_hold, ...
// Individual enable bits can be accessed by signals.reset, signals.feed_hold, ...
// Each bit indicates a control signal, where triggered is 1 and not triggered is 0.
// axes_signals_t is defined in grbl/system.h.
inline static control_signals_t systemGetState (void)
{
    control_signals_t signals = {0};

#if ESTOP_ENABLE
    signals.e_stop = (Reset.reg->DR & Reset.bit) != 0;
#else
    signals.reset = (Reset.reg->DR & Reset.bit) != 0;
#endif
    signals.feed_hold = (FeedHold.reg->DR & FeedHold.bit) != 0;
    signals.cycle_start = (CycleStart.reg->DR & CycleStart.bit) != 0;
#ifdef SAFETY_DOOR_PIN
    signals.safety_door_ajar = (SafetyDoor.reg->DR & SafetyDoor.bit) != 0;
#endif

    if(settings.control_invert.value)
        signals.value ^= settings.control_invert.value;

    return signals;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away)
{
    probe_invert = !settings.flags.invert_probe_pin;

    if (is_probe_away)
        probe_invert = !probe_invert;
}

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {
        .connected = On
    };

    state.triggered = ((Probe.reg->DR & Probe.bit) != 0) ^ probe_invert;

    return state;
}

// Static spindle (off, on cw & on ccw)

inline static void spindle_off ()
{
    DIGITAL_OUT(spindleEnable, settings.spindle.invert.on);
}

inline static void spindle_on ()
{
    DIGITAL_OUT(spindleEnable, !settings.spindle.invert.on);
}

inline static void spindle_dir (bool ccw)
{
    DIGITAL_OUT(spindleDir, ccw ^ settings.spindle.invert.ccw);
}

// Start or stop spindle.
static void spindleSetState (spindle_state_t state, float rpm)
{
    if (!state.on)
        spindle_off();
    else {
        if(hal.driver_cap.spindle_dir)
            spindle_dir(state.ccw);
        spindle_on();
    }
}

// Variable spindle control functions

// Set spindle speed.
static void spindle_set_speed (uint_fast16_t pwm_value)
{
    if (pwm_value == spindle_pwm.off_value) {
        if(settings.spindle.disable_with_zero_speed)
            spindle_off();
        pwmEnabled = false;
        if(spindle_pwm.always_on) {
#if SPINDLEPWMPIN == 12
            TMR1_COMP21 = spindle_pwm.off_value;
            TMR1_CMPLD11 = spindle_pwm.period - spindle_pwm.off_value;
            TMR1_CTRL1 |= TMR_CTRL_CM(0b001);
#else // 13
            TMR2_COMP20 = spindle_pwm.off_value;
            TMR2_CMPLD10 = spindle_pwm.period - spindle_pwm.off_value;
            TMR2_CTRL0 |= TMR_CTRL_CM(0b001);
#endif

        } else {
#if SPINDLEPWMPIN == 12
            TMR1_CTRL1 &= ~TMR_CTRL_CM(0b111);
            TMR1_SCTRL1 &= ~TMR_SCTRL_VAL; //  set TMR_SCTRL_VAL || TMR_SCTRL_OPS if inverted PWM
            TMR1_SCTRL1 |= TMR_SCTRL_FORCE;
#else // 13
            TMR2_CTRL0 &= ~TMR_CTRL_CM(0b111);
            TMR2_SCTRL0 &= ~TMR_SCTRL_VAL; //  set TMR_SCTRL_VAL || TMR_SCTRL_OPS if inverted PWM
            TMR2_SCTRL0 |= TMR_SCTRL_FORCE;
#endif
        }
     } else {
        if(!pwmEnabled) {
            spindle_on();
            pwmEnabled = true;
        }
#if SPINDLEPWMPIN == 12
        TMR1_COMP21 = pwm_value;
        TMR1_CMPLD11 = spindle_pwm.period - pwm_value;
        TMR1_CTRL1 |= TMR_CTRL_CM(0b001);
#else // 13
        TMR2_COMP20 = pwm_value;
        TMR2_CMPLD10 = spindle_pwm.period - pwm_value;
        TMR2_CTRL0 |= TMR_CTRL_CM(0b001);
#endif
    }
}

#ifdef SPINDLE_PWM_DIRECT

// Convert spindle speed to PWM value.
static uint_fast16_t spindleGetPWM (float rpm)
{
    return spindle_compute_pwm_value(&spindle_pwm, rpm, false);
}

#else

// Update spindle speed.
static void spindleUpdateRPM (float rpm)
{
    spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
}

#endif

// Start or stop spindle.
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

// Returns spindle state in a spindle_state_t variable.
// spindle_state_t is defined in grbl/spindle_control.h
static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {0};

    state.on = (spindleEnable.reg->DR & spindleEnable.bit) != 0;

    if(hal.driver_cap.spindle_dir)
        state.ccw = (spindleDir.reg->DR & spindleDir.bit) != 0;
 
    state.value ^= settings.spindle.invert.mask;
    if(pwmEnabled)
        state.on |= pwmEnabled;

    state.value ^= settings.spindle.invert.mask;

    return state;
}

// end spindle code

// Start/stop coolant (and mist if enabled).
// coolant_state_t is defined in grbl/coolant_control.h.
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;

    DIGITAL_OUT(Flood, mode.flood);
    DIGITAL_OUT(Mist, mode.mist);
}

// Returns coolant state in a coolant_state_t variable.
// coolant_state_t is defined in grbl/coolant_control.h.
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

    state.flood = (Flood.reg->DR & Flood.bit) != 0;
    state.mist = (Mist.reg->DR & Mist.bit) != 0;
 
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

// Configures perhipherals when settings are initialized or changed
static void settings_changed (settings_t *settings)
{
    if(IOInitDone) {

        stepperEnable(settings->steppers.deenergize);

        if(hal.driver_cap.variable_spindle && spindle_precompute_pwm_values(&spindle_pwm, F_BUS_ACTUAL / 2)) {
#if SPINDLEPWMPIN == 12
            TMR1_COMP11 = spindle_pwm.period;
            TMR1_CMPLD11 = spindle_pwm.period;
#else // 13
            TMR2_COMP10 = spindle_pwm.period;
            TMR2_CMPLD10 = spindle_pwm.period;
#endif
            hal.spindle_set_state = spindleSetStateVariable;
        } else
            hal.spindle_set_state = spindleSetState;

        // Stepper pulse timeout setup.
        // When the stepper pulse is delayed either two timers or a timer that supports multiple
        // compare registers is required.
        TMR4_CSCTRL0 &= ~(TMR_CSCTRL_TCF1|TMR_CSCTRL_TCF2);
        if(settings->steppers.pulse_delay_microseconds) {
            TMR4_COMP10 = F_BUS_MHZ * settings->steppers.pulse_delay_microseconds;
            TMR4_COMP20 = F_BUS_MHZ * settings->steppers.pulse_microseconds;
            TMR4_CSCTRL0 |= TMR_CSCTRL_TCF2EN;
            TMR4_CTRL0 |= TMR_CTRL_OUTMODE(0b100);
            hal.stepper_pulse_start = stepperPulseStartDelayed;
            attachInterruptVector(IRQ_QTIMER4, stepper_pulse_isr_delayed);
        } else {
            hal.stepper_pulse_start = stepperPulseStart;
            TMR4_COMP10 = F_BUS_MHZ * settings->steppers.pulse_microseconds;
            TMR4_CSCTRL0 &= ~TMR_CSCTRL_TCF2EN;
            TMR4_CTRL0 &= ~TMR_CTRL_OUTMODE(0b000);
            attachInterruptVector(IRQ_QTIMER4, stepper_pulse_isr);
        }

        /****************************************
         *  Control, limit & probe pins config  *
         ****************************************/

        control_signals_t control_fei;
        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        bool pullup;
        uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);
        input_signal_t *signal;

        NVIC_DISABLE_IRQ(IRQ_GPIO6789);

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

                case Input_SafetyDoor:
                    pullup = !settings->control_disable_pullup.safety_door_ajar;
                    signal->debounce = hal.driver_cap.software_debounce;
                    signal->irq_mode = control_fei.safety_door_ajar ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_Probe:
                    pullup = hal.driver_cap.probe_pull_up;
                    break;

                case Input_LimitX:
                    pullup = !settings->limits.disable_pullup.x;
                    signal->debounce = hal.driver_cap.software_debounce;
                    signal->irq_mode = limit_fei.x ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitY:
                    pullup = !settings->limits.disable_pullup.y;
                    signal->irq_mode = limit_fei.y ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitZ:
                    pullup = !settings->limits.disable_pullup.z;
                    signal->irq_mode = limit_fei.z ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#ifdef A_LIMIT_PIN
                case Input_LimitA:
                    pullup = !settings->limits.disable_pullup.a;
                    signal->irq_mode = limit_fei.a ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#ifdef B_LIMIT_PIN
                case Input_LimitB:
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
                    signal->irq_mode = IRQ_Mode_Change;
                    break;
#endif
#if QEI_ENABLE
                case Input_QEI_A:
                    signal->irq_mode = IRQ_Mode_Change;
                    break;

                case Input_QEI_B:
                    signal->irq_mode = IRQ_Mode_Change;
                    break;

  #if QEI_INDEX_ENABLED
                case Input_QEI_Index:
                    signal->irq_mode = IRQ_Mode_None;
                    break;
  #endif

  #if QEI_SELECT_ENABLED
                case Input_QEI_Select:
                    signal->debounce = hal.driver_cap.software_debounce;
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

        NVIC_ENABLE_IRQ(IRQ_GPIO6789);
    }
}

void pinModeOutput (gpio_t *gpio, uint8_t pin)
{
    pinMode(pin, OUTPUT);
    gpio->reg = (gpio_reg_t *)digital_pin_to_info_PGM[pin].reg;
    gpio->bit = digital_pin_to_info_PGM[pin].mask;
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
#ifdef DRIVER_SETTINGS
    if(hal.eeprom.driver_area.address != 0) {
        if(!hal.eeprom.memcpy_from_with_checksum((uint8_t *)&driver_settings, hal.eeprom.driver_area.address, sizeof(driver_settings)))
            hal.driver_settings_restore();
      #if TRINAMIC_ENABLE && CNC_BOOSTERPACK // Trinamic BoosterPack does not support mixed drivers
        driver_settings.trinamic.driver_enable.mask = AXES_BITMASK;
      #endif
    }
#endif

    /******************
     *  Stepper init  *
     ******************/

    PIT_MCR = 0x00;
	CCM_CCGR1 |= CCM_CCGR1_PIT(CCM_CCGR_ON);

	attachInterruptVector(IRQ_PIT, stepper_driver_isr);
	NVIC_SET_PRIORITY(IRQ_PIT, 2);
	NVIC_ENABLE_IRQ(IRQ_PIT);

    TMR4_ENBL = 0;
    TMR4_LOAD0 = 0;
    TMR4_CTRL0 = TMR_CTRL_PCS(0b1000) | TMR_CTRL_ONCE | TMR_CTRL_LENGTH;
    TMR4_CSCTRL0 = TMR_CSCTRL_TCF1EN;

	attachInterruptVector(IRQ_QTIMER4, stepper_pulse_isr);
	NVIC_SET_PRIORITY(IRQ_QTIMER4, 0);
	NVIC_ENABLE_IRQ(IRQ_QTIMER4);

    TMR4_ENBL = 1;

    pinModeOutput(&stepX, X_STEP_PIN);
    pinModeOutput(&stepY, Y_STEP_PIN);
    pinModeOutput(&stepZ, Z_STEP_PIN);

    pinModeOutput(&dirX, X_DIRECTION_PIN);
    pinModeOutput(&dirY, Y_DIRECTION_PIN);
    pinModeOutput(&dirZ, Z_DIRECTION_PIN);

#ifdef A_AXIS
    pinModeOutput(&stepA, A_STEP_PIN);
    pinModeOutput(&dirA, A_DIRECTION_PIN);
#endif

#ifdef B_AXIS
    pinModeOutput(&stepB, B_STEP_PIN);
    pinModeOutput(&dirB, B_DIRECTION_PIN);
#endif

    pinModeOutput(&steppersEnable, STEPPERS_ENABLE_PIN);
#ifdef STEPPERS_ENABLE_Y_PIN
    pinModeOutput(&steppersEnableY, STEPPERS_ENABLE_Y_PIN);
#endif
#ifdef STEPPERS_ENABLE_Z_PIN
    pinModeOutput(&steppersEnableZ, STEPPERS_ENABLE_Z_PIN);
#endif
#ifdef STEPPERS_ENABLE_A_PIN
    pinModeOutput(&steppersEnableA, STEPPERS_ENABLE_A_PIN);
#endif
#ifdef STEPPERS_ENABLE_B_PIN
    pinModeOutput(&steppersEnableB, STEPPERS_ENABLE_B_PIN);
#endif

   /****************************
    *  Software debounce init  *
    ****************************/

    if(hal.driver_cap.software_debounce) {

        TMR3_ENBL = 0;
        TMR3_LOAD0 = 0;
        TMR3_CTRL0 = TMR_CTRL_PCS(0b1111) | TMR_CTRL_ONCE | TMR_CTRL_LENGTH;
        TMR3_COMP10 = (40000 * 128) / F_BUS_MHZ; // 150 MHz -> 40ms
        TMR3_CSCTRL0 = TMR_CSCTRL_TCF1EN;

        attachInterruptVector(IRQ_QTIMER3, debounce_isr);
        NVIC_SET_PRIORITY(IRQ_QTIMER3, 4);
        NVIC_ENABLE_IRQ(IRQ_QTIMER3);

        TMR3_ENBL = 1;
    }

   /***********************
    *  Control pins init  *
    ***********************/

	attachInterruptVector(IRQ_GPIO6789, gpio_isr);

   /***********************
    *  Coolant pins init  *
    ***********************/

    pinModeOutput(&Flood, COOLANT_FLOOD_PIN);
    pinModeOutput(&Mist, COOLANT_MIST_PIN);

   /******************
    *  Spindle init  *
    ******************/

    pinModeOutput(&spindleEnable, SPINDLE_ENABLE_PIN);
    pinModeOutput(&spindleDir, SPINDLE_DIRECTION_PIN);

#if SPINDLEPWMPIN == 12
    TMR1_ENBL = 0;
    TMR1_LOAD1 = 0;
    TMR1_CTRL1 = TMR_CTRL_PCS(0b1001) | TMR_CTRL_OUTMODE(0b100) | TMR_CTRL_LENGTH;
    TMR1_SCTRL1 = TMR_SCTRL_OEN | TMR_SCTRL_FORCE; //  set TMR_SCTRL_VAL || TMR_SCTRL_OPS if inverted PWM
    TMR1_ENBL = 1 << 1;
#else // 13
    TMR2_ENBL = 0;
    TMR2_LOAD0 = 0;
    TMR2_CTRL0 = TMR_CTRL_PCS(0b1001) | TMR_CTRL_OUTMODE(0b100) | TMR_CTRL_LENGTH;
    TMR2_SCTRL0 = TMR_SCTRL_OEN | TMR_SCTRL_FORCE; //  set TMR_SCTRL_VAL || TMR_SCTRL_OPS if inverted PWM
    TMR2_ENBL = 1;
#endif

    *(portConfigRegister(SPINDLEPWMPIN)) = 1;

  // Set defaults

    IOInitDone = settings->version == 16;

    settings_changed(settings);

    hal.stepper_go_idle(true);
    hal.spindle_set_state((spindle_state_t){0}, 0.0f);
    hal.coolant_set_state((coolant_state_t){0});

#if IOPORTS_ENABLE
    ioports_init();
#endif

    return IOInitDone;
}

#if EEPROM_ENABLE == 0

// EEPROM emulation - stores settings in flash

bool nvsRead (uint8_t *dest)
{
// assert size ? E2END

    eeprom_read_block(dest, 0, hal.eeprom.size);

    return true; //?;
}

bool nvsWrite (uint8_t *source)
{
    eeprom_write_block(source, 0, hal.eeprom.size);

    return true; //?;
}

// End EEPROM emulation

#endif

#if QEI_SELECT_ENABLED || KEYPAD_ENABLE || USB_SERIAL_GRBL > 0
static void execute_realtime (uint_fast16_t state)
{
#if USB_SERIAL_GRBL > 0
    usb_execute_realtime(state);
#endif
#if KEYPAD_ENABLE
    keypad_process_keypress(state);
#endif
#if QEI_SELECT_ENABLED
    encoder_execute_realtime(state);
#endif
}
#endif

#ifdef DEBUGOUT
void debugOut (bool on)
{
    digitalWrite(13, on); // LED
}
#endif

// Initialize HAL pointers, setup serial comms and enable EEPROM.
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done.
bool driver_init (void)
{
    static char options[30];

    // Chain our systick isr to the Arduino handler

    if(systick_isr_org == NULL) 
        systick_isr_org = _VectorsRam[15];
    _VectorsRam[15] = systick_isr;

    // Enable lazy stacking of FPU registers here if a FPU is available.

 //   FPU->FPCCR = (FPU->FPCCR & ~FPU_FPCCR_LSPEN_Msk) | FPU_FPCCR_ASPEN_Msk;  // enable lazy stacking

    options[0] = '\0';

#if USB_SERIAL_GRBL == 1
    strcat(options, "USB.1 ");
#endif
#if USB_SERIAL_GRBL == 2
    strcat(options, "USB.2 ");
#endif
#if KEYPAD
    strcat(options, "KEYPAD ");
#endif
#if IOPORTS_ENABLE
    strcat(options, "IOPORTS ");
#endif
    if(*options != '\0')
        options[strlen(options) - 1] = '\0';

    hal.info = "IMXRT1062";
    hal.driver_version = "200606";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.driver_options = *options == '\0' ? NULL : options;
    hal.driver_setup = driver_setup;
    hal.f_step_timer = 24000000;
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

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
    hal.probe_configure_invert_mask = probeConfigure;

    hal.spindle_set_state = spindleSetState;
    hal.spindle_get_state = spindleGetState;
#ifdef SPINDLE_PWM_DIRECT
    hal.spindle_get_pwm = spindleGetPWM;
    hal.spindle_update_pwm = spindle_set_speed;
#else
    hal.spindle_update_rpm = spindleUpdateRPM;
#endif

    hal.system_control_get_state = systemGetState;

#if USB_SERIAL_GRBL
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
    hal.stream.write = serialWriteS;
    hal.stream.write_all = serialWriteS;
    hal.stream.get_rx_buffer_available = serialRxFree;
    hal.stream.reset_read_buffer = serialRxFlush;
    hal.stream.cancel_read_buffer = serialRxCancel;
    hal.stream.suspend_read = serialSuspendInput;
#endif

#if EEPROM_ENABLE
    eepromInit(); 
    hal.eeprom.type = EEPROM_Physical;
    hal.eeprom.get_byte = eepromGetByte;
    hal.eeprom.put_byte = eepromPutByte;
    hal.eeprom.memcpy_to_with_checksum = eepromWriteBlockWithChecksum;
    hal.eeprom.memcpy_from_with_checksum = eepromReadBlockWithChecksum;
#else // use Arduino emulated EEPROM in flash
    eeprom_initialize();
    hal.eeprom.type = EEPROM_Emulated;
    hal.eeprom.memcpy_from_flash = nvsRead;
    hal.eeprom.memcpy_to_flash = nvsWrite;
#endif

    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;

#if QEI_SELECT_ENABLED || KEYPAD_ENABLE || USB_SERIAL_GRBL > 0
    hal.execute_realtime = execute_realtime;
#endif

#if QEI_ENABLE
    hal.encoder_state_changed = encoder_changed;
#endif

#ifdef DEBUGOUT
    hal.debug_out = debugOut;
#endif

  // Driver capabilities, used for announcing and negotiating (with Grbl) driver functionality.
  // See driver_cap_t union i grbl/hal.h for available flags.

#ifdef SPINDLE_DIRECTION_PIN
    hal.driver_cap.spindle_dir = On;
#endif
    hal.driver_cap.variable_spindle = On;
#ifdef COOLANT_MIST_PIN
    hal.driver_cap.mist_control = On;
#endif
#if ESTOP_ENABLE
    hal.driver_cap.e_stop = On;
#endif
#ifdef SAFETY_DOOR_PIN
    hal.driver_cap.safety_door = On;
#endif
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 6;
}

/* interrupt handlers */

// Main stepper driver.
static void stepper_driver_isr (void)
{
    if(PIT_TFLG0 & PIT_TFLG_TIF) {
        PIT_TFLG0 |= PIT_TFLG_TIF;
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
// This interrupt is enabled when Grbl sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
static void stepper_pulse_isr (void)
{
    TMR4_CSCTRL0 &= ~TMR_CSCTRL_TCF1;

    set_step_outputs((axes_signals_t){0});
}

static void stepper_pulse_isr_delayed (void)
{
    if(TMR4_CSCTRL0 & TMR_CSCTRL_TCF1) {
        TMR4_CSCTRL0 &= ~TMR_CSCTRL_TCF1;
        set_step_outputs(next_step_outbits);
    } else {
        TMR4_CSCTRL0 &= ~TMR_CSCTRL_TCF2;
        set_step_outputs((axes_signals_t){0});
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

static void debounce_isr (void)
{
    uint8_t grp = 0;
    input_signal_t *signal;

    TMR3_CSCTRL0 &= ~TMR_CSCTRL_TCF1;

    while((signal = get_debounce())) {

        signal->gpio.reg->IMR |= signal->gpio.bit;

        if(((signal->gpio.reg->DR & signal->gpio.bit) != 0) == (signal->irq_mode == IRQ_Mode_Falling ? 0 : 1))
            grp |= signal->group;
    }

    if(grp & INPUT_GROUP_LIMIT)
        hal.limit_interrupt_callback(limitsGetState());

    if(grp & INPUT_GROUP_CONTROL)
        hal.control_interrupt_callback(systemGetState());

#if QEI_SELECT_ENABLED

    if(grp & INPUT_GROUP_QEI_SELECT) {
        qei.encoder.changed.select = On;
        hal.encoder_state_changed(&qei.encoder);
    }

#endif

}

  //GPIO intr process
static void gpio_isr (void)
{
    bool debounce = false;
    uint8_t grp = 0;
    uint32_t intr_status[4];

    // Get masked interrupt status
    intr_status[0] = ((gpio_reg_t *)&GPIO6_DR)->ISR & ((gpio_reg_t *)&GPIO6_DR)->IMR;
    intr_status[1] = ((gpio_reg_t *)&GPIO7_DR)->ISR & ((gpio_reg_t *)&GPIO7_DR)->IMR;
    intr_status[2] = ((gpio_reg_t *)&GPIO8_DR)->ISR & ((gpio_reg_t *)&GPIO8_DR)->IMR;
    intr_status[3] = ((gpio_reg_t *)&GPIO9_DR)->ISR & ((gpio_reg_t *)&GPIO9_DR)->IMR;

    // Clear interrupts
    ((gpio_reg_t *)&GPIO6_DR)->ISR = intr_status[0];
    ((gpio_reg_t *)&GPIO7_DR)->ISR = intr_status[1];
    ((gpio_reg_t *)&GPIO8_DR)->ISR = intr_status[2];
    ((gpio_reg_t *)&GPIO9_DR)->ISR = intr_status[3];

    uint32_t i = sizeof(inputpin) / sizeof(input_signal_t);
    do {
        if(inputpin[--i].irq_mode != IRQ_Mode_None) {

            if(intr_status[inputpin[i].offset] & inputpin[i].gpio.bit) {
                inputpin[i].active = true;
                if(inputpin[i].debounce && enqueue_debounce(&inputpin[i])) {
                    inputpin[i].gpio.reg->IMR &= ~inputpin[i].gpio.bit;
                    debounce = true;
                } else {
#if QEI_ENABLE
                    if(inputpin[i].group & INPUT_GROUP_QEI) {
                        QEI_A.reg->IMR &= ~QEI_A.bit;       // Switch off
                        QEI_B.reg->IMR &= ~QEI_B.bit;       // encoder interrupts.
                        qei.iflags.a = inputpin[i].port == &QEI_A;
                        qei.iflags.b = inputpin[i].port == &QEI_B;
                        qei.debounce = 1;
                        qei.initial_debounce = true;
                    } else
#endif
                    grp |= inputpin[i].group;
                }
            }
        }
    } while(i);

    if(debounce) {
        TMR3_CTRL0 |= TMR_CTRL_CM(0b001); 
    }

    if(grp & INPUT_GROUP_LIMIT)
        hal.limit_interrupt_callback(limitsGetState());

    if(grp & INPUT_GROUP_CONTROL)
        hal.control_interrupt_callback(systemGetState());

#if QEI_SELECT_ENABLED

    if(grp & INPUT_GROUP_QEI_SELECT) {
        qei.encoder.changed.select = On;
        hal.encoder_state_changed(&qei.encoder);
    }

#endif

#if MPG_MODE_ENABLE

    static bool mpg_mutex = false;

    if((grp & INPUT_GROUP_MPG) && !mpg_mutex) {
        mpg_mutex = true;
        modeChange();
        // hal.delay_ms(50, modeChange);
        mpg_mutex = false;
    }
#endif

#if KEYPAD_ENABLE
    if(grp & INPUT_GROUP_KEYPAD)
        keypad_keyclick_handler(gpio_get_level(KEYPAD_STROBE_PIN));
#endif
}

#if QEI_ENABLE

void encoder_debounce (void)
{
    qei.debounce = 1;
    qei.initial_debounce = false;
    qei.state.a = (QEI_A.reg->DR & QEI_A.bit) != 0;
    qei.state.b = (QEI_B.reg->DR & QEI_B.bit) != 0;
}

void encoder_update (void)
{
    qei_state_t state = {0};

    state.a = (QEI_A.reg->DR & QEI_A.bit) != 0;
    state.b = (QEI_B.reg->DR & QEI_B.bit) != 0;

    if(state.pins == qei.state.pins) {

        if(qei.iflags.a) {

            if(qei.state.a)
                qei.encoder.position = qei.encoder.position + (qei.state.b ? 1 : -1);
            else
                qei.encoder.position = qei.encoder.position + (qei.state.b ? -1 : 1);

            qei.encoder.changed.position = hal.encoder_state_changed != NULL;
        }

        if(qei.iflags.b) {

            if(qei.state.b)
                qei.encoder.position = qei.encoder.position + (qei.state.a ? -1 : 1);
            else
                qei.encoder.position = qei.encoder.position + (qei.state.a ? 1 : -1);

            qei.encoder.changed.position = hal.encoder_state_changed != NULL;
        }

        if(qei.encoder.changed.position)
            hal.encoder_state_changed(&qei.encoder);
    }

    // Clear and reenable encoder interrupts
    QEI_A.reg->ISR = QEI_A.bit;
    QEI_B.reg->ISR = QEI_B.bit;
    QEI_A.reg->IMR |= QEI_A.bit;
    QEI_B.reg->IMR |= QEI_B.bit;
}
#endif

// Interrupt handler for 1 ms interval timer
static void systick_isr (void)
{
#if USB_SERIAL_GRBL == 2
    systick_isr_org();
//    usb_serial_poll();
#endif

#if QEI_ENABLE
    if(qei.debounce && !(--qei.debounce)) {
        if(qei.initial_debounce)
            encoder_debounce();
        else
            encoder_update();
    }
#endif

    if(grbl_delay.ms && !(--grbl_delay.ms)) {
        if(grbl_delay.callback) {
            grbl_delay.callback();
            grbl_delay.callback = NULL;
        }
    }
}
