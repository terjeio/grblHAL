/*
  driver.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Template driver code for ARM processors

  Part of GrblHAL

  By Terje Io, public domain

*/

#include "eeprom.h"
#include "serial.h"

#include "grbl/grbl.h"

static bool pwmEnabled = false, IOInitDone = false;
static axes_signals_t next_step_outbits;
static spindle_pwm_t spindle_pwm;
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup

// Inverts the probe pin state depending on user settings and probing cycle mode.
static uint8_t probe_invert;

static void spindle_set_speed (uint_fast16_t pwm_value);

// Interrupt handler prototypes
// Interrupt handlers needs to be registered, possibly by modifying a system specific startup file.
// It is possible to relocate the interrupt dispatch table from flash to RAM and programatically attach handlers.
// See the driver for SAMD21 for an example, relocation is done in the driver_init() function.
// Also, if a MCU specific driver library is used this might have functions to programatically attach handlers.

static void stepper_driver_isr (void);
static void stepper_pulse_isr (void);
static void stepper_pulse_isr_delayed (void);
static void limit_isr (void);
static void control_isr (void);
static void systick_isr (void);


// Millisecond resolution delay function
// Will return immediately if a callback function is provided
static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if(ms) {
        delay.ms = ms;
        // SYSTICK_ENABLE(); // Enable systick timer
        if(!(delay.callback = callback))
            while(delay.ms);
    } else {
        if(delay.ms) {
            delay.callback = NULL;
            delay.ms = 1;
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

    // GPIO_WRITE(STEP_PORT, step_outbits.value); // Output step bits.
}


// Set stepper direction ouput pins.
// dir_outbits.value (or dir_outbits.mask) are: bit0 -> X, bit1 -> Y...
// Individual direction bits can be accessed by dir_outbits.x, dir_outbits.y, ...
inline static void set_dir_outputs (axes_signals_t dir_outbits)
{
    dir_outbits.value ^= settings.steppers.dir_invert.mask;

    // GPIO_WRITE(STEP_PORT, dir_outbits.value);    // Output direction bits.
}


// Enable steppers.
// enable.value (or enable.mask) are: bit0 -> X, bit1 -> Y...
// Individual enable bits can be accessed by enable.x, enable.y, ...
// NOTE: if a common signal is used to enable all drivers enable.x should be used to set the signal.
static void stepperEnable (axes_signals_t enable)
{
    enable.value ^= settings.steppers.enable_invert.mask;

    // NOTE: many drivers are enabled by a active low signal
    // GPIO_WRITE(ENABLE_PORT, enable.value);    // Output stepper enable bits.
}

// Starts stepper driver timer and forces a stepper driver interrupt callback.
static void stepperWakeUp (void)
{
    // Enable stepper drivers.
    stepperEnable((axes_signals_t){AXES_BITMASK});

    // STEPPERTIMER_LOAD(500);      // Load stepper timer with a sensible timeout value allowing stepper drivers time to wake up.
                                    // NOTE: timeout should be long enough to allow the IRQ handler time to update the load
                                    //       value before it fires again.
    // STEPPERTIMER_IRQ_ENABLE();   // Enable stepper timer IRQ.
    // STEPPERTIMER_START();        // Start stepper timer.
}

// Disables stepper driver interrupts and reset outputs.
static void stepperGoIdle (bool clear_signals)
{
    // STEPPERTIMER_IRQ_DISABLE();  // Disable stepper timer IRQ.
    // STEPPERTIMER_STOP();         // Stop stepper timer.

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
    // STEPPERTIMER_LOAD(cycles_per_tick);  // Set the stepper timer timeout time.
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
        // STEPPULSETIMER_START();        // Start step pulse timer.
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
        // STEPPULSETIMER_START();        // Start step pulse timer.
    }
}


// Enable/disable limit pins interrupt.
// NOTE: the homing parameter is indended for configuring advanced
//        stepper drivers for sensorless homing.
static void limitsEnable (bool on, bool homing)
{
//    if (on && settings.limits.flags.hard_enabled)
        // GPIO_IRQ_ENABLE(LIMITS_PORT); // Enable limit pins change interrupts.
//    else
        // GPIO_IRQ_DISABLE(LIMITS_PORT); // Disable limit pins change interrupts.
}

// Returns limit state as an axes_signals_t bitmap variable.
// signals.value (or signals.mask) are: bit0 -> X, bit1 -> Y...
// Individual signals bits can be accessed by signals.x, signals.y, ...
// Each bit indicates a limit signal, where triggered is 1 and not triggered is 0.
// axes_signals_t is defined in grbl/nuts_bolts.h.
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};

	signals.min.value = settings.limits.invert.value

    // Read limits pins, either to a temp variable or directly to signals.vaue if no remapping is required.
    // Single bits may be also assigned directly by reading them via the bit band region.
    // signals.value = GPIO_READ(LIMITS_PORT);

    if (settings.limits.invert.mask)
        signals.min.value ^= settings.limits.invert.mask;

    return signals;
}

// Returns system state as a control_signals_t bitmap variable.
// signals.value (or signals.mask) are: bit0 -> reset, bit1 -> feed_hold, ...
// Individual enable bits can be accessed by signals.reset, signals.feed_hold, ...
// Each bit indicates a control signal, where triggered is 1 and not triggered is 0.
// axes_signals_t is defined in grbl/system.h.
inline static control_signals_t systemGetState (void)
{
    control_signals_t signals = {settings.control_invert.value};

    // Read control signal pins, either to a temp variable or directly to signals.vaue if no remapping is required.
    // Single bits may be also assigned directly by reading them via the bit band region.
    // signals.value = GPIO_READ(CONTROL_PORT);

    if(settings.control_invert.value)
        signals.value ^= settings.control_invert.value;

    return signals;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
  probe_invert = settings.flags.invert_probe_pin ? 0 : PROBE_PIN;

  if (is_probe_away)
      probe_invert ^= PROBE_PIN;
}

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {
        .connected = On
    };

//    state.triggered = (GPIO_READ(PROBE_PORT, PROBE_PIN) ^ probe_invert) != 0;

    return state;
}

// Static spindle (off, on cw & on ccw)

inline static void spindle_off (void)
{
    // GPIO_WRITE(SPINDLE_PORT, SPINDLE_ENABLE_PIN) = settings.spindle.invert.on ? SPINDLE_ENABLE_PIN : 0;
}

inline static void spindle_on (void)
{
    // GPIO_WRITE(SPINDLE_PORT, SPINDLE_ENABLE_PIN) = settings.spindle.invert.on ? 0 : SPINDLE_ENABLE_PIN);
}

inline static void spindle_dir (bool ccw)
{
    // GPIO_WRITE(SPINDLE_PORT, SPINDLE_DIRECTION_PIN) = (ccw ^ settings.spindle.invert.ccw) ? SPINDLE_DIRECTION_PIN : 0);
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
        // SPINDLEPWM_TIMER_SET(spindle_pwm.off_value);
        // SPINDLEPWM_TIMER_START();
        } else {
        // SPINDLEPWM_TIMER_STOP();
        // NOTE: code may be added to ensure spindle output are at the correct level.
        }
     } else {
        if(!pwmEnabled) {
            spindle_on();
            pwmEnabled = true;
        }
        // SPINDLEPWM_TIMER_SET(pwm_value);
        // SPINDLEPWM_TIMER_START();
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

//    state.on = GPIO_READ(SPINDLE_PORT, SPINDLE_ENABLE_PIN) != 0;
//    if(hal.driver_cap.spindle_dir)
//        state.ccw = GPIO_READ((SPINDLE_PORT, SPINDLE_DIRECTION_PIN) != 0;
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

    // GPIO_WRITE(COOLANT_PORT, FLOOD_PIN) = mode.flood; // Set flood output according to mode.flood value: 0 is off, 1 is on
    // GPIO_WRITE(COOLANT_PORT, MIST_PIN) = mode.mist; // Set mist output according to mode.mist value: 0 is off, 1 is on
}

// Returns coolant state in a coolant_state_t variable.
// coolant_state_t is defined in grbl/coolant_control.h.
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

    // Read state back from GPIO pins
    // mode.flood = GPIO_READ(COOLANT_PORT, FLOOD_PIN);
    // mode.mist = GPIO_READ(COOLANT_PORT, MIST_PIN);

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
    //
    hal.driver_cap.variable_spindle = spindle_precompute_pwm_values(&spindle_pwm, 120000000UL);

    if(IOInitDone) {

        stepperEnable(settings->steppers.deenergize);

        if(hal.driver_cap.variable_spindle) {
            // Set spindle PWM timer timeout value to spindle_pwm.period here
            hal.spindle.set_state = spindleSetStateVariable;
        } else
            hal.spindle.set_state = spindleSetState;

        // Stepper pulse timeout setup.
        // When the stepper pulse is delayed either two timers or a timer that supports multiple
        // compare registers is required.
        if(settings->steppers.pulse_delay_microseconds) {
            // Configure step pulse timer(s) for delayed pulse here.
            hal.stepper.pulse_start = stepperPulseStartDelayed;
        } else {
            // Configure step pulse timer for pulse off here.
            hal.stepper.pulse_start = stepperPulseStart;
        }

       /*************************
        *  Control pins config  *
        ************************/

        // Configure pullup/pulldown and interrupt type (falling/rising edge) here.


       /***********************
        *  Limit pins config  *
        ***********************/

        // Configure pullup/pulldown and interrupt type (falling/rising edge) here.


       /********************
        *  Probe pin init  *
        ********************/

        // Configure pullup/pulldown here.
    }
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
    // System init

    // Enable peripherals to use here (if required).

    /******************
     *  Stepper init  *
     ******************/

    // Configure stepper driver timer here.

    // Configure step pulse timer here.

   /****************************
    *  Software debounce init  *
    ****************************/

    if(hal.driver_cap.software_debounce) {
        // Configure software debounce here. Additional code in GPIO IRQ handlers is required.
    }

   /***********************
    *  Control pins init  *
    ***********************/

    // Configure control pins here. Pullup/pulldown and interrupt type is set up in the settings_changed() function.

   /*********************
    *  Limit pins init  *
    *********************/

    // Configure limit pins here. Pullup/pulldown and interrupt type is set up in the settings_changed() function.

   /********************
    *  Probe pin init  *
    ********************/

    // Configure probe pin here. Pullup/pulldown is set up in the settings_changed() function.

   /***********************
    *  Coolant pins init  *
    ***********************/

    // Configure coolant pin(s) here.

   /******************
    *  Spindle init  *
    ******************/

    // Configure spindle pin(s) and spindle PWM timer here.

  // Set defaults

    IOInitDone = settings->version == 17;

    settings_changed(settings);

    hal.stepper.go_idle(true);
    hal.spindle.set_state((spindle_state_t){0}, 0.0f);
    hal.coolant.set_state((coolant_state_t){0});

    return IOInitDone;
}

// Initialize HAL pointers, setup serial comms and enable EEPROM.
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done.
bool driver_init (void)
{
    // If CMSIS is used this may be a good place to initialize the system. Comment out or remove if not available or done already.
    SystemInit();

    // Set up systick timer with a 1ms period
    // and set SysTick IRQ to lowest priority.
    // NOTE: the following code assumes CMSIS is used, if not so this has to be changed.
    NVIC_SetPriority(SysTick_IRQn, (1 << __NVIC_PRIO_BITS) - 1);

    SysTick->LOAD = (SystemCoreClock / 1000) - 1;
    SysTick->VAL = 0;
    SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_TICKINT_Msk;

    // end systick timer setup.

    // Enable EEPROM peripheral here if available.

    // Enable lazy stacking of FPU registers here if a FPU is available.

    // Enable serial communication.
    serialInit();

    hal.info = "my driver name"; // Typically set to MCU or board name
	hal.driver_version = "YYMMDD"; // Set to build date
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.driver_setup = driver_setup;
    hal.f_step_timer = SystemCoreClock; // NOTE: SystemCoreClock is a CMSIS definition, ...
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

    hal.spindle.set_state = spindleSetState;
    hal.spindle.get_state = spindleGetState;
#ifdef SPINDLE_PWM_DIRECT
    hal.spindle.get_pwm = spindleGetPWM;
    hal.spindle.update_pwm = spindle_set_speed;
#else
    hal.spindle._update_rpm = spindleUpdateRPM;
#endif

    hal.control.get_state = systemGetState;

    hal.stream.read = serialGetC;
    hal.stream.write = serialWriteS;
    hal.stream.write_all = serialWriteS;
    hal.stream.get_rx_buffer_available = serialRxFree;
    hal.stream.reset_read_buffer = serialRxFlush;
    hal.stream.cancel_read_buffer = serialRxCancel;
    hal.stream.suspend_read = serialSuspendInput;

    // If EEPROM is available for settings storage uncomment the following line:

    // eeprom_init();

    // end EEPROM available

    // Settings may also be stored in flash.
    // Quite a few drivers has code for that: SAMD21, SAM3X8E, ESP32, STM32F1xx and LPC1769
    // Check out the source for these for howto examples.
    // Note: many drivers has code examples for using external EEPROM, typically via I2C interface.

    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;

  // Driver capabilities, used for announcing and negotiating (with Grbl) driver functionality.
  // See driver_cap_t union i grbl/hal.h for available flags.

    hal.driver_cap.safety_door = On;
    hal.driver_cap.spindle_dir = On;
    hal.driver_cap.variable_spindle = On;
    hal.driver_cap.mist_control = On;
//    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;


    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 7;
}

/* interrupt handlers */

// Main stepper driver.
static void stepper_driver_isr (void)
{
    // STEPPERTIMER_IRQ_CLEAR(); // Clear stepper timer interrupt.
    hal.stepper.interrupt_callback();
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
    // STEPPULSETIMER_STOP(); // Stop step pulse timer.
    set_step_outputs((axes_signals_t){0});
}

static void stepper_pulse_isr_delayed (void)
{
    if(STEP_PULSE_ON)
        set_step_outputs(next_step_outbits);
    else {
        // STEPPULSETIMER_STOP(); // Stop step pulse timer.
        set_step_outputs((axes_signals_t){0});
    }
}

// Limit pins ISR.
static void limit_isr (void)
{
//  GPIO_IRQ_CLEAR(LIMIT_PORT);
    hal.limits.interrupt_callback(limitsGetState());
}

// Control pins ISR.
static void control_isr (void)
{
//  GPIO_IRQ_CLEAR(CONTROL_PORT);
    hal.control.interrupt_callback(systemGetState());
}

// Interrupt handler for 1 ms interval timer
static void systick_isr (void)
{
    if(!(--delay.ms)) {
        // SYSTICK_DISABLE; // Disable systick timer
        if(delay.callback) {
            delay.callback();
            delay.callback = NULL;
        }
    }
}
