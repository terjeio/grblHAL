/*
  I2CKeypad.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver for Cypress PSoC 5 (CY8CKIT-059)

  Part of Grbl

  Copyright (c) 2017 Terje Io

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

#include "project.h"
#include "serial.h"
#include "i2c_keypad.h"
#include "grbl.h"

#define HAS_KEYPAD //uncomment to enable I2C keypad for jogging etc.

// prescale step counter to 20Mhz (80 / (STEPPER_DRIVER_PRESCALER + 1))
#define STEPPER_DRIVER_PRESCALER 3
#define INTERRUPT_FREQ 1000u
#define SYSTICK_INTERRUPT_VECTOR_NUMBER 15u

static volatile uint8_t curr_step_outbits = 0;
static volatile uint32_t ms_count = 1;
static uint32_t curr_spindle_pwm = 0;
static bool spindlePWM = false;
static spindle_pwm_t spindle_pwm;
static axes_signals_t next_step_outbits;
static void (*delayCallback)(void) = 0;

#ifdef STEP_PULSE_DELAY
static uint8_t next_step_outbits, step_port_invert_mask, dir_port_invert_mask;
#endif

// Interrupt handler prototypes
static void stepper_driver_isr (void);
static void stepper_pulse_isr (void);
static void limit_isr (void);
static void control_isr (void);
static void systick_isr (void);

static void driver_delay_ms (uint32_t ms, void (*callback)(void)) {
    if((ms_count = ms) > 0) {
		DelayTimer_Start();
		if(!(delayCallback = callback))
		    while(ms_count);
	} else if(callback)
		callback();
}

// Non-variable spindle

// Start or stop spindle, called from spindle_run() and protocol_execute_realtime()
static void spindleSetStateFixed (spindle_state_t state, float rpm, uint8_t speed_ovr)
{
    rpm = rpm;              // stop compiler complaining
    speed_ovr = speed_ovr;  // stop compiler complaining
    
    SpindleOutput_Write(state.value);
}

// Variable spindle

// Called by spindle_set_state() and step segment generator. Keep routine small and efficient.
static uint32_t spindleComputePWMValue (float rpm, uint8_t speed_ovr)
{
    uint32_t pwm_value;

    rpm *= (0.010f * speed_ovr); // Scale by spindle speed override value.
    // Calculate PWM register value based on rpm max/min settings and programmed rpm.
    if ((settings.rpm_min >= settings.rpm_max) || (rpm >= settings.rpm_max)) {
        // No PWM range possible. Set simple on/off spindle control pin state.
        sys.spindle_speed = settings.rpm_max;
        pwm_value = spindle_pwm.max_value - 1;
    } else if (rpm <= settings.rpm_min) {
        if (rpm == 0.0f) { // S0 disables spindle
            sys.spindle_speed = 0.0f;
            pwm_value = spindle_pwm.off_value;
        } else { // Set minimum PWM output
            sys.spindle_speed = settings.rpm_min;
            pwm_value = spindle_pwm.min_value;
        }
    } else {
        // Compute intermediate PWM value with linear spindle speed model.
        // NOTE: A nonlinear model could be installed here, if required, but keep it VERY light-weight.
        sys.spindle_speed = rpm;
        pwm_value = (uint32_t)floorf((rpm - settings.rpm_min) * spindle_pwm.pwm_gradient) + spindle_pwm.min_value;
        if(pwm_value >= spindle_pwm.max_value)
        	pwm_value = spindle_pwm.max_value - 1;
    }

    return pwm_value;
}

// Set spindle speed. Note: spindle direction must be kept if stopped or restarted
static uint32_t spindleSetSpeed (uint32_t pwm_value)
{
    if (pwm_value == hal.spindle_pwm_off) {
        if(settings.flags.spindle_disable_with_zero_speed)
            SpindleOutput_Write(SpindleOutput_Read() & 0x02);
    } else {
        if(!(SpindleOutput_Read() & 0x01))
            SpindleOutput_Write(SpindleOutput_Read() | 0x01);
        SpindlePWM_WriteCompare(pwm_value);
    }
    
    return pwm_value;
}

// Start or stop spindle, called from spindle_run() and protocol_execute_realtime()
static void spindleSetStateVariable (spindle_state_t state, float rpm, uint8_t speed_ovr)
{
    uint32_t new_pwm = spindleComputePWMValue(rpm, speed_ovr);

    if (!state.on || new_pwm == spindle_pwm.off_value)
        SpindleOutput_Write(SpindleOutput_Read() & 0x02); // Keep direction!
    else { // Alarm if direction change without stopping first?
        SpindleOutput_Write(state.value);
        spindleSetSpeed(new_pwm);
    }
}

// end Variable spindle

static spindle_state_t spindleGetState (void)
{   
    return (spindle_state_t)SpindleOutput_Read();
}

// end spindle code

// Enable/disable steppers, called from st_wake_up() and st_go_idle()
static void stepperEnable (bool on) {
    StepperEnable_Write(on);
}

// Sets up for a step pulse and forces a stepper driver interrupt, called from st_wake_up()
// NOTE: delay and pulse_time are # of microseconds
static void stepperWakeUp () 
{
/*
    if(pulse_delay) {
        pulse_time += pulse_delay;
        TimerMatchSet(TIMER2_BASE, TIMER_A, pulse_time - pulse_delay);
    }
*/
    // Enable stepper drivers.
    StepperEnable_Write(on);
    StepperTimer_WritePeriod(5000); // dummy
    StepperTimer_Enable();
    Stepper_Interrupt_SetPending();
//    hal.stepper_interrupt_callback();

}

// Disables stepper driver interrups, called from st_go_idle()
static void stepperGoIdle (void) {
    StepperTimer_Stop();
}

// Sets up stepper driver interrupt timeout, called from stepper_driver_interrupt_handler()
static void stepperCyclesPerTick (uint32_t cycles_per_tick) {
//        StepperTimer_Stop();
//        StepperTimer_WriteCounter(cycles_per_tick < (1UL << 24) /*< 65536 (4.1ms @ 16MHz)*/ ? cycles_per_tick : 0xFFFFFF /*Just set the slowest speed possible.*/);
        StepperTimer_WritePeriod(cycles_per_tick < (1UL << 24) /*< 65536 (4.1ms @ 16MHz)*/ ? cycles_per_tick : 0xFFFFFF /*Just set the slowest speed possible.*/);
//    Control_Reg_1_Write(1);
//    Control_Reg_1_Write(0);
//        StepperTimer_Enable();
}

// Set stepper pulse output pins, called from st_reset()
inline static void stepperSetStepOutputs (axes_signals_t step_outbits)
{
    StepOutput_Write(step_outbits.value);
}

// Set stepper direction output pins, called from st_reset()
inline static void stepperSetDirOutputs (axes_signals_t dir_outbits)
{
    DirOutput_Write(dir_outbits.value);
}

// Sets stepper direction and pulse pins and starts a step pulse, called from stepper_driver_interrupt_handler()
// When delayed pulse the step register is written in the step delay interrupt handler
static void stepperPulseStart (axes_signals_t dir_outbits, axes_signals_t step_outbits, uint32_t spindle_pwm)
{
    
	if(spindlePWM && spindle_pwm != curr_spindle_pwm)
		curr_spindle_pwm = spindleSetSpeed(spindle_pwm);

    StepOutput_Write(step_outbits.value);
    DirOutput_Write(dir_outbits.value);
}

static void stepperPulseStartDelayed (axes_signals_t dir_outbits, axes_signals_t step_outbits, uint32_t spindle_pwm)
{
	if(spindlePWM && spindle_pwm != curr_spindle_pwm)
		curr_spindle_pwm = spindleSetSpeed(spindle_pwm);

    stepperSetDirOutputs(dir_outbits);
    next_step_outbits = step_outbits; // Store out_bits
//    TimerEnable(TIMER2_BASE, TIMER_A);
}

// Disable limit pins interrupt, called from mc_homing_cycle()
static void limitsEnable (bool on)
{
    if(on)
        Homing_Interrupt_Enable();
    else
        Homing_Interrupt_Disable();
}

// Returns limit state as a bit-wise uint8 variable. Each bit indicates an axis limit, where
// triggered is 1 and not triggered is 0. Invert mask is applied. Axes are defined by their
// number in bit position, i.e. Z_AXIS is (1<<2) or bit 2, and Y_AXIS is (1<<1) or bit 1.
inline static axes_signals_t limitsGetState()
{
	return (axes_signals_t)HomingSignals_Read();
}

static control_signals_t systemGetState (void)
{
	return (control_signals_t)ControlSignals_Read();
}

// Called by probe_init() and the mc_probe() routines. Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigureInvertMask(bool is_probe_away)
{
    ProbeInvert_Write(is_probe_away);
}

// Returns the probe pin state. Triggered = true. Called by gcode parser and probe state monitor.
bool probeGetState (void) {
    return ProbeSignal_Read() != 0;
}

// Start/stop coolant (and mist if enabled), called by coolant_run() and protocol_execute_realtime()
static void coolantSetState (coolant_state_t mode)
{
    CoolantOutput_Write(mode.value & 0x03);
}

static coolant_state_t coolantGetState (void)
{
    return (coolant_state_t)CoolantOutput_Read();
}

void eepromPutByte (uint32_t addr, uint8_t new_value) {
    EEPROM_WriteByte(new_value, addr); 
}

static void eepromWriteBlockWithChecksum (uint32_t destination, uint8_t *source, uint32_t size) {
  unsigned char checksum = 0;
  for(; size > 0; size--) { 
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += *source;
    EEPROM_WriteByte(*(source++), destination++); 
  }
  EEPROM_WriteByte(checksum, destination);
}

bool eepromReadBlockWithChecksum (uint8_t *destination, uint32_t source, uint32_t size) {
  unsigned char data, checksum = 0;
  for(; size > 0; size--) { 
    data = EEPROM_ReadByte(source++);
    checksum = (checksum << 1) || (checksum >> 7);
    checksum += data;    
    *(destination++) = data; 
  }
  return checksum == EEPROM_ReadByte(source);
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint8_t *ptr, uint8_t bits)
{
	CyGlobalIntDisable;
	*ptr |= bits;
	CyGlobalIntEnable;
}

static uint8_t bitsClearAtomic (volatile uint8_t *ptr, uint8_t bits)
{
	CyGlobalIntDisable;
    uint8_t prev = *ptr;
	*ptr &= ~bits;
	CyGlobalIntEnable;
	return prev;
}

static uint8_t valueSetAtomic (volatile uint8_t *ptr, uint8_t value)
{
	CyGlobalIntDisable;
    uint8_t prev = *ptr;
    *ptr = value;
	CyGlobalIntEnable;
    return prev;
}

// Callback to inform settings has been changed, called by settings_store_global_setting()
// Used to (re)configure hardware and set up helper variables
void settings_changed (settings_t *settings) {

    //TODO: disable interrupts while reconfigure?

    StepPulseClock_SetDivider(hal.f_step_timer / 1000000UL * settings->pulse_microseconds);

    DirInvert_Write(settings->dir_invert_mask.value);
    StepInvert_Write(settings->step_invert_mask.value);
    StepperEnableInvert_Write(settings->flags.invert_st_enable);
    SpindleInvert_Write(settings->spindle_invert_mask.value);
    CoolantInvert_Write(settings->coolant_invert_mask.value);

    // Homing (limit) inputs
    XHome_Write(settings->limit_disable_pullup_mask.x ? 0 : 1);
    XHome_SetDriveMode(settings->limit_disable_pullup_mask.x ? XHome_DM_RES_DWN : XHome_DM_RES_UP);
    YHome_Write(settings->limit_disable_pullup_mask.y ? 0 : 1);
    YHome_SetDriveMode(settings->limit_disable_pullup_mask.y ? YHome_DM_RES_DWN : YHome_DM_RES_UP);
    ZHome_Write(settings->limit_disable_pullup_mask.z ? 0 : 1);
    ZHome_SetDriveMode(settings->limit_disable_pullup_mask.z ? ZHome_DM_RES_DWN : ZHome_DM_RES_UP);
    HomingSignalsInvert_Write(settings->limit_invert_mask.value);

    // Control inputs
    Reset_Write(settings->control_disable_pullup_mask.reset ? 0 : 1);
    Reset_SetDriveMode(settings->control_disable_pullup_mask.reset ? Reset_DM_RES_DWN : Reset_DM_RES_UP);
    FeedHold_Write(settings->control_disable_pullup_mask.feed_hold ? 0 : 1);
    FeedHold_SetDriveMode(settings->control_disable_pullup_mask.feed_hold ? FeedHold_DM_RES_DWN : FeedHold_DM_RES_UP);
    CycleStart_Write(settings->control_disable_pullup_mask.cycle_start ? 0 : 1);
    CycleStart_SetDriveMode(settings->control_disable_pullup_mask.cycle_start ? CycleStart_DM_RES_DWN : CycleStart_DM_RES_UP);
    SafetyDoor_Write(settings->control_disable_pullup_mask.safety_door_ajar ? 0 : 1);
    SafetyDoor_SetDriveMode(settings->control_disable_pullup_mask.safety_door_ajar ? SafetyDoor_DM_RES_DWN : SafetyDoor_DM_RES_UP);
    ControlSignalsInvert_Write(settings->control_invert_mask.value);

    // Probe input
    ProbeInvert_Write(settings->flags.disable_probe_pullup ? 0 : 1);
    Probe_SetDriveMode(settings->flags.disable_probe_pullup ? Probe_DM_RES_DWN : Probe_DM_RES_UP);
    Probe_Write(settings->flags.disable_probe_pullup ? 0 : 1);

    spindle_pwm.period = (uint32_t)(hal.f_step_timer / settings->spindle_pwm_freq);
    spindle_pwm.off_value = (uint32_t)(spindle_pwm.period * settings->spindle_pwm_off_value / 100.0f);
    spindle_pwm.min_value = (uint32_t)(spindle_pwm.period * settings->spindle_pwm_min_value / 100.0f);
    spindle_pwm.max_value = (uint32_t)(spindle_pwm.period * settings->spindle_pwm_max_value / 100.0f);
    spindle_pwm.pwm_gradient = (float)(spindle_pwm.max_value - spindle_pwm.min_value) / (settings->rpm_max - settings->rpm_min);

    hal.spindle_pwm_off = spindle_pwm.off_value;

    if(spindlePWM)
        SpindlePWM_WritePeriod(spindle_pwm.period);

}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{

    StepPulseClock_Start();
    StepperTimer_Init();
    Stepper_Interrupt_SetVector(stepper_driver_isr);
    Stepper_Interrupt_SetPriority(1);
    Stepper_Interrupt_Enable();

    if(hal.driver_cap.step_pulse_delay) {
	//    TimerIntRegister(TIMER2_BASE, TIMER_A, stepper_pulse_isr_delayed);
	//    TimerIntEnable(TIMER2_BASE, TIMER_TIMA_TIMEOUT|TIMER_TIMA_MATCH);
	    hal.stepper_pulse_start = &stepperPulseStartDelayed;
    }
    
    Control_Interrupt_StartEx(control_isr);
    ControlSignals_InterruptEnable();
    
    Homing_Interrupt_SetVector(limit_isr);
    
    if((spindlePWM = hal.driver_cap.variable_spindle)) {
        SpindlePWM_Start();
        SpindlePWM_WritePeriod(spindle_pwm.period);
    } else
        hal.spindle_set_status = &spindleSetStateFixed;

//    CyIntSetSysVector(SYSTICK_INTERRUPT_VECTOR_NUMBER, systick_isr);
//    SysTick_Config(BCLK__BUS_CLK__HZ / INTERRUPT_FREQ);

    DelayTimer_Interrupt_SetVector(systick_isr);
    DelayTimer_Interrupt_SetPriority(7);
    DelayTimer_Interrupt_Enable();
    DelayTimer_Start();

    if(spindlePWM)
        spindleSetStateVariable((spindle_state_t){0}, spindle_pwm.off_value, DEFAULT_SPINDLE_SPEED_OVERRIDE);
    else
        spindleSetStateFixed((spindle_state_t){0}, 0, 0);

    coolantSetState((coolant_state_t){0});
    stepperSetDirOutputs((axes_signals_t){0});

#ifdef HAS_KEYPAD

   /*********************
    *  I2C KeyPad init  *
	*********************/

	I2C_keypad_setup();

#endif

    return settings->version == 12;
}

// Initialize HAL pointers
// NOTE: Grbl is not yet (configured from EEPROM data), driver_setup() will be called when done
bool driver_init (void) {

    serialInit();
    EEPROM_Start();
    
	hal.driver_setup = &driver_setup;
	hal.f_step_timer = 24000000UL;
	hal.rx_buffer_size = RX_BUFFER_SIZE;
	hal.delay_milliseconds = &driver_delay_ms;
    hal.settings_changed = &settings_changed;

	hal.stepper_wake_up = &stepperWakeUp;
	hal.stepper_go_idle = &stepperGoIdle;
	hal.stepper_enable = &stepperEnable;
	hal.stepper_set_outputs = &stepperSetStepOutputs;
	hal.stepper_set_directions = &stepperSetDirOutputs;
	hal.stepper_cycles_per_tick = &stepperCyclesPerTick;
	hal.stepper_pulse_start = &stepperPulseStart;

	hal.limits_enable = &limitsEnable;
	hal.limits_get_state = &limitsGetState;

	hal.coolant_set_state = &coolantSetState;
	hal.coolant_get_state = &coolantGetState;

	hal.probe_get_state = &probeGetState;
	hal.probe_configure_invert_mask = &probeConfigureInvertMask;

	hal.spindle_set_status = &spindleSetStateVariable;
	hal.spindle_get_state = &spindleGetState;
	hal.spindle_set_speed = &spindleSetSpeed;
	hal.spindle_compute_pwm_value = &spindleComputePWMValue;

	hal.system_control_get_state = &systemGetState;

    hal.serial_read = &serialGetC;
    hal.serial_write = (void (*)(uint8_t))&serialPutC;
    hal.serial_write_string = &serialWriteS;
    hal.serial_get_rx_buffer_available = &serialRxFree;
    hal.serial_reset_read_buffer = &serialRxFlush;
    hal.serial_cancel_read_buffer = &serialRxCancel;

    hal.eeprom.type = EEPROM_Physical;
	hal.eeprom.get_byte = (uint8_t (*)(uint32_t))&EEPROM_ReadByte;
	hal.eeprom.put_byte = &eepromPutByte;
	hal.eeprom.memcpy_to_with_checksum = &eepromWriteBlockWithChecksum;
	hal.eeprom.memcpy_from_with_checksum = &eepromReadBlockWithChecksum;

	hal.set_bits_atomic = &bitsSetAtomic;
	hal.clear_bits_atomic = &bitsClearAtomic;
	hal.set_value_atomic = &valueSetAtomic;

#ifdef HAS_KEYPAD
    hal.execute_realtime = &process_keypress;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

    hal.driver_cap.variable_spindle = on;
    hal.driver_cap.mist_control = on;
    hal.driver_cap.software_debounce = on;
    hal.driver_cap.step_pulse_delay = on;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = on;
    hal.driver_cap.limits_pull_up = on;
    hal.driver_cap.probe_pull_up = on;

    // no need to move version check before init - compiler will fail any mismatch for existing entries
	return hal.version == 3;
}

/* interrupt handlers */

// Main stepper driver
static void stepper_driver_isr (void) {

    StepperTimer_ReadStatusRegister(); // Clear interrupt

	hal.stepper_interrupt_callback();

}

// This interrupt is enabled when Grbl sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
// NOTE: TivaC has a shared interrupt for match and timeout
static void stepper_pulse_isr (void) {

    //Stepper_Timer_ReadStatusRegister();

#ifdef STEP_PULSE_DELAY
	uint32_t iflags = TimerIntStatus(TIMER2_BASE, true);
	TimerIntClear(TIMER2_BASE, iflags); // clear interrupt flags
	GPIOPinWrite(STEP_PORT, STEP_MASK, iflags & TIMER_TIMA_MATCH ? next_step_outbits : step_port_invert_mask);
#else
//	TimerIntClear(TIMER2_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag
	//GPIOPinWrite(STEP_PORT, STEP_MASK, step_port_invert_mask);
#endif
}

static void limit_isr (void)
{
    hal.limit_interrupt_callback((axes_signals_t)HomingSignals_Read());
}

static void control_isr (void) {
    hal.control_interrupt_callback((control_signals_t)ControlSignals_Read());
}

// Interrupt handler for 1 ms interval timer
static void systick_isr (void) {
    DelayTimer_ReadStatusRegister();
	if(!(--ms_count)) {
		DelayTimer_Stop();
		if(delayCallback) {
			delayCallback();
            delayCallback = 0;
        }
	}
}
