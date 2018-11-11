/*
  driver.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Texas Instruments Tiva C (TM4C1294MCPDT) ARM processor/LaunchPad

  Part of Grbl

  Copyright (c) 2018 Terje Io

  Some parts
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

#include "../GRBL/grbl.h"

#include "tiva.h"
#include "driver.h"
#include "eeprom.h"
#include "serial.h"

#ifdef SDCARD_SUPPORT
#include "sdcard.h"
#endif

//#include "usermcodes.h"
//#include "keypad.h"
//#include "atc.h"

#ifdef FreeRTOS
#include "enet.h"
#include "TCPStream.h"
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#endif

// prescale step counter to 20Mhz (120 / (STEPPER_DRIVER_PRESCALER + 1))
#define STEPPER_DRIVER_PRESCALER 5
#define STEPPER_PULSE_PRESCALER (120 - 1)

//#define SPINDLE_SYNC // do NOT enable - not complete!

#ifdef PWM_RAMPED

#define SPINDLE_RAMP_STEP_INCR 20 // timer compare register change per ramp step
#define SPINDLE_RAMP_STEP_TIME 2  // ms

typedef struct {
    volatile uint32_t ms_cfg;
    volatile uint32_t ms_count;
    int32_t pwm_current;
    int32_t pwm_target;
    int32_t pwm_step;
} pwm_ramp_t;

static pwm_ramp_t pwm_ramp;
#endif

#ifdef LASER_PPI

laser_ppi_t laser;

static void ppi_timeout_isr (void);

#endif

#ifdef SPINDLE_SYNC

typedef struct {                     // Set when last encoder pulse count did not match at last index
    float block_start;
    float prev_pos;
    float dpp; // distance per pulse in mm
    uint32_t timer_value_start;
    uint_fast8_t segment_id;
    uint32_t segments;
} spindle_sync_t;

static void stepperPulseStartSyncronized (stepper_t *stepper);

#endif

#ifdef ENABLE_TRINAMIC

driver_settings_t driver_settings;

#endif

#define INPUT_RESET         0
#define INPUT_FEED_HOLD     1
#define INPUT_CYCLE_START   2
#define INPUT_SAFETY_DOOR   3
#define INPUT_PROBE         4
#define INPUT_LIMIT_X       5
#define INPUT_LIMIT_Y       6
#define INPUT_LIMIT_Z       7
#define INPUT_LIMIT_A       8
#define INPUT_LIMIT_B       9
#define INPUT_LIMIT_C      10

typedef struct {
    uint32_t port;
    uint8_t pin;
    bool invert;
    volatile bool active;
    volatile bool debounce;
} state_signal_t;

state_signal_t inputpin[] = {
    {CONTROL_PORT, RESET_PIN, false, false, false},
    {CONTROL_PORT, FEED_HOLD_PIN, false, false, false},
    {CONTROL_PORT, CYCLE_START_PIN, false, false, false},
    {CONTROL_PORT_SD, SAFETY_DOOR_PIN, false, false, false},
    {PROBE_PORT, PROBE_PIN, false, false, false},
    {LIMIT_PORT_X, X_LIMIT_PIN, false, false, false},
    {LIMIT_PORT_YZ, Y_LIMIT_PIN, false, false, false},
    {LIMIT_PORT_YZ, Z_LIMIT_PIN, false, false, false}
#ifdef CNC_BOOSTERPACK2
,   {LIMIT_PORT_A, A_LIMIT_PIN, false, false, false},
    {LIMIT_PORT_B, B_LIMIT_PIN, false, false, false},
    {LIMIT_PORT_C, C_LIMIT_PIN, false, false, false}
#endif
};

static volatile uint32_t ms_count = 1; // NOTE: initial value 1 is for "resetting" systick timer
static bool pwmEnabled = false, IOInitDone = false;
static axes_signals_t next_step_outbits;
static spindle_pwm_t spindle_pwm;

static uint16_t step_prescaler[3] = {
    STEPPER_DRIVER_PRESCALER,
    STEPPER_DRIVER_PRESCALER + 8,
    STEPPER_DRIVER_PRESCALER + 64
};

// Inverts the probe pin state depending on user settings and probing cycle mode.
static uint8_t probe_invert;

#if STEP_OUTMODE == GPIO_MAP

    static const uint8_t c_step_outmap[8] = {
        0,
        X_STEP_PIN,
        Y_STEP_PIN,
        X_STEP_PIN|Y_STEP_PIN,
        Z_STEP_PIN,
        X_STEP_PIN|Z_STEP_PIN,
        Y_STEP_PIN|Z_STEP_PIN,
        X_STEP_PIN|Y_STEP_PIN|Z_STEP_PIN
    };

    static uint8_t step_outmap[8];

#endif

#if DIRECTION_OUTMODE == GPIO_MAP

    static const uint8_t c_dir_outmap[8] = {
        0,
        X_DIRECTION_PIN,
        Y_DIRECTION_PIN,
        X_DIRECTION_PIN|Y_DIRECTION_PIN,
        Z_DIRECTION_PIN,
        X_DIRECTION_PIN|Z_DIRECTION_PIN,
        Y_DIRECTION_PIN|Z_DIRECTION_PIN,
        X_DIRECTION_PIN|Y_DIRECTION_PIN|Z_DIRECTION_PIN
    };

    static uint8_t dir_outmap[8];

#endif

#ifdef CNC_BOOSTERPACK2

    static const uint8_t c_dir_outmap2[8] = {
        0,
        A_DIRECTION_PIN,
        B_DIRECTION_PIN,
        A_DIRECTION_PIN|B_DIRECTION_PIN,
        C_DIRECTION_PIN,
        A_DIRECTION_PIN|C_DIRECTION_PIN,
        B_DIRECTION_PIN|C_DIRECTION_PIN,
        A_DIRECTION_PIN|B_DIRECTION_PIN|C_DIRECTION_PIN
    };

    static uint8_t dir_outmap2[8];

#endif

#ifdef FreeRTOS
static void enetStreamWriteS (const char *data)
{
    TCPStreamWriteS(data);
    serialWriteS(data);
}
#endif

void selectStream (stream_setting_t stream)
{
    switch(stream) {

#ifdef FreeRTOS
        case StreamSetting_Ethernet:
            hal.stream_read = TCPStreamGetC;
            hal.stream_write = TCPStreamWriteS;
            hal.stream_write_all = enetStreamWriteS;
            hal.stream_get_rx_buffer_available = TCPStreamRxFree;
            hal.stream_reset_read_buffer = TCPStreamRxFlush;
            hal.stream_cancel_read_buffer = TCPStreamRxCancel;
            break;
#endif
        case StreamSetting_Serial:
            hal.stream_read = serialGetC;
            hal.stream_write = serialWriteS;
            hal.stream_write_all = serialWriteS;
            hal.stream_get_rx_buffer_available = serialRxFree;
            hal.stream_reset_read_buffer = serialRxFlush;
            hal.stream_cancel_read_buffer = serialRxCancel;
            hal.stream_suspend_read = serialSuspendInput;
            break;

        default:
            break;
    }
}

static uint_fast16_t spindleSetSpeed (uint_fast16_t pwm_value);

// Interrupt handler prototypes

static void stepper_driver_isr (void);
static void stepper_pulse_isr (void);
static void stepper_pulse_isr_delayed (void);
static void mode_select_isr (void);
#ifdef CNC_BOOSTERPACK
static void limit_yz_isr (void);
static void limit_debounced_yz_isr (void);
static void limit_x_isr (void);
static void limit_debounced_x_isr (void);
#ifdef CNC_BOOSTERPACK2
static void limit_a_isr (void);
static void limit_debounced_a_isr (void);
//static void limit_b_isr (void);
//static void limit_debounced_b_isr (void);
static void limit_c_isr (void);
static void limit_debounced_c_isr (void);
#endif
#else
static void limit_isr (void);
static void limit_debounced_isr (void);
#endif
static void control_isr (void);
static void control_isr_sd (void);
static void software_debounce_isr (void);

#ifdef DEBUGOUT
static void debug_out (bool enable)
{
    GPIOPinWrite(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, enable ? SPINDLE_DIRECTION_PIN : 0);
}
#endif

#ifdef FreeRTOS

static TimerHandle_t xDelayTimer = NULL;

void vTimerCallback (TimerHandle_t xTimer)
{
    configASSERT( pxTimer );

    void (*callback)(void) = (void (*)(void)) pvTimerGetTimerID(xTimer);

    if(callback)
        callback();

    xTimerDelete(xDelayTimer, 3);
    xDelayTimer = NULL;
}
static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if(callback) {
        if(xDelayTimer) {
            xTimerDelete(xDelayTimer, 3);
            xDelayTimer = NULL;
        }
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xDelayTimer = xTimerCreate("msDelay", pdMS_TO_TICKS(ms), pdFALSE, callback, vTimerCallback);
        xTimerStartFromISR(xDelayTimer, &xHigherPriorityTaskWoken);
    } else {
        if(xDelayTimer) {
            xTimerDelete(xDelayTimer, 3);
            xDelayTimer = NULL;
        }
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}
#else

static void systick_isr (void);

static void (*delayCallback)(void) = 0;

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if(ms) {
        ms_count = ms;
        SysTickEnable();
        if(!(delayCallback = callback))
            while(ms_count);
    } else {
        if(ms_count) {
            delayCallback = 0;
            ms_count = 1;
        }
        if(callback)
            callback();
    }
}
#endif

// Enable/disable steppers
static void stepperEnable (axes_signals_t enable)
{
    enable.mask ^= settings.steppers.enable_invert.mask;
#ifdef CNC_BOOSTERPACK
    GPIOPinWrite(STEPPERS_DISABLE_XY_PORT, STEPPERS_DISABLE_XY_PIN, enable.x ? STEPPERS_DISABLE_XY_PIN : 0);
    GPIOPinWrite(STEPPERS_DISABLE_Z_PORT, STEPPERS_DISABLE_Z_PIN, enable.z ? STEPPERS_DISABLE_Z_PIN : 0);
 #ifdef CNC_BOOSTERPACK2
    GPIOPinWrite(STEPPERS_DISABLE_AC_PORT, STEPPERS_DISABLE_AC_PIN, enable.a ? STEPPERS_DISABLE_AC_PIN : 0);
    GPIOPinWrite(STEPPERS_DISABLE_B_PORT, STEPPERS_DISABLE_B_PIN, enable.b ? STEPPERS_DISABLE_B_PIN : 0);
 #endif
#else
    GPIOPinWrite(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_PIN, enable.x ? STEPPERS_DISABLE_PIN : 0);
#endif
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    if(settings.steppers.pulse_delay_microseconds) {
        TimerMatchSet(PULSE_TIMER_BASE, TIMER_A, settings.steppers.pulse_delay_microseconds);
        TimerLoadSet(PULSE_TIMER_BASE, TIMER_A, settings.steppers.pulse_microseconds + settings.steppers.pulse_delay_microseconds - 1);
    } else
        TimerLoadSet(PULSE_TIMER_BASE, TIMER_A, settings.steppers.pulse_microseconds - 1);

#ifdef LASER_PPI
    laser.next_pulse = 0;
#endif

    // Enable stepper drivers.
    stepperEnable((axes_signals_t){AXES_BITMASK});

    TimerLoadSet(STEPPER_TIMER_BASE, TIMER_A, 5000);    // dummy...
    TimerEnable(STEPPER_TIMER_BASE, TIMER_A);
    IntPendSet(STEPPER_TIMER_INT);                  // force immediate Timer1 interrupt
}

// Disables stepper driver interrupts
static void stepperGoIdle (void)
{
    TimerDisable(STEPPER_TIMER_BASE, TIMER_A);
}

// Sets up stepper driver interrupt timeout, AMASS version
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    TimerLoadSet(STEPPER_TIMER_BASE, TIMER_A, cycles_per_tick < (1UL << 16) /*< 65536 (4.1ms @ 16MHz)*/ ? cycles_per_tick : 0xFFFF /*Just set the slowest speed possible.*/);
}

// Sets up stepper driver interrupt timeout, "Normal" version
// TODO: can use stepperCyclesPerTick and 32bit timer?
static void stepperCyclesPerTickPrescaled (uint32_t cycles_per_tick)
{
    uint32_t prescaler;
    // Compute step timing and timer prescalar for normal step generation.
    if (cycles_per_tick < (1UL << 16)) // < 65536  (4.1ms @ 16MHz)
        prescaler = step_prescaler[0]; // prescaler: 0
    else if (cycles_per_tick < (1UL << 19)) { // < 524288 (32.8ms@16MHz)
        prescaler = step_prescaler[1]; // prescaler: 8
        cycles_per_tick = cycles_per_tick >> 3;
    } else {
        prescaler = step_prescaler[2]; // prescaler: 64
        cycles_per_tick = cycles_per_tick >> 6;
    }
    TimerPrescaleSet(STEPPER_TIMER_BASE, TIMER_A, prescaler);
    TimerLoadSet(STEPPER_TIMER_BASE, TIMER_A, cycles_per_tick < (1UL << 16) /*< 65536 (4.1ms @ 16MHz)*/ ? cycles_per_tick : 0xFFFF /*Just set the slowest speed possible.*/);
}

// Set stepper pulse output pins
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z...
// Mapping to registers can be done by
// 1. bitbanding. Pros: can assign pins to different ports, no RMW needed. Cons: overhead, pin changes not synchronous
// 2. bit shift. Pros: fast, Cons: bits must be consecutive
// 3. lookup table. Pros: signal inversions done at setup, Cons: slower than bit shift
inline static void stepperSetStepOutputs (axes_signals_t step_outbits)
{
#ifdef CNC_BOOSTERPACK2
    step_outbits.value ^= settings.step_invert.mask;
    GPIOPinWrite(STEP_PORT, HWSTEP_MASK, step_outbits.value << STEP_OUTMODE);
    GPIOPinWrite(STEP_PORT_AB, HWSTEP_MASK_AB, step_outbits.value >> 3);
  #ifdef C_AXIS
    GPIOPinWrite(STEP_PORT_C, C_STEP_PIN, step_outbits.c ? C_STEP_PIN : 0);
  #endif
#else
  #if STEP_OUTMODE == GPIO_MAP
    GPIOPinWrite(STEP_PORT, HWSTEP_MASK, step_outmap[step_outbits.value]);
  #else
    GPIOPinWrite(STEP_PORT, HWSTEP_MASK, (step_outbits.value ^ settings.steppers.step_invert.mask) << STEP_OUTMODE);
  #endif
#endif
}

// Set stepper direction output pins
// NOTE: see note for stepperSetStepOutputs()
inline static void stepperSetDirOutputs (axes_signals_t dir_outbits)
{
#ifdef CNC_BOOSTERPACK2
    dir_outbits.value ^= settings.dir_invert.mask;
    GPIOPinWrite(DIRECTION_PORT, HWDIRECTION_MASK, dir_outmap[dir_outbits.value & 0x07]);
    GPIOPinWrite(DIRECTION_PORT2, HWDIRECTION_MASK2, dir_outmap2[dir_outbits.value >> 3]);
#else
  #if DIRECTION_OUTMODE == GPIO_MAP
    GPIOPinWrite(DIRECTION_PORT, HWDIRECTION_MASK, dir_outmap[dir_outbits.value]);
  #else
    GPIOPinWrite(DIRECTION_PORT, HWDIRECTION_MASK, (dir_outbits.value ^ settings.dir_invert.mask) << DIRECTION_OUTMODE);
  #endif
#endif
}

// Sets stepper direction and pulse pins and starts a step pulse
static void stepperPulseStart (stepper_t *stepper)
{
    static uint_fast16_t current_pwm = 0;

#ifdef SPINDLE_SYNC
    if(stepper->new_block) {
        if(stepper->exec_segment->spindle_sync) {
            hal.stepper_pulse_start = stepperPulseStartSyncronized;
            stepperPulseStartSyncronized(stepper);
            return;
        }
        stepper->new_block = false;
        stepperSetDirOutputs(stepper->dir_outbits);
    }
#endif

    if(stepper->spindle_pwm != current_pwm)
        current_pwm = spindleSetSpeed(stepper->spindle_pwm);

    if(stepper->new_block) {
        stepper->new_block = false;
        stepperSetDirOutputs(stepper->dir_outbits);
    }

    stepperSetStepOutputs(stepper->step_outbits);
    TimerEnable(PULSE_TIMER_BASE, TIMER_A);
}

#ifdef SPINDLE_SYNC

// Sets stepper direction and pulse pins and starts a step pulse
// When delayed pulse the step register is written in the step delay interrupt handler
static void stepperPulseStartSyncronized (stepper_t *stepper)
{
    static spindle_sync_t spindle_sync;

    if(stepper->new_block) {
        if(!stepper->exec_segment->spindle_sync) {
            hal.stepper_pulse_start = stepperPulseStart;
            stepperPulseStart(stepper);
            return;
        } else {
            spindle_sync.dpp = stepper->exec_block->programmed_rate * 120.0f;
            spindle_sync.prev_pos = 0.0f;
            spindle_sync.timer_value_start = 123;
            spindle_sync.block_start = 2.33f;
            spindle_sync.segments = 0;
            spindle_sync.segment_id = stepper->exec_segment->id + 1; // force recalc
        }
        stepper->new_block = false;
        stepperSetDirOutputs(stepper->dir_outbits);
    }

    stepperSetStepOutputs(stepper->step_outbits);

    if(spindle_sync.segment_id != stepper->exec_segment->id) {

        spindle_sync.segment_id = stepper->exec_segment->id;

        float dist = stepper->exec_segment->target_position - spindle_sync.prev_pos;

        float epulses = dist * spindle_sync.dpp;

        sys.pid_log.target[spindle_sync.segments] = stepper->exec_segment->target_position;

        spindle_sync.segments++;


 //       float current_pos = (spindleGetData(true).angular_position - spindle_sync.block_start) * stepper->exec_block->programmed_rate;

        spindle_sync.prev_pos = stepper->exec_segment->target_position;

    }

    TimerEnable(PULSE_TIMER_BASE, TIMER_A);
}
#endif

// Sets stepper direction and pulse pins and starts a step pulse with an initial delay
// When delayed pulse the step register is written in the step delay interrupt handler
static void stepperPulseStartDelayed (stepper_t *stepper)
{
    static uint_fast16_t current_pwm = 0;

    if(stepper->spindle_pwm != current_pwm)
        current_pwm = spindleSetSpeed(stepper->spindle_pwm);

    if(stepper->new_block) {
        stepper->new_block = false;
        stepperSetDirOutputs(stepper->dir_outbits);
    }

    next_step_outbits = stepper->step_outbits; // Store out_bits
    TimerEnable(PULSE_TIMER_BASE, TIMER_A);
}

#ifdef CONSTANT_SURFACE_SPEED_OPTION

// Sets stepper direction and pulse pins and starts a step pulse with an initial delay
// When delayed pulse the step register is written in the step delay interrupt handler
static void stepperPulseStartCSS (stepper_t *stepper)
{
    static uint_fast16_t current_pwm = 0, new_pwm = 0;
    static float pwm_delta = 0.0f, pwm_offset = 0.0f;

    if(stepper->new_block) {
        stepper->new_block = false;
        stepperSetDirOutputs(stepper->dir_outbits);
        pwm_offset = 0.0f;
        pwm_delta = stepper->exec_block->pwm_adjust;
        current_pwm = new_pwm = spindleSetSpeed(stepper->spindle_pwm);
    } else if(stepper->step_outbits.x && pwm_delta != 0.0f) {
        pwm_offset += pwm_delta;
        if(new_pwm + (int16_t)pwm_offset != current_pwm)
            current_pwm = spindleSetSpeed(new_pwm + (int16_t)pwm_offset);
    }

    stepperSetStepOutputs(stepper->step_outbits);
    TimerEnable(PULSE_TIMER_BASE, TIMER_A);
}

#endif

#ifdef LASER_PPI

static void spindleOn ();

// Sets stepper direction and pulse pins and starts a step pulse with an initial delay
// When delayed pulse the step register is written in the step delay interrupt handler
static void stepperPulseStartPPI (stepper_t *stepper)
{
    static uint_fast16_t current_pwm = 0;

    if(stepper->new_block) {
        stepper->new_block = false;
        stepperSetDirOutputs(stepper->dir_outbits);
        uint_fast16_t steps_per_pulse = stepper->exec_block->steps_per_mm * 25.4f / laser.ppi;
        if(laser.next_pulse && laser.steps_per_pulse)
            laser.next_pulse = laser.next_pulse * steps_per_pulse / laser.steps_per_pulse;
        laser.steps_per_pulse = steps_per_pulse;
    }

    if(stepper->spindle_pwm != current_pwm) {
        current_pwm = spindleSetSpeed(stepper->spindle_pwm);
        laser.next_pulse = 0;
    }

    if(laser.next_pulse == 0) {
        laser.next_pulse = laser.steps_per_pulse;
        if(current_pwm != hal.spindle_pwm_off) {
            spindleOn();
            TimerEnable(LASER_PPI_TIMER_BASE, TIMER_A);
            // TODO: T2CCP0 - use timer timeout to switch off CCP output w/o using interrupt? single shot PWM?
        }
    } else
        laser.next_pulse--;

    stepperSetStepOutputs(stepper->step_outbits);
    TimerEnable(PULSE_TIMER_BASE, TIMER_A);
}
#endif

// Enable/disable limit pins interrupt
static void limitsEnable (bool on)
{
#ifdef CNC_BOOSTERPACK
    uint32_t i;
  #ifdef CNC_BOOSTERPACK2
    for(i = INPUT_LIMIT_X; i <= INPUT_LIMIT_C; i++) {
  #else
    for(i = INPUT_LIMIT_X; i <= INPUT_LIMIT_Z; i++) {
  #endif
        GPIOIntClear(inputpin[i].port, inputpin[i].pin);     // Clear any pending interrupt
        if (on && settings.limits.flags.hard_enabled)
            GPIOIntEnable(inputpin[i].port, inputpin[i].pin);
        else
            GPIOIntDisable(inputpin[i].port, inputpin[i].pin);
    }
#else
    if (on && settings.flags.hard_limit_enable)
        GPIOIntEnable(LIMIT_PORT, HWLIMIT_MASK); // Enable Pin Change Interrupt
    else
        GPIOIntDisable(LIMIT_PORT, HWLIMIT_MASK); // Disable Pin Change Interrupt
#endif
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static axes_signals_t limitsGetState()
{
#ifdef CNC_BOOSTERPACK // Change to bit-band read to avoid call overhead!
    axes_signals_t signals = {0};
    uint32_t flags = GPIOPinRead(LIMIT_PORT_YZ, HWLIMIT_MASK_YZ);

    signals.x = GPIOPinRead(LIMIT_PORT_X, X_LIMIT_PIN) != 0;
    signals.y = (flags & Y_LIMIT_PIN) != 0;
    signals.z = (flags & Z_LIMIT_PIN) != 0;
#ifdef A_AXIS
    signals.a = GPIOPinRead(LIMIT_PORT_A, A_LIMIT_PIN) != 0;
#endif
#ifdef B_AXIS
    signals.b = GPIOPinRead(LIMIT_PORT_B, B_LIMIT_PIN) != 0;
#endif
#ifdef C_AXIS
    signals.c = GPIOPinRead(LIMIT_PORT_C, C_LIMIT_PIN) != 0;
#endif
#else
    uint32_t flags = GPIOPinRead(LIMIT_PORT, HWLIMIT_MASK);
    axes_signals_t signals;

    signals.x = (flags & X_LIMIT_PIN) != 0;
    signals.y = (flags & Y_LIMIT_PIN) != 0;
    signals.z = (flags & Z_LIMIT_PIN) != 0;
#endif

    if (settings.limits.invert.value)
        signals.value ^= settings.limits.invert.value;

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
inline static control_signals_t systemGetState (void)
{
    uint32_t flags = GPIOPinRead(CONTROL_PORT, HWCONTROL_MASK);
    control_signals_t signals = {0};

    signals.reset = (flags & RESET_PIN) != 0;
    signals.feed_hold = (flags & FEED_HOLD_PIN) != 0;
    signals.cycle_start = (flags & CYCLE_START_PIN) != 0;
#ifdef CNC_BOOSTERPACK
    signals.safety_door_ajar = GPIOPinRead(CONTROL_PORT_SD, SAFETY_DOOR_PIN) != 0;
#else
    signals.safety_door_ajar = (flags & SAFETY_DOOR_PIN) != 0;
#endif

    if(settings.control_invert.value)
        signals.value ^= settings.control_invert.value;

    return signals;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure(bool is_probe_away)
{
  probe_invert = settings.flags.invert_probe_pin ? 0 : PROBE_PIN;

  if (is_probe_away)
      probe_invert ^= PROBE_PIN;

  GPIOIntTypeSet(PROBE_PORT, PROBE_PIN, probe_invert ? GPIO_FALLING_EDGE : GPIO_RISING_EDGE);
  GPIOIntEnable(PROBE_PORT, PROBE_PIN);
}

// Returns the probe pin state. Triggered = true.
bool probeGetState (void)
{   //return probeState; // TODO: check out using interrupt instead (we want to trap trigger and not risk losing it due to bouncing)
    return (((uint8_t)GPIOPinRead(PROBE_PORT, PROBE_PIN)) ^ probe_invert) != 0;
}

// Static spindle (off, on cw & on ccw)

inline static void spindleOff ()
{
    GPIOPinWrite(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? SPINDLE_ENABLE_PIN : 0);
}

inline static void spindleOn ()
{
    GPIOPinWrite(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? 0 : SPINDLE_ENABLE_PIN);
}

inline static void spindleDir (bool ccw)
{
    GPIOPinWrite(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, (ccw ^ settings.spindle.invert.ccw) ? SPINDLE_DIRECTION_PIN : 0);
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
    return spindle_compute_pwm_value(&spindle_pwm, rpm, speed_ovr);
}

// Sets spindle speed
#ifdef PWM_RAMPED
static uint_fast16_t spindleSetSpeed (uint_fast16_t pwm_value)
{
    if (pwm_value == hal.spindle_pwm_off) {
        pwm_ramp.pwm_target = 0;
        pwm_ramp.pwm_step = -SPINDLE_RAMP_STEP_INCR;
        pwm_ramp.ms_count = 0;
        pwm_ramp.ms_cfg = SPINDLE_RAMP_STEP_TIME;
        SysTickEnable();
     } else {

        if(!pwmEnabled) {
            spindleOn();
            pwmEnabled = true;
            pwm_ramp.pwm_current = spindle_pwm.min_value;
            pwm_ramp.ms_count = 0;
            TimerMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period - pwm_ramp.pwm_current + 15);
            TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period);
            TimerEnable(SPINDLE_PWM_TIMER_BASE, TIMER_B); // Ensure PWM output is enabled.
//            TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, false);
        }
        pwm_ramp.pwm_target = pwm_value;
        pwm_ramp.pwm_step = pwm_ramp.pwm_target < pwm_ramp.pwm_current ? -SPINDLE_RAMP_STEP_INCR : SPINDLE_RAMP_STEP_INCR;
        pwm_ramp.ms_cfg = SPINDLE_RAMP_STEP_TIME;
        TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, false);
        SysTickEnable();
    }

    return pwm_value;
}
#else
static uint_fast16_t spindleSetSpeed (uint_fast16_t pwm_value)
{
    if (pwm_value == hal.spindle_pwm_off) {
        if(settings.spindle.disable_with_zero_speed)
            spindleOff();
        TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period + 20000);
        TimerDisable(SPINDLE_PWM_TIMER_BASE, TIMER_B); // Disable PWM. Output voltage is zero.
        if(pwmEnabled)
            TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, true);
        pwmEnabled = false;
     } else {
        TimerMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period - pwm_value);
        if(!pwmEnabled) {
            spindleOn();
            pwmEnabled = true;
            TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period);
            TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, false);
            TimerEnable(SPINDLE_PWM_TIMER_BASE, TIMER_B); // Ensure PWM output is enabled.
        }
    }

    return pwm_value;
}
#endif

// Start or stop spindle
static void spindleSetStateVariable (spindle_state_t state, float rpm, uint8_t speed_ovr)
{
    if (!state.on || rpm == 0.0f) {
        spindleSetSpeed(hal.spindle_pwm_off);
        spindleOff();
    } else {
        spindleDir(state.ccw);
        spindleSetSpeed(spindleComputePWMValue(rpm, speed_ovr));
    }
}

#ifdef SPINDLE_SYNC
static spindle_data_t spindleGetData (spindle_data_request_t request)
{
    static spindle_data_t spindle_data;

    spindle_data.rpm = GPIOPinRead(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN) ? 300.0f : 0.0f;
    spindle_data.angular_position = 0.0f;
    spindle_data.index_count++;

    return spindle_data;
}

static void spindleDataReset (void)
{
}

#endif

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {0};

    state.on = pwmEnabled || GPIOPinRead(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN) != 0;
    state.ccw = hal.driver_cap.spindle_dir && GPIOPinRead(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN) != 0;
    state.value ^= settings.spindle.invert.mask;
#ifdef PWM_RAMPED
    state.at_speed = pwm_ramp.pwm_current == pwm_ramp.pwm_target;
#endif
#ifdef SPINDLE_SYNC
    state.at_speed = spindleGetData(SpindleData_RPM).rpm == (state.on ? 300.0f : 0.0f);
#endif

    return state;
}

// end spindle code

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;
    GPIOPinWrite(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN, mode.flood ? COOLANT_FLOOD_PIN : 0);
    GPIOPinWrite(COOLANT_MIST_PORT, COOLANT_MIST_PIN, mode.mist ? COOLANT_MIST_PIN : 0);
}

// Returns coolant state in a coolant_state_t variable
static coolant_state_t coolantGetState (void)
{
    coolant_state_t state = {0};

    state.flood = GPIOPinRead(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN) != 0;
    state.mist  = GPIOPinRead(COOLANT_MIST_PORT, COOLANT_MIST_PIN) != 0;
    state.value ^= settings.coolant_invert.mask;

    return state;
}

static void showMessage (const char *msg)
{
    hal.stream_write("[MSG:");
    hal.stream_write(msg);
    hal.stream_write("]\r\n");
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    IntMasterDisable();
    *ptr |= bits;
    IntMasterEnable();
}

static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    IntMasterDisable();
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    IntMasterEnable();
    return prev;
}

static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    IntMasterDisable();
    uint_fast16_t prev = *ptr;
    *ptr = value;
    IntMasterEnable();
    return prev;
}

static void modeSelect (bool mpg_mode)
{
#ifndef FreeRTOS

    // Deny entering MPG mode if busy
    if(mpg_mode == sys.mpg_mode || (mpg_mode && (gc_state.file_run || sys.state != STATE_IDLE)))
        return;

//    BITBAND_PERI(MODE_PORT->OUT, MODE_LED_PIN) = mpg_mode;

    serialSelect(mpg_mode);

    if(mpg_mode) {
        hal.stream_read = serial2GetC;
        hal.stream_get_rx_buffer_available = serial2RxFree;
        hal.stream_cancel_read_buffer = serial2RxCancel;
        hal.stream_reset_read_buffer = serial2RxFlush;
    } else {
        hal.stream_read = serialGetC;
        hal.stream_get_rx_buffer_available = serialRxFree;
        hal.stream_cancel_read_buffer = serialRxCancel;
        hal.stream_reset_read_buffer = serialRxFlush;
    }

    hal.stream_reset_read_buffer();

    // Report WCO on first status report request from MPG processor
    if(mpg_mode)
        sys.report.wco_counter = 1;

    // Force a realtime status report
    hal.protocol_process_realtime('?');

    sys.mpg_mode = mpg_mode;
    sys.report.mpg_mode = true;
#endif
}

static void modechange (void)
{
    modeSelect(GPIOPinRead(MODE_PORT, MODE_SWITCH_PIN) == 0);
    GPIOIntEnable(MODE_PORT, MODE_SWITCH_PIN);
}

// Configures perhipherals when settings are initialized or changed
static void settings_changed (settings_t *settings)
{
    spindle_precompute_pwm_values(&spindle_pwm, 120000000UL);

    hal.spindle_pwm_off = spindle_pwm.off_value;

#if (STEP_OUTMODE == GPIO_MAP) || (DIRECTION_OUTMODE == GPIO_MAP)
    uint8_t i;
#endif

#if STEP_OUTMODE == GPIO_MAP
    for(i = 0; i < sizeof(step_outmap); i++)
        step_outmap[i] = c_step_outmap[i] ^ c_step_outmap[settings->step_invert.mask];
#endif

#if DIRECTION_OUTMODE == GPIO_MAP
    for(i = 0; i < sizeof(dir_outmap); i++)
        dir_outmap[i] = c_dir_outmap[i] ^ c_dir_outmap[settings->steppers.dir_invert.mask & 0x07];
#ifdef CNC_BOOSTERPACK2
    for(i = 0; i < sizeof(dir_outmap2); i++)
        dir_outmap2[i] = c_dir_outmap2[i] ^ c_dir_outmap2[settings->dir_invert.mask >> 3];
#endif
#endif

    if(IOInitDone) {

        // NOTE: some interfaces may defer actual stream switch until stack is operational
        switch(settings->stream) {

            case StreamSetting_Serial:
                selectStream(StreamSetting_Serial);
                break;

#ifdef FreeRTOS
            case StreamSetting_Ethernet:;
                static bool enet_ok = false;
                if(!enet_ok)
                    enet_ok = enet_init();
                break;
#endif
            default:
                break;
        }

        stepperEnable(settings->steppers.deenergize);

        if(hal.driver_cap.variable_spindle)
            TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_A, spindle_pwm.period);

        if(settings->steppers.pulse_delay_microseconds) {
            TimerIntRegister(PULSE_TIMER_BASE, TIMER_A, stepper_pulse_isr_delayed);
            TimerIntEnable(PULSE_TIMER_BASE, TIMER_TIMA_TIMEOUT|TIMER_TIMA_MATCH);
            hal.stepper_pulse_start = stepperPulseStartDelayed;
        } else {
            TimerIntRegister(PULSE_TIMER_BASE, TIMER_A, stepper_pulse_isr);
            TimerIntEnable(PULSE_TIMER_BASE, TIMER_TIMA_TIMEOUT);
          #ifdef CONSTANT_SURFACE_SPEED_OPTION
            hal.stepper_pulse_start = stepperPulseStartCSS;
          #else
            hal.stepper_pulse_start = stepperPulseStart;
          #endif
        }

      #ifdef ENABLE_TRINAMIC
        trinamic_configure();
      #endif

      #ifdef LASER_PPI
        if(!settings->flags.laser_mode)
            laser_ppi_mode(false);
      #endif

        /****************************************
         *  Control, limit & probe pins config  *
         ****************************************/

        bool pullup;
        uint32_t i = sizeof(inputpin) / sizeof(state_signal_t);

        control_signals_t control_fei;
        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        do {
            switch(--i) {

                case INPUT_RESET:
                    pullup = !settings->control_disable_pullup.reset;
                    inputpin[i].invert = control_fei.reset;
                    break;

                case INPUT_FEED_HOLD:
                    pullup = !settings->control_disable_pullup.feed_hold;
                    inputpin[i].invert = control_fei.feed_hold;
                    break;

                case INPUT_CYCLE_START:
                    pullup = !settings->control_disable_pullup.cycle_start;
                    inputpin[i].invert = control_fei.cycle_start;
                    break;

                case INPUT_SAFETY_DOOR:
                    pullup = !settings->control_disable_pullup.safety_door_ajar;
                    inputpin[i].invert = control_fei.safety_door_ajar;
                    break;

                case INPUT_PROBE:
                    pullup = hal.driver_cap.probe_pull_up;
                    inputpin[i].invert = false;
                    break;

                case INPUT_LIMIT_X:
                    pullup = !settings->limits.disable_pullup.x;
                    inputpin[i].invert = limit_fei.x;
                    break;

                case INPUT_LIMIT_Y:
                    pullup = !settings->limits.disable_pullup.y;
                    inputpin[i].invert = limit_fei.y;
                    break;

                case INPUT_LIMIT_Z:
                    pullup = !settings->limits.disable_pullup.z;
                    inputpin[i].invert = limit_fei.z;
                    break;

                case INPUT_LIMIT_A:
                    pullup = !settings->limits.disable_pullup.a;
                    inputpin[i].invert = limit_fei.a;
                    break;

                case INPUT_LIMIT_B:
                    pullup = !settings->limits.disable_pullup.b;
                    inputpin[i].invert = limit_fei.b;
                    break;

                case INPUT_LIMIT_C:
                    pullup = !settings->limits.disable_pullup.c;
                    inputpin[i].invert = limit_fei.c;
                    break;

            }

            GPIOIntDisable(inputpin[i].port, inputpin[i].pin);    // Disable pin change interrupt
            GPIOPadConfigSet(inputpin[i].port, inputpin[i].pin, GPIO_STRENGTH_2MA, pullup ? GPIO_PIN_TYPE_STD_WPU : GPIO_PIN_TYPE_STD_WPD);
            GPIOIntTypeSet(inputpin[i].port, inputpin[i].pin, inputpin[i].invert ? GPIO_FALLING_EDGE : GPIO_RISING_EDGE);
            GPIOIntClear(inputpin[i].port, inputpin[i].pin);     // Clear any pending interrupt

            if(i <= INPUT_SAFETY_DOOR)
                GPIOIntEnable(inputpin[i].port, inputpin[i].pin);    // Enable pin change interrupt for control pins

            inputpin[i].active   = GPIOPinRead(inputpin[i].port, inputpin[i].pin) == (inputpin[i].invert ? 0 : inputpin[i].pin);
            inputpin[i].debounce = false;

        } while(i);

#ifdef SERIAL2_MOD
       if(sys.mpg_mode != !(GPIOPinRead(MODE_PORT, MODE_SWITCH_PIN) == MODE_SWITCH_PIN))
            modeSelect(true);
       GPIOIntEnable(MODE_PORT, MODE_SWITCH_PIN);
#endif

    }
}

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
  // System init

#ifndef BACKCHANNEL
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
#endif
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOG);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOH);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOK);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOM);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    SysCtlPeripheralEnable(STEPPER_TIMER_PERIPH);
    SysCtlPeripheralEnable(PULSE_TIMER_PERIPH);


    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);

    SysCtlDelay(26); // wait a bit for peripherals to wake up

    /********************************************************
     * Read driver specific setting from persistent storage *
     ********************************************************/

#if defined(ENABLE_TRINAMIC)
    if(hal.eeprom.driver_area.address != 0) {
        if(!hal.eeprom.memcpy_from_with_checksum((uint8_t *)&driver_settings, hal.eeprom.driver_area.address, sizeof(driver_settings)))
            hal.driver_settings_restore(SETTINGS_RESTORE_DRIVER_PARAMETERS);
    }
#endif

    /******************
     *  Stepper init  *
     ******************/

    // Unlock GPIOF0, used for stepper disable Z control
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;

    GPIOPinTypeGPIOOutput(STEP_PORT, HWSTEP_MASK);
    GPIOPadConfigSet(STEP_PORT, HWSTEP_MASK, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOOutput(DIRECTION_PORT, HWDIRECTION_MASK);
    GPIOPadConfigSet(DIRECTION_PORT, HWDIRECTION_MASK, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

#ifdef CNC_BOOSTERPACK2
    GPIOPinTypeGPIOOutput(STEP_PORT_AB, HWSTEP_MASK_AB);
    GPIOPadConfigSet(STEP_PORT_AB, HWSTEP_MASK_AB, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    GPIOPinTypeGPIOOutput(STEP_PORT_C, C_STEP_PIN);
    GPIOPadConfigSet(STEP_PORT_C, C_STEP_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOOutput(DIRECTION_PORT2, HWDIRECTION_MASK2);
    GPIOPadConfigSet(DIRECTION_PORT2, HWDIRECTION_MASK2, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

#endif

#ifdef CNC_BOOSTERPACK
    // Unlock GPIOD7, used for disable XY
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_7;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    GPIOPinTypeGPIOOutput(STEPPERS_DISABLE_XY_PORT, STEPPERS_DISABLE_XY_PIN);
    GPIOPinTypeGPIOOutput(STEPPERS_DISABLE_Z_PORT, STEPPERS_DISABLE_Z_PIN);
  #ifdef CNC_BOOSTERPACK2
    GPIOPinTypeGPIOOutput(STEPPERS_DISABLE_AC_PORT, STEPPERS_DISABLE_AC_PIN);
    GPIOPinTypeGPIOOutput(STEPPERS_DISABLE_B_PORT, STEPPERS_DISABLE_B_PIN);
  #endif
#else
    GPIOPinTypeGPIOOutput(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_PIN);
#endif

    // Configure stepper driver timer
    TimerConfigure(STEPPER_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PERIODIC);
    IntPrioritySet(STEPPER_TIMER_INT, 0x20); // lower priority than for Timer2 (which resets the step-dir signal)
    TimerControlStall(STEPPER_TIMER_BASE, TIMER_A, true); //timer1 will stall in debug mode
    TimerIntRegister(STEPPER_TIMER_BASE, TIMER_A, stepper_driver_isr);
    TimerIntClear(STEPPER_TIMER_BASE, 0xFFFF);
    IntPendClear(STEPPER_TIMER_INT);
    TimerPrescaleSet(STEPPER_TIMER_BASE, TIMER_A, STEPPER_DRIVER_PRESCALER); // 20 MHz clock
    TimerIntEnable(STEPPER_TIMER_BASE, TIMER_TIMA_TIMEOUT);

    // Configure step pulse timer
//  TimerClockSourceSet(PULSE_TIMER_BASE, TIMER_CLOCK_SYSTEM);
    TimerConfigure(PULSE_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_ONE_SHOT);
    IntPrioritySet(PULSE_TIMER_INT, 0x00); // highest priority - higher than for Timer1 (which sets the step-dir output)
    TimerControlStall(PULSE_TIMER_BASE, TIMER_A, true); //timer2 will stall in debug mode
    TimerIntClear(PULSE_TIMER_BASE, 0xFFFF);
    IntPendClear(PULSE_TIMER_INT);
    TimerPrescaleSet(PULSE_TIMER_BASE, TIMER_A, STEPPER_PULSE_PRESCALER); // for 1uS per count

#ifdef CNC_BOOSTERPACK_A4998
    GPIOPinTypeGPIOOutput(STEPPERS_VDD_PORT, STEPPERS_VDD_PIN);
    GPIOPadConfigSet(STEPPERS_VDD_PORT, STEPPERS_VDD_PIN, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    GPIOPinWrite(STEPPERS_VDD_PORT, STEPPERS_VDD_PIN, STEPPERS_VDD_PIN);
  #ifdef CNC_BOOSTERPACK2
    GPIOPinTypeGPIOOutput(STEPPERS_VDD_PORT2, STEPPERS_VDD_PIN2);
    GPIOPadConfigSet(STEPPERS_VDD_PORT2, STEPPERS_VDD_PIN2, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    GPIOPinWrite(STEPPERS_VDD_PORT2, STEPPERS_VDD_PIN2, STEPPERS_VDD_PIN2);
  #endif
#endif

#ifdef LASER_PPI

   /********************************
    *  PPI mode pulse width timer  *
    ********************************/

    laser.ppi = 600.0f;
    laser.pulse_length = 1500;

    SysCtlPeripheralEnable(LASER_PPI_TIMER_PERIPH);
    SysCtlDelay(26); // wait a bit for peripherals to wake up
    TimerConfigure(LASER_PPI_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_ONE_SHOT);
    IntPrioritySet(LASER_PPI_TIMER_INT, 0x40); // lower priority than for Timer2 (which resets the step-dir signal)
    TimerControlStall(LASER_PPI_TIMER_BASE, TIMER_A, true); //TIMER5 will stall in debug mode
    TimerIntClear(LASER_PPI_TIMER_BASE, 0xFFFF);
    IntPendClear(LASER_PPI_TIMER_INT);
    TimerPrescaleSet(LASER_PPI_TIMER_BASE, TIMER_A, 79); // for 1uS per count
    TimerIntRegister(LASER_PPI_TIMER_BASE, TIMER_A, ppi_timeout_isr);
    TimerLoadSet(LASER_PPI_TIMER_BASE, TIMER_A, laser.pulse_length);
    TimerIntEnable(LASER_PPI_TIMER_BASE, TIMER_TIMA_TIMEOUT|TIMER_TIMA_MATCH);

#endif

   /****************************
    *  Software debounce init  *
    ****************************/

    if(hal.driver_cap.software_debounce) {
        SysCtlPeripheralEnable(DEBOUNCE_TIMER_PERIPH);
        SysCtlDelay(26); // wait a bit for peripherals to wake up
        IntPrioritySet(DEBOUNCE_TIMER_INT, 0x40); // lower priority than for Timer2 (which resets the step-dir signal)
        TimerConfigure(DEBOUNCE_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_ONE_SHOT);
        TimerControlStall(DEBOUNCE_TIMER_BASE, TIMER_A, true); //timer2 will stall in debug mode
        TimerIntRegister(DEBOUNCE_TIMER_BASE, TIMER_A, software_debounce_isr);
        TimerIntClear(DEBOUNCE_TIMER_BASE, 0xFFFF);
        IntPendClear(DEBOUNCE_TIMER_INT);
        TimerPrescaleSet(DEBOUNCE_TIMER_BASE, TIMER_A, 119); // configure for 1us per count
        TimerLoadSet(DEBOUNCE_TIMER_BASE, TIMER_A, 32000);  // and for a total of 32ms
        TimerIntEnable(DEBOUNCE_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    }

    /******************************************
     *  Control, limit & probe pins dir init  *
     ******************************************/

    uint32_t i = sizeof(inputpin) / sizeof(state_signal_t);

    do {
        i--;
        GPIOPinTypeGPIOInput(inputpin[i].port, inputpin[i].pin);
    } while(i);

   /***********************
    *  Control pins init  *
    ***********************/

    GPIOIntRegister(CONTROL_PORT, control_isr);             // Register interrupt handler
#ifdef CNC_BOOSTERPACK
    GPIOIntRegister(CONTROL_PORT_SD, control_isr_sd);       // Register interrupt handler for safety door
#endif

   /*********************
    *  Limit pins init  *
    *********************/
    // Register a call-back function(s) for interrupt
#ifdef CNC_BOOSTERPACK
    GPIOIntRegister(LIMIT_PORT_X, hal.driver_cap.software_debounce ? limit_debounced_x_isr : limit_x_isr);
    GPIOIntRegister(LIMIT_PORT_YZ, hal.driver_cap.software_debounce ? limit_debounced_yz_isr : limit_yz_isr);
  #ifdef CNC_BOOSTERPACK2
    GPIOIntRegister(LIMIT_PORT_A, hal.driver_cap.software_debounce ? limit_debounced_a_isr : limit_a_isr);
//    GPIOIntRegister(LIMIT_PORT_B, hal.driver_cap.software_debounce ? limit_debounced_b_isr : limit_b_isr); Handled by control isr
    GPIOIntRegister(LIMIT_PORT_C, hal.driver_cap.software_debounce ? limit_debounced_c_isr : limit_c_isr);
  #endif
#else
    GPIOIntRegister(LIMIT_PORT, hal.driver_cap.software_debounce ? limit_isr_debounced : limit_isr); // Register a call-back funcion for interrupt
#endif

   /***********************
    *  Coolant pins init  *
    ***********************/

    GPIOPinTypeGPIOOutput(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN);
    GPIOPinTypeGPIOOutput(COOLANT_MIST_PORT, COOLANT_MIST_PIN);
    GPIOPadConfigSet(COOLANT_FLOOD_PORT, COOLANT_FLOOD_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    GPIOPadConfigSet(COOLANT_MIST_PORT, COOLANT_MIST_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

    if(hal.driver_cap.amass_level == 0)
        hal.stepper_cycles_per_tick = &stepperCyclesPerTickPrescaled;

   /******************
    *  Spindle init  *
    ******************/

    GPIOPinTypeGPIOOutput(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN);
    GPIOPadConfigSet(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOOutput(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN);
    GPIOPadConfigSet(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

    if((hal.driver_cap.variable_spindle)) {
        SysCtlPeripheralEnable(SPINDLE_PWM_TIMER_PERIPH);
        SysCtlDelay(26); // wait a bit for peripherals to wake up
        TimerClockSourceSet(SPINDLE_PWM_TIMER_BASE, TIMER_CLOCK_SYSTEM);
        TimerConfigure(SPINDLE_PWM_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_B_PWM);
//      TimerControlStall(SPINDLE_PWM_TIMER_BASE, TIMER_B, false); //timer1 will stall in debug mode
//      TimerPrescaleSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, STEPPER_DRIVER_PRESCALER); // 20 MHz clock
//      TimerPrescaleMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, STEPPER_DRIVER_PRESCALER);
        TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, false);
        GPIOPinConfigure(SPINDLEPWM_MAP);
        GPIOPinTypeTimer(SPINDLEPPORT, SPINDLEPPIN);
        GPIOPadConfigSet(SPINDLEPPORT, SPINDLEPPIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
      #ifdef PWM_RAMPED
        pwm_ramp.ms_cfg = pwm_ramp.pwm_current = pwm_ramp.pwm_target = 0;
      #endif
    } else
        hal.spindle_set_state = &spindleSetState;

#ifdef _KEYPAD_H_

   /*********************
    *  I2C KeyPad init  *
    *********************/

    keypad_setup();

#endif

#ifdef _SDCARD_H_
    sdcard_init();
#endif

#ifdef ENABLE_TRINAMIC
    trinamic_init();
#endif

    /*********************
     * Mode select input *
     *********************/

    GPIOPadConfigSet(MODE_PORT, MODE_SWITCH_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(MODE_PORT, MODE_SWITCH_PIN, GPIO_BOTH_EDGES);
    GPIOIntClear(MODE_PORT, MODE_SWITCH_PIN);     // Clear any pending interrupt
    GPIOIntRegister(MODE_PORT, mode_select_isr);

  // Set defaults

    IOInitDone = settings->version == 14;

    settings_changed(settings);

    spindleSetState((spindle_state_t){0}, spindle_pwm.off_value, DEFAULT_SPINDLE_RPM_OVERRIDE);
    coolantSetState((coolant_state_t){0});
    stepperSetDirOutputs((axes_signals_t){0});

    return IOInitDone;
}

#if defined(ENABLE_TRINAMIC)

static bool driver_setting (uint_fast16_t param, float value, char *svalue)
{
    bool claimed = false;

#ifdef ENABLE_TRINAMIC
    if(!claimed)
        claimed = trinamic_setting(param, value, svalue);
#endif

    if(claimed)
        hal.eeprom.memcpy_to_with_checksum(hal.eeprom.driver_area.address, (uint8_t *)&driver_settings, sizeof(driver_settings));

    return claimed;
}

static void driver_settings_report (bool axis_settings, axis_setting_type_t setting_type, uint8_t axis_idx)
{
#ifdef ENABLE_TRINAMIC
    trinamic_settings_report(axis_settings, setting_type, axis_idx);
#endif
}

void driver_settings_restore (uint8_t restore_flag)
{
    if(restore_flag & SETTINGS_RESTORE_DRIVER_PARAMETERS) {
#ifdef ENABLE_TRINAMIC
        trinamic_settings_restore(restore_flag);
#endif
        hal.eeprom.memcpy_to_with_checksum(hal.eeprom.driver_area.address, (uint8_t *)&driver_settings, sizeof(driver_settings));
    }
}

#endif

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{
#ifdef FreeRTOS
    hal.f_step_timer = configCPU_CLOCK_HZ;
#else
    FPUEnable();
    FPULazyStackingEnable();
    hal.f_step_timer = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ|SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_CFG_VCO_480, 120000000);

    // Set up systick timer with a 1ms period
    SysTickPeriodSet((hal.f_step_timer / 1000) - 1);
    SysTickIntRegister(systick_isr);
    SysTickIntEnable();
    SysTickEnable();
#endif
    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
    SysCtlDelay(26); // wait a bit for peripheral to wake up
    EEPROMInit();

    serialInit();

#ifdef FreeRTOS
    hal.info = "TM4C1294NCPDT FreeRTOS";
#else
    hal.info = "TM4C1294NCPDT";
#endif
    hal.driver_setup = driver_setup;
    hal.f_step_timer = hal.f_step_timer / (STEPPER_DRIVER_PRESCALER + 1);
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

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
    hal.probe_configure_invert_mask = probeConfigure;

    hal.spindle_set_state = spindleSetStateVariable;
    hal.spindle_get_state = spindleGetState;
    hal.spindle_set_speed = spindleSetSpeed;
    hal.spindle_compute_pwm_value = spindleComputePWMValue;
#ifdef SPINDLE_SYNC
    hal.spindle_get_data = spindleGetData;
    hal.spindle_reset_data = spindleDataReset;
#endif

    hal.system_control_get_state = systemGetState;

    selectStream(StreamSetting_Serial);

    hal.eeprom.type = EEPROM_Physical;
    hal.eeprom.get_byte = eepromGetByte;
    hal.eeprom.put_byte = eepromPutByte;
    hal.eeprom.memcpy_to_with_checksum = eepromWriteBlockWithChecksum;
    hal.eeprom.memcpy_from_with_checksum = eepromReadBlockWithChecksum;

#if defined(ENABLE_TRINAMIC)
    assert(EEPROM_ADDR_TOOL_TABLE - (sizeof(driver_settings_t) + 2) > EEPROM_ADDR_GLOBAL + sizeof(settings_t) + 1);

    hal.eeprom.driver_area.address = EEPROM_ADDR_TOOL_TABLE - (sizeof(driver_settings_t) + 2);
    hal.eeprom.driver_area.size = sizeof(driver_settings_t);

    hal.driver_setting = driver_setting;
    hal.driver_settings_report = driver_settings_report;
    hal.driver_settings_restore = driver_settings_restore;

    hal.driver_mcode_check = trimamic_MCodeCheck;
    hal.driver_mcode_validate = trimamic_MCodeValidate;
    hal.driver_mcode_execute = trimamic_MCodeExecute;
#endif

    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;

#ifdef _USERMCODES_H_
    hal.driver_mcode_check = userMCodeCheck;
    hal.driver_mcode_validate = userMCodeValidate;
    hal.driver_mcode_execute = userMCodeExecute;
#endif

    hal.show_message = showMessage;

#ifdef DEBUGOUT
    hal.debug_out = debug_out;
#endif

#ifdef _KEYPAD_H_
    hal.execute_realtime = process_keypress;
    hal.driver_setting = driver_setting;
    hal.driver_settings_restore = driver_settings_restore;
    hal.driver_settings_report = driver_settings_report;
#endif

#ifdef _ATC_H_
    hal.tool_select = atc_tool_selected;
    hal.tool_change = atc_tool_change;
#endif

#ifdef _SDCARD_H_
    hal.driver_reset = sdcard_reset;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

    hal.driver_cap.spindle_dir = On;
    hal.driver_cap.variable_spindle = On;
#ifdef PWM_RAMPED
    hal.driver_cap.spindle_at_speed = On;
#endif
#ifdef SPINDLE_SYNC
    hal.driver_cap.spindle_sync = On;
    hal.driver_cap.spindle_at_speed = On;
#endif
    hal.driver_cap.mist_control = On;
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;
#ifdef LASER_PPI
    hal.driver_cap.laser_ppi_mode = On;
#endif
#ifdef CONSTANT_SURFACE_SPEED_OPTION
    hal.driver_cap.constant_surface_speed = On;
#endif
#ifdef _SDCARD_H_
    hal.driver_cap.sd_card = On;
#endif
#ifdef FreeRTOS
    hal.driver_cap.ethernet = On;
#endif

    // no need to move version check before init - compiler will fail any mismatch for existing entries
    return hal.version == 4;
}

/* interrupt handlers */

// Main stepper driver
static void stepper_driver_isr (void)
{
    TimerIntClear(STEPPER_TIMER_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag
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
// This interrupt is enabled when Grbl sets the motor port bits to execute
// a step. This ISR resets the motor port after a short period (settings.pulse_microseconds)
// completing one step cycle.
// NOTE: TivaC has a shared interrupt for match and timeout
static void stepper_pulse_isr (void)
{
    TimerIntClear(PULSE_TIMER_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag
    stepperSetStepOutputs(settings.steppers.step_invert);
}

static void stepper_pulse_isr_delayed (void)
{
    uint32_t iflags = TimerIntStatus(PULSE_TIMER_BASE, true);
    TimerIntClear(PULSE_TIMER_BASE, iflags); // clear interrupt flags
    stepperSetStepOutputs(iflags & TIMER_TIMA_MATCH ? next_step_outbits : settings.steppers.step_invert);
}

#ifdef LASER_PPI

void laser_ppi_mode (bool on)
{
    if(on)
        hal.stepper_pulse_start = stepperPulseStartPPI;
    else
        hal.stepper_pulse_start = settings.pulse_delay_microseconds ? stepperPulseStartDelayed : stepperPulseStart;
    gc_set_laser_ppimode(on);
}

// Switches off the spindle (laser) after laser.pulse_length time has elapsed
static void ppi_timeout_isr (void)
{
    TimerIntClear(LASER_PPI_TIMER_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag
    spindleOff();
}
#endif

static void software_debounce_isr (void)
{
    TimerIntClear(DEBOUNCE_TIMER_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag

    axes_signals_t state = limitsGetState();

    if(state.value) //TODO: add check for limit swicthes having same state as when limit_isr were invoked?
        hal.limit_interrupt_callback(state);
}

#ifdef CNC_BOOSTERPACK

static void limit_yz_isr (void)
{
    uint32_t iflags = GPIOIntStatus(LIMIT_PORT_YZ, true);

    GPIOIntClear(LIMIT_PORT_YZ, iflags);
    if(iflags & HWLIMIT_MASK_YZ)
        hal.limit_interrupt_callback(limitsGetState());
}

static void limit_debounced_yz_isr (void)
{
    uint32_t iflags = GPIOIntStatus(LIMIT_PORT_YZ, true);

    GPIOIntClear(LIMIT_PORT_YZ, iflags);
    if(iflags & HWLIMIT_MASK_YZ) {
/*
        // TODO: disable interrupts here and reenable in software_debounce_isr?
        if(iflags & Y_LIMIT_PIN)
            inputpin[INPUT_LIMIT_Y].debounce = true;

        if(iflags & Z_LIMIT_PIN)
            inputpin[INPUT_LIMIT_Z].debounce = true;
*/
        TimerLoadSet(DEBOUNCE_TIMER_BASE, TIMER_A, 32000);  // 32ms
        TimerEnable(DEBOUNCE_TIMER_BASE, TIMER_A);
    }
}

static void limit_x_isr (void)
{
    uint32_t iflags = GPIOIntStatus(LIMIT_PORT_X, true);

    GPIOIntClear(LIMIT_PORT_X, iflags);
    if(iflags & X_LIMIT_PIN)
        hal.limit_interrupt_callback(limitsGetState());
}

static void limit_debounced_x_isr (void)
{
    uint32_t iflags = GPIOIntStatus(LIMIT_PORT_X, true);

    GPIOIntClear(LIMIT_PORT_X, iflags);
    if(iflags & X_LIMIT_PIN) {
/*
        // TODO: disable interrupts here and reenable in software_debounce_isr?
        inputpin[INPUT_LIMIT_X].debounce = true;

*/
        TimerLoadSet(DEBOUNCE_TIMER_BASE, TIMER_A, 32000);  // 32ms
        TimerEnable(DEBOUNCE_TIMER_BASE, TIMER_A);
    }
}

static void mode_select_isr (void)
{
    GPIOIntDisable(MODE_PORT, MODE_SWITCH_PIN); // Disable mode pin interrupt
    GPIOIntClear(MODE_PORT, MODE_SWITCH_PIN);   // and clear it

    driver_delay_ms(50, modechange);            // Debounce 50ms before attempting mode change
}

  #ifdef CNC_BOOSTERPACK2

    static void limit_a_isr (void)
    {
        uint32_t iflags = GPIOIntStatus(LIMIT_PORT_A, true);

        GPIOIntClear(LIMIT_PORT_A, iflags);
        if(iflags & A_LIMIT_PIN)
            hal.limit_interrupt_callback(limitsGetState());
    }

    static void limit_debounced_a_isr (void)
    {
        uint32_t iflags = GPIOIntStatus(LIMIT_PORT_A, true);

        GPIOIntClear(LIMIT_PORT_A, iflags);
        if(iflags & A_LIMIT_PIN) {
    /*
            // TODO: disable interrupts here and reenable in software_debounce_isr?
            inputpin[INPUT_LIMIT_X].debounce = true;

    */
            TimerLoadSet(DEBOUNCE_TIMER_BASE, TIMER_A, 32000);  // 32ms
            TimerEnable(DEBOUNCE_TIMER_BASE, TIMER_A);
        }
    }
/* NOTE: handled by control isr
    static void limit_b_isr (void)
    {
        uint32_t iflags = GPIOIntStatus(LIMIT_PORT_B, true);

        GPIOIntClear(LIMIT_PORT_B, iflags);
        if(iflags & B_LIMIT_PIN)
            hal.limit_interrupt_callback(limitsGetState());
    }

    static void limit_debounced_b_isr (void)
    {
        uint32_t iflags = GPIOIntStatus(LIMIT_PORT_B, true);

        GPIOIntClear(LIMIT_PORT_B, iflags);
        if(iflags & B_LIMIT_PIN) {
            // TODO: disable interrupts here and reenable in software_debounce_isr?
            //inputpin[INPUT_LIMIT_X].debounce = true;
            TimerLoadSet(DEBOUNCE_TIMER_BASE, TIMER_A, 32000);  // 32ms
            TimerEnable(DEBOUNCE_TIMER_BASE, TIMER_A);
        }
    }
*/
    static void limit_c_isr (void)
    {
        uint32_t iflags = GPIOIntStatus(LIMIT_PORT_C, true);

        GPIOIntClear(LIMIT_PORT_C, iflags);
        if(iflags & C_LIMIT_PIN)
            hal.limit_interrupt_callback(limitsGetState());
    }

    static void limit_debounced_c_isr (void)
    {
        uint32_t iflags = GPIOIntStatus(LIMIT_PORT_C, true);

        GPIOIntClear(LIMIT_PORT_C, iflags);
        if(iflags & C_LIMIT_PIN) {
    /*
            // TODO: disable interrupts here and reenable in software_debounce_isr?
            inputpin[INPUT_LIMIT_X].debounce = true;

    */
            TimerLoadSet(DEBOUNCE_TIMER_BASE, TIMER_A, 32000);  // 32ms
            TimerEnable(DEBOUNCE_TIMER_BASE, TIMER_A);
        }
    }
  #endif

#else

static void limit_isr (void)
{
    uint32_t iflags = GPIOIntStatus(LIMIT_PORT_YZ, true);

    GPIOIntClear(LIMIT_PORT_YZ, iflags);
    if(iflags & HWLIMIT_MASK_YZ)
        hal.limit_interrupt_callback(limitsGetState());
}

static void limit_isr_debounced (void)
{
    uint32_t iflags = GPIOIntStatus(LIMIT_PORT_YZ, true);

    GPIOIntClear(LIMIT_PORT_YZ, iflags);
    if(iflags & HWLIMIT_MASK_YZ) {
        // TODO: disable interrups here and reenable in software_debounce_isr?
        TimerLoadSet(DEBOUNCE_TIMER_BASE, TIMER_A, 32000);  // 32ms
        TimerEnable(DEBOUNCE_TIMER_BASE, TIMER_A);
    }
}
#endif

static void control_isr (void)
{
// No debounce??
    uint32_t iflags = GPIOIntStatus(CONTROL_PORT, true) & HWCONTROL_MASK;

    if(iflags) {
        GPIOIntClear(CONTROL_PORT, iflags);
        hal.control_interrupt_callback(systemGetState());
    }
}

static void control_isr_sd (void)
{
// No debounce??
    uint32_t iflags = GPIOIntStatus(CONTROL_PORT_SD, true);

    GPIOIntClear(CONTROL_PORT_SD, iflags);

#ifdef CNC_BOOSTERPACK2
    if(iflags & B_LIMIT_PIN) {
        if(hal.driver_cap.software_debounce) {
            TimerLoadSet(DEBOUNCE_TIMER_BASE, TIMER_A, 32000);  // 32ms
            TimerEnable(DEBOUNCE_TIMER_BASE, TIMER_A);
        } else
            hal.limit_interrupt_callback(limitsGetState());
    }
#endif
    if(iflags & SAFETY_DOOR_PIN)
        hal.control_interrupt_callback(systemGetState());
}

#ifndef FreeRTOS
// Interrupt handler for 1 ms interval timer
#ifdef PWM_RAMPED
static void systick_isr (void)
{
    if(pwm_ramp.ms_cfg) {
        if(++pwm_ramp.ms_count == pwm_ramp.ms_cfg) {

            pwm_ramp.ms_count = 0;
            pwm_ramp.pwm_current += pwm_ramp.pwm_step;

            if(pwm_ramp.pwm_step < 0) { // decrease speed

                if(pwm_ramp.pwm_current < pwm_ramp.pwm_target)
                    pwm_ramp.pwm_current = pwm_ramp.pwm_target;

                if(pwm_ramp.pwm_current == 0) { // stop?
                    if(settings.flags.spindle_disable_with_zero_speed)
                        spindleOff();
                    TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period + 20000);
                    TimerDisable(SPINDLE_PWM_TIMER_BASE, TIMER_B); // Disable PWM. Output voltage is zero.
                    if(pwmEnabled)
                        TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, true);
                    pwmEnabled = false;
                } else
                    TimerMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period - pwm_ramp.pwm_current); // use LUT?
            } else {
                 if(pwm_ramp.pwm_current > pwm_ramp.pwm_target)
                     pwm_ramp.pwm_current = pwm_ramp.pwm_target;
                TimerMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period - pwm_ramp.pwm_current); // use LUT?
            }
            if(pwm_ramp.pwm_current == pwm_ramp.pwm_target)
                pwm_ramp.ms_cfg = 0;
        }
    }

    if(ms_count && !(--ms_count) && delayCallback) {
        delayCallback();
        delayCallback = 0;
    }

    if(!(ms_count || pwm_ramp.ms_cfg))
        SysTickDisable();
}
#else
static void systick_isr (void)
{
    if(!(--ms_count)) {
        SysTickDisable();
        if(delayCallback) {
            delayCallback();
            delayCallback = 0;
        }
    }
}
#endif
#endif // FreeRTOS
