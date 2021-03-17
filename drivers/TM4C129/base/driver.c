/*
  driver.c - main driver

  For Texas Instruments SimpleLink ARM processors/LaunchPads

  Part of grblHAL

  Copyright (c) 2018-2021 Terje Io

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


#include "driver.h"
#include "eeprom.h"
#include "serial.h"

#include "grbl/limits.h"
#include "grbl/protocol.h"
#include "grbl/state_machine.h"

#ifdef FreeRTOS
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#endif

#if KEYPAD_ENABLE
#include "keypad/keypad.h"
static void keyclick_int_handler (void);
#endif

#if TRINAMIC_ENABLE
static void trinamic_diag1_isr (void);
#endif

#ifdef I2C_ENABLE
#include "i2c.h"
#endif

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#endif

#if ETHERNET_ENABLE
  #include "shared/ethernet/enet.h"
  #if TELNET_ENABLE
    #include "networking/TCPStream.h"
  #endif
  #if WEBSOCKET_ENABLE
    #include "networking/WsStream.h"
  #endif
#endif

#define USE_32BIT_TIMER 1

#if USE_32BIT_TIMER
#define USE_PIOSC 0 // enabling PIOSC does not work!
#define STEPPER_TIMER TIMER_BOTH
#else
#define STEPPER_TIMER TIMER_A
// prescale step counter to 20Mhz (120 / (STEPPER_DRIVER_PRESCALER + 1))
#define STEPPER_DRIVER_PRESCALER 5

static uint16_t step_prescaler[3] = {
    STEPPER_DRIVER_PRESCALER,
    STEPPER_DRIVER_PRESCALER + 8,
    STEPPER_DRIVER_PRESCALER + 64
};
#endif

#define STEPPER_PULSE_PRESCALER (12 - 1)

#if ETHERNET_ENABLE

static network_services_t services = {0};

static void enetStreamWriteS (const char *data)
{
#if TELNET_ENABLE
    if(services.telnet)
        TCPStreamWriteS(data);
#endif
#if WEBSOCKET_ENABLE
    if(services.websocket)
        WsStreamWriteS(data);
#endif
    serialWriteS(data);
}

  #if TELNET_ENABLE
    const io_stream_t ethernet_stream = {
        .type = StreamType_Telnet,
        .read = TCPStreamGetC,
        .write = TCPStreamWriteS,
        .write_all = enetStreamWriteS,
        .get_rx_buffer_available = TCPStreamRxFree,
        .reset_read_buffer = TCPStreamRxFlush,
        .cancel_read_buffer = TCPStreamRxCancel,
        .suspend_read = TCPStreamSuspendInput,
        .enqueue_realtime_command = protocol_enqueue_realtime_command
    };
  #endif

  #if WEBSOCKET_ENABLE
    const io_stream_t websocket_stream = {
        .type = StreamType_WebSocket,
        .read = WsStreamGetC,
        .write = WsStreamWriteS,
        .write_all = enetStreamWriteS,
        .get_rx_buffer_available = WsStreamRxFree,
        .reset_read_buffer = WsStreamRxFlush,
        .cancel_read_buffer = WsStreamRxCancel,
        .suspend_read = WsStreamSuspendInput,
        .enqueue_realtime_command = protocol_enqueue_realtime_command
    };
  #endif

#endif // ETHERNET_ENABLE

const io_stream_t serial_stream = {
    .type = StreamType_Serial,
    .read = serialGetC,
    .write = serialWriteS,
#if ETHERNET_ENABLE
    .write_all = enetStreamWriteS,
#else
    .write_all = serialWriteS,
#endif
    .get_rx_buffer_available = serialRxFree,
    .reset_read_buffer = serialRxFlush,
    .cancel_read_buffer = serialRxCancel,
    .enqueue_realtime_command = protocol_enqueue_realtime_command,
#if M6_ENABLE
    .suspend_read = serialSuspendInput
#else
    .suspend_read = NULL
#endif
};


#if PWM_RAMPED

#define SPINDLE_RAMP_STEP_INCR 20 // timer compare register change per ramp step
#define SPINDLE_RAMP_STEP_TIME 2  // ms

typedef struct {
    volatile uint32_t ms_cfg;
    volatile uint32_t delay_ms;
    int32_t pwm_current;
    int32_t pwm_target;
    int32_t pwm_step;
} pwm_ramp_t;

static pwm_ramp_t pwm_ramp;
#endif

#if LASER_PPI

laser_ppi_t laser;

static void ppi_timeout_isr (void);

#endif

#if SPINDLE_SYNC_ENABLE

typedef struct {                     // Set when last encoder pulse count did not match at last index
    float block_start;
    float prev_pos;
    float dpp; // distance per pulse in mm
    void (*stepper_pulse_start_normal)(stepper_t *stepper);
    uint32_t timer_value_start;
    uint_fast8_t segment_id;
    uint32_t segments;
} spindle_sync_t;

static void stepperPulseStartSyncronized (stepper_t *stepper);

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
    {CONTROL_PORT_RST, RESET_PIN, false, false, false},
    {CONTROL_PORT_FH_CS, FEED_HOLD_PIN, false, false, false},
    {CONTROL_PORT_FH_CS, CYCLE_START_PIN, false, false, false},
#ifdef SAFETY_DOOR_PIN
    {CONTROL_PORT_SD, SAFETY_DOOR_PIN, false, false, false},
#endif
    {PROBE_PORT, PROBE_PIN, false, false, false},
    {LIMIT_PORT_X, X_LIMIT_PIN, false, false, false},
    {LIMIT_PORT_YZ, Y_LIMIT_PIN, false, false, false},
    {LIMIT_PORT_YZ, Z_LIMIT_PIN, false, false, false}
#if CNC_BOOSTERPACK2
,   {LIMIT_PORT_A, A_LIMIT_PIN, false, false, false},
    {LIMIT_PORT_B, B_LIMIT_PIN, false, false, false},
    {LIMIT_PORT_C, C_LIMIT_PIN, false, false, false}
#endif
};

static bool pwmEnabled = false, IOInitDone = false;
static uint32_t pulse_length, pulse_delay;
#ifndef FreeRTOS
static volatile uint32_t elapsed_tics = 0;
#endif
static axes_signals_t next_step_outbits;
static spindle_pwm_t spindle_pwm = {0};
static delay_t delay = { .ms = 1, .callback = NULL }; // NOTE: initial ms set to 1 for "resetting" systick timer on startup
static probe_state_t probe = {
    .connected = On
};

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

#if CNC_BOOSTERPACK2

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

void selectStream (stream_type_t stream)
{
    static stream_type_t active_stream = StreamType_Serial;

    switch(stream) {

#if TELNET_ENABLE
        case StreamType_Telnet:
            hal.stream.write_all("[MSG:TELNET STREAM ACTIVE]" ASCII_EOL);
            memcpy(&hal.stream, &ethernet_stream, sizeof(io_stream_t));
            services.telnet = On;
            break;
#endif
#if WEBSOCKET_ENABLE
        case StreamType_WebSocket:
            hal.stream.write_all("[MSG:WEBSOCKET STREAM ACTIVE]" ASCII_EOL);
            memcpy(&hal.stream, &websocket_stream, sizeof(io_stream_t));
            services.websocket = On;
            break;
#endif
        case StreamType_Serial:
            memcpy(&hal.stream, &serial_stream, sizeof(io_stream_t));
#if ETHERNET_ENABLE
            services.mask = 0;
#endif
            if(active_stream != StreamType_Serial)
                hal.stream.write_all("[MSG:SERIAL STREAM ACTIVE]" ASCII_EOL);
            break;

        default:
            break;
    }

    active_stream = stream;
}

static void spindle_set_speed (uint_fast16_t pwm_value);

// Interrupt handler prototypes

static void stepper_driver_isr (void);
static void stepper_pulse_isr (void);
static void stepper_pulse_isr_delayed (void);
static void mode_select_isr (void);
#if CNC_BOOSTERPACK
static void limit_yz_isr (void);
static void limit_debounced_yz_isr (void);
static void limit_x_isr (void);
static void limit_debounced_x_isr (void);
#if CNC_BOOSTERPACK2
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

/*
boolean bCalledFromInterrupt (void)
{
    return (SCB->ICSR & SCBICSRVECTACTIVE_Msk) != 0;
}
*/

void vTimerCallback (TimerHandle_t xTimer)
{
    if(delay.callback)
        delay.callback();
}

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if(callback) {

        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//        if(xPortIsInsideInterrupt())

        xTimerStopFromISR(xDelayTimer, &xHigherPriorityTaskWoken);

        if(delay.callback)
            delay.callback();

        delay.callback = callback;

        xTimerChangePeriodFromISR(xDelayTimer, pdMS_TO_TICKS(ms), &xHigherPriorityTaskWoken);
        xTimerStartFromISR(xDelayTimer, &xHigherPriorityTaskWoken);
    } else
        vTaskDelay(pdMS_TO_TICKS(ms));
}
#else

static void systick_isr (void);

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if(ms) {
        delay.ms = ms;
        SysTickEnable();
        if(!(delay.callback = callback))
            while(delay.ms);
    } else {
        if(delay.ms) {
            delay.callback = 0;
            delay.ms = 1;
        }
        if(callback)
            callback();
    }
}
#endif

// Set stepper pulse output pins
// NOTE: step_outbits are: bit0 -> X, bit1 -> Y, bit2 -> Z...
// Mapping to registers can be done by
// 1. bitbanding. Pros: can assign pins to different ports, no RMW needed. Cons: overhead, pin changes not synchronous
// 2. bit shift. Pros: fast, Cons: bits must be consecutive
// 3. lookup table. Pros: signal inversions done at setup, Cons: slower than bit shift
inline static __attribute__((always_inline)) void set_step_outputs (axes_signals_t step_outbits)
{
#if STEP_OUTMODE == GPIO_BITBAND
    step_outbits.value ^= settings.steppers.step_invert.mask;
    HWREGBITW(&STEP_OUT_X->DATA, X_STEP_BIT) = step_outbits.x;
    HWREGBITW(&STEP_OUT_Y->DATA, Y_STEP_BIT) = step_outbits.y;
    HWREGBITW(&STEP_OUT_Z->DATA, Z_STEP_BIT) = step_outbits.z;
#ifdef A_AXIS
    HWREGBITW(&STEP_OUT_A->DATA, A_STEP_BIT) = step_outbits.a;
#endif
#ifdef B_AXIS
    HWREGBITW(&STEP_OUT_B->DATA, B_STEP_BIT) = step_outbits.b;
#endif
#ifdef C_AXIS
    HWREGBITW(&STEP_OUT_C->DATA, C_STEP_BIT) = step_outbits.c;
#endif
#else
#if CNC_BOOSTERPACK2
    step_outbits.value ^= settings.steppers.step_invert.mask;
    GPIOPinWrite(STEP_PORT, HWSTEP_MASK, step_outbits.value << STEP_OUTMODE);
    GPIOPinWrite(STEP_PORT_AB, HWSTEP_MASK_AB, step_outbits.value >> (3 - STEP_OUTMODE_2));
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
#endif
}

// Set stepper direction output pins
// NOTE: see note for set_step_outputs()
inline static __attribute__((always_inline)) void set_dir_outputs (axes_signals_t dir_outbits)
{
#if STEP_OUTMODE == GPIO_BITBAND
    dir_outbits.value ^= settings.steppers.dir_invert.mask;
    HWREGBITW(&DIRECTION_OUT_X->DATA, X_DIRECTION_BIT) = dir_outbits.x;
    HWREGBITW(&DIRECTION_OUT_Y->DATA, Y_DIRECTION_BIT) = dir_outbits.y;
    HWREGBITW(&DIRECTION_OUT_Z->DATA, Z_DIRECTION_BIT) = dir_outbits.z;
#ifdef A_AXIS
    HWREGBITW(&DIRECTION_OUT_A->DATA, A_DIRECTION_BIT) = dir_outbits.a;
#endif
#ifdef B_AXIS
    HWREGBITW(&DIRECTION_OUT_B->DATA, B_DIRECTION_BIT) = dir_outbits.b;
#endif
#ifdef C_AXIS
    HWREGBITW(&DIRECTION_OUT_C->DATA, C_DIRECTION_BIT) = dir_outbits.c;
#endif
#else
#if CNC_BOOSTERPACK2
    dir_outbits.value ^= settings.steppers.dir_invert.mask;
    GPIOPinWrite(DIRECTION_PORT, HWDIRECTION_MASK, dir_outmap[dir_outbits.value & 0x07]);
    GPIOPinWrite(DIRECTION_PORT2, HWDIRECTION_MASK2, dir_outmap2[dir_outbits.value >> 3]);
#else
  #if DIRECTION_OUTMODE == GPIO_MAP
    GPIOPinWrite(DIRECTION_PORT, HWDIRECTION_MASK, dir_outmap[dir_outbits.value]);
  #else
    GPIOPinWrite(DIRECTION_PORT, HWDIRECTION_MASK, (dir_outbits.value ^ settings.steppers.dir_invert.mask) << DIRECTION_OUTMODE);
  #endif
#endif
#endif
}

// Enable/disable steppers
static void stepperEnable (axes_signals_t enable)
{
    enable.mask ^= settings.steppers.enable_invert.mask;
#if TRINAMIC_ENABLE && TRINAMIC_I2C
    axes_signals_t tmc_enable = trinamic_stepper_enable(enable);
  #if !CNC_BOOSTERPACK // Trinamic BoosterPack does not support mixed drivers
    if(!tmc_enable.z)
        GPIOPinWrite(STEPPERS_DISABLE_Z_PORT, STEPPERS_DISABLE_Z_PIN, enable.z ? STEPPERS_DISABLE_Z_PIN : 0);
    if(!tmc_enable.x)
        GPIOPinWrite(STEPPERS_DISABLE_Z_PORT, STEPPERS_DISABLE_Z_PIN, enable.z ? STEPPERS_DISABLE_Z_PIN : 0);
  #endif
#elif CNC_BOOSTERPACK
    GPIOPinWrite(STEPPERS_DISABLE_XY_PORT, STEPPERS_DISABLE_XY_PIN, enable.x ? STEPPERS_DISABLE_XY_PIN : 0);
    GPIOPinWrite(STEPPERS_DISABLE_Z_PORT, STEPPERS_DISABLE_Z_PIN, enable.z ? STEPPERS_DISABLE_Z_PIN : 0);
 #if CNC_BOOSTERPACK2
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
    TimerLoadSet(PULSE_TIMER_BASE, TIMER_A, pulse_length);

#if LASER_PPI
    laser.next_pulse = 0;
#endif

    // Enable stepper drivers.
    stepperEnable((axes_signals_t){AXES_BITMASK});

    TimerLoadSet(STEPPER_TIMER_BASE, TIMER_A, 5000);    // dummy...
    TimerEnable(STEPPER_TIMER_BASE, STEPPER_TIMER);

    hal.stepper.interrupt_callback(); // Start the show...
}

// Disables stepper driver interrupts and reset outputs
static void stepperGoIdle (bool clear_signals)
{
    TimerDisable(STEPPER_TIMER_BASE, STEPPER_TIMER);

    if(clear_signals) {
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}

// Sets up stepper driver interrupt timeout, AMASS version
static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
#if USE_32BIT_TIMER
#if USE_PIOSC
// Limit min steps/s to about 2 (hal.f_step_timer @ 16 MHz)
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    TimerLoadSet(STEPPER_TIMER_BASE, TIMER_A, cycles_per_tick < (1UL << 20) ? cycles_per_tick : (1UL << 20) - 1UL);
  #else
    TimerLoadSet(STEPPER_TIMER_BASE, TIMER_A, cycles_per_tick < (1UL << 23) ? cycles_per_tick : (1UL << 23) - 1UL);
  #endif
#else
// Limit min steps/s to about 2 (hal.f_step_timer @ 120 MHz)
  #ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    TimerLoadSet(STEPPER_TIMER_BASE, TIMER_A, cycles_per_tick < (1UL << 23) ? cycles_per_tick : (1UL << 23) - 1UL);
  #else
    TimerLoadSet(STEPPER_TIMER_BASE, TIMER_A, cycles_per_tick < (1UL << 26) ? cycles_per_tick : (1UL << 26) - 1UL);
  #endif
#endif
#else
    TimerLoadSet(STEPPER_TIMER_BASE, TIMER_A, cycles_per_tick < (1UL << 16) ? cycles_per_tick : 0xFFFF);
#endif
}

#if !USE_32BIT_TIMER
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
    TimerPrescaleSet(STEPPER_TIMER_BASE, STEPPER_TIMER, prescaler);
    TimerLoadSet(STEPPER_TIMER_BASE, STEPPER_TIMER, cycles_per_tick < (1UL << 16) /*< 65536 (4.1ms @ 16MHz)*/ ? cycles_per_tick : 0xFFFF /*Just set the slowest speed possible.*/);
}
#endif

// "Normal" version: Sets stepper direction and pulse pins and starts a step pulse a few nanoseconds later.
// If spindle synchronized motion switch to PID version.
static void stepperPulseStart (stepper_t *stepper)
{
#if SPINDLE_SYNC_ENABLE
    if(stepper->new_block) {
        if(stepper->exec_segment->spindle_sync) {
            spindle_tracker.stepper_pulse_start_normal = hal.stepper_pulse_start;
            hal.stepper_pulse_start = stepperPulseStartSyncronized;
            stepperPulseStartSyncronized(stepper);
            return;
        }
        set_dir_outputs(stepper->dir_outbits);
    }
#else
    if(stepper->dir_change)
        set_dir_outputs(stepper->dir_outbits);
#endif

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        TimerEnable(PULSE_TIMER_BASE, TIMER_A);
    }
}

// Delayed pulse version: sets stepper direction and pulse pins and starts a step pulse with an initial delay.
// If spindle synchronized motion switch to PID version.
// TODO: only delay after setting dir outputs?
static void stepperPulseStartDelayed (stepper_t *stepper)
{
#if SPINDLE_SYNC_ENABLE
    if(stepper->new_block) {
        if(stepper->exec_segment->spindle_sync) {
            spindle_tracker.stepper_pulse_start_normal = hal.stepper_pulse_start;
            hal.stepper_pulse_start = stepperPulseStartSyncronized;
            stepperPulseStartSyncronized(stepper);
            return;
        }
        set_dir_outputs(stepper->dir_outbits);
    }
#else
    if(stepper->dir_change) {

        set_dir_outputs(stepper->dir_outbits);

        if(stepper->step_outbits.value) {
            next_step_outbits = stepper->step_outbits; // Store out_bits
            IntRegister(PULSE_TIMER_INT, stepper_pulse_isr_delayed);
            TimerLoadSet(PULSE_TIMER_BASE, TIMER_A, pulse_delay);
            TimerEnable(PULSE_TIMER_BASE, TIMER_A);
        }

       return;
    }
#endif

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        TimerEnable(PULSE_TIMER_BASE, TIMER_A);
    }
}

#if SPINDLE_SYNC_ENABLE

#error Spindle sync code not ready!

// Spindle sync version: sets stepper direction and pulse pins and starts a step pulse.
// Switches back to "normal" version if spindle synchronized motion is finished.
// TODO: add delayed pulse handling...
static void stepperPulseStartSyncronized (stepper_t *stepper)
{
    static spindle_sync_t spindle_sync;

    if(stepper->new_block) {
        if(!stepper->exec_segment->spindle_sync) {
            hal.stepper_pulse_start = spindle_tracker.stepper_pulse_start_normal;
            hal.stepper_pulse_start(stepper);
            return;
        } else {
            spindle_sync.dpp = stepper->exec_block->programmed_rate * 120.0f;
            spindle_sync.prev_pos = 0.0f;
            spindle_sync.timer_value_start = 123;
            spindle_sync.block_start = 2.33f;
            spindle_sync.segments = 0;
            spindle_sync.segment_id = stepper->exec_segment->id + 1; // force recalc
        }
        set_dir_outputs(stepper->dir_outbits);
    }

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        TimerEnable(PULSE_TIMER_BASE, TIMER_A);
    }

    if(spindle_sync.segment_id != stepper->exec_segment->id) {

        spindle_sync.segment_id = stepper->exec_segment->id;

        float dist = stepper->exec_segment->target_position - spindle_sync.prev_pos;

        float epulses = dist * spindle_sync.dpp;

        sys.pid_log.target[spindle_sync.segments] = stepper->exec_segment->target_position;

        spindle_sync.segments++;

 //       float current_pos = (spindleGetData(true).angular_position - spindle_sync.block_start) * stepper->exec_block->programmed_rate;

        spindle_sync.prev_pos = stepper->exec_segment->target_position;
    }
}
#endif

#if LASER_PPI

static void spindle_on ();

// Sets stepper direction and pulse pins and starts a step pulse with an initial delay
// When delayed pulse the step register is written in the step delay interrupt handler
static void stepperPulseStartPPI (stepper_t *stepper)
{
    static uint_fast16_t current_pwm = 0;

    if(stepper->new_block) {
        stepper->new_block = false;
        set_dir_outputs(stepper->dir_outbits);
        uint_fast16_t steps_per_pulse = stepper->exec_block->steps_per_mm * 25.4f / laser.ppi;
        if(laser.next_pulse && laser.steps_per_pulse)
            laser.next_pulse = laser.next_pulse * steps_per_pulse / laser.steps_per_pulse;
        laser.steps_per_pulse = steps_per_pulse;
    }

    if(stepper->step_outbits.value) {
        if(stepper->spindle_pwm != current_pwm) {
            current_pwm = spindle_set_speed(stepper->spindle_pwm);
            laser.next_pulse = 0;
        }

        if(laser.next_pulse == 0) {
            laser.next_pulse = laser.steps_per_pulse;
            if(current_pwm != spindle_pwm.off_value) {
                spindle_on();
                TimerEnable(LASER_PPI_TIMER_BASE, TIMER_A);
                // TODO: T2CCP0 - use timer timeout to switch off CCP output w/o using interrupt? single shot PWM?
            }
        } else
            laser.next_pulse--;

        set_step_outputs(stepper->step_outbits);
        TimerEnable(PULSE_TIMER_BASE, TIMER_A);
    }
}
#endif

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing)
{
#if CNC_BOOSTERPACK
    uint32_t i;
  #if CNC_BOOSTERPACK2
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

#if TRINAMIC_ENABLE
    trinamic_homing(homing);
#endif
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};
    signals.min.value = settings.limits.invert.value;

#if CNC_BOOSTERPACK // Change to bit-band read to avoid call overhead!
    uint32_t flags = GPIOPinRead(LIMIT_PORT_YZ, HWLIMIT_MASK_YZ);

    signals.min.x = GPIOPinRead(LIMIT_PORT_X, X_LIMIT_PIN) != 0;
    signals.min.y = (flags & Y_LIMIT_PIN) != 0;
    signals.min.z = (flags & Z_LIMIT_PIN) != 0;
#ifdef A_AXIS
    signals.min.a = GPIOPinRead(LIMIT_PORT_A, A_LIMIT_PIN) != 0;
#endif
#ifdef B_AXIS
    signals.min.b = GPIOPinRead(LIMIT_PORT_B, B_LIMIT_PIN) != 0;
#endif
#ifdef C_AXIS
    signals.min.c = GPIOPinRead(LIMIT_PORT_C, C_LIMIT_PIN) != 0;
#endif
#else
    uint32_t flags = GPIOPinRead(LIMIT_PORT, HWLIMIT_MASK);

    signals.min.x = (flags & X_LIMIT_PIN) != 0;
    signals.min.y = (flags & Y_LIMIT_PIN) != 0;
    signals.min.z = (flags & Z_LIMIT_PIN) != 0;
#endif

    if (settings.limits.invert.value)
        signals.min.value ^= settings.limits.invert.value;

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
inline static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.value = settings.control_invert.value;

#if CNC_BOOSTERPACK
    uint32_t flags = GPIOPinRead(CONTROL_PORT_FH_CS, HWCONTROL_MASK);
    signals.feed_hold = (flags & FEED_HOLD_PIN) != 0;
    signals.cycle_start = (flags & CYCLE_START_PIN) != 0;
  #if CNC_BOOSTERPACK_SHORTS
    signals.reset = (flags & RESET_PIN) != 0;
  #else
    signals.reset = GPIOPinRead(CONTROL_PORT_RST, RESET_PIN) != 0;
  #endif
  #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    signals.safety_door_ajar = GPIOPinRead(CONTROL_PORT_SD, SAFETY_DOOR_PIN) != 0;
  #endif
#else
    uint32_t flags = GPIOPinRead(CONTROL_PORT, HWCONTROL_MASK);

    signals.reset = (flags & RESET_PIN) != 0;
    signals.feed_hold = (flags & FEED_HOLD_PIN) != 0;
    signals.cycle_start = (flags & CYCLE_START_PIN) != 0;
  #ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    signals.safety_door_ajar = (flags & SAFETY_DOOR_PIN) != 0;
  #endif
#endif

    if(settings.control_invert.value)
        signals.value ^= settings.control_invert.value;

    return signals;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure (bool is_probe_away, bool probing)
{
    probe.triggered = Off;
    probe.is_probing = probing;
    probe.inverted = is_probe_away ? !settings.probe.invert_probe_pin : settings.probe.invert_probe_pin;
/*
    GPIOIntDisable(PROBE_PORT, PROBE_PIN);
    GPIOIntTypeSet(PROBE_PORT, PROBE_PIN, probe.inverted ? GPIO_FALLING_EDGE : GPIO_RISING_EDGE);
    if(probing)
        GPIOIntEnable(PROBE_PORT, PROBE_PIN);
        */
}

// Returns the probe connected and pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;
    //state.triggered = probeState; // TODO: check out using interrupt instead (we want to trap trigger and not risk losing it due to bouncing)
    state.triggered = !!GPIOPinRead(PROBE_PORT, PROBE_PIN) ^ probe.inverted;

    return state;
}

// Static spindle (off, on cw & on ccw)

inline static void spindle_off ()
{
    GPIOPinWrite(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? SPINDLE_ENABLE_PIN : 0);
}

inline static void spindle_on ()
{
    GPIOPinWrite(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? 0 : SPINDLE_ENABLE_PIN);
}

inline static void spindle_dir (bool ccw)
{
    GPIOPinWrite(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, (ccw ^ settings.spindle.invert.ccw) ? SPINDLE_DIRECTION_PIN : 0);
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
#if PWM_RAMPED

static void spindle_set_speed (uint_fast16_t pwm_value)
{
    if (pwm_value == spindle_pwm.off_value) {
        pwm_ramp.pwm_target = 0;
        pwm_ramp.pwm_step = -SPINDLE_RAMP_STEP_INCR;
        pwm_ramp.delay_ms = 0;
        pwm_ramp.ms_cfg = SPINDLE_RAMP_STEP_TIME;
        SysTickEnable();
     } else {

        if(!pwmEnabled) {
            spindle_on();
            pwmEnabled = true;
            pwm_ramp.pwm_current = spindle_pwm.min_value;
            pwm_ramp.delay_ms = 0;
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
}

#else

static void spindle_set_speed (uint_fast16_t pwm_value)
{
    if (pwm_value == spindle_pwm.off_value) {
        pwmEnabled = false;
        if(settings.spindle.flags.pwm_action == SpindleAction_DisableWithZeroSPeed)
            spindle_off();
        if(spindle_pwm.always_on) {
            TimerPrescaleMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.off_value >> 16);
            TimerMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.off_value & 0xFFFF);
            TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, !settings.spindle.invert.pwm);
            TimerEnable(SPINDLE_PWM_TIMER_BASE, TIMER_B); // Ensure PWM output is enabled.
        } else {
            uint_fast16_t pwm = spindle_pwm.period + 20000;
            TimerPrescaleSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, pwm >> 16);
            TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, pwm & 0xFFFF);
            if(!pwmEnabled)
                TimerEnable(SPINDLE_PWM_TIMER_BASE, TIMER_B);                                   // Ensure PWM output is enabled to
            TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, !settings.spindle.invert.pwm);   // ensure correct output level.
            TimerDisable(SPINDLE_PWM_TIMER_BASE, TIMER_B);                                      // Disable PWM.
        }
     } else {
        TimerPrescaleMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, pwm_value >> 16);
        TimerMatchSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, pwm_value & 0xFFFF);
        if(!pwmEnabled) {
            spindle_on();
            pwmEnabled = true;
            TimerPrescaleSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period >> 16);
            TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period & 0xFFFF);
            TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, !settings.spindle.invert.pwm);
            TimerEnable(SPINDLE_PWM_TIMER_BASE, TIMER_B); // Ensure PWM output is enabled.
        }
    }
}

#endif

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

#if SPINDLE_SYNC_ENABLE

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

    state.on = GPIOPinRead(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN) != 0;
    if(hal.driver_cap.spindle_dir)
        state.ccw = GPIOPinRead(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN) != 0;
    state.value ^= settings.spindle.invert.mask;
    if(pwmEnabled)
        state.on = On;
#if PWM_RAMPED
    state.at_speed = pwm_ramp.pwm_current == pwm_ramp.pwm_target;
#endif
#if SPINDLE_SYNC_ENABLE
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

static void enable_irq (void)
{
    IntMasterEnable();
}

static void disable_irq (void)
{
    IntMasterDisable();
}

static void modeSelect (bool mpg_mode)
{
    static stream_type_t normal_stream = StreamType_Serial;

    // Deny entering MPG mode if busy
    if(mpg_mode == sys.mpg_mode || (mpg_mode && (gc_state.file_run || !(state_get() == STATE_IDLE || (state_get() & (STATE_ALARM|STATE_ESTOP)))))) {
        hal.stream.enqueue_realtime_command(CMD_STATUS_REPORT_ALL);
        return;
    }

    serialSelect(mpg_mode);

    if(mpg_mode) {
        normal_stream = hal.stream.type;
        hal.stream.read = serial2GetC;
        hal.stream.get_rx_buffer_available = serial2RxFree;
        hal.stream.cancel_read_buffer = serial2RxCancel;
        hal.stream.reset_read_buffer = serial2RxFlush;
    } else
        selectStream(normal_stream);

    hal.stream.reset_read_buffer();

    sys.mpg_mode = mpg_mode;
    sys.report.mpg_mode = On;

    // Force a realtime status report, all reports when MPG mode active
    hal.stream.enqueue_realtime_command(mpg_mode ? CMD_STATUS_REPORT_ALL : CMD_STATUS_REPORT);
}

static void modechange (void)
{
    modeSelect(GPIOPinRead(MODE_PORT, MODE_SWITCH_PIN) == 0);
    GPIOIntEnable(MODE_PORT, MODE_SWITCH_PIN);
}

#ifndef FreeRTOS

uint32_t getElapsedTicks (void)
{
    return elapsed_tics;
}

#endif

// Configures perhipherals when settings are initialized or changed
static void settings_changed (settings_t *settings)
{
    spindle_pwm.offset = -1;
    hal.driver_cap.variable_spindle = spindle_precompute_pwm_values(&spindle_pwm, configCPU_CLOCK_HZ);

#if (STEP_OUTMODE == GPIO_MAP) || (DIRECTION_OUTMODE == GPIO_MAP)
    uint8_t i;
#endif

#if STEP_OUTMODE == GPIO_MAP
    for(i = 0; i < sizeof(step_outmap); i++)
        step_outmap[i] = c_step_outmap[i ^ settings->step_invert.mask];
#endif

#if DIRECTION_OUTMODE == GPIO_MAP
    for(i = 0; i < sizeof(dir_outmap); i++)
        dir_outmap[i] = c_dir_outmap[i ^ settings->steppers.dir_invert.mask & 0x07];
#if CNC_BOOSTERPACK2
    for(i = 0; i < sizeof(dir_outmap2); i++)
        dir_outmap2[i] = c_dir_outmap2[i ^ settings->steppers.dir_invert.mask >> 3];
#endif
#endif

    if(IOInitDone) {

#if ETHERNET_ENABLE

        static bool enet_ok = false;
        if(!enet_ok)
            enet_ok = enet_start();

#endif

        stepperEnable(settings->steppers.deenergize);

        if(hal.driver_cap.variable_spindle) {
            TimerPrescaleSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period >> 16);
            TimerLoadSet(SPINDLE_PWM_TIMER_BASE, TIMER_B, spindle_pwm.period & 0xFFFF);
            hal.spindle.set_state = spindleSetStateVariable;
        } else
            hal.spindle.set_state = spindleSetState;

        pulse_length = (uint32_t)(10.0f * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY)) - 1;

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            int32_t delay = (uint32_t)(10.0f * (settings->steppers.pulse_delay_microseconds - 1.2f)) - 1;
            pulse_delay = delay < 2 ? 2 : delay;
            hal.stepper.pulse_start = stepperPulseStartDelayed;
        } else
            hal.stepper.pulse_start = stepperPulseStart;

        TimerIntRegister(PULSE_TIMER_BASE, TIMER_A, stepper_pulse_isr);
        TimerIntEnable(PULSE_TIMER_BASE, TIMER_TIMA_TIMEOUT);

      #if LASER_PPI
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
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPION);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOP);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOQ);
    SysCtlPeripheralEnable(STEPPER_TIMER_PERIPH);
    SysCtlPeripheralEnable(PULSE_TIMER_PERIPH);

    SysCtlDelay(26); // wait a bit for peripherals to wake up

    /******************
     *  Stepper init  *
     ******************/
#ifndef __MSP432E401Y__
    // Unlock GPIOF0, used for stepper disable Z control
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= GPIO_PIN_0;
    HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
#endif

    GPIOPinTypeGPIOOutput(STEP_PORT, HWSTEP_MASK);
    GPIOPadConfigSet(STEP_PORT, HWSTEP_MASK, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOOutput(DIRECTION_PORT, HWDIRECTION_MASK);
    GPIOPadConfigSet(DIRECTION_PORT, HWDIRECTION_MASK, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

#if CNC_BOOSTERPACK2
    GPIOPinTypeGPIOOutput(STEP_PORT_AB, HWSTEP_MASK_AB);
    GPIOPadConfigSet(STEP_PORT_AB, HWSTEP_MASK_AB, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
    GPIOPinTypeGPIOOutput(STEP_PORT_C, C_STEP_PIN);
    GPIOPadConfigSet(STEP_PORT_C, C_STEP_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOOutput(DIRECTION_PORT2, HWDIRECTION_MASK2);
    GPIOPadConfigSet(DIRECTION_PORT2, HWDIRECTION_MASK2, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

#endif

#if CNC_BOOSTERPACK

 #ifndef __MSP432E401Y__
    // Unlock GPIOD7, used for disable XY
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= GPIO_PIN_7;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;
 #endif

 #if !TRINAMIC_ENABLE
    GPIOPinTypeGPIOOutput(STEPPERS_DISABLE_XY_PORT, STEPPERS_DISABLE_XY_PIN);
    GPIOPinTypeGPIOOutput(STEPPERS_DISABLE_Z_PORT, STEPPERS_DISABLE_Z_PIN);
  #if CNC_BOOSTERPACK2
    GPIOPinTypeGPIOOutput(STEPPERS_DISABLE_AC_PORT, STEPPERS_DISABLE_AC_PIN);
    GPIOPinTypeGPIOOutput(STEPPERS_DISABLE_B_PORT, STEPPERS_DISABLE_B_PIN);
  #endif
 #endif
#else
    GPIOPinTypeGPIOOutput(STEPPERS_DISABLE_PORT, STEPPERS_DISABLE_PIN);
#endif

    // Configure stepper driver timer
#if USE_32BIT_TIMER
  #if USE_PIOSC
    TimerClockSourceSet(STEPPER_TIMER_BASE, TIMER_CLOCK_PIOSC);
  #endif
    TimerConfigure(STEPPER_TIMER_BASE, TIMER_CFG_PERIODIC);
#else
    TimerConfigure(STEPPER_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_PERIODIC);
    TimerPrescaleSet(STEPPER_TIMER_BASE, TIMER_A, STEPPER_DRIVER_PRESCALER); // 20 MHz step timer clock
#endif
    IntPrioritySet(STEPPER_TIMER_INT, 0x20);                    // lower priority than for step timer (which resets the step-dir signal)
    TimerControlStall(STEPPER_TIMER_BASE, STEPPER_TIMER, true); // timer will stall in debug mode
    TimerIntRegister(STEPPER_TIMER_BASE, STEPPER_TIMER, stepper_driver_isr);
    TimerIntClear(STEPPER_TIMER_BASE, 0xFFFF);
    IntPendClear(STEPPER_TIMER_INT);
    TimerIntEnable(STEPPER_TIMER_BASE, TIMER_TIMA_TIMEOUT);

    // Configure step pulse timer
//  TimerClockSourceSet(PULSE_TIMER_BASE, TIMER_CLOCK_SYSTEM);
    TimerConfigure(PULSE_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_ONE_SHOT);
    IntPrioritySet(PULSE_TIMER_INT, 0x00);              // highest priority - higher than for stepper timer
    TimerControlStall(PULSE_TIMER_BASE, TIMER_A, true); // timer will stall in debug mode
    TimerIntClear(PULSE_TIMER_BASE, 0xFFFF);
    IntPendClear(PULSE_TIMER_INT);
    TimerPrescaleSet(PULSE_TIMER_BASE, TIMER_A, STEPPER_PULSE_PRESCALER); // for 0.1uS per count

#if CNC_BOOSTERPACK_A4998
    GPIOPinTypeGPIOOutput(STEPPERS_VDD_PORT, STEPPERS_VDD_PIN);
    GPIOPadConfigSet(STEPPERS_VDD_PORT, STEPPERS_VDD_PIN, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    GPIOPinWrite(STEPPERS_VDD_PORT, STEPPERS_VDD_PIN, STEPPERS_VDD_PIN);
  #if CNC_BOOSTERPACK2
    GPIOPinTypeGPIOOutput(STEPPERS_VDD_PORT2, STEPPERS_VDD_PIN2);
    GPIOPadConfigSet(STEPPERS_VDD_PORT2, STEPPERS_VDD_PIN2, GPIO_STRENGTH_12MA, GPIO_PIN_TYPE_STD);
    GPIOPinWrite(STEPPERS_VDD_PORT2, STEPPERS_VDD_PIN2, STEPPERS_VDD_PIN2);
  #endif
#endif

#if LASER_PPI

   /********************************
    *  PPI mode pulse width timer  *
    ********************************/

    laser.ppi = 600.0f;
    laser.pulse_length = 1500;

    SysCtlPeripheralEnable(LASER_PPI_TIMER_PERIPH);
    SysCtlDelay(26); // wait a bit for peripherals to wake up
    TimerConfigure(LASER_PPI_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_A_ONE_SHOT);
    IntPrioritySet(LASER_PPI_TIMER_INT, 0x40);              // lower priority than for step timer (which resets the step-dir signal)
    TimerControlStall(LASER_PPI_TIMER_BASE, TIMER_A, true); // timer will stall in debug mode
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

#if CNC_BOOSTERPACK
#if CNC_BOOSTERPACK_SHORTS
    GPIOIntRegister(CONTROL_PORT, control_isr);             // Register interrupt handler
#else
    GPIOIntRegister(CONTROL_PORT_FH_CS, control_isr);             // Register interrupt handler
#endif
    GPIOIntRegister(CONTROL_PORT_SD, control_isr_sd);       // Register interrupt handler for safety door
#else
    GPIOIntRegister(CONTROL_PORT, control_isr);             // Register interrupt handler
#endif

   /*********************
    *  Limit pins init  *
    *********************/
    // Register a call-back function(s) for interrupt
#if CNC_BOOSTERPACK
    GPIOIntRegister(LIMIT_PORT_X, hal.driver_cap.software_debounce ? limit_debounced_x_isr : limit_x_isr);
    GPIOIntRegister(LIMIT_PORT_YZ, hal.driver_cap.software_debounce ? limit_debounced_yz_isr : limit_yz_isr);
  #if CNC_BOOSTERPACK2
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

#if !USE_32BIT_TIMER
    if(hal.driver_cap.amass_level == 0)
        hal.stepper_cycles_per_tick = &stepperCyclesPerTickPrescaled;
#endif

   /******************
    *  Spindle init  *
    ******************/

    GPIOPinTypeGPIOOutput(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN);
    GPIOPadConfigSet(SPINDLE_ENABLE_PORT, SPINDLE_ENABLE_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOOutput(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN);
    GPIOPadConfigSet(SPINDLE_DIRECTION_PORT, SPINDLE_DIRECTION_PIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);

    SysCtlPeripheralEnable(SPINDLE_PWM_TIMER_PERIPH);
    SysCtlDelay(26); // wait a bit for peripherals to wake up
    TimerClockSourceSet(SPINDLE_PWM_TIMER_BASE, TIMER_CLOCK_SYSTEM);
    TimerConfigure(SPINDLE_PWM_TIMER_BASE, TIMER_CFG_SPLIT_PAIR|TIMER_CFG_B_PWM);
    TimerControlLevel(SPINDLE_PWM_TIMER_BASE, TIMER_B, false);
    GPIOPinConfigure(SPINDLEPWM_MAP);
    GPIOPinTypeTimer(SPINDLEPPORT, SPINDLEPPIN);
    GPIOPadConfigSet(SPINDLEPPORT, SPINDLEPPIN, GPIO_STRENGTH_8MA, GPIO_PIN_TYPE_STD);
  #if PWM_RAMPED
    pwm_ramp.ms_cfg = pwm_ramp.pwm_current = pwm_ramp.pwm_target = 0;
  #endif

#if SDCARD_ENABLE
  #ifdef __MSP432E401Y__
    SDFatFS_init();
  #endif
    sdcard_init();
#endif

#if KEYPAD_ENABLE

   /*********************
    *  I2C KeyPad init  *
    *********************/

    GPIOPinTypeGPIOInput(KEYINTR_PORT, KEYINTR_PIN);
    GPIOPadConfigSet(KEYINTR_PORT, KEYINTR_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); // -> WPU

    GPIOIntRegister(KEYINTR_PORT, keyclick_int_handler);
    GPIOIntTypeSet(KEYINTR_PORT, KEYINTR_PIN, GPIO_BOTH_EDGES);
    GPIOIntEnable(KEYINTR_PORT, KEYINTR_PIN);

#endif

#if TRINAMIC_ENABLE
/*
    // Configure input pin for DIAG1 signal (with pullup) and enable interrupt
    GPIOPinTypeGPIOInput(TRINAMIC_DIAG_IRQ_PORT, TRINAMIC_DIAG_IRQ_PIN);
    GPIOIntRegister(TRINAMIC_DIAG_IRQ_PORT, trinamic_diag1_isr); // Register a call-back function for interrupt
    GPIOPadConfigSet(TRINAMIC_DIAG_IRQ_PORT, TRINAMIC_DIAG_IRQ_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(TRINAMIC_DIAG_IRQ_PORT, TRINAMIC_DIAG_IRQ_PIN, GPIO_FALLING_EDGE);
    GPIOIntEnable(TRINAMIC_DIAG_IRQ_PORT, TRINAMIC_DIAG_IRQ_PIN);
*/
  #if TRINAMIC_I2C
  // Configure input pin for WARN signal (with pullup) and enable interrupt
    GPIOPinTypeGPIOInput(TRINAMIC_WARN_IRQ_PORT, TRINAMIC_WARN_IRQ_PIN);
//    GPIOIntRegister(TRINAMIC_WARN_IRQ_PORT, trinamic_warn_isr); // Register a call-back function for interrupt NOTE: same port as limit X
    GPIOPadConfigSet(TRINAMIC_WARN_IRQ_PORT, TRINAMIC_WARN_IRQ_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(TRINAMIC_WARN_IRQ_PORT, TRINAMIC_WARN_IRQ_PIN, GPIO_FALLING_EDGE);
    GPIOIntEnable(TRINAMIC_WARN_IRQ_PORT, TRINAMIC_WARN_IRQ_PIN);
  #endif

#endif

    /*********************
     * Mode select input *
     *********************/

    GPIOPadConfigSet(MODE_PORT, MODE_SWITCH_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    GPIOIntTypeSet(MODE_PORT, MODE_SWITCH_PIN, GPIO_BOTH_EDGES);
    GPIOIntClear(MODE_PORT, MODE_SWITCH_PIN);     // Clear any pending interrupt
    GPIOIntRegister(MODE_PORT, mode_select_isr);

  // Set defaults

    IOInitDone = settings->version == 19;

    hal.settings_changed(settings);
    hal.stepper.go_idle(true);
    hal.spindle.set_state((spindle_state_t){0}, 0.0f);
    hal.coolant.set_state((coolant_state_t){0});
    set_dir_outputs((axes_signals_t){0});

    return IOInitDone;
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{
#ifdef FreeRTOS
    hal.f_step_timer = configCPU_CLOCK_HZ;
    xDelayTimer = xTimerCreate("msDelay", pdMS_TO_TICKS(10), pdFALSE, NULL, vTimerCallback);
#else
    FPUEnable();
    FPULazyStackingEnable();
    hal.f_step_timer = SysCtlClockFreqSet(SYSCTL_XTAL_25MHZ|SYSCTL_OSC_MAIN|SYSCTL_USE_PLL|SYSCTL_CFG_VCO_480, 120000000);

    // Set up systick timer with a 1ms period
    SysTickPeriodSet((hal.f_step_timer / 1000) - 1);
    SysTickIntRegister(systick_isr);
    IntPrioritySet(FAULT_SYSTICK, 0x40);
    SysTickIntEnable();
    SysTickEnable();
#endif

#if USE_32BIT_TIMER && USE_PIOSC
    // Enable hibernation module, used to calibrate 16 MHZ PIOSC
    SysCtlPeripheralEnable(SYSCTL_PERIPH_HIBERNATE);
    SysCtlDelay(26); // wait a bit for peripheral to wake up
    HibernateEnableExpClk(hal.f_step_timer);
    HibernateClockConfig(HIBERNATE_OSC_LOWDRIVE);
    HibernateRTCEnable();
#endif

    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors
    SysCtlPeripheralEnable(SYSCTL_PERIPH_EEPROM0);
    SysCtlDelay(26); // wait a bit for peripheral to wake up
    EEPROMInit();

#if USE_32BIT_TIMER && USE_PIOSC
    // Moved after EEPROM enable for giving time for 32768 Hz crystal to stabilize
    SysCtlPIOSCCalibrate(SYSCTL_PIOSC_CAL_AUTO);
//    HibernateRTCDisable();
//    HibernateClockConfig(HIBERNATE_OSC_DISABLE);
//    HibernateDisable();
#endif

    serialInit();

#if I2C_ENABLE
    I2CInit();
#endif

#ifdef __MSP432E401Y__
  #ifdef FreeRTOS
    hal.info = "MSP432E401Y FreeRTOS";
  #else
    hal.info = "MSP432E401Y";
  #endif
#else // TM4C1294
  #ifdef FreeRTOS
    hal.info = "TM4C1294NCPDT FreeRTOS";
  #else
    hal.info = "TM4C1294NCPDT";
  #endif
#endif
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.driver_version = "210314";
    hal.driver_setup = driver_setup;
#if !USE_32BIT_TIMER
    hal.f_step_timer = hal.f_step_timer / (STEPPER_DRIVER_PRESCALER + 1);
#elif USE_PIOSC
    hal.f_step_timer = 16000000UL;
#endif
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
    hal.spindle.update_rpm = spindleUpdateRPM;
#endif
#if SPINDLE_SYNC_ENABLE
    hal.spindle.get_data = spindleGetData;
    hal.spindle.reset_data = spindleDataReset;
#endif

    hal.control.get_state = systemGetState;

    selectStream(StreamType_Serial);

    eeprom_init();

    hal.irq_enable = enable_irq;
    hal.irq_disable = disable_irq;
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
#ifdef FreeRTOS
    hal.get_elapsed_ticks = xTaskGetTickCountFromISR;
#else
    hal.get_elapsed_ticks = getElapsedTicks;
#endif

#ifdef DEBUGOUT
    hal.debug_out = debug_out;
#endif

#ifdef _ATC_H_
    hal.tool_select = atc_tool_selected;
    hal.tool_change = atc_tool_change;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

#ifdef SAFETY_DOOR_PIN
    hal.signals_cap.safety_door_ajar = On;
#endif
    hal.driver_cap.spindle_dir = On;
    hal.driver_cap.variable_spindle = On;
    hal.driver_cap.spindle_pwm_invert = On;
#if PWM_RAMPED
    hal.driver_cap.spindle_at_speed = On;
#endif
#if SPINDLE_SYNC_ENABLE
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
#if LASER_PPI
    hal.driver_cap.laser_ppi_mode = On;
#endif

#if ETHERNET_ENABLE
    enet_init();
#endif

#if TRINAMIC_ENABLE
    trinamic_init();
#endif

#if KEYPAD_ENABLE
    keypad_init();
#endif

    my_plugin_init();

    // no need to move version check before init - compiler will fail any signature mismatch for existing entries
    return hal.version == 8;
}

/* interrupt handlers */

// Main stepper driver
static void stepper_driver_isr (void)
{
    TimerIntClear(STEPPER_TIMER_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag
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
// NOTE: TivaC has a shared interrupt for match and timeout
static void stepper_pulse_isr (void)
{
    TimerIntClear(PULSE_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    set_step_outputs((axes_signals_t){0});
}

static void stepper_pulse_isr_delayed (void)
{
    TimerIntClear(PULSE_TIMER_BASE, TIMER_TIMA_TIMEOUT);
    IntRegister(PULSE_TIMER_INT, stepper_pulse_isr);

    set_step_outputs(next_step_outbits);

    TimerLoadSet(PULSE_TIMER_BASE, TIMER_A, pulse_length);
    TimerEnable(PULSE_TIMER_BASE, TIMER_A);
}

#if LASER_PPI

void laser_ppi_mode (bool on)
{
    if(on)
        hal.stepper_pulse_start = stepperPulseStartPPI;
    else
        hal.stepper_pulse_start = settings.pulse_delay_microseconds > 0.0f ? stepperPulseStartDelayed : stepperPulseStart;
    gc_set_laser_ppimode(on);
}

// Switches off the spindle (laser) after laser.pulse_length time has elapsed
static void ppi_timeout_isr (void)
{
    TimerIntClear(LASER_PPI_TIMER_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag
    spindle_off();
}
#endif

static void software_debounce_isr (void)
{
    TimerIntClear(DEBOUNCE_TIMER_BASE, TIMER_TIMA_TIMEOUT); // clear interrupt flag

    limit_signals_t state = limitsGetState();
    if(limit_signals_merge(state).value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
        hal.limits.interrupt_callback(state);
}

#if CNC_BOOSTERPACK

static void limit_yz_isr (void)
{
    uint32_t iflags = GPIOIntStatus(LIMIT_PORT_YZ, true);

    GPIOIntClear(LIMIT_PORT_YZ, iflags);
    if(iflags & HWLIMIT_MASK_YZ)
        hal.limits.interrupt_callback(limitsGetState());
#if CNC_BOOSTERPACK_SHORTS
    if(iflags & RESET_PIN)
        hal.control.interrupt_callback(systemGetState());
#endif
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
#if !CNC_BOOSTERPACK_SHORTS
    if(iflags & RESET_PIN)
        hal.control.interrupt_callback(systemGetState());
#endif
}

static void limit_x_isr (void)
{
    uint32_t iflags = GPIOIntStatus(LIMIT_PORT_X, true);

    GPIOIntClear(LIMIT_PORT_X, iflags);

    if(iflags & X_LIMIT_PIN)
        hal.limits.interrupt_callback(limitsGetState());

#if TRINAMIC_ENABLE && TRINAMIC_I2C
    if(iflags & TRINAMIC_WARN_IRQ_PIN)
        trinamic_warn_handler();
#endif
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


#if KEYPAD_ENABLE

static void keyclick_int_handler (void)
{
    uint32_t iflags = GPIOIntStatus(KEYINTR_PORT, true);

    GPIOIntClear(KEYINTR_PORT, iflags);

    if(iflags & KEYINTR_PIN)
        keypad_keyclick_handler(GPIOPinRead(KEYINTR_PORT, KEYINTR_PIN) != 0);
}

#endif

/*
#if TRINAMIC_ENABLE && TRINAMIC_ENABLE != 2130

static void trinamic_diag1_isr (void)
{
    uint32_t iflags = GPIOIntStatus(TRINAMIC_DIAG_IRQ_PORT, true);

    GPIOIntClear(TRINAMIC_DIAG_IRQ_PORT, iflags);

    if(iflags & TRINAMIC_DIAG_IRQ_PIN)
        trinamic_fault_handler();
}

#endif
*/
#if CNC_BOOSTERPACK2

    static void limit_a_isr (void)
    {
        uint32_t iflags = GPIOIntStatus(LIMIT_PORT_A, true);

        GPIOIntClear(LIMIT_PORT_A, iflags);
        if(iflags & A_LIMIT_PIN)
            hal.limits.interrupt_callback(limitsGetState());
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
            hal.limits.interrupt_callback(limitsGetState());
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
            hal.limits.interrupt_callback(limitsGetState());
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
        hal.limits.interrupt_callback(limitsGetState());
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
#if CNC_BOOSTERPACK
  #if CNC_BOOSTERPACK_SHORTS
    uint32_t iflags = GPIOIntStatus(CONTROL_PORT, true) & HWCONTROL_MASK;
    if(iflags) {
        GPIOIntClear(CONTROL_PORT, iflags);
        hal.control.interrupt_callback(systemGetState());
    }
  #else
    uint32_t iflags = GPIOIntStatus(CONTROL_PORT_FH_CS, true) & HWCONTROL_MASK;
    if(iflags) {
        GPIOIntClear(CONTROL_PORT_FH_CS, iflags);
        hal.control.interrupt_callback(systemGetState());
    }
  #endif
#else
    uint32_t iflags = GPIOIntStatus(CONTROL_PORT, true) & HWCONTROL_MASK;
    if(iflags) {
        GPIOIntClear(CONTROL_PORT, iflags);
        hal.control.interrupt_callback(systemGetState());
    }
#endif
}

static void control_isr_sd (void)
{
// No debounce??
    uint32_t iflags = GPIOIntStatus(CONTROL_PORT_SD, true);

    GPIOIntClear(CONTROL_PORT_SD, iflags);

#if CNC_BOOSTERPACK2
    if(iflags & B_LIMIT_PIN) {
        if(hal.driver_cap.software_debounce) {
            TimerLoadSet(DEBOUNCE_TIMER_BASE, TIMER_A, 32000);  // 32ms
            TimerEnable(DEBOUNCE_TIMER_BASE, TIMER_A);
        } else
            hal.limits.interrupt_callback(limitsGetState());
    }
#endif
#ifdef SAFETY_DOOR_PIN
    if(iflags & SAFETY_DOOR_PIN)
        hal.control.interrupt_callback(systemGetState());
#endif
}

#ifndef FreeRTOS
// Interrupt handler for 1 ms interval timer
#if PWM_RAMPED
static void systick_isr (void)
{
    elapsed_tics++;
    
    if(pwm_ramp.ms_cfg) {
        if(++pwm_ramp.delay_ms == pwm_ramp.ms_cfg) {

            pwm_ramp.delay_ms = 0;
            pwm_ramp.pwm_current += pwm_ramp.pwm_step;

            if(pwm_ramp.pwm_step < 0) { // decrease speed

                if(pwm_ramp.pwm_current < pwm_ramp.pwm_target)
                    pwm_ramp.pwm_current = pwm_ramp.pwm_target;

                if(pwm_ramp.pwm_current == 0) { // stop?
                    if(settings.flags.spindle_disable_with_zero_speed)
                        spindle_off();
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

    if(delay.ms && !(--delay.ms) && delay.callback) {
        delay.callback();
        delay.callback = NULL;
    }
}
#else
static void systick_isr (void)
{
    elapsed_tics++;

    if(delay.ms && !(--delay.ms) && delay.callback) {
        delay.callback();
        delay.callback = NULL;
    }
}
#endif
#endif // FreeRTOS
