/*
  driver.c - driver code for IMXRT1062 processor (on Teensy 4.0/4.1 board)

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

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

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "uart.h"
#include "driver.h"
#include "grbl/protocol.h"
#include "grbl/limits.h"

#ifdef I2C_PORT
#include "i2c.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#else
#include "avr/eeprom.h"
#endif

#if IOPORTS_ENABLE
#include "ioports.h"
#endif

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#endif

#if KEYPAD_ENABLE
#include "keypad/keypad.h"
#endif

#if PLASMA_ENABLE
#include "plasma/thc.h"
#endif

#if PPI_ENABLE
#include "laser/ppi.h"
static void ppi_timeout_isr (void);
#endif

#if ODOMETER_ENABLE
#include "odometer/odometer.h"
#endif

#if ETHERNET_ENABLE
  #include "enet.h"
  #if TELNET_ENABLE
    #include "networking/TCPStream.h"
  #endif
  #if WEBSOCKET_ENABLE
    #include "networking/WsStream.h"
  #endif
#endif

#if USB_SERIAL_CDC == 1
#include "usb_serial_ard.h"
#elif USB_SERIAL_CDC == 2
#include "usb_serial_pjrc.h"
#endif

#if defined(ENABLE_SAFETY_DOOR_INPUT_PIN) && defined(SAFETY_DOOR_PIN)
#define SAFETY_DOOR_ENABLE 1
#else
#define SAFETY_DOOR_ENABLE 0
#ifdef SAFETY_DOOR_PIN
//#define LIMITS_OVERRIDE_PIN SAFETY_DOOR_PIN
#endif
#endif

#define DEBOUNCE_QUEUE 8 // Must be a power of 2

#define F_BUS_MHZ (F_BUS_ACTUAL / 1000000)

#if X_AUTO_SQUARE || Y_AUTO_SQUARE || Z_AUTO_SQUARE
#define SQUARING_ENABLED
#endif

#if defined(X2_LIMIT_PIN) || defined(Y2_LIMIT_PIN) || defined(Z2_LIMIT_PIN) || defined(A2_LIMIT_PIN) || defined(B2_LIMIT_PIN)
#define DUAL_LIMIT_SWITCHES
#else
  #ifdef SQUARING_ENABLED
    #error "Squaring requires at least one axis with dual switch inputs!"
  #endif
#endif

#define INPUT_GROUP_CONTROL       (1 << 0)
#define INPUT_GROUP_PROBE         (1 << 1)
#define INPUT_GROUP_LIMIT         (1 << 2)
#define INPUT_GROUP_KEYPAD        (1 << 3)
#define INPUT_GROUP_MPG           (1 << 4)
#define INPUT_GROUP_QEI           (1 << 5)
#define INPUT_GROUP_QEI_SELECT    (1 << 6)
#define INPUT_GROUP_SPINDLE_INDEX (1 << 7)

typedef struct {
    volatile uint_fast8_t head;
    volatile uint_fast8_t tail;
    input_signal_t *signal[DEBOUNCE_QUEUE];
} debounce_queue_t;

#if QEI_ENABLE

#define QEI_DEBOUNCE 3
#define QEI_VELOCITY_TIMEOUT 100

typedef union {
    uint_fast8_t pins;
    struct {
        uint_fast8_t a :1,
                     b :1;
    };
} qei_state_t;

typedef struct {
    encoder_t encoder;
    int32_t count;
    int32_t vel_count;
    uint_fast16_t state;
    volatile uint32_t dbl_click_timeout;
    volatile uint32_t vel_timeout;
    uint32_t vel_timestamp;
} qei_t;

static qei_t qei = {0};

#endif

static debounce_queue_t debounce_queue = {0};

// Standard inputs
static gpio_t Reset, FeedHold, CycleStart, Probe, LimitX, LimitY, LimitZ;

// Standard outputs
static gpio_t Mist, Flood, stepX, stepY, stepZ, dirX, dirY, dirZ;

#if !VFD_SPINDLE
static gpio_t spindleEnable, spindleDir;
#endif

// Optional I/O
#if SAFETY_DOOR_ENABLE
static gpio_t SafetyDoor;
#endif
#ifdef LIMITS_OVERRIDE_PIN
static gpio_t LimitsOverride;
#endif
#ifdef A_AXIS
static gpio_t stepA, dirA, LimitA;
#ifdef A_ENABLE_PIN
static gpio_t enableA;
#endif
#endif
#ifdef B_AXIS
static gpio_t stepB, dirB, LimitB;
#ifdef B_ENABLE_PIN
static gpio_t enableB;
#endif
#endif
#ifdef C_AXIS
static gpio_t stepC, dirC;
#ifdef C_ENABLE_PIN
static gpio_t enableC;
#endif
#ifdef C_LIMIT_PIN
static gpio_t LimitC;
#endif
#endif
#ifdef STEPPERS_ENABLE_PIN
static gpio_t steppersEnable;
#endif
#ifdef X_ENABLE_PIN
static gpio_t enableX;
#endif
#ifdef Y_ENABLE_PIN
static gpio_t enableY;
#endif
#ifdef Z_ENABLE_PIN
static gpio_t enableZ;
#endif
#if KEYPAD_ENABLE
static gpio_t KeypadStrobe;
#endif
#if MPG_MODE_ENABLE
static gpio_t ModeSelect;
#endif
#if QEI_ENABLE
static bool qei_enable = false;
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

#ifdef X2_STEP_PIN
  static gpio_t stepX2;
#endif
#ifdef X2_DIRECTION_PIN
  static gpio_t dirX2;
#endif
#ifdef X2_ENABLE_PIN
  static gpio_t enableX2;
#endif
#ifdef X2_LIMIT_PIN
  static gpio_t LimitX2;
#endif

#ifdef Y2_STEP_PIN
  static gpio_t stepY2;
#endif
#ifdef Y2_DIRECTION_PIN
  static gpio_t dirY2;
#endif
#ifdef Y2_ENABLE_PIN
  static gpio_t enableY2;
#endif
#ifdef Y2_LIMIT_PIN
  static gpio_t LimitY2;
#endif

#ifdef Z2_STEP_PIN
  static gpio_t stepZ2;
#endif
#ifdef Z2_DIRECTION_PIN
  static gpio_t dirZ2;
#endif
#ifdef Z2_ENABLE_PIN
  static gpio_t enableZ2;
#endif
#ifdef Z2_LIMIT_PIN
  static gpio_t LimitZ2;
#endif
#ifdef SPINDLE_INDEX_PIN
  static gpio_t SpindleIndex;
#endif

static input_signal_t inputpin[] = {
#if ESTOP_ENABLE
    { .id = Input_EStop,          .port = &Reset,          .pin = RESET_PIN,           .group = INPUT_GROUP_CONTROL },
#else
    { .id = Input_Reset,          .port = &Reset,          .pin = RESET_PIN,           .group = INPUT_GROUP_CONTROL },
#endif
    { .id = Input_FeedHold,       .port = &FeedHold,       .pin = FEED_HOLD_PIN,       .group = INPUT_GROUP_CONTROL },
    { .id = Input_CycleStart,     .port = &CycleStart,     .pin = CYCLE_START_PIN,     .group = INPUT_GROUP_CONTROL },
#if SAFETY_DOOR_ENABLE
    { .id = Input_SafetyDoor,     .port = &SafetyDoor,     .pin = SAFETY_DOOR_PIN,     .group = INPUT_GROUP_CONTROL },
#endif
#if defined(LIMITS_OVERRIDE_PIN)
    { .id = Input_LimitsOverride, .port = &LimitsOverride, .pin = LIMITS_OVERRIDE_PIN, .group = INPUT_GROUP_CONTROL },
#endif
    { .id = Input_Probe,          .port = &Probe,          .pin = PROBE_PIN,           .group = INPUT_GROUP_PROBE },
    { .id = Input_LimitX,         .port = &LimitX,         .pin = X_LIMIT_PIN,         .group = INPUT_GROUP_LIMIT },
#ifdef X2_LIMIT_PIN
    { .id = Input_LimitX_Max,     .port = &LimitX2,        .pin = X2_LIMIT_PIN,        .group = INPUT_GROUP_LIMIT },
#endif
    { .id = Input_LimitY,         .port = &LimitY,         .pin = Y_LIMIT_PIN,         .group = INPUT_GROUP_LIMIT },
#ifdef Y2_LIMIT_PIN
    { .id = Input_LimitY_Max,     .port = &LimitY2,        .pin = Y2_LIMIT_PIN,        .group = INPUT_GROUP_LIMIT },
#endif
    { .id = Input_LimitZ,         .port = &LimitZ,         .pin = Z_LIMIT_PIN,         .group = INPUT_GROUP_LIMIT }
#ifdef Z2_LIMIT_PIN
  , { .id = Input_LimitZ_Max,     .port = &LimitZ2,        .pin = Z2_LIMIT_PIN,        .group = INPUT_GROUP_LIMIT }
#endif
#ifdef A_LIMIT_PIN
  , { .id = Input_LimitA,         .port = &LimitA,         .pin = A_LIMIT_PIN,         .group = INPUT_GROUP_LIMIT }
#endif
#ifdef B_LIMIT_PIN
  , { .id = Input_LimitB,         .port = &LimitB,         .pin = B_LIMIT_PIN,         .group = INPUT_GROUP_LIMIT }
#endif
#ifdef C_LIMIT_PIN
  , { .id = Input_LimitC,         .port = &LimitC,         .pin = C_LIMIT_PIN,         .group = INPUT_GROUP_LIMIT }
#endif
#if MPG_MODE_ENABLE
  ,  { .id = Input_ModeSelect,    .port = &ModeSelect,     .pin = MODE_PIN,            .group = INPUT_GROUP_MPG }
#endif
#if KEYPAD_ENABLE && defined(KEYPAD_STROBE_PIN)
  , { .id = Input_KeypadStrobe,   .port = &KeypadStrobe,   .pin = KEYPAD_STROBE_PIN,   .group = INPUT_GROUP_KEYPAD }
#endif
#ifdef SPINDLE_INDEX_PIN
  , { .id = Input_SpindleIndex,   .port = &SpindleIndex,   .pin = SPINDLE_INDEX_PIN,   .group = INPUT_GROUP_SPINDLE_INDEX }
#endif
#if QEI_ENABLE
  , { .id = Input_QEI_A,          .port = &QEI_A,          .pin = QEI_A_PIN,           .group = INPUT_GROUP_QEI }
  , { .id = Input_QEI_B,          .port = &QEI_B,          .pin = QEI_B_PIN,           .group = INPUT_GROUP_QEI }
  #if QEI_SELECT_ENABLED
  , { .id = Input_QEI_Select,     .port = &QEI_Select,     .pin = QEI_SELECT_PIN,      .group = INPUT_GROUP_QEI_SELECT }
  #endif
  #if QEI_INDEX_ENABLED
  , { .id = Input_QEI_Index,      .port = &QEI_Index,      .pin = QEI_INDEX_PIN,       .group = INPUT_GROUP_QEI }
  #endif
#endif
};

#if USB_SERIAL_CDC || QEI_ENABLE
#define ADD_MSEVENT 1
static volatile bool ms_event = false;
#else
#define ADD_MSEVENT 0
#endif
static bool IOInitDone = false;
static uint16_t pulse_length, pulse_delay;
static axes_signals_t next_step_outbits;
static delay_t grbl_delay = { .ms = 0, .callback = NULL };
static probe_state_t probe = {
    .connected = On
};
#ifdef SQUARING_ENABLED
static axes_signals_t motors_1 = {AXES_BITMASK}, motors_2 = {AXES_BITMASK};
#endif

#if !VFD_SPINDLE
static bool pwmEnabled = false;
static spindle_pwm_t spindle_pwm;

static void spindle_set_speed (uint_fast16_t pwm_value);
#endif

#if MODBUS_ENABLE
static modbus_stream_t modbus_stream = {0};
#endif

#ifdef SPINDLE_SYNC_ENABLE

#include "grbl/spindle_sync.h"

static spindle_data_t spindle_data;
static spindle_encoder_t spindle_encoder = {
    .tics_per_irq = 4
};
static spindle_sync_t spindle_tracker;
static volatile bool spindleLock = false;

static void stepperPulseStartSynchronized (stepper_t *stepper);
static void spindleDataReset (void);
static spindle_data_t *spindleGetData (spindle_data_request_t request);
static void spindle_pulse_isr (void);

#endif

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
#if USB_SERIAL_CDC
    if(!(services.telnet || services.websocket)) // TODO: check if usb connection is up?
        usb_serialWriteS(data);
#else
    serialWriteS(data);
#endif
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

#if USB_SERIAL_CDC
    const io_stream_t serial_stream = {
        .type = StreamType_Serial,
        .read = usb_serialGetC,
        .write = usb_serialWriteS,
    #if ETHERNET_ENABLE
        .write_all = enetStreamWriteS,
    #else
        .write_all = usb_serialWriteS,
    #endif
        .get_rx_buffer_available = usb_serialRxFree,
        .reset_read_buffer = usb_serialRxFlush,
        .cancel_read_buffer = usb_serialRxCancel,
        .suspend_read = usb_serialSuspendInput,
        .enqueue_realtime_command = protocol_enqueue_realtime_command
    };
#else
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
    .suspend_read = serialSuspendInput,
    .enqueue_realtime_command = protocol_enqueue_realtime_command
};
#endif

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
static void driver_delay_ms (uint32_t ms, delay_callback_ptr callback)
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

// Set stepper pulse output pins.
// step_outbits.value (or step_outbits.mask) are: bit0 -> X, bit1 -> Y...
// Individual step bits can be accessed by step_outbits.x, step_outbits.y, ...
#ifdef SQUARING_ENABLED
inline static __attribute__((always_inline)) void set_step_outputs (axes_signals_t step_outbits_1)
{
    axes_signals_t step_outbits_2;

    step_outbits_2.mask = (step_outbits_1.mask & motors_2.mask) ^ settings.steppers.step_invert.mask;
    step_outbits_1.mask = (step_outbits_1.mask & motors_1.mask) ^ settings.steppers.step_invert.mask;

    DIGITAL_OUT(stepX, step_outbits_1.x);
#ifdef X2_STEP_PIN
    DIGITAL_OUT(stepX2, step_outbits_2.x);
#endif

    DIGITAL_OUT(stepY, step_outbits_1.y);
#ifdef Y2_STEP_PIN
    DIGITAL_OUT(stepY2, step_outbits_2.y);
#endif

    DIGITAL_OUT(stepZ, step_outbits_1.z);
#ifdef Z2_STEP_PIN
    DIGITAL_OUT(stepZ2, step_outbits_2.z);
#endif

#ifdef A_AXIS
    DIGITAL_OUT(stepA, step_outbits_1.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(stepB, step_outbits_1.b);
#endif
}
#else
inline static __attribute__((always_inline)) void set_step_outputs (axes_signals_t step_outbits)
{
    step_outbits.value ^= settings.steppers.step_invert.mask;

    DIGITAL_OUT(stepX, step_outbits.x);
#ifdef X2_STEP_PIN
    DIGITAL_OUT(stepX2, step_outbits.x);
#endif

    DIGITAL_OUT(stepY, step_outbits.y);
#ifdef Y2_STEP_PIN
    DIGITAL_OUT(stepY2, step_outbits.y);
#endif

    DIGITAL_OUT(stepZ, step_outbits.z);
#ifdef Z2_STEP_PIN
    DIGITAL_OUT(stepZ2, step_outbits.z);
#endif

#ifdef A_AXIS
    DIGITAL_OUT(stepA, step_outbits.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(stepB, step_outbits.b);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(stepC, step_outbits.c);
#endif
}
#endif

// Set stepper direction ouput pins.
// dir_outbits.value (or dir_outbits.mask) are: bit0 -> X, bit1 -> Y...
// Individual direction bits can be accessed by dir_outbits.x, dir_outbits.y, ...
inline static __attribute__((always_inline)) void set_dir_outputs (axes_signals_t dir_outbits)
{
    dir_outbits.value ^= settings.steppers.dir_invert.mask;

    DIGITAL_OUT(dirX, dir_outbits.x);
#ifdef X2_DIRECTION_PIN
    DIGITAL_OUT(dirX2, dir_outbits.x);
#endif

    DIGITAL_OUT(dirY, dir_outbits.y);
#ifdef Y2_DIRECTION_PIN
    DIGITAL_OUT(dirY2, dir_outbits.y);
#endif

    DIGITAL_OUT(dirZ, dir_outbits.z);
#ifdef Z2_DIRECTION_PIN
    DIGITAL_OUT(dirZ2, dir_outbits.z);
#endif

#ifdef A_AXIS
    DIGITAL_OUT(dirA, dir_outbits.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(dirB, dir_outbits.b);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(dirC, dir_outbits.c);
#endif
}

// Enable steppers.
// enable.value (or enable.mask) are: bit0 -> X, bit1 -> Y...
// Individual enable bits can be accessed by enable.x, enable.y, ...
// NOTE: if a common signal is used to enable all drivers enable.x should be used to set the signal.
static void stepperEnable (axes_signals_t enable)
{
    enable.value ^= settings.steppers.enable_invert.mask;

#ifdef STEPPERS_ENABLE_PIN
    DIGITAL_OUT(steppersEnable, enable.x)
#endif

#ifdef X_ENABLE_PIN
    DIGITAL_OUT(enableX, enable.x)
#endif
#ifdef X2_ENABLE_PIN
    DIGITAL_OUT(enableX2, enable.x)
#endif

#ifdef Y_ENABLE_PIN
    DIGITAL_OUT(enableY, enable.y)
#endif
#ifdef Y2_ENABLE_PIN
    DIGITAL_OUT(enableY2, enable.y)
#endif

#ifdef Z_ENABLE_PIN
    DIGITAL_OUT(enableZ, enable.z)
#endif
#ifdef Z2_ENABLE_PIN
    DIGITAL_OUT(enableZ2, enable.z)
#endif

#ifdef A_ENABLE_PIN
    DIGITAL_OUT(enableA, enable.a)
#endif
#ifdef B_ENABLE_PIN
    DIGITAL_OUT(enableB, enable.b)
#endif
#ifdef C_ENABLE_PIN
    DIGITAL_OUT(enableC, enable.c)
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

// Start a stepper pulse, no delay version.
// stepper_t struct is defined in grbl/stepper.h
static void stepperPulseStart (stepper_t *stepper)
{
#ifdef SPINDLE_SYNC_ENABLE
    if(stepper->new_block && stepper->exec_segment->spindle_sync) {
        spindle_tracker.stepper_pulse_start_normal = hal.stepper.pulse_start;
        hal.stepper.pulse_start = stepperPulseStartSynchronized;
        hal.stepper.pulse_start(stepper);
        return;
    }
#endif

    if(stepper->dir_change)
        set_dir_outputs(stepper->dir_outbits);

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        TMR4_CTRL0 |= TMR_CTRL_CM(0b001);
    }
}

// Start a stepper pulse, delay version.
// Note: delay is only added when there is a direction change and a pulse to be output.
//       In the delayed step pulse interrupt handler the pulses are output and
//       normal (no delay) operation is resumed.
// stepper_t struct is defined in grbl/stepper.h
static void stepperPulseStartDelayed (stepper_t *stepper)
{
#ifdef SPINDLE_SYNC_ENABLE
    if(stepper->new_block && stepper->exec_segment->spindle_sync) {
        spindle_tracker.stepper_pulse_start_normal = hal.stepper.pulse_start;
        hal.stepper.pulse_start = stepperPulseStartSynchronized;
        hal.stepper.pulse_start(stepper);
        return;
    }
#endif

    if(stepper->dir_change) {

        set_dir_outputs(stepper->dir_outbits);

        if(stepper->step_outbits.value) {

            next_step_outbits = stepper->step_outbits; // Store out_bits

            attachInterruptVector(IRQ_QTIMER4, stepper_pulse_isr_delayed);

            TMR4_COMP10 = pulse_delay;
            TMR4_CTRL0 |= TMR_CTRL_CM(0b001);
        }

        return;
    }

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        TMR4_CTRL0 |= TMR_CTRL_CM(0b001);
    }
}

#ifdef SPINDLE_SYNC_ENABLE

// Spindle sync version: sets stepper direction and pulse pins and starts a step pulse.
// Switches back to "normal" version if spindle synchronized motion is finished.
// TODO: add delayed pulse handling...
static void stepperPulseStartSynchronized (stepper_t *stepper)
{
    static bool sync = false;
    static float block_start;

    if(stepper->new_block) {
        if(!stepper->exec_segment->spindle_sync) {
            hal.stepper.pulse_start = spindle_tracker.stepper_pulse_start_normal;
            hal.stepper.pulse_start(stepper);
            return;
        }
        sync = true;
        set_dir_outputs(stepper->dir_outbits);
        spindle_tracker.programmed_rate = stepper->exec_block->programmed_rate;
        spindle_tracker.steps_per_mm = stepper->exec_block->steps_per_mm;
        spindle_tracker.segment_id = 0;
        spindle_tracker.prev_pos = 0.0f;
        block_start = spindleGetData(SpindleData_AngularPosition)->angular_position * spindle_tracker.programmed_rate;
        pidf_reset(&spindle_tracker.pid);
#ifdef PID_LOG
        sys.pid_log.idx = 0;
        sys.pid_log.setpoint = 100.0f;
#endif
    }

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
        TMR4_CTRL0 |= TMR_CTRL_CM(0b001);
    }

    if(spindle_tracker.segment_id != stepper->exec_segment->id) {

        spindle_tracker.segment_id = stepper->exec_segment->id;

        if(!stepper->new_block) {  // adjust this segments total time for any positional error since last segment

            float actual_pos;

            if(stepper->exec_segment->cruising) {

                float dt = (float)hal.f_step_timer / (float)(stepper->exec_segment->cycles_per_tick * stepper->exec_segment->n_step);
                actual_pos = spindleGetData(SpindleData_AngularPosition)->angular_position * spindle_tracker.programmed_rate;

                if(sync) {
                    spindle_tracker.pid.sample_rate_prev = dt;
//                    block_start += (actual_pos - spindle_tracker.block_start) - spindle_tracker.prev_pos;
//                    block_start += spindle_tracker.prev_pos;
                    sync = false;
                }

                actual_pos -= block_start;
                int32_t step_delta = (int32_t)(pidf(&spindle_tracker.pid, spindle_tracker.prev_pos, actual_pos, dt) * spindle_tracker.steps_per_mm);


                int32_t ticks = (((int32_t)stepper->step_count + step_delta) * (int32_t)stepper->exec_segment->cycles_per_tick) / (int32_t)stepper->step_count;

                stepper->exec_segment->cycles_per_tick = (uint32_t)max(ticks, spindle_tracker.min_cycles_per_tick);

                stepperCyclesPerTick(stepper->exec_segment->cycles_per_tick);
           } else
               actual_pos = spindle_tracker.prev_pos;

#ifdef PID_LOG
            if(sys.pid_log.idx < PID_LOG) {

                sys.pid_log.target[sys.pid_log.idx] = spindle_tracker.prev_pos;
                sys.pid_log.actual[sys.pid_log.idx] = actual_pos; // - spindle_tracker.prev_pos;

            //    spindle_tracker.log[sys.pid_log.idx] = STEPPER_TIMER->BGLOAD << stepper->amass_level;
            //    spindle_tracker.pos[sys.pid_log.idx] = stepper->exec_segment->cycles_per_tick  stepper->amass_level;
            //    spindle_tracker.pos[sys.pid_log.idx] = stepper->exec_segment->cycles_per_tick * stepper->step_count;
            //    STEPPER_TIMER->BGLOAD = STEPPER_TIMER->LOAD;

             //   spindle_tracker.pos[sys.pid_log.idx] = spindle_tracker.prev_pos;

                sys.pid_log.idx++;
            }
#endif
        }

        spindle_tracker.prev_pos = stepper->exec_segment->target_position;
    }
}

#endif

#if PLASMA_ENABLE

#if PLASMA_ENABLE
static void output_pulse_isr(void);
#endif

static axes_signals_t pulse_output = {0};

void stepperOutputStep (axes_signals_t step_outbits, axes_signals_t dir_outbits)
{
    pulse_output = step_outbits;
    dir_outbits.value ^= settings.steppers.dir_invert.mask;

    DIGITAL_OUT(dirZ, dir_outbits.z);
    TMR2_CTRL0 |= TMR_CTRL_CM(0b001);
}

#endif

#ifdef DUAL_LIMIT_SWITCHES

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
// Dual limit switch inputs per axis version. Only one needs to be dual input!
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};

    signals.min.mask = signals.min2.mask = settings.limits.invert.mask;

    signals.min.x = (LimitX.reg->DR & LimitX.bit) != 0;
#ifdef X2_LIMIT_PIN
    signals.min2.x = (LimitX2.reg->DR & LimitX2.bit) != 0;
#endif

    signals.min.y = (LimitY.reg->DR & LimitY.bit) != 0;
#ifdef Y2_LIMIT_PIN
    signals.min2.y = (LimitY2.reg->DR & LimitY2.bit) != 0;
#endif

    signals.min.z = (LimitZ.reg->DR & LimitZ.bit) != 0;
#ifdef Z2_LIMIT_PIN
    signals.min2.z = (LimitZ2.reg->DR & LimitZ2.bit) != 0;
#endif

#ifdef A_LIMIT_PIN
    signals.min.a = (LimitA.reg->DR & LimitA.bit) != 0;
#endif
#ifdef B_LIMIT_PIN
    signals.min.b = (LimitB.reg->DR & LimitB.bit) != 0;
#endif
#ifdef C_LIMIT_PIN
    signals.min.c = (LimitC.reg->DR & LimitC.bit) != 0;
#endif

    if(settings.limits.invert.mask) {
        signals.min.value ^= settings.limits.invert.mask;
        signals.min2.value ^= settings.limits.invert.mask;
    }

    return signals;
}
#else // SINGLE INPUT LIMIT SWITCHES

// Returns limit state as an axes_signals_t bitmap variable.
// signals.value (or signals.mask) are: bit0 -> X, bit1 -> Y...
// Individual signals bits can be accessed by signals.x, signals.y, ...
// Each bit indicates a limit signal, where triggered is 1 and not triggered is 0.
// axes_signals_t is defined in grbl/nuts_bolts.h.
inline static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};

    signals.min.mask = settings.limits.invert.mask;

    signals.min.x = (LimitX.reg->DR & LimitX.bit) != 0;
    signals.min.y = (LimitY.reg->DR & LimitY.bit) != 0;
    signals.min.z = (LimitZ.reg->DR & LimitZ.bit) != 0;
#ifdef A_LIMIT_PIN
    signals.min.a = (LimitA.reg->DR & LimitA.bit) != 0;
#endif
#ifdef B_LIMIT_PIN
    signals.min.b = (LimitB.reg->DR & LimitB.bit) != 0;
#endif
#ifdef C_LIMIT_PIN
    signals.min.c = (LimitC.reg->DR & LimitC.bit) != 0;
#endif

    if (settings.limits.invert.mask)
        signals.min.value ^= settings.limits.invert.mask;

    return signals;
}

#endif

#ifdef SQUARING_ENABLED

static axes_signals_t getAutoSquaredAxes (void)
{
    axes_signals_t ganged = {0};

    #if X_AUTO_SQUARE
    ganged.x = On;
#endif
#if Y_AUTO_SQUARE
    ganged.y = On;
#endif
#if Z_AUTO_SQUARE
    ganged.z = On;
#endif

    return ganged;
}

// Enable/disable motors for auto squaring of ganged axes
static void StepperDisableMotors (axes_signals_t axes, squaring_mode_t mode)
{
    motors_1.mask = (mode == SquaringMode_A || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
    motors_2.mask = (mode == SquaringMode_B || mode == SquaringMode_Both ? axes.mask : 0) ^ AXES_BITMASK;
}

#endif

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

// Returns system state as a control_signals_t bitmap variable.
// signals.value (or signals.mask) are: bit0 -> reset, bit1 -> feed_hold, ...
// Individual enable bits can be accessed by signals.reset, signals.feed_hold, ...
// Each bit indicates a control signal, where triggered is 1 and not triggered is 0.
// axes_signals_t is defined in grbl/system.h.
inline static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.value = settings.control_invert.value;

#if ESTOP_ENABLE
    signals.e_stop = (Reset.reg->DR & Reset.bit) != 0;
#else
    signals.reset = (Reset.reg->DR & Reset.bit) != 0;
#endif
    signals.feed_hold = (FeedHold.reg->DR & FeedHold.bit) != 0;
    signals.cycle_start = (CycleStart.reg->DR & CycleStart.bit) != 0;
#if SAFETY_DOOR_ENABLE
    signals.safety_door_ajar = (SafetyDoor.reg->DR & SafetyDoor.bit) != 0;
#endif

    if(settings.control_invert.value)
        signals.value ^= settings.control_invert.value;

#ifdef LIMITS_OVERRIDE_PIN
    signals.limits_override = (LimitsOverride.reg->DR & LimitsOverride.bit) == 0;
#endif

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

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;
    state.triggered = !!(Probe.reg->DR & Probe.bit) ^ probe.inverted;

    return state;
}

#if !VFD_SPINDLE

// Static spindle (off, on cw & on ccw)

inline static void spindle_off ()
{
    DIGITAL_OUT(spindleEnable, settings.spindle.invert.on);
}

inline static void spindle_on ()
{
    DIGITAL_OUT(spindleEnable, !settings.spindle.invert.on);
#ifdef SPINDLE_SYNC_ENABLE
    spindleDataReset();
#endif
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
        if(settings.spindle.flags.pwm_action == SpindleAction_DisableWithZeroSPeed)
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

#ifdef SPINDLE_SYNC_ENABLE
    if(settings.spindle.at_speed_tolerance > 0.0f) {
        float tolerance = rpm * settings.spindle.at_speed_tolerance / 100.0f;
        spindle_data.rpm_low_limit = rpm - tolerance;
        spindle_data.rpm_high_limit = rpm + tolerance;
    }
    spindle_data.rpm_programmed = spindle_data.rpm = rpm;
#endif
}

// Returns spindle state in a spindle_state_t variable.
// spindle_state_t is defined in grbl/spindle_control.h
static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {settings.spindle.invert.mask};

    state.on = (spindleEnable.reg->DR & spindleEnable.bit) != 0;

    if(hal.driver_cap.spindle_dir)
        state.ccw = (spindleDir.reg->DR & spindleDir.bit) != 0;
 
    state.value ^= settings.spindle.invert.mask;

    if(pwmEnabled)
        state.on |= pwmEnabled;

#ifdef SPINDLE_SYNC_ENABLE
    float rpm = spindleGetData(SpindleData_RPM)->rpm;
    state.at_speed = settings.spindle.at_speed_tolerance <= 0.0f || (rpm >= spindle_data.rpm_low_limit && rpm <= spindle_data.rpm_high_limit);
#endif

    return state;
}

#if PPI_ENABLE

static void spindlePulseOn (uint_fast16_t pulse_length)
{
    static uint_fast16_t plen = 0;

    if(plen != pulse_length) {
        plen = pulse_length;
        PPI_TIMER.CH[0].COMP1 = (uint16_t)((pulse_length * F_BUS_MHZ) / 128);
    }

    spindle_on();
    PPI_TIMER.CH[0].CTRL |= TMR_CTRL_CM(0b001);
}

#endif

#ifdef SPINDLE_SYNC_ENABLE

static spindle_data_t *spindleGetData (spindle_data_request_t request)
{
    bool stopped;
    uint32_t pulse_length, rpm_timer_delta;
    spindle_encoder_counter_t encoder;

    __disable_irq();

    memcpy(&encoder, &spindle_encoder.counter, sizeof(spindle_encoder_counter_t));

    pulse_length = spindle_encoder.timer.pulse_length / spindle_encoder.tics_per_irq;
    rpm_timer_delta = GPT1_CNT - spindle_encoder.timer.last_pulse;

    __enable_irq();

    // If no (4) spindle pulses during last 250 ms assume RPM is 0
    if((stopped = ((pulse_length == 0) || (rpm_timer_delta > spindle_encoder.maximum_tt)))) {
        spindle_data.rpm = 0.0f;
        rpm_timer_delta = (GPT2_CNT - spindle_encoder.counter.last_count) * pulse_length;
    }

    switch(request) {

        case SpindleData_Counters:
            spindle_data.pulse_count = GPT2_CNT;
            spindle_data.index_count = encoder.index_count;
            spindle_data.error_count = spindle_encoder.error_count;
            break;

        case SpindleData_RPM:
            if(!stopped)
                spindle_data.rpm = spindle_encoder.rpm_factor / (float)pulse_length;
            break;

        case SpindleData_AngularPosition:;
            while(spindleLock);
            int32_t d = encoder.last_count - encoder.last_index;
            spindle_data.angular_position = (float)encoder.index_count +
                    ((float)(d) +
                             (pulse_length == 0 ? 0.0f : (float)rpm_timer_delta / (float)pulse_length)) *
                                spindle_encoder.pulse_distance;
            break;
    }

    return &spindle_data;
}

static void spindleDataReset (void)
{
    while(spindleLock);

    uint32_t timeout = millis() + 1000; // 1 second

    uint32_t index_count = spindle_data.index_count + 2;
    if(spindleGetData(SpindleData_RPM)->rpm > 0.0f) { // wait for index pulse if running

        while(index_count != spindle_data.index_count && millis() <= timeout);

//        if(uwTick > timeout)
//            alarm?
    }

    GPT2_CR &= ~GPT_CR_EN;  // Reset timer
    GPT1_CR &= ~GPT_CR_EN;  // Reset timer
    GPT1_PR = 24;
    GPT1_CR |= GPT_CR_EN;

    spindle_encoder.timer.last_pulse =
    spindle_encoder.timer.last_index = GPT1_CNT;

    spindle_encoder.timer.pulse_length =
    spindle_encoder.counter.last_count =
    spindle_encoder.counter.last_index =
    spindle_encoder.counter.pulse_count =
    spindle_encoder.counter.index_count =
    spindle_encoder.error_count = 0;

    // Spindle pulse counter
    GPT2_OCR1 = spindle_encoder.tics_per_irq;
    GPT2_CR |= GPT_CR_EN;
}

#endif

// end spindle code

#endif

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

#if ETHERNET_ENABLE
static void reportIP (bool newopt)
{
    if(!newopt && (services.telnet || services.websocket)) {
        hal.stream.write("[NETCON:");
        hal.stream.write(services.telnet ? "Telnet" : "Websocket");
        hal.stream.write("]" ASCII_EOL);
    }
}
#endif

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

static void enable_irq (void)
{
    __enable_irq();
}

static void disable_irq (void)
{
    __disable_irq();
}

// Configures perhipherals when settings are initialized or changed
static void settings_changed (settings_t *settings)
{
    if(IOInitDone) {

        stepperEnable(settings->steppers.deenergize);

#ifdef SQUARING_ENABLED
        hal.stepper.disable_motors((axes_signals_t){0}, SquaringMode_Both);
#endif

#if !PLASMA_ENABLE && !defined(SPINDLE_RPM_CONTROLLED)

        if(hal.driver_cap.variable_spindle && spindle_precompute_pwm_values(&spindle_pwm, F_BUS_ACTUAL / 2)) {
  #if SPINDLEPWMPIN == 12
            TMR1_COMP11 = spindle_pwm.period;
            TMR1_CMPLD11 = spindle_pwm.period;
  #else // 13
            TMR2_COMP10 = spindle_pwm.period;
            TMR2_CMPLD10 = spindle_pwm.period;
  #endif
            hal.spindle.set_state = spindleSetStateVariable;
        } else
            hal.spindle.set_state = spindleSetState;
#elif !VFD_SPINDLE
        hal.spindle.set_state = spindleSetState;
#endif

#ifdef SPINDLE_SYNC_ENABLE

        if((hal.spindle.get_data = settings->spindle.ppr > 0 ? spindleGetData : NULL) && spindle_encoder.ppr != settings->spindle.ppr) {

            hal.spindle.reset_data = spindleDataReset;
            hal.spindle.set_state((spindle_state_t){0}, 0.0f);

            pidf_init(&spindle_tracker.pid, &settings->position.pid);

            float timer_resolution = 1.0f / 1000000.0f; // 1 us resolution

            spindle_tracker.min_cycles_per_tick = (int32_t)ceilf(settings->steppers.pulse_microseconds * 2.0f + settings->steppers.pulse_delay_microseconds);
            spindle_encoder.ppr = settings->spindle.ppr;
            spindle_encoder.tics_per_irq = max(1, spindle_encoder.ppr / 32);
            spindle_encoder.pulse_distance = 1.0f / spindle_encoder.ppr;
            spindle_encoder.maximum_tt = (uint32_t)(2.0f / timer_resolution) / spindle_encoder.tics_per_irq;
            spindle_encoder.rpm_factor = 60.0f / ((timer_resolution * (float)spindle_encoder.ppr));
            spindleDataReset();
        }

#endif

        // Stepper pulse timeout setup.
        TMR4_CSCTRL0 &= ~(TMR_CSCTRL_TCF1|TMR_CSCTRL_TCF2);

        pulse_length = (uint16_t)((float)F_BUS_MHZ * (settings->steppers.pulse_microseconds - STEP_PULSE_LATENCY));

        if(hal.driver_cap.step_pulse_delay && settings->steppers.pulse_delay_microseconds > 0.0f) {
            float delay = settings->steppers.pulse_delay_microseconds - STEP_PULSE_LATENCY;
            if(delay <= STEP_PULSE_LATENCY)
                delay = STEP_PULSE_LATENCY + 0.2f;
            pulse_delay = (uint16_t)((float)F_BUS_MHZ * delay);
            hal.stepper.pulse_start = stepperPulseStartDelayed;
        } else
            hal.stepper.pulse_start = stepperPulseStart;

        TMR4_COMP10 = pulse_length;
        TMR4_CSCTRL0 &= ~TMR_CSCTRL_TCF2EN;
        TMR4_CTRL0 &= ~TMR_CTRL_OUTMODE(0b000);
        attachInterruptVector(IRQ_QTIMER4, stepper_pulse_isr);

#if PLASMA_ENABLE
        TMR2_CSCTRL0 &= ~(TMR_CSCTRL_TCF1|TMR_CSCTRL_TCF2);
        TMR2_COMP10 = pulse_length;
        TMR2_CSCTRL0 &= ~TMR_CSCTRL_TCF2EN;
        TMR2_CTRL0 &= ~TMR_CTRL_OUTMODE(0b000);
#endif

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

#if SAFETY_DOOR_ENABLE
                case Input_SafetyDoor:
                    pullup = !settings->control_disable_pullup.safety_door_ajar;
                    signal->debounce = hal.driver_cap.software_debounce;
                    signal->irq_mode = control_fei.safety_door_ajar ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#ifdef LIMITS_OVERRIDE_PIN
                case Input_LimitsOverride:
                    pullup = true;
                    signal->debounce = false;
                    break;
#endif
                case Input_Probe:
                    pullup = hal.driver_cap.probe_pull_up;
                    break;

                case Input_LimitX:
                case Input_LimitX_Max:
                    pullup = !settings->limits.disable_pullup.x;
                    signal->debounce = hal.driver_cap.software_debounce;
                    signal->irq_mode = limit_fei.x ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitY:
                case Input_LimitY_Max:
                    pullup = !settings->limits.disable_pullup.y;
                    signal->irq_mode = limit_fei.y ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;

                case Input_LimitZ:
                case Input_LimitZ_Max:
                    pullup = !settings->limits.disable_pullup.z;
                    signal->irq_mode = limit_fei.z ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#ifdef A_LIMIT_PIN
                case Input_LimitA:
                case Input_LimitA_Max:
                    pullup = !settings->limits.disable_pullup.a;
                    signal->irq_mode = limit_fei.a ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#ifdef B_LIMIT_PIN
                case Input_LimitB:
                case Input_LimitB_Max:
                    pullup = !settings->limits.disable_pullup.b;
                    signal->irq_mode = limit_fei.b ? IRQ_Mode_Falling : IRQ_Mode_Rising;
                    break;
#endif
#ifdef C_LIMIT_PIN
                case Input_LimitC:
                case Input_LimitC_Max:
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
                    pullup = true;
                    signal->irq_mode = IRQ_Mode_Change;
                    break;
#endif
#ifdef SPINDLE_INDEX_PIN
                case Input_SpindleIndex:
                    pullup = false;
                    signal->irq_mode = IRQ_Mode_Rising;
                    break;
#endif
#if QEI_ENABLE
                case Input_QEI_A:
                    if(qei_enable)
                        signal->irq_mode = IRQ_Mode_Change;
                    break;

                case Input_QEI_B:
                    if(qei_enable)
                        signal->irq_mode = IRQ_Mode_Change;
                    break;

  #if QEI_INDEX_ENABLED
                case Input_QEI_Index:
                    if(qei_enable)
                        signal->irq_mode = IRQ_Mode_None;
                    break;
  #endif

  #if QEI_SELECT_ENABLED
                case Input_QEI_Select:
                    signal->debounce = hal.driver_cap.software_debounce;
                    if(qei_enable)
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

#if QEI_ENABLE

static void qei_update (void)
{
    const uint8_t encoder_valid_state[] = {0, 1, 1, 0, 1, 0, 0, 1, 1, 0, 0, 1, 0, 1, 1, 0};

    uint_fast8_t idx;
    qei_state_t state = {0};

    state.a = (QEI_A.reg->DR & QEI_A.bit) != 0;
    state.b = (QEI_B.reg->DR & QEI_B.bit) != 0;

    idx = (((qei.state << 2) & 0x0F) | state.pins);

    if(encoder_valid_state[idx] ) {

        qei.state = ((qei.state << 4) | idx) & 0xFF;

        if (qei.state == 0x42 || qei.state == 0xD4 || qei.state == 0x2B || qei.state == 0xBD) {
            qei.count--;
            if(qei.vel_timeout == 0) {
                qei.encoder.event.position_changed = hal.encoder.on_event != NULL;
                hal.encoder.on_event(&qei.encoder, qei.count);
            }
        } else if(qei.state == 0x81 || qei.state == 0x17 || qei.state == 0xE8 || qei.state == 0x7E) {
            qei.count++;
            if(qei.vel_timeout == 0) {
                qei.encoder.event.position_changed = hal.encoder.on_event != NULL;
                hal.encoder.on_event(&qei.encoder, qei.count);
            }
        }
    }

}

static void qei_reset (uint_fast8_t id)
{
    qei.vel_timeout = 0;
    qei.count = qei.vel_count = 0;
    qei.vel_timestamp = millis();
    qei.vel_timeout = qei.encoder.axis != 0xFF ? QEI_VELOCITY_TIMEOUT : 0;
}

// dummy handler, called on events if plugin init fails
static void encoder_event (encoder_t *encoder, int32_t position)
{
    UNUSED(position);
    encoder->event.events = 0;
}

#endif

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
#if TRINAMIC_ENABLE && defined(BOARD_CNC_BOOSTERPACK) // Trinamic BoosterPack does not support mixed drivers
    driver_settings.trinamic.driver_enable.mask = AXES_BITMASK;
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

#if PLASMA_ENABLE
    TMR2_ENBL = 0;
    TMR2_LOAD0 = 0;
    TMR2_CTRL0 = TMR_CTRL_PCS(0b1000) | TMR_CTRL_ONCE | TMR_CTRL_LENGTH;
    TMR2_CSCTRL0 = TMR_CSCTRL_TCF1EN;

    attachInterruptVector(IRQ_QTIMER2, output_pulse_isr);
    NVIC_SET_PRIORITY(IRQ_QTIMER2, 0);
    NVIC_ENABLE_IRQ(IRQ_QTIMER2);

    TMR2_ENBL = 1;
#endif

    pinModeOutput(&stepX, X_STEP_PIN);
    pinModeOutput(&dirX, X_DIRECTION_PIN);
#ifdef X_ENABLE_PIN
    pinModeOutput(&enableX, X_ENABLE_PIN);
#endif
#ifdef X2_STEP_PIN
    pinModeOutput(&stepX2, X2_STEP_PIN);
#endif
#ifdef X2_DIRECTION_PIN
    pinModeOutput(&dirX2, X2_DIRECTION_PIN);
#endif
#ifdef X2_ENABLE_PIN
    pinModeOutput(&enableX2, X2_ENABLE_PIN);
#endif

    pinModeOutput(&stepY, Y_STEP_PIN);
    pinModeOutput(&dirY, Y_DIRECTION_PIN);
#ifdef Y_ENABLE_PIN
    pinModeOutput(&enableY, Y_ENABLE_PIN);
#endif
#ifdef Y2_STEP_PIN
    pinModeOutput(&stepY2, Y2_STEP_PIN);
#endif
#ifdef Y2_DIRECTION_PIN
    pinModeOutput(&dirY2, Y2_DIRECTION_PIN);
#endif
#ifdef Y2_ENABLE_PIN
    pinModeOutput(&enableY2, Y2_ENABLE_PIN);
#endif

    pinModeOutput(&stepZ, Z_STEP_PIN);
    pinModeOutput(&dirZ, Z_DIRECTION_PIN);
#ifdef Z_ENABLE_PIN
    pinModeOutput(&enableZ, Z_ENABLE_PIN);
#endif
#ifdef Z2_STEP_PIN
    pinModeOutput(&stepZ2, Z2_STEP_PIN);
#endif
#ifdef Z2_DIRECTION_PIN
    pinModeOutput(&dirZ2, Z2_DIRECTION_PIN);
#endif
#ifdef Z2_ENABLE_PIN
    pinModeOutput(&enableZ2, Z2_ENABLE_PIN);
#endif

#ifdef A_AXIS
    pinModeOutput(&stepA, A_STEP_PIN);
    pinModeOutput(&dirA, A_DIRECTION_PIN);
  #ifdef A_ENABLE_PIN
    pinModeOutput(&enableA, A_ENABLE_PIN);
  #endif
#endif

#ifdef B_AXIS
    pinModeOutput(&stepB, B_STEP_PIN);
    pinModeOutput(&dirB, B_DIRECTION_PIN);
  #ifdef B_ENABLE_PIN
    pinModeOutput(&enableB, B_ENABLE_PIN);
  #endif
#endif

#ifdef C_AXIS
    pinModeOutput(&stepC, C_STEP_PIN);
    pinModeOutput(&dirC, C_DIRECTION_PIN);
  #ifdef C_ENABLE_PIN
    pinModeOutput(&enableC, C_ENABLE_PIN);
  #endif
#endif

#ifdef STEPPERS_ENABLE_PIN
    pinModeOutput(&steppersEnable, STEPPERS_ENABLE_PIN);
#endif

   /****************************
    *  Software debounce init  *
    ****************************/

    if(hal.driver_cap.software_debounce) {

        TMR3_ENBL = 0;
        TMR3_LOAD0 = 0;
        TMR3_CTRL0 = TMR_CTRL_PCS(0b1111) | TMR_CTRL_ONCE | TMR_CTRL_LENGTH;
        TMR3_COMP10 = (uint16_t)((40000UL * F_BUS_MHZ) / 128); // 150 MHz -> 40ms
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

#if !VFD_SPINDLE

   /******************
    *  Spindle init  *
    ******************/

    pinModeOutput(&spindleEnable, SPINDLE_ENABLE_PIN);
    pinModeOutput(&spindleDir, SPINDLE_DIRECTION_PIN);

#if !PLASMA_ENABLE

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

#ifdef SPINDLE_SYNC_ENABLE

    CCM_CCGR1 |= CCM_CCGR1_GPT1_BUS(CCM_CCGR_ON);
    CCM_CMEOR |= CCM_CMEOR_MOD_EN_OV_GPT;

    // Free running timer
    GPT1_CR = 0;
    GPT1_CR |= GPT_CR_SWR;
    while(GPT1_CR & GPT_CR_SWR);
    GPT1_CR = GPT_CR_CLKSRC(1);
    GPT1_CR |= GPT_CR_FRR|GPT_CR_ENMOD|GPT_CR_EN;
    GPT1_PR = 150;

  #if SPINDLE_PULSE_PIN == 14

    CCM_CCGR0 |= CCM_CCGR0_GPT2_BUS(CCM_CCGR_ON);

    IOMUXC_SW_MUX_CTL_PAD_GPIO_AD_B1_02 = 8;
    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 = IOMUXC_PAD_DSE(7) | IOMUXC_PAD_PKE | IOMUXC_PAD_PUE | IOMUXC_PAD_PUS(3) | IOMUXC_PAD_HYS;
//    IOMUXC_SW_PAD_CTL_PAD_GPIO_AD_B1_02 = 0x001d0b0;
    IOMUXC_GPT2_IPP_IND_CLKIN_SELECT_INPUT = 1;

    // Spindle pulse counter
    GPT2_CR = 0;
    GPT2_CR |= GPT_CR_SWR;
    while(GPT2_CR & GPT_CR_SWR);
    GPT2_CR = GPT_CR_CLKSRC(3);
    GPT2_CR |= GPT_CR_ENMOD;
    GPT2_CR |= GPT_CR_FRR|GPT_CR_EN;
    GPT2_OCR1 = spindle_encoder.tics_per_irq;
    GPT2_IR = GPT_IR_OF1IE;

    attachInterruptVector(IRQ_GPT2, spindle_pulse_isr);
    NVIC_SET_PRIORITY(IRQ_GPT2, 1);
    NVIC_ENABLE_IRQ(IRQ_GPT2);

  #endif

#endif // SPINDLE_SYNC_ENABLE

#if PPI_ENABLE

    PPI_TIMER.ENBL = 0;
    PPI_TIMER.CH[0].LOAD = 0;
    PPI_TIMER.CH[0].COMP1 = (uint16_t)((1500UL * F_BUS_MHZ) / 128);
    PPI_TIMER.CH[0].CTRL = TMR_CTRL_PCS(0b1111) | TMR_CTRL_ONCE | TMR_CTRL_LENGTH;
    PPI_TIMER.CH[0].CSCTRL = TMR_CSCTRL_TCF1EN;

    attachInterruptVector(PPI_TIMERIRQ, ppi_timeout_isr);
    NVIC_SET_PRIORITY(PPI_TIMERIRQ, 3);
    NVIC_ENABLE_IRQ(PPI_TIMERIRQ);

    PPI_TIMER.ENBL = 1;

    ppi_init();

  #endif // PPI_ENABLE

#endif // !PLASMA_ENABLE

#endif // !VFD_SPINDLE

  // Set defaults

    IOInitDone = settings->version == 19;

    hal.settings_changed(settings);
    hal.stepper.go_idle(true);

#if IOPORTS_ENABLE
    ioports_init();
#endif

#if SDCARD_ENABLE
    sdcard_init();
#endif

#if ETHERNET_ENABLE
    grbl_enet_start();
#endif

#if QEI_ENABLE
    if(qei_enable)
        encoder_start(&qei.encoder);
#endif

    return IOInitDone;
}

#if EEPROM_ENABLE == 0

// EEPROM emulation - stores settings in flash

bool nvsRead (uint8_t *dest)
{
// assert size ? E2END

    eeprom_read_block(dest, 0, hal.nvs.size);

    return true; //?;
}

bool nvsWrite (uint8_t *source)
{
    eeprom_write_block(source, 0, hal.nvs.size);

    return true; //?;
}

// End EEPROM emulation

#endif

#if ETHERNET_ENABLE || ADD_MSEVENT

static void execute_realtime (uint_fast16_t state)
{
#if ADD_MSEVENT
    if(ms_event) {

        ms_event = false;

  #if USB_SERIAL_CDC
        usb_execute_realtime(state);
  #endif

  #if QEI_ENABLE
        if(qei.vel_timeout && !(--qei.vel_timeout)) {
            qei.encoder.velocity = abs(qei.count - qei.vel_count) * 1000 / (millis() - qei.vel_timestamp);
            qei.vel_timestamp = millis();
            qei.vel_timeout = QEI_VELOCITY_TIMEOUT;
            if((qei.encoder.event.position_changed = !qei.dbl_click_timeout || qei.encoder.velocity == 0))
                hal.encoder.on_event(&qei.encoder, qei.count);
            qei.vel_count = qei.count;
        }

        if(qei.dbl_click_timeout && !(--qei.dbl_click_timeout)) {
            qei.encoder.event.click = On;
            hal.encoder.on_event(&qei.encoder, qei.count);
        }
  #endif
    }
#endif // ADD_MSEVENT

#if ETHERNET_ENABLE
    grbl_enet_poll();
#endif
}

#endif

#ifdef DEBUGOUT
void debugOut (bool on)
{
    digitalWrite(13, on); // LED
}
#endif

#ifdef UART_DEBUG
void uart_debug_write (char *s)
{
    serialWriteS(s);
    while(serialTxCount()); // Wait until message is delivered
}
#endif

// Cold restart (T4.x has no reset button)
static void reboot (void)
{
    SCB_AIRCR = 0x05FA0004;
}

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

#if USB_SERIAL_CDC == 1
    strcat(options, "USB.1 ");
#endif
#if USB_SERIAL_CDC == 2
    strcat(options, "USB.2 ");
#endif

    if(*options != '\0')
        options[strlen(options) - 1] = '\0';

    hal.info = "iMXRT1062";
    hal.driver_version = "210313";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.driver_options = *options == '\0' ? NULL : options;
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
#ifdef SQUARING_ENABLED
    hal.stepper.get_auto_squared = getAutoSquaredAxes;
    hal.stepper.disable_motors = StepperDisableMotors;
#endif

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;
    hal.homing.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

    hal.probe.configure = probeConfigure;
    hal.probe.get_state = probeGetState;

#if !VFD_SPINDLE
    hal.spindle.set_state = spindleSetState;
    hal.spindle.get_state = spindleGetState;
  #ifdef SPINDLE_PWM_DIRECT
    hal.spindle.get_pwm = spindleGetPWM;
    hal.spindle.update_pwm = spindle_set_speed;
  #else
    hal.spindle.update_rpm = spindleUpdateRPM;
  #endif
  #if PPI_ENABLE
    hal.spindle.pulse_on = spindlePulseOn;
  #endif
#endif
#ifdef SPINDLE_SYNC_ENABLE
    hal.driver_cap.spindle_sync = On;
    hal.driver_cap.spindle_at_speed = On;
#endif
    hal.control.get_state = systemGetState;

#if ETHERNET_ENABLE
    grbl.on_report_options = reportIP;
#endif

#if USB_SERIAL_CDC
    usb_serialInit();
#else
    serialInit(115200);
#endif

    selectStream(StreamType_Serial);

#ifdef I2C_PORT
    i2c_init();
#endif

#if EEPROM_ENABLE
    i2c_eeprom_init();
#else // use Arduino emulated EEPROM in flash
    eeprom_initialize();
    hal.nvs.type = NVS_Flash;
    hal.nvs.memcpy_from_flash = nvsRead;
    hal.nvs.memcpy_to_flash = nvsWrite;
#endif

    hal.reboot = reboot;
    hal.irq_enable = enable_irq;
    hal.irq_disable = disable_irq;
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = millis;

#if ETHERNET_ENABLE || ADD_MSEVENT
    grbl.on_execute_realtime = execute_realtime;
#endif

#if QEI_ENABLE
    hal.encoder.reset = qei_reset;
    hal.encoder.on_event = encoder_event;
#endif

#if MODBUS_ENABLE
    modbus_stream.write = serialWrite;
    modbus_stream.read = serialGetC;
    modbus_stream.flush_rx_buffer = serialRxFlush;
    modbus_stream.flush_tx_buffer = serialTxFlush;
    modbus_stream.get_rx_buffer_count = serialRxCount;
    modbus_stream.get_tx_buffer_count = serialTxCount;
    modbus_stream.set_baud_rate = serialSetBaudRate;

    bool modbus = modbus_init(&modbus_stream);

#if SPINDLE_HUANYANG
    if(modbus)
        huanyang_init(&modbus_stream);
#endif

#endif

  // Driver capabilities, used for announcing and negotiating (with Grbl) driver functionality.
  // See driver_cap_t union i grbl/hal.h for available flags.

#if ESTOP_ENABLE
    hal.signals_cap.e_stop = On;
#endif
#if SAFETY_DOOR_ENABLE
    hal.signals_cap.safety_door_ajar = On;
#endif
#ifdef LIMITS_OVERRIDE_PIN
    hal.signals_cap.limits_override = On;
#endif

#if !VFD_SPINDLE && !PLASMA_ENABLE
  #ifdef SPINDLE_DIRECTION_PIN
    hal.driver_cap.spindle_dir = On;
  #endif
    hal.driver_cap.variable_spindle = On;
#endif
#ifdef COOLANT_MIST_PIN
    hal.driver_cap.mist_control = On;
#endif
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;

#ifdef DEBUGOUT
    hal.debug_out = debugOut;
#endif

#ifdef UART_DEBUG
    serialInit(115200);
    uart_debug_write(ASCII_EOL "UART Debug:" ASCII_EOL);
#endif

#ifdef HAS_BOARD_INIT
    board_init();
#endif

#if ETHERNET_ENABLE
    grbl_enet_init();
#endif

#if KEYPAD_ENABLE
    keypad_init();
#endif

#if QEI_ENABLE
    qei_enable = encoder_init(QEI_ENABLE);
#endif

#if PLASMA_ENABLE
    hal.stepper.output_step = stepperOutputStep;
    plasma_init();
#endif

    my_plugin_init();

#if ODOMETER_ENABLE
    odometer_init(); // NOTE: this *must* be last plugin to be initialized as it claims storage at the end of NVS.
#endif

    // No need to move version check before init.
    // Compiler will fail any signature mismatch for existing entries.
    return hal.version == 8;
}

/* interrupt handlers */

// Main stepper driver.
static void stepper_driver_isr (void)
{
    if(PIT_TFLG0 & PIT_TFLG_TIF) {
        PIT_TFLG0 |= PIT_TFLG_TIF;
        hal.stepper.interrupt_callback();
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
    TMR4_CSCTRL0 &= ~TMR_CSCTRL_TCF1;

    set_step_outputs(next_step_outbits);

    attachInterruptVector(IRQ_QTIMER4, stepper_pulse_isr);
    TMR4_COMP10 = pulse_length;
    TMR4_CTRL0 |= TMR_CTRL_CM(0b001);
}

#if defined(SPINDLE_SYNC_ENABLE) && SPINDLE_PULSE_PIN == 14

static void spindle_pulse_isr (void)
{
    uint32_t tval = GPT1_CNT;

    GPT2_SR |= GPT_SR_OF1; // clear interrupt flag
    GPT2_OCR1 += spindle_encoder.tics_per_irq;

    spindleLock = true;

    spindle_encoder.counter.pulse_count = GPT2_CNT;
    spindle_encoder.counter.last_count = spindle_encoder.counter.pulse_count;
    spindle_encoder.timer.pulse_length = tval - spindle_encoder.timer.last_pulse;
    spindle_encoder.timer.last_pulse = tval;

    spindleLock = false;
}

#endif

#if PLASMA_ENABLE
static void output_pulse_isr(void)
{
    axes_signals_t output = {pulse_output.mask ^ settings.steppers.dir_invert.mask};

    TMR2_CSCTRL0 &= ~TMR_CSCTRL_TCF1;

    DIGITAL_OUT(stepZ, output.z);

    if(pulse_output.value) {
        pulse_output.value = 0;
        TMR2_CTRL0 |= TMR_CTRL_CM(0b001);
    }
}
#endif

#if PPI_ENABLE
// Switches off the spindle (laser) after laser.pulse_length time has elapsed
static void ppi_timeout_isr (void)
{
    PPI_TIMER.CH[0].CSCTRL &= ~TMR_CSCTRL_TCF1;
    spindle_off();
}
#endif

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
        hal.limits.interrupt_callback(limitsGetState());

    if(grp & INPUT_GROUP_CONTROL)
        hal.control.interrupt_callback(systemGetState());

#if QEI_SELECT_ENABLED

    if(grp & INPUT_GROUP_QEI_SELECT) {
        if(!qei.dbl_click_timeout)
            qei.dbl_click_timeout = qei.encoder.settings->dbl_click_window;
        else if(qei.dbl_click_timeout < qei.encoder.settings->dbl_click_window - 40) {
            qei.dbl_click_timeout = 0;
            qei.encoder.event.dbl_click = On;
            hal.encoder.on_event(&qei.encoder, qei.count);
        }
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
                        qei_update();
                        /*
                        QEI_A.reg->IMR &= ~QEI_A.bit;       // Switch off
                        QEI_B.reg->IMR &= ~QEI_B.bit;       // encoder interrupts.
                        qei.iflags.a = inputpin[i].port == &QEI_A;
                        qei.iflags.b = inputpin[i].port == &QEI_B;
                        qei.debounce = QEI_DEBOUNCE;
                        qei.initial_debounce = true;
                        */
                    } else
#endif

#if defined(SPINDLE_SYNC_ENABLE) && defined(SPINDLE_INDEX_PIN)
                    if(inputpin[i].group & INPUT_GROUP_SPINDLE_INDEX) {
                        spindleLock = true;
                        spindle_encoder.counter.index_count++;
                        spindle_encoder.counter.last_index = GPT2_CNT;
                        spindle_encoder.timer.last_index = GPT1_CNT;
                        spindleLock = false;
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

    if(grp & INPUT_GROUP_LIMIT) {
        limit_signals_t state = limitsGetState();
        if(limit_signals_merge(state).value) //TODO: add check for limit switches having same state as when limit_isr were invoked?
            hal.limits.interrupt_callback(state);
    }

    if(grp & INPUT_GROUP_CONTROL)
        hal.control.interrupt_callback(systemGetState());

#if QEI_SELECT_ENABLED

    if(grp & INPUT_GROUP_QEI_SELECT) {
        if(!qei.dbl_click_timeout)
            qei.dbl_click_timeout = qei.encoder.settings->dbl_click_window;
        else if(qei.dbl_click_timeout < qei.encoder.settings->dbl_click_window - 40) {
            qei.dbl_click_timeout = 0;
            qei.encoder.event.dbl_click = On;
            hal.encoder.on_event(&qei.encoder, qei.count);
        }
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
        keypad_keyclick_handler(!(KeypadStrobe.reg->DR & KeypadStrobe.bit));
#endif
}

// Interrupt handler for 1 ms interval timer
static void systick_isr (void)
{
    systick_isr_org();

#if ADD_MSEVENT
    ms_event = true;
#endif

    if(grbl_delay.ms && !(--grbl_delay.ms)) {
        if(grbl_delay.callback) {
            grbl_delay.callback();
            grbl_delay.callback = NULL;
        }
    }
}
