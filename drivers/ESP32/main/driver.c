/*
  driver.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

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

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "grbl/limits.h"

#include "driver.h"
#include "esp32-hal-uart.h"
#include "nvs.h"
#include "grbl/protocol.h"
#include "esp_log.h"
//#include "grbl_esp32_if/grbl_esp32_if.h"

#ifdef USE_I2S_OUT
#include "i2s_out.h"
#endif

#if WIFI_ENABLE
#include "wifi.h"
#endif

#if WEBUI_ENABLE
#include "webui/response.h"
#endif

#if TELNET_ENABLE
#include "networking/TCPStream.h"
#endif

#if WEBSOCKET_ENABLE
#include "networking/WsStream.h"
#endif

#if BLUETOOTH_ENABLE
#include "bluetooth.h"
#endif

#if SDCARD_ENABLE
#include "sdcard/sdcard.h"
#include "esp_vfs_fat.h"
#endif

#if KEYPAD_ENABLE
#include "keypad/keypad.h"
#endif

#if IOEXPAND_ENABLE
#include "ioexpand.h"
#endif

#if EEPROM_ENABLE
#include "eeprom/eeprom.h"
#endif

#if I2C_ENABLE
#include "i2c.h"
#endif

#ifndef VFD_SPINDLE
static uint32_t pwm_max_value;
static bool pwmEnabled = false;
static spindle_pwm_t spindle_pwm;
#else
#undef SPINDLE_RPM_CONTROLLED
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"

// prescale step counter to 20Mhz
#define STEPPER_DRIVER_PRESCALER 4

#if PWM_RAMPED

#define SPINDLE_RAMP_STEP_INCR 20 // timer compare register change per ramp step
#define SPINDLE_RAMP_STEP_TIME 2  // ms

typedef struct {
    volatile uint32_t ms_cfg;
    volatile uint32_t ms_count;
    uint32_t pwm_current;
    uint32_t pwm_target;
    uint32_t pwm_step;
} pwm_ramp_t;

static pwm_ramp_t pwm_ramp;
#endif

typedef enum {
    Input_Probe = 0,
    Input_Reset,
    Input_FeedHold,
    Input_CycleStart,
    Input_SafetyDoor,
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
    Input_KeypadStrobe
} input_t;

typedef struct {
    input_t id;
    uint8_t pin;
    uint8_t group;
    uint32_t mask;
    uint8_t offset;
    bool invert;
    volatile bool active;
    volatile bool debounce;
} state_signal_t;


#if MPG_MODE_ENABLE
static io_stream_t prev_stream = {0};
#endif

const io_stream_t serial_stream = {
    .type = StreamType_Serial,
    .read = serialRead,
    .write = serialWriteS,
    .write_all = serialWriteS,
    .get_rx_buffer_available = serialRXFree,
    .reset_read_buffer = serialFlush,
    .cancel_read_buffer = serialCancel,
    .suspend_read = serialSuspendInput,
    .enqueue_realtime_command = protocol_enqueue_realtime_command
};

#if WIFI_ENABLE

static network_services_t services = {0};

void tcpStreamWriteS (const char *data)
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
const io_stream_t telnet_stream = {
    .type = StreamType_Telnet,
    .read = TCPStreamGetC,
    .write = TCPStreamWriteS,
    .write_all = tcpStreamWriteS,
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
    .write_all = tcpStreamWriteS,
    .get_rx_buffer_available = WsStreamRxFree,
    .reset_read_buffer = WsStreamRxFlush,
    .cancel_read_buffer = WsStreamRxCancel,
    .suspend_read = WsStreamSuspendInput,
    .enqueue_realtime_command = protocol_enqueue_realtime_command,
};
#endif

#endif // WIFI_ENABLE

#if BLUETOOTH_ENABLE
void btStreamWriteS (const char *data)
{
    BTStreamWriteS(data);
    serialWriteS(data);
}

const io_stream_t bluetooth_stream = {
    .type = StreamType_Bluetooth,
    .read = BTStreamGetC,
    .write = BTStreamWriteS,
    .write_all = btStreamWriteS,
    .get_rx_buffer_available = BTStreamRXFree,
    .reset_read_buffer = BTStreamFlush,
    .cancel_read_buffer = BTStreamCancel,
    .suspend_read = serialSuspendInput,
    .enqueue_realtime_command = protocol_enqueue_realtime_command
};

#endif

#define INPUT_GROUP_CONTROL (1 << 0)
#define INPUT_GROUP_PROBE   (1 << 1)
#define INPUT_GROUP_LIMIT   (1 << 2)
#define INPUT_GROUP_KEYPAD  (1 << 3)
#define INPUT_GROUP_MPG     (1 << 4)

state_signal_t inputpin[] = {
#ifdef RESET_PIN
    { .id = Input_Reset,        .pin = RESET_PIN,       .group = INPUT_GROUP_CONTROL },
#endif
#ifdef FEED_HOLD_PIN
    { .id = Input_FeedHold,     .pin = FEED_HOLD_PIN,   .group = INPUT_GROUP_CONTROL },
#endif
#ifdef CYCLE_START_PIN
    { .id = Input_CycleStart,   .pin = CYCLE_START_PIN, .group = INPUT_GROUP_CONTROL },
#endif
#ifdef SAFETY_DOOR_PIN
    { .id = Input_SafetyDoor,   .pin = SAFETY_DOOR_PIN, .group = INPUT_GROUP_CONTROL },
#endif
#ifdef PROBE_PIN
    { .id = Input_Probe,        .pin = PROBE_PIN,       .group = INPUT_GROUP_PROBE },
#endif
    { .id = Input_LimitX,       .pin = X_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT },
    { .id = Input_LimitY,       .pin = Y_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT },
    { .id = Input_LimitZ,       .pin = Z_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT }
#ifdef A_LIMIT_PIN
  , { .id = Input_LimitA,       .pin = A_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT }
#endif
#ifdef B_LIMIT_PIN
  , { .id = Input_LimitB,       .pin = B_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT }
#endif
#ifdef C_LIMIT_PIN
  , { .id = Input_LimitC,       .pin = C_LIMIT_PIN,     .group = INPUT_GROUP_LIMIT }
#endif
#if MPG_MODE_ENABLE
  , { .id = Input_ModeSelect,   .pin = MPG_ENABLE_PIN,  .group = INPUT_GROUP_MPG }
#endif
#if KEYPAD_ENABLE
  , { .id = Input_KeypadStrobe, .pin = KEYPAD_STROBE_PIN, .group = INPUT_GROUP_KEYPAD }
#endif
};

gpio_num_t outputpin[] =
{
#ifdef STEPPERS_DISABLE_PIN
    STEPPERS_DISABLE_PIN,
#endif
#if defined(SPINDLE_ENABLE_PIN) && SPINDLE_ENABLE_PIN != IOEXPAND
    SPINDLE_ENABLE_PIN,
#endif
#if defined(SPINDLE_DIRECTION_PIN) && SPINDLE_DIRECTION_PIN != IOEXPAND
    SPINDLE_DIRECTION_PIN,
#endif
#if defined(COOLANT_FLOOD_PIN) && COOLANT_FLOOD_PIN != IOEXPAND
    COOLANT_FLOOD_PIN,
#endif
#if defined(COOLANT_MIST_PIN) && COOLANT_MIST_PIN != IOEXPAND
    COOLANT_MIST_PIN,
#endif
    X_DIRECTION_PIN,
    Y_DIRECTION_PIN,
    Z_DIRECTION_PIN
};

static volatile uint32_t ms_count = 1; // NOTE: initial value 1 is for "resetting" systick timer
static bool IOInitDone = false;
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;
#if PROBE_ENABLE
static probe_state_t probe = {
    .connected = On
};
#endif

#ifdef USE_I2S_OUT
#define DIGITAL_IN(pin) i2s_out_state(pin)
#define DIGITAL_OUT(pin, state) i2s_out_write(pin, state)
uint32_t i2s_step_length = I2S_OUT_USEC_PER_PULSE, i2s_step_samples = 1;
#else
#define DIGITAL_IN(pin) gpio_get_level(pin)
#define DIGITAL_OUT(pin, state) gpio_set_level(pin, state)
#endif

#if IOEXPAND_ENABLE
static ioexpand_t iopins = {0};
#endif

#ifndef VFD_SPINDLE

static void spindle_set_speed (uint_fast16_t pwm_value);

static ledc_timer_config_t ledTimerConfig = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 5000
};

static ledc_channel_config_t ledConfig = {
    .gpio_num = SPINDLEPWMPIN,
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0,
    .duty = 0,  /*!< LEDC channel duty, the range of duty setting is [0, (2**duty_resolution)] */
    .hpoint = 0
};

#endif

#if MODBUS_ENABLE
static modbus_stream_t modbus_stream = {0};
#endif

// Interrupt handler prototypes
static void gpio_isr (void *arg);
static void stepper_driver_isr (void *arg);

static TimerHandle_t xDelayTimer = NULL, debounceTimer = NULL;

static void activateStream (const io_stream_t *stream)
{
#if MPG_MODE_ENABLE
    if(hal.stream.type == StreamType_MPG) {
        hal.stream.write_all = stream->write_all;
        if(prev_stream.reset_read_buffer != NULL)
            prev_stream.reset_read_buffer();
        memcpy(&prev_stream, stream, sizeof(io_stream_t));
    } else
#endif
        memcpy(&hal.stream, stream, sizeof(io_stream_t));
}

void selectStream (stream_type_t stream)
{
    static stream_type_t active_stream = StreamType_Serial;

    switch(stream) {

#if BLUETOOTH_ENABLE
        case StreamType_Bluetooth:
            activateStream(&bluetooth_stream);
//            services.bluetooth = On;
            break;
#endif

#if TELNET_ENABLE
        case StreamType_Telnet:
            hal.stream.write_all("[MSG:TELNET STREAM ACTIVE]\r\n");
            activateStream(&telnet_stream);
            services.telnet = On;
            break;
#endif

#if WEBSOCKET_ENABLE
        case StreamType_WebSocket:
            hal.stream.write_all("[MSG:WEBSOCKET STREAM ACTIVE]\r\n");
            activateStream(&websocket_stream);
            services.websocket = On;
            break;
#endif

        case StreamType_Serial:
            activateStream(&serial_stream);
#if WIFI_ENABLE
            services.mask = 0;
#endif
            if(active_stream != StreamType_Serial)
                hal.stream.write_all("[MSG:SERIAL STREAM ACTIVE]\r\n");
            break;

        default:
            break;
    }

    active_stream = stream;
}

void initRMT (settings_t *settings)
{
    rmt_item32_t rmtItem[2];

    rmt_config_t rmtConfig = {
        .rmt_mode = RMT_MODE_TX,
        .clk_div = 20,
        .mem_block_num = 1,
        .tx_config.loop_en = false,
        .tx_config.carrier_en = false,
        .tx_config.carrier_freq_hz = 0,
        .tx_config.carrier_duty_percent = 50,
        .tx_config.carrier_level = RMT_CARRIER_LEVEL_LOW,
        .tx_config.idle_output_en = true
    };

    rmtItem[0].duration0 = (uint32_t)(settings->steppers.pulse_delay_microseconds > 0.0f ? 4.0f * settings->steppers.pulse_delay_microseconds : 1.0f);
    rmtItem[0].duration1 = (uint32_t)(4.0f * settings->steppers.pulse_microseconds);
    rmtItem[1].duration0 = 0;
    rmtItem[1].duration1 = 0;

    uint32_t channel;
    for(channel = 0; channel < N_AXIS; channel++) {

        rmtConfig.channel = channel;

        switch(channel) {
            case 0:
                rmtConfig.tx_config.idle_level = settings->steppers.step_invert.x;
                rmtConfig.gpio_num = X_STEP_PIN;
                break;
            case 1:
                rmtConfig.tx_config.idle_level = settings->steppers.step_invert.y;
                rmtConfig.gpio_num = Y_STEP_PIN;
                break;
            case 2:
                rmtConfig.tx_config.idle_level = settings->steppers.step_invert.z;
                rmtConfig.gpio_num = Z_STEP_PIN;
                break;
        }
        rmtItem[0].level0 = rmtConfig.tx_config.idle_level;
        rmtItem[0].level1 = !rmtConfig.tx_config.idle_level;
        rmt_config(&rmtConfig);
        rmt_fill_tx_items(rmtConfig.channel, &rmtItem[0], 2, 0);
    }
}

void vTimerCallback (TimerHandle_t xTimer)
{
    void (*callback)(void) = (void (*)(void))pvTimerGetTimerID(xTimer);

    if(callback)
        callback();

    xTimerDelete(xDelayTimer, 3);
    xDelayTimer = NULL;
}

IRAM_ATTR static void driver_delay_ms (uint32_t ms, void (*callback)(void))
{
    if(callback) {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if(xDelayTimer) {
            xTimerDelete(xDelayTimer, 3);
            xDelayTimer = NULL;
        }
        xDelayTimer = xTimerCreate("msDelay", pdMS_TO_TICKS(ms), pdFALSE, callback, vTimerCallback);
        xTimerStartFromISR(xDelayTimer, &xHigherPriorityTaskWoken);
        if(xHigherPriorityTaskWoken)
            portYIELD_FROM_ISR();
    } else {
        if(xDelayTimer) {
            xTimerDelete(xDelayTimer, 3);
            xDelayTimer = NULL;
        }
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
}

#ifdef DEBUGOUT
static void debug_out (bool enable)
{
    gpio_set_level(STEPPERS_DISABLE_PIN, enable);
}
#endif

// Set stepper direction output pins
// NOTE: see note for set_step_outputs()
inline IRAM_ATTR static void set_dir_outputs (axes_signals_t dir_outbits)
{
    dir_outbits.value ^= settings.steppers.dir_invert.mask;
    DIGITAL_OUT(X_DIRECTION_PIN, dir_outbits.x);
    DIGITAL_OUT(Y_DIRECTION_PIN, dir_outbits.y);
    DIGITAL_OUT(Z_DIRECTION_PIN, dir_outbits.z);
#ifdef A_AXIS
    DIGITAL_OUT(A_DIRECTION_PIN, dir_outbits.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(B_DIRECTION_PIN, dir_outbits.b);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(C_DIRECTION_PIN, dir_outbits.c);
#endif
}

// Enable/disable steppers
static void stepperEnable (axes_signals_t enable)
{
    enable.mask ^= settings.steppers.enable_invert.mask;

#if TRINAMIC_ENABLE && TRINAMIC_I2C
    axes_signals_t tmc_enable = trinamic_stepper_enable(enable);
 #if !CNC_BOOSTERPACK // Trinamic BoosterPack does not support mixed drivers
  #if IOEXPAND_ENABLE
    if(!tmc_enable.x)
        iopins.stepper_enable_x = enable.x;
    if(!tmc_enable.y)
        iopins.stepper_enable_y = enable.y;
    if(!tmc_enable.z)
        iopins.stepper_enable_z = enable.z;
  #endif
 #endif
#elif IOEXPAND_ENABLE // TODO: read from expander?
    iopins.stepper_enable_x = enable.x;
    iopins.stepper_enable_y = enable.y;
    iopins.stepper_enable_z = enable.z;
    ioexpand_out(iopins);
#elif defined(STEPPERS_DISABLE_PIN)
    DIGITAL_OUT(STEPPERS_DISABLE_PIN, enable.x);
#else
    DIGITAL_OUT(X_DISABLE_PIN, enable.x);
    DIGITAL_OUT(Y_DISABLE_PIN, enable.y);
    DIGITAL_OUT(Z_DISABLE_PIN, enable.z);
#ifdef A_AXIS
    DIGITAL_OUT(A_DISABLE_PIN, enable.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(B_DISABLE_PIN, enable.b);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(C_DISABLE_PIN, enable.c);
#endif
#endif
}

#ifdef USE_I2S_OUT

// Set stepper pulse output pins
inline __attribute__((always_inline)) IRAM_ATTR static void i2s_set_step_outputs (axes_signals_t step_outbits)
{
    step_outbits.value ^= settings.steppers.step_invert.mask;
    DIGITAL_OUT(X_STEP_PIN, step_outbits.x);
    DIGITAL_OUT(Y_STEP_PIN, step_outbits.y);
    DIGITAL_OUT(Z_STEP_PIN, step_outbits.z);
#ifdef A_AXIS
    DIGITAL_OUT(A_STEP_PIN, step_outbits.a);
#endif
#ifdef B_AXIS
    DIGITAL_OUT(B_STEP_PIN, step_outbits.b);
#endif
#ifdef C_AXIS
    DIGITAL_OUT(C_STEP_PIN, step_outbits.c);
#endif
}

IRAM_ATTR static void I2S_stepperGoIdle (bool clear_signals)
{
    if(clear_signals) {
        i2s_set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
        i2s_out_reset();
    }

    i2s_out_set_passthrough();
}

IRAM_ATTR static void I2S_stepperCyclesPerTick (uint32_t cycles_per_tick)
{
    i2s_out_set_pulse_period(cycles_per_tick);
}

// Sets stepper direction and pulse pins and starts a step pulse
IRAM_ATTR static void I2S_stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_change)
        set_dir_outputs(stepper->dir_outbits);

    if(stepper->step_outbits.value) {
        i2s_set_step_outputs(stepper->step_outbits);
        i2s_out_push_sample(i2s_step_samples);
        i2s_set_step_outputs((axes_signals_t){0});
    }
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void I2S_stepperWakeUp (void)
{
    // Enable stepper drivers.
    stepperEnable((axes_signals_t){AXES_BITMASK});
    i2s_out_set_stepping();
}

#else

// Set stepper pulse output pins
inline IRAM_ATTR static void set_step_outputs (axes_signals_t step_outbits)
{
    if(step_outbits.x) {
        RMT.conf_ch[0].conf1.mem_rd_rst = 1;
        RMT.conf_ch[0].conf1.tx_start = 1;
    }

    if(step_outbits.y) {
        RMT.conf_ch[1].conf1.mem_rd_rst = 1;
        RMT.conf_ch[1].conf1.tx_start = 1;
    }

    if(step_outbits.z) {
        RMT.conf_ch[2].conf1.mem_rd_rst = 1;
        RMT.conf_ch[2].conf1.tx_start = 1;
    }
}

#endif

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
    // Enable stepper drivers.
    stepperEnable((axes_signals_t){AXES_BITMASK});

    timer_set_counter_value(STEP_TIMER_GROUP, STEP_TIMER_INDEX, 0x00000000ULL);
//  timer_set_alarm_value(STEP_TIMER_GROUP, STEP_TIMER_INDEX, 5000ULL);
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarm_high = 0;
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarm_low = 5000UL;

    timer_start(STEP_TIMER_GROUP, STEP_TIMER_INDEX);
    TIMERG0.hw_timer[STEP_TIMER_INDEX].config.alarm_en = TIMER_ALARM_EN;
}

// Disables stepper driver interrupts
IRAM_ATTR static void stepperGoIdle (bool clear_signals)
{
    timer_pause(STEP_TIMER_GROUP, STEP_TIMER_INDEX);

    if(clear_signals) {
#ifdef USE_I2S_OUT
        i2s_set_step_outputs((axes_signals_t){0});
#else
        set_step_outputs((axes_signals_t){0});
#endif
        set_dir_outputs((axes_signals_t){0});
    }
}

// Sets up stepper driver interrupt timeout
IRAM_ATTR static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
// Limit min steps/s to about 2 (hal.f_step_timer @ 20MHz)
#ifdef ADAPTIVE_MULTI_AXIS_STEP_SMOOTHING
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarm_low = cycles_per_tick < (1UL << 18) ? cycles_per_tick : (1UL << 18) - 1UL;
#else
    TIMERG0.hw_timer[STEP_TIMER_INDEX].alarm_low = cycles_per_tick < (1UL << 23) ? cycles_per_tick : (1UL << 23) - 1UL;
#endif
}

// Sets stepper direction and pulse pins and starts a step pulse
IRAM_ATTR static void stepperPulseStart (stepper_t *stepper)
{
    if(stepper->dir_change)
        set_dir_outputs(stepper->dir_outbits);

    if(stepper->step_outbits.value) {
#ifdef USE_I2S_OUT
        uint64_t step_pulse_start_time = esp_timer_get_time();
        i2s_set_step_outputs(stepper->step_outbits);
        while (esp_timer_get_time() - step_pulse_start_time < i2s_step_length) {
            __asm__ __volatile__ ("nop");  // spin here until time to turn off step
        }
        i2s_set_step_outputs((axes_signals_t){0});
#else
        set_step_outputs(stepper->step_outbits);
#endif
    }
}

#ifdef USE_I2S_OUT

static void i2s_set_streaming_mode (bool stream)
{
    if(!stream && hal.stepper.wake_up == I2S_stepperWakeUp) {
        i2s_out_set_passthrough();
        i2s_out_delay();
    }

    if(stream) {
        hal.stepper.wake_up = I2S_stepperWakeUp;
        hal.stepper.go_idle = I2S_stepperGoIdle;
        hal.stepper.cycles_per_tick = I2S_stepperCyclesPerTick;
        hal.stepper.pulse_start = I2S_stepperPulseStart;
    } else {
        hal.stepper.wake_up = stepperWakeUp;
        hal.stepper.go_idle = stepperGoIdle;
        hal.stepper.cycles_per_tick = stepperCyclesPerTick;
        hal.stepper.pulse_start = stepperPulseStart;
    }
}

#endif

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing)
{
#ifdef USE_I2S_OUT
    i2s_set_streaming_mode(!homing);
#endif

    uint32_t i = sizeof(inputpin) / sizeof(state_signal_t);
    do {
        if(inputpin[--i].group == INPUT_GROUP_LIMIT)
            gpio_set_intr_type(inputpin[i].pin, on ? (inputpin[i].invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE) : GPIO_INTR_DISABLE);
    } while(i);

#if TRINAMIC_ENABLE
    trinamic_homing(homing);
#endif
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline IRAM_ATTR static limit_signals_t limitsGetState()
{
    limit_signals_t signals = {0};

    signals.min.x = gpio_get_level(X_LIMIT_PIN);
    signals.min.y = gpio_get_level(Y_LIMIT_PIN);
    signals.min.z = gpio_get_level(Z_LIMIT_PIN);
#ifdef A_LIMIT_PIN
    signals.min.a = gpio_get_level(A_LIMIT_PIN);
#endif
#ifdef B_LIMIT_PIN
    signals.min.b = gpio_get_level(B_LIMIT_PIN);
#endif
#ifdef C_LIMIT_PIN
    signals.min.c = gpio_get_level(C_LIMIT_PIN);
#endif

    if (settings.limits.invert.value)
        signals.min.value ^= settings.limits.invert.value;

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
inline IRAM_ATTR static control_signals_t systemGetState (void)
{
    control_signals_t signals;

    signals.value = settings.control_invert.value;

#ifdef RESET_PIN
    signals.reset = gpio_get_level(RESET_PIN);
#endif
#ifdef FEED_HOLD_PIN
    signals.feed_hold = gpio_get_level(FEED_HOLD_PIN);
#endif
#ifdef CYCLE_START_PIN
    signals.cycle_start = gpio_get_level(CYCLE_START_PIN);
#endif
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
    signals.safety_door_ajar = gpio_get_level(SAFETY_DOOR_PIN);
#endif

    if(settings.control_invert.value)
        signals.value ^= settings.control_invert.value;

    return signals;
}

#ifdef PROBE_PIN

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure(bool is_probe_away, bool probing)
{
#ifdef USE_I2S_OUT
    i2s_set_streaming_mode(!probing);
#endif

    probe.triggered = Off;
    probe.is_probing = probing;
    probe.inverted = is_probe_away ? !settings.probe.invert_probe_pin : settings.probe.invert_probe_pin;

#if PROBE_ISR
    gpio_set_intr_type(inputpin[INPUT_PROBE].pin, probe_invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE);
    inputpin[INPUT_PROBE].active = false;
#endif
}

// Returns the probe connected and triggered pin states.
probe_state_t probeGetState (void)
{
    probe_state_t state = {0};

    state.connected = probe.connected;

#if PROBE_ISR
    // TODO: verify!
    inputpin[INPUT_PROBE].active = inputpin[INPUT_PROBE].active || ((uint8_t)gpio_get_level(PROBE_PIN) ^ probe.inverted);
    state.triggered = inputpin[INPUT_PROBE].active;
#else
    state.triggered = (uint8_t)gpio_get_level(PROBE_PIN) ^ probe.inverted;
#endif

    return state;
}

#endif

#ifndef VFD_SPINDLE

// Static spindle (off, on cw & on ccw)
IRAM_ATTR inline static void spindle_off (void)
{
#if IOEXPAND_ENABLE
    iopins.spindle_on = settings.spindle.invert.on ? On : Off;
    ioexpand_out(iopins);
#else
    gpio_set_level(SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? 1 : 0);
#endif
}

IRAM_ATTR inline static void spindle_on (void)
{
#if IOEXPAND_ENABLE
    iopins.spindle_on = settings.spindle.invert.on ? Off : On;
    ioexpand_out(iopins);
#else
    gpio_set_level(SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? 0 : 1);
#endif
}

IRAM_ATTR inline static void spindle_dir (bool ccw)
{
    if(hal.driver_cap.spindle_dir) {
#if IOEXPAND_ENABLE
        iopins.spindle_dir = (ccw ^ settings.spindle.invert.ccw) ? On : Off;
        ioexpand_out(iopins);
#elif defined(SPINDLE_DIRECTION_PIN)
        gpio_set_level(SPINDLE_DIRECTION_PIN, (ccw ^ settings.spindle.invert.ccw) ? 1 : 0);
#endif
    }
}

// Start or stop spindle
IRAM_ATTR static void spindleSetState (spindle_state_t state, float rpm)
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
IRAM_ATTR static void spindle_set_speed (uint_fast16_t pwm_value)
{
    if (pwm_value == spindle_pwm.off_value) {
        if(settings.spindle.flags.pwm_action == SpindleAction_DisableWithZeroSPeed)
            spindle_off();
#if PWM_RAMPED
        pwm_ramp.pwm_target = pwm_value;
        ledc_set_fade_step_and_start(ledConfig.speed_mode, ledConfig.channel, pwm_ramp.pwm_target, 1, 4, LEDC_FADE_NO_WAIT);
#else
        if(spindle_pwm.always_on) {
            ledc_set_duty(ledConfig.speed_mode, ledConfig.channel, spindle_pwm.off_value);
            ledc_update_duty(ledConfig.speed_mode, ledConfig.channel);
        } else
            ledc_stop(ledConfig.speed_mode, ledConfig.channel, settings.spindle.invert.pwm ? 1 : 0);
#endif
        pwmEnabled = false;
     } else {
#if PWM_RAMPED
         pwm_ramp.pwm_target = pwm_value;
         ledc_set_fade_step_and_start(ledConfig.speed_mode, ledConfig.channel, pwm_ramp.pwm_target, 1, 4, LEDC_FADE_NO_WAIT);
#else
         ledc_set_duty(ledConfig.speed_mode, ledConfig.channel, settings.spindle.invert.pwm ? pwm_max_value - pwm_value : pwm_value);
         ledc_update_duty(ledConfig.speed_mode, ledConfig.channel);
#endif
        if(!pwmEnabled) {
            spindle_on();
            pwmEnabled = true;
        }
    }
}

#ifdef SPINDLE_PWM_DIRECT

static uint_fast16_t spindleGetPWM (float rpm)
{
    return spindle_compute_pwm_value(&spindle_pwm, rpm, false);
}

#else // Only enable if (when?) ESP IDF supports FPU access in ISR !!

IRAM_ATTR static void spindleUpdateRPM (float rpm)
{
    spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
}

#endif

// Start or stop spindle, variable version

IRAM_ATTR void __attribute__ ((noinline)) _setSpeed (spindle_state_t state, float rpm)
{
    spindle_dir(state.ccw);
    spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
}

IRAM_ATTR static void spindleSetStateVariable (spindle_state_t state, float rpm)
{
    if (!state.on || memcmp(&rpm, &FZERO, sizeof(float)) == 0) { // rpm == 0.0f cannot be used, causes intermittent panic on soft reset!
        spindle_set_speed(spindle_pwm.off_value);
        spindle_off();
    } else
        _setSpeed(state, rpm);
}

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {0};

#if IOEXPAND_ENABLE // TODO: read from expander?
    state.on = iopins.spindle_on;
    state.ccw = hal.driver_cap.spindle_dir && iopins.spindle_dir;
#else
    state.on = gpio_get_level(SPINDLE_ENABLE_PIN) != 0;
  #if defined(SPINDLE_DIRECTION_PIN)
    state.ccw = hal.driver_cap.spindle_dir && gpio_get_level(SPINDLE_DIRECTION_PIN) != 0;
  #endif
#endif
    state.value ^= settings.spindle.invert.mask;
    state.on |= pwmEnabled;

#if PWM_RAMPED
    state.at_speed = ledc_get_duty(ledConfig.speed_mode, ledConfig.channel) == pwm_ramp.pwm_target;
#endif
    return state;
}

// end spindle code

#endif

// Start/stop coolant (and mist if enabled)
IRAM_ATTR static void coolantSetState (coolant_state_t mode)
{
    mode.value ^= settings.coolant_invert.mask;
#if IOEXPAND_ENABLE
    iopins.flood_on = mode.flood;
    iopins.mist_on = mode.mist;
    ioexpand_out(iopins);
#else
  #ifdef COOLANT_FLOOD_PIN
    gpio_set_level(COOLANT_FLOOD_PIN, mode.flood ? 1 : 0);
  #endif
  #ifdef COOLANT_MIST_PIN
    gpio_set_level(COOLANT_MIST_PIN, mode.mist ? 1 : 0);
  #endif
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
  #ifdef COOLANT_FLOOD_PIN
    state.flood = gpio_get_level(COOLANT_FLOOD_PIN);
  #endif
  #ifdef COOLANT_MIST_PIN
    state.mist  = gpio_get_level(COOLANT_MIST_PIN);
  #endif
#endif

    state.value ^= settings.coolant_invert.mask;

    return state;
}

// Helper functions for setting/clearing/inverting individual bits atomically (uninterruptable)
IRAM_ATTR static void bitsSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    portENTER_CRITICAL(&mux);
    *ptr |= bits;
    portEXIT_CRITICAL(&mux);
}

IRAM_ATTR static uint_fast16_t bitsClearAtomic (volatile uint_fast16_t *ptr, uint_fast16_t bits)
{
    portENTER_CRITICAL(&mux);
    uint_fast16_t prev = *ptr;
    *ptr &= ~bits;
    portEXIT_CRITICAL(&mux);
    return prev;
}

IRAM_ATTR static uint_fast16_t valueSetAtomic (volatile uint_fast16_t *ptr, uint_fast16_t value)
{
    portENTER_CRITICAL(&mux);
    uint_fast16_t prev = *ptr;
    *ptr = value;
    portEXIT_CRITICAL(&mux);
    return prev;
}

static void enable_irq (void)
{
    portEXIT_CRITICAL(&mux);
}

static void disable_irq (void)
{
    portENTER_CRITICAL(&mux);
}

#if MPG_MODE_ENABLE

IRAM_ATTR static void modeSelect (bool mpg_mode)
{
    // Deny entering MPG mode if busy
    if(mpg_mode == sys.mpg_mode || (mpg_mode && (gc_state.file_run || !(sys.state == STATE_IDLE || (sys.state & (STATE_ALARM|STATE_ESTOP)))))) {
        hal.stream.enqueue_realtime_command(CMD_STATUS_REPORT_ALL);
        return;
    }

    serialSelect(mpg_mode);

    if(mpg_mode) {
        memcpy(&prev_stream, &hal.stream, sizeof(io_stream_t));
        hal.stream.type = StreamType_MPG;
        hal.stream.read = serial2Read;
        hal.stream.write = serial_stream.write;
        hal.stream.get_rx_buffer_available = serial2RXFree;
        hal.stream.reset_read_buffer = serial2Flush;
        hal.stream.cancel_read_buffer = serial2Cancel;
        hal.stream.suspend_read = serial2SuspendInput;
    } else if(hal.stream.read != NULL)
        memcpy(&hal.stream, &prev_stream, sizeof(io_stream_t));

    hal.stream.reset_read_buffer();

    sys.mpg_mode = mpg_mode;
    sys.report.mpg_mode = On;

    // Force a realtime status report, all reports when MPG mode active
    hal.stream.enqueue_realtime_command(mpg_mode ? CMD_STATUS_REPORT_ALL : CMD_STATUS_REPORT);
}

IRAM_ATTR static void modeChange(void)
{
    modeSelect(!gpio_get_level(MPG_ENABLE_PIN));
}

IRAM_ATTR static void modeEnable (void)
{
    if(sys.mpg_mode == gpio_get_level(MPG_ENABLE_PIN))
        modeSelect(true);
}

#endif

void debounceTimerCallback (TimerHandle_t xTimer)
{
      uint8_t grp = 0;

      uint32_t i = sizeof(inputpin) / sizeof(state_signal_t);
      do {
          i--;
          if(inputpin[i].debounce && inputpin[i].active) {
              inputpin[i].active = false; //gpio_get_level(inputpin[i].pin) == (inputpin[i].invert ? 0 : 1);
              grp |= inputpin[i].group;
          }
      } while(i);

//    printf("Debounce %d %d\n", grp, limitsGetState().value);

      if(grp & INPUT_GROUP_LIMIT)
            hal.limits.interrupt_callback(limitsGetState());

      if(grp & INPUT_GROUP_CONTROL)
          hal.control.interrupt_callback(systemGetState());
}


// Configures perhipherals when settings are initialized or changed
static void settings_changed (settings_t *settings)
{

#ifndef VFD_SPINDLE

    if((hal.driver_cap.variable_spindle = settings->spindle.rpm_max > settings->spindle.rpm_min)) {

        if(ledTimerConfig.freq_hz != (uint32_t)settings->spindle.pwm_freq) {
            ledTimerConfig.freq_hz = (uint32_t)settings->spindle.pwm_freq;
            if(ledTimerConfig.freq_hz <= 100) {
                if(ledTimerConfig.duty_resolution != LEDC_TIMER_16_BIT) {
                    ledTimerConfig.duty_resolution = LEDC_TIMER_16_BIT;
                    ledc_timer_config(&ledTimerConfig);
                }
            } else if(ledTimerConfig.duty_resolution != LEDC_TIMER_10_BIT) {
                ledTimerConfig.duty_resolution = LEDC_TIMER_10_BIT;
                ledc_timer_config(&ledTimerConfig);
            }
        }

        pwm_max_value = (1UL << ledTimerConfig.duty_resolution) - 1;

        spindle_pwm.period = (uint32_t)(80000000UL / settings->spindle.pwm_freq);
        if(settings->spindle.pwm_off_value == 0.0f)
            spindle_pwm.off_value = settings->spindle.invert.pwm ? pwm_max_value : 0;
        else {
            spindle_pwm.off_value = (uint32_t)(pwm_max_value * settings->spindle.pwm_off_value / 100.0f);
            if(settings->spindle.invert.pwm)
                spindle_pwm.off_value = pwm_max_value - spindle_pwm.off_value;
        }
        spindle_pwm.min_value = (uint32_t)(pwm_max_value * settings->spindle.pwm_min_value / 100.0f);
        spindle_pwm.max_value = (uint32_t)(pwm_max_value * settings->spindle.pwm_max_value / 100.0f) + (settings->spindle.invert.pwm ? -1 : 1);
        spindle_pwm.pwm_gradient = (float)(spindle_pwm.max_value - spindle_pwm.min_value) / (settings->spindle.rpm_max - settings->spindle.rpm_min);
        spindle_pwm.always_on = settings->spindle.pwm_off_value != 0.0f;

        ledc_set_freq(ledTimerConfig.speed_mode, ledTimerConfig.timer_num, ledTimerConfig.freq_hz);
    }

#endif

    if(IOInitDone) {

      #ifndef VFD_SPINDLE
        hal.spindle.set_state = hal.driver_cap.variable_spindle ? spindleSetStateVariable : spindleSetState;
      #endif

#if WIFI_ENABLE

        static bool wifi_ok = false;

        if(!wifi_ok)
            wifi_ok = wifi_start();

        // TODO: start/stop services...
#endif

#if BLUETOOTH_ENABLE
        static bool bluetooth_ok = false;
        if(!bluetooth_ok)
            bluetooth_ok = bluetooth_start();
        // else report error?
#endif

        stepperEnable(settings->steppers.deenergize);

        /*********************
         * Step pulse config *
         *********************/

#ifdef USE_I2S_OUT
        i2s_step_length = (uint32_t)(settings->steppers.pulse_microseconds);
        if(i2s_step_length < I2S_OUT_USEC_PER_PULSE)
            i2s_step_length = I2S_OUT_USEC_PER_PULSE;
        i2s_step_samples = i2s_step_length / I2S_OUT_USEC_PER_PULSE; // round up?
#else
        initRMT(settings);
#endif

        /****************************************
         *  Control, limit & probe pins config  *
         ****************************************/

        bool pullup = true;
        control_signals_t control_fei;
        gpio_config_t config;

        control_fei.mask = settings->control_disable_pullup.mask ^ settings->control_invert.mask;

        axes_signals_t limit_fei;
        limit_fei.mask = settings->limits.disable_pullup.mask ^ settings->limits.invert.mask;

        uint32_t i = sizeof(inputpin) / sizeof(state_signal_t);

        do {

            config.intr_type = GPIO_INTR_DISABLE;

            switch(inputpin[--i].id) {

                case Input_Reset:
                    pullup = !settings->control_disable_pullup.reset;
                    inputpin[i].invert = control_fei.reset;
                    config.intr_type = inputpin[i].invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
                    break;

                case Input_FeedHold:
                    pullup = !settings->control_disable_pullup.feed_hold;
                    inputpin[i].invert = control_fei.feed_hold;
                    config.intr_type = inputpin[i].invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
                    break;

                case Input_CycleStart:
                    pullup = !settings->control_disable_pullup.cycle_start;
                    inputpin[i].invert = control_fei.cycle_start;
                    config.intr_type = inputpin[i].invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
                    break;

                case Input_SafetyDoor:
                    pullup = !settings->control_disable_pullup.safety_door_ajar;
                    inputpin[i].invert = control_fei.safety_door_ajar;
                    config.intr_type = inputpin[i].invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
                    break;
#ifdef PROBE_PIN
                case Input_Probe:
                    pullup = hal.driver_cap.probe_pull_up;
                    inputpin[i].invert = false;
                    break;
#endif
                case Input_LimitX:
                    pullup = !settings->limits.disable_pullup.x;
                    inputpin[i].invert = limit_fei.x;
                    break;

                case Input_LimitY:
                    pullup = !settings->limits.disable_pullup.y;
                    inputpin[i].invert = limit_fei.y;
                    break;

                case Input_LimitZ:
                    pullup = !settings->limits.disable_pullup.z;
                    inputpin[i].invert = limit_fei.z;
                    break;
#ifdef A_LIMIT_PIN
                case Input_LimitA:
                    pullup = !settings->limits.disable_pullup.a;
                    inputpin[i].invert = limit_fei.a;
                    break;
#endif
#ifdef B_LIMIT_PIN
                case Input_LimitB:
                    pullup = !settings->limits.disable_pullup.b;
                    inputpin[i].invert = limit_fei.b;
                    break;
#endif
#ifdef C_LIMIT_PIN
                case Input_LimitC:
                    pullup = !settings->limits.disable_pullup.c;
                    inputpin[i].invert = limit_fei.c;
                    break;
#endif
#if MPG_MODE_ENABLE
                case Input_ModeSelect:
                    pullup = true;
                    inputpin[i].invert = false;
                    config.intr_type = GPIO_INTR_ANYEDGE;
                    break;
#endif
#if KEYPAD_ENABLE
                case Input_KeypadStrobe:
                    pullup = true;
                    inputpin[i].invert = false;
                    config.intr_type = GPIO_INTR_ANYEDGE;
                    break;
#endif
                default:
                    break;

            }

            if(inputpin[i].pin != 0xFF) {

                gpio_intr_disable(inputpin[i].pin);

                config.pin_bit_mask = 1ULL << inputpin[i].pin;
                config.mode = GPIO_MODE_INPUT;
                config.pull_up_en = pullup && inputpin[i].pin < 34 ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
                config.pull_down_en = pullup || inputpin[i].pin >= 34 ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;

                inputpin[i].offset = config.pin_bit_mask > (1ULL << 31) ? 1 : 0;
                inputpin[i].mask = inputpin[i].offset == 0 ? (uint32_t)config.pin_bit_mask : (uint32_t)(config.pin_bit_mask >> 32);

    //            printf("IN %d - %d - %d : %x\n", inputpin[i].pin,  inputpin[i].offset, inputpin[i].mask, inputpin[i].invert);

                gpio_config(&config);

                inputpin[i].active   = gpio_get_level(inputpin[i].pin) == (inputpin[i].invert ? 0 : 1);
                inputpin[i].debounce = hal.driver_cap.software_debounce && !(inputpin[i].group == INPUT_GROUP_PROBE || inputpin[i].group == INPUT_GROUP_KEYPAD || inputpin[i].group == INPUT_GROUP_MPG);
            }
        } while(i);

#if MPG_MODE_ENABLE
        if(hal.driver_cap.mpg_mode)
            // Delay mode enable a bit so grbl can finish startup and MPG controller can check ready status
            hal.delay_ms(50, modeEnable);
#endif
    }
}

#if WIFI_ENABLE
static void reportConnection (bool newopt)
{
    if(!newopt && (services.telnet || services.websocket)) {
        hal.stream.write("[NETCON:");
        hal.stream.write(services.telnet ? "Telnet" : "Websocket");
        hal.stream.write("]" ASCII_EOL);
    }
}
#endif

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
    /******************
     *  Stepper init  *
     ******************/

    timer_config_t timerConfig = {
        .divider     = STEPPER_DRIVER_PRESCALER,
        .counter_dir = TIMER_COUNT_UP,
        .counter_en  = TIMER_PAUSE,
        .alarm_en    = TIMER_ALARM_EN,
        .intr_type   = TIMER_INTR_LEVEL,
        .auto_reload = true
    };

    timer_init(STEP_TIMER_GROUP, STEP_TIMER_INDEX, &timerConfig);
    timer_set_counter_value(STEP_TIMER_GROUP, STEP_TIMER_INDEX, 0ULL);
    timer_isr_register(STEP_TIMER_GROUP, STEP_TIMER_INDEX, stepper_driver_isr, 0, ESP_INTR_FLAG_IRAM, NULL);
    timer_enable_intr(STEP_TIMER_GROUP, STEP_TIMER_INDEX);

    /********************
     *  Output signals  *
     ********************/

    uint32_t idx;
    for(idx = 0; idx < N_AXIS; idx++)
        rmt_set_source_clk(idx, RMT_BASECLK_APB);

    uint32_t mask = 0; // this is insane...
    idx = sizeof(outputpin) / sizeof(gpio_num_t);
    do {
        mask |= (1ULL << outputpin[--idx]);
    } while(idx);

    gpio_config_t gpioConfig = {
        .pin_bit_mask = mask,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&gpioConfig);

#if MPG_MODE_ENABLE
ccc
    /************************
     *  MPG mode (pre)init  *
     ************************/

    // Set as output low (until boot is complete)
    gpioConfig.pin_bit_mask = (1ULL << MPG_ENABLE_PIN);
    gpio_config(&gpioConfig);
    gpio_set_level(MPG_ENABLE_PIN, 0);

    serial2Init(BAUD_RATE);
#endif

   /****************************
    *  Software debounce init  *
    ****************************/

    if(hal.driver_cap.software_debounce)
        debounceTimer = xTimerCreate("debounce", pdMS_TO_TICKS(32), pdFALSE, NULL, debounceTimerCallback);

    /******************************************
     *  Control, limit & probe pins dir init  *
     ******************************************/

    gpio_isr_register(gpio_isr, NULL, (int)ESP_INTR_FLAG_IRAM, NULL);

#ifndef VFD_SPINDLE

    /******************
    *  Spindle init  *
    ******************/

#if PWM_RAMPED
    ledc_fade_func_install(ESP_INTR_FLAG_IRAM);
#endif
    ledConfig.speed_mode = ledTimerConfig.speed_mode;
    ledc_timer_config(&ledTimerConfig);
    ledc_channel_config(&ledConfig);

    /**/

#endif

#if SDCARD_ENABLE

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5
//        .allocation_unit_size = 16 * 1024
    };

    slot_config.gpio_miso = PIN_NUM_MISO;
    slot_config.gpio_mosi = PIN_NUM_MOSI;
    slot_config.gpio_sck  = PIN_NUM_CLK;
    slot_config.gpio_cs   = PIN_NUM_CS;

    host.max_freq_khz = 20000; //SDMMC_FREQ_DEFAULT; //SDMMC_FREQ_PROBING; 19000;

    sdmmc_card_t* card;
    esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &card);

    sdcard_init();
#endif

#if IOEXPAND_ENABLE
    ioexpand_init();
#endif

#if WEBUI_ENABLE
    webui_init();
#endif

  // Set defaults

    IOInitDone = settings->version == 19;

    hal.settings_changed(settings);
    hal.stepper.go_idle(true);

    return IOInitDone;
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{
    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

    serialInit();

    hal.info = "ESP32";
    hal.driver_version = "210314";
#ifdef BOARD_NAME
    hal.board = BOARD_NAME;
#endif
    hal.driver_setup = driver_setup;
    hal.f_step_timer = rtc_clk_apb_freq_get() / STEPPER_DRIVER_PRESCALER; // 20 MHz
    hal.rx_buffer_size = RX_BUFFER_SIZE;
    hal.delay_ms = driver_delay_ms;
    hal.settings_changed = settings_changed;

#ifndef USE_I2S_OUT
    hal.stepper.wake_up = stepperWakeUp;
    hal.stepper.go_idle = stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = stepperCyclesPerTick;
    hal.stepper.pulse_start = stepperPulseStart;
#else
    hal.stepper.wake_up = I2S_stepperWakeUp;
    hal.stepper.go_idle = I2S_stepperGoIdle;
    hal.stepper.enable = stepperEnable;
    hal.stepper.cycles_per_tick = I2S_stepperCyclesPerTick;
    hal.stepper.pulse_start = I2S_stepperPulseStart;
    i2s_out_init();
    i2s_out_set_pulse_callback(hal.stepper.interrupt_callback);
#endif

    hal.limits.enable = limitsEnable;
    hal.limits.get_state = limitsGetState;

    hal.coolant.set_state = coolantSetState;
    hal.coolant.get_state = coolantGetState;

#ifdef PROBE_PIN
    hal.probe.get_state = probeGetState;
    hal.probe.configure = probeConfigure;
#endif

#ifndef VFD_SPINDLE
    hal.spindle.set_state = spindleSetState;
    hal.spindle.get_state = spindleGetState;
  #ifdef SPINDLE_PWM_DIRECT
    hal.spindle.get_pwm = spindleGetPWM;
    hal.spindle.update_pwm = spindle_set_speed;
  #else
    hal.spindle.update_rpm = spindleUpdateRPM; // NOTE: fails in laser mode as ESP32 does not handle FPU access in ISRs!
  #endif
#endif

    hal.control.get_state = systemGetState;

    selectStream(StreamType_Serial);

#if I2C_ENABLE
    I2CInit();
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

//    hal.reboot = esp_restart; crashes the MCU...
    hal.irq_enable = enable_irq;
    hal.irq_disable = disable_irq;
    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;
    hal.get_elapsed_ticks = xTaskGetTickCountFromISR;

#ifdef DEBUGOUT
    hal.debug_out = debug_out;
#endif

#if WIFI_ENABLE
    grbl.on_report_options = reportConnection;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

#ifndef VFD_SPINDLE
  #if IOEXPAND_ENABLE || defined(SPINDLE_DIRECTION_PIN)
    hal.driver_cap.spindle_dir = On;
  #endif
    hal.driver_cap.variable_spindle = On;
    hal.driver_cap.spindle_pwm_invert = On;
  #if PWM_RAMPED
    hal.driver_cap.spindle_at_speed = On;
  #endif
  #if IOEXPAND_ENABLE || defined(COOLANT_MIST_PIN)
    hal.driver_cap.mist_control = On;
  #endif
#endif
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;
#ifdef SAFETY_DOOR_PIN
    hal.signals_cap.safety_door_ajar = On;
#endif
#if MPG_MODE_ENABLE
    hal.driver_cap.mpg_mode = On;
#endif

#if MODBUS_ENABLE

    modbus_stream.write = serial2Write;
    modbus_stream.read = serial2Read;
    modbus_stream.flush_rx_buffer = serial2Flush;
    modbus_stream.flush_tx_buffer = serial2Flush;
    modbus_stream.get_rx_buffer_count = serial2Available;
    modbus_stream.get_tx_buffer_count = serial2txCount;
    modbus_stream.set_baud_rate = serial2SetBaudRate;

    bool modbus = modbus_init(&modbus_stream);

#if SPINDLE_HUANYANG > 0
    if(modbus)
        huanyang_init(&modbus_stream);
#endif

#endif

#if WIFI_ENABLE
    wifi_init();
#endif

#if BLUETOOTH_ENABLE
    bluetooth_init();
#endif

#if TRINAMIC_ENABLE
    trinamic_init();
#endif

#if KEYPAD_ENABLE
    keypad_init();
#endif

//    grbl_esp32_if_init();

    my_plugin_init();

    // no need to move version check before init - compiler will fail any mismatch for existing entries
    return hal.version == 8;
}

/* interrupt handlers */

// Main stepper driver
IRAM_ATTR static void stepper_driver_isr (void *arg)
{
    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[STEP_TIMER_INDEX].config.alarm_en = TIMER_ALARM_EN;

    hal.stepper.interrupt_callback();
}

  //GPIO intr process
IRAM_ATTR static void gpio_isr (void *arg)
{
  bool debounce = false;
  uint8_t grp = 0;
  uint32_t intr_status[2];
  intr_status[0] = READ_PERI_REG(GPIO_STATUS_REG);          // get interrupt status for GPIO0-31
  intr_status[1] = READ_PERI_REG(GPIO_STATUS1_REG);         // get interrupt status for GPIO32-39
  SET_PERI_REG_MASK(GPIO_STATUS_W1TC_REG, intr_status[0]);  // clear intr for gpio0-gpio31
  SET_PERI_REG_MASK(GPIO_STATUS1_W1TC_REG, intr_status[1]); // clear intr for gpio32-39

  uint32_t i = sizeof(inputpin) / sizeof(state_signal_t);
  do {
      i--;
      if(intr_status[inputpin[i].offset] & inputpin[i].mask) {
          inputpin[i].active = true;
          if(inputpin[i].debounce)
              debounce = true;
          else
              grp |= inputpin[i].group;
      }
  } while(i);

  if(debounce) {
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      xTimerStartFromISR(debounceTimer, &xHigherPriorityTaskWoken);
  }

  if(grp & INPUT_GROUP_LIMIT)
      hal.limits.interrupt_callback(limitsGetState());

  if(grp & INPUT_GROUP_CONTROL)
      hal.control.interrupt_callback(systemGetState());

#if MPG_MODE_ENABLE

  static bool mpg_mutex = false;

  if((grp & INPUT_GROUP_MPG) && !mpg_mutex) {
      mpg_mutex = true;
      modeChange();
     // hal.delay_ms(50, modeChange); // causes intermittent panic... stacked calls due to debounce?
      mpg_mutex = false;
  }
#endif

#if KEYPAD_ENABLE
  if(grp & INPUT_GROUP_KEYPAD)
      keypad_keyclick_handler(gpio_get_level(KEYPAD_STROBE_PIN));
#endif
}
