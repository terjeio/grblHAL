/*
  driver.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of GrblHAL

  Copyright (c) 2018-2020 Terje Io

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

#include "driver.h"
#include "esp32-hal-uart.h"

#if MPG_MODE_ENABLE
#endif

#include "serial.h"
#include "nvs.h"
#include "esp_log.h"

#if WIFI_ENABLE
#include "wifi.h"
#endif

#if WEBUI_ENABLE
#include "webui/response.h"
#endif

#if TELNET_ENABLE
#include "networking/TCPSTream.h"
#endif

#if WEBSOCKET_ENABLE
#include "networking/WsSTream.h"
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
#include "eeprom.h"
#endif

#ifdef I2C_PORT
#include "i2c.h"
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

#ifdef DRIVER_SETTINGS 
driver_settings_t driver_settings;
#endif

typedef struct {
    uint8_t pin;
    uint8_t group;
    uint32_t mask;
    uint8_t offset;
    bool invert;
    volatile bool active;
    volatile bool debounce;
} state_signal_t;

const io_stream_t serial_stream = {
    .type = StreamType_Serial,
    .read = uartRead,
    .write = uartWriteS,
    .write_all = uartWriteS,
    .get_rx_buffer_available = uartRXFree,
    .reset_read_buffer = uartFlush,
    .cancel_read_buffer = uartCancel,
    .suspend_read = uartSuspendInput,
    .enqueue_realtime_command = protocol_enqueue_realtime_command
};

const io_stream_t serial2_stream = {
    .type = StreamType_Serial,
    .read = uart2Read,
    .write = uart2WriteS,
    .write_all = uart2WriteS,
    .get_rx_buffer_available = uart2RXFree,
    .reset_read_buffer = uart2Flush,
    .cancel_read_buffer = uart2Cancel,
    .suspend_read = uart2SuspendInput,
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
    uartWriteS(data);
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
    .suspend_read = uartSuspendInput,
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
    .enqueue_realtime_command = protocol_enqueue_realtime_command,
#if M6_ENABLE
    .suspend_read = NULL // for now...
#else
    .suspend_read = NULL
#endif
};
#endif

#endif // WIFI_ENABLE

#if BLUETOOTH_ENABLE
void btStreamWriteS (const char *data)
{
    BTStreamWriteS(data);
    uartWriteS(data);
}

const io_stream_t bluetooth_stream = {
    .type = StreamType_Bluetooth,
    .read = BTStreamGetC,
    .write = BTStreamWriteS,
    .write_all = btStreamWriteS,
    .get_rx_buffer_available = BTStreamRXFree,
    .reset_read_buffer = BTStreamFlush,
    .cancel_read_buffer = BTStreamCancel,
    .suspend_read = uartSuspendInput,
    .enqueue_realtime_command = protocol_enqueue_realtime_command
};

#endif


#define INPUT_GROUP_CONTROL 1
#define INPUT_GROUP_PROBE   2
#define INPUT_GROUP_LIMIT   4
#define INPUT_GROUP_MPG     5
#define INPUT_GROUP_KEYPAD  8

#define INPUT_RESET         0
#define INPUT_FEED_HOLD     1
#define INPUT_CYCLE_START   2
#define INPUT_SAFETY_DOOR   3
#define INPUT_PROBE         4
#define INPUT_LIMIT_X       5
#define INPUT_LIMIT_Y       6
#define INPUT_LIMIT_Z       7
#define INPUT_KEYPAD        8


state_signal_t inputpin[] = {
    { .pin = RESET_PIN, .group = INPUT_GROUP_CONTROL },
    { .pin = FEED_HOLD_PIN, .group = INPUT_GROUP_CONTROL },
    { .pin = CYCLE_START_PIN, .group = INPUT_GROUP_CONTROL },
    { .pin = SAFETY_DOOR_PIN, .group = INPUT_GROUP_CONTROL },
    { .pin = PROBE_PIN, .group = INPUT_GROUP_PROBE },
    { .pin = X_LIMIT_PIN, .group = INPUT_GROUP_LIMIT },
    { .pin = Y_LIMIT_PIN, .group = INPUT_GROUP_LIMIT },
    { .pin = Z_LIMIT_PIN, .group = INPUT_GROUP_LIMIT }
#if KEYPAD_ENABLE
  , { .pin = KEYPAD_STROBE_PIN, .group = INPUT_GROUP_KEYPAD }
#endif
};

static volatile uint32_t ms_count = 1; // NOTE: initial value 1 is for "resetting" systick timer
static uint32_t pwm_max_value;
static bool pwmEnabled = false, IOInitDone = false;
static spindle_pwm_t spindle_pwm;

// Inverts the probe pin state depending on user settings and probing cycle mode.
static uint8_t probe_invert;

#if IOEXPAND_ENABLE
static ioexpand_t iopins = {0};
#endif

static void spindle_set_speed (uint_fast16_t pwm_value);

static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

static ledc_timer_config_t ledTimerConfig = {
    .speed_mode = LEDC_HIGH_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_10_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = (uint32_t)DEFAULT_SPINDLE_PWM_FREQ
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
// Interrupt handler prototypes
static void stepper_driver_isr (void *arg);
static void gpio_isr (void *arg);
static void mode_isr_handler(void* arg);
static void gpio_task_mode(void* arg);

static TimerHandle_t xDelayTimer = NULL, debounceTimer = NULL;
static TaskHandle_t xStepperTask = NULL;

void selectStream (stream_type_t stream)
{
    static stream_type_t active_stream = StreamType_Serial;

    switch(stream) {

#if BLUETOOTH_ENABLE
        case StreamType_Bluetooth:
            memcpy(&hal.stream, &bluetooth_stream, sizeof(io_stream_t));
            services.bluetooth = On;
            break;
#endif

#if TELNET_ENABLE
        case StreamType_Telnet:
            memcpy(&hal.stream, &telnet_stream, sizeof(io_stream_t));
            services.telnet = On;
            hal.stream.write_all("[MSG:TELNET STREAM ACTIVE]\r\n");
            break;
#endif

#if WEBSOCKET_ENABLE
        case StreamType_WebSocket:
            memcpy(&hal.stream, &websocket_stream, sizeof(io_stream_t));
            services.websocket = On;
            hal.stream.write_all("[MSG:WEBSOCKET STREAM ACTIVE]\r\n");
            break;
#endif

        case StreamType_Serial:
            memcpy(&hal.stream, &serial_stream, sizeof(io_stream_t));
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

    rmtItem[0].duration0 = settings->steppers.pulse_delay_microseconds ? 4 * settings->steppers.pulse_delay_microseconds : 1;
    rmtItem[0].duration1 = 4 * settings->steppers.pulse_microseconds;
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


// Set stepper pulse output pins
inline IRAM_ATTR static void set_step_outputs (axes_signals_t step_outbits)
{
#ifdef xDEBUGOUT
    if(step_outbits.value)
        hal.debug_out(true);
#endif
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
/*
    step_outbits.value ^= settings.steppers.step_invert.mask;
    gpio_set_level(X_STEP_PIN, step_outbits.x);
    gpio_set_level(Y_STEP_PIN, step_outbits.y);
    gpio_set_level(Z_STEP_PIN, step_outbits.z);
*/
}

// Set stepper direction output pins
// NOTE: see note for set_step_outputs()
inline IRAM_ATTR static void set_dir_outputs (axes_signals_t dir_outbits)
{
    dir_outbits.value ^= settings.steppers.dir_invert.mask;
    gpio_set_level(X_DIRECTION_PIN, dir_outbits.x);
    gpio_set_level(Y_DIRECTION_PIN, dir_outbits.y);
    gpio_set_level(Z_DIRECTION_PIN, dir_outbits.z);
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
#else
    gpio_set_level(STEPPERS_DISABLE_PIN, enable.x);
#endif
}

// Starts stepper driver ISR timer and forces a stepper driver interrupt callback
static void stepperWakeUp (void)
{
#ifdef LASER_PPI
    laser.next_pulse = 0;
#endif
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
        set_step_outputs((axes_signals_t){0});
        set_dir_outputs((axes_signals_t){0});
    }
}

// Sets up stepper driver interrupt timeout
IRAM_ATTR static void stepperCyclesPerTick (uint32_t cycles_per_tick)
{
#ifdef xDEBUGOUT
    if(cycles_per_tick > 1500000UL)
        hal.debug_out(true);
#endif
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
    if(stepper->new_block) {
        stepper->new_block = false;
        set_dir_outputs(stepper->dir_outbits);
    }

    if(stepper->step_outbits.value) {
        set_step_outputs(stepper->step_outbits);
#ifdef DEBUGOUT
        hal.debug_out(false);
#endif
    }
}

// Enable/disable limit pins interrupt
static void limitsEnable (bool on, bool homing)
{
    uint32_t i;
    for(i = INPUT_LIMIT_X; i <= INPUT_LIMIT_Z; i++) {
        gpio_set_intr_type(inputpin[i].pin, on ? (inputpin[i].invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE) : GPIO_INTR_DISABLE);
    }
#if TRINAMIC_ENABLE
    trinamic_homing(homing);
#endif
}

// Returns limit state as an axes_signals_t variable.
// Each bitfield bit indicates an axis limit, where triggered is 1 and not triggered is 0.
inline IRAM_ATTR static axes_signals_t limitsGetState()
{
    axes_signals_t signals;

    signals.x = gpio_get_level(X_LIMIT_PIN);
    signals.y = gpio_get_level(Y_LIMIT_PIN);
    signals.z = gpio_get_level(Z_LIMIT_PIN);

    if (settings.limits.invert.value)
        signals.value ^= settings.limits.invert.value;

    return signals;
}

// Returns system state as a control_signals_t variable.
// Each bitfield bit indicates a control signal, where triggered is 1 and not triggered is 0.
inline IRAM_ATTR static control_signals_t systemGetState (void)
{
    control_signals_t signals = {0};

    signals.reset = gpio_get_level(RESET_PIN);
    signals.feed_hold = gpio_get_level(FEED_HOLD_PIN);
    signals.cycle_start = gpio_get_level(CYCLE_START_PIN);
    signals.safety_door_ajar = gpio_get_level(SAFETY_DOOR_PIN);

    if(settings.control_invert.value)
        signals.value ^= settings.control_invert.value;

    return signals;
}

// Sets up the probe pin invert mask to
// appropriately set the pin logic according to setting for normal-high/normal-low operation
// and the probing cycle modes for toward-workpiece/away-from-workpiece.
static void probeConfigure(bool is_probe_away)
{
  probe_invert = settings.flags.invert_probe_pin ? 0 : 1;

  if(is_probe_away)
      probe_invert ^= 1;
#if PROBE_ISR
    gpio_set_intr_type(inputpin[INPUT_PROBE].pin, probe_invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE);
    inputpin[INPUT_PROBE].active = false;
#endif
}

// Returns the probe pin state. Triggered = true.
bool probeGetState (void)
{
#if PROBE_ISR
    // TODO: verify!
    inputpin[INPUT_PROBE].active = inputpin[INPUT_PROBE].active || ((uint8_t)gpio_get_level(PROBE_PIN) ^ probe_invert);
    return inputpin[INPUT_PROBE].active;
#else
    return (uint8_t)gpio_get_level(PROBE_PIN) ^ probe_invert;
#endif
}

// Static spindle (off, on cw & on ccw)
inline static void spindle_off ()
{
#if IOEXPAND_ENABLE
    iopins.spindle_on = settings.spindle.invert.on ? On : Off;
    ioexpand_out(iopins);
#else
    gpio_set_level(SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? 1 : 0);
#endif
}

inline static void spindle_on ()
{
#if IOEXPAND_ENABLE
    iopins.spindle_on = settings.spindle.invert.on ? Off : On;
    ioexpand_out(iopins);
#else
   gpio_set_level(SPINDLE_ENABLE_PIN, settings.spindle.invert.on ? 0 : 1);
#endif
}


inline static void spindle_dir (bool ccw)
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
IRAM_ATTR static void spindle_set_speed (uint_fast16_t pwm_value)
{
    if (pwm_value == spindle_pwm.off_value) {
        if(settings.spindle.disable_with_zero_speed)
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

#if MPG_MODE_ENABLE
SemaphoreHandle_t xSemaphore = NULL;
#define ESP_INTR_FLAG_DEFAULT 0

static void modeSelect (bool mpg_mode)
{

  hal.stream.write_all("ModeSelect togglated\n");
  if(mpg_mode){
    hal.stream.write_all("\tmode true\n");
    // TODO check to be sure idle and can switch stream
    memcpy(&hal.stream, &serial2_stream, sizeof(io_stream_t));
    hal.stream.write_all("\thello mpg\n");
    uart2WriteS("to you friend");

  }
  else{
    hal.stream.write_all("\tmode false\n");
    // TODO ensure idle to switch stream
    memcpy(&hal.stream, &serial_stream, sizeof(io_stream_t));
  }
}

static void modeChange (void)
{
    //modeSelect(gpio_get_level(MPG_ENABLE_PIN));
    bool mpgmode = gpio_get_level(MPG_ENABLE_PIN);
    serialSelect(mpgmode);

    if(mpgmode){
      uart2WriteS("uart1, nice!");
    }else{

      uartWriteS("sup uart0");
    }
    modeSelect(mpgmode);
}

static void modeEnable (void)
{
    //
#if KEYPAD_ENABLE
    //KEYPAD_PORT->IE |= KEYPAD_IRQ_BIT;
#endif
}

#endif // end MPG_MODE_ENABLE

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

// Start/stop coolant (and mist if enabled)
static void coolantSetState (coolant_state_t mode)
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

static void showMessage (const char *msg)
{
    hal.stream.write("[MSG:");
    hal.stream.write(msg);
    hal.stream.write("]\r\n");
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

//    printf("Debounce %d %d\n", grp, systemGetState().value);

      if(grp & INPUT_GROUP_LIMIT)
            hal.limit_interrupt_callback(limitsGetState());

      if(grp & INPUT_GROUP_CONTROL)
          hal.control_interrupt_callback(systemGetState());
}


// Configures perhipherals when settings are initialized or changed
static void settings_changed (settings_t *settings)
{
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
        spindle_pwm.max_value = (uint32_t)(pwm_max_value * settings->spindle.pwm_max_value / 100.0f);
        spindle_pwm.pwm_gradient = (float)(spindle_pwm.max_value - spindle_pwm.min_value) / (settings->spindle.rpm_max - settings->spindle.rpm_min);
        spindle_pwm.always_on = settings->spindle.pwm_off_value != 0.0f;

        ledc_set_freq(ledTimerConfig.speed_mode, ledTimerConfig.timer_num, ledTimerConfig.freq_hz);
    }

    if(IOInitDone) {

      #if TRINAMIC_ENABLE
        trinamic_configure();
      #endif

        hal.spindle_set_state = hal.driver_cap.variable_spindle ? spindleSetStateVariable : spindleSetState;

#if WIFI_ENABLE

        static bool wifi_ok = false;

        if(!wifi_ok)
            wifi_ok = wifi_init(&driver_settings.wifi);

        // TODO: start/stop services...
#endif

#if BLUETOOTH_ENABLE
        static bool bluetooth_ok = false;
        if(!bluetooth_ok && driver_settings.bluetooth.device_name[0] != '\0')
            bluetooth_ok = bluetooth_init(&driver_settings.bluetooth);
        // else report error?
#endif

        stepperEnable(settings->steppers.deenergize);

        /*********************
         * Step pulse config *
         *********************/

        initRMT(settings);

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

            switch(--i) {

                case INPUT_RESET:
                    pullup = !settings->control_disable_pullup.reset;
                    inputpin[i].invert = control_fei.reset;
                    config.intr_type = inputpin[i].invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
                    break;

                case INPUT_FEED_HOLD:
                    pullup = !settings->control_disable_pullup.feed_hold;
                    inputpin[i].invert = control_fei.feed_hold;
                    config.intr_type = inputpin[i].invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
                    break;

                case INPUT_CYCLE_START:
                    pullup = !settings->control_disable_pullup.cycle_start;
                    inputpin[i].invert = control_fei.cycle_start;
                    config.intr_type = inputpin[i].invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
                    break;

                case INPUT_SAFETY_DOOR:
                    pullup = !settings->control_disable_pullup.safety_door_ajar;
                    inputpin[i].invert = control_fei.safety_door_ajar;
                    config.intr_type = inputpin[i].invert ? GPIO_INTR_NEGEDGE : GPIO_INTR_POSEDGE;
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

#if KEYPAD_ENABLE
                case INPUT_KEYPAD:
                    pullup = true;
                    inputpin[i].invert = false;
                    config.intr_type = GPIO_INTR_ANYEDGE;
                    break;
#endif
            }

            if(inputpin[i].pin != 0xFF) {
                pullup = pullup && inputpin[i].pin < 34;

                gpio_intr_disable(inputpin[i].pin);

                config.pin_bit_mask = 1ULL << inputpin[i].pin;
                config.mode = GPIO_MODE_INPUT;
                config.pull_up_en = pullup ? GPIO_PULLUP_ENABLE : GPIO_PULLUP_DISABLE;
                config.pull_down_en = pullup ? GPIO_PULLDOWN_DISABLE : GPIO_PULLDOWN_ENABLE;

                inputpin[i].offset = config.pin_bit_mask > (1ULL << 31) ? 1 : 0;
                inputpin[i].mask = inputpin[i].offset == 0 ? (uint32_t)config.pin_bit_mask : (uint32_t)(config.pin_bit_mask >> 32);

    //            printf("IN %d - %d - %d : %x\n", inputpin[i].pin,  inputpin[i].offset, inputpin[i].mask, inputpin[i].invert);

                gpio_config(&config);

                inputpin[i].active   = gpio_get_level(inputpin[i].pin) == (inputpin[i].invert ? 0 : 1);
                inputpin[i].debounce = hal.driver_cap.software_debounce && !(inputpin[i].group == INPUT_GROUP_PROBE || inputpin[i].group == INPUT_GROUP_KEYPAD);
            }
        } while(i);
    }
}

void vStepperTask (void *pvParameters)
{
    while(true) {
        vTaskSuspend(NULL);
        hal.stepper_interrupt_callback();
    }
}

#if WIFI_ENABLE
static void reportIP (void)
{
    hal.stream.write("[IP:");
    hal.stream.write(wifi_get_ip());
    hal.stream.write("]\r\n");
}
#endif

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
    /********************************************************
     * Read driver specific setting from persistent storage *
     ********************************************************/
     
#ifdef DRIVER_SETTINGS
    if(hal.eeprom.type != EEPROM_None) {
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

    xTaskCreatePinnedToCore(vStepperTask, "Stepper", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES, &xStepperTask, 1);

    /********************
     *  Output signals  *
     ********************/

    uint32_t channel;
    for(channel = 0; channel < N_AXIS; channel++)
        rmt_set_source_clk(channel, RMT_BASECLK_APB);

    gpio_config_t gpioConfig = {
#if IOEXPAND_ENABLE
        .pin_bit_mask = DIRECTION_MASK,
#else
  #ifdef COOLANT_MASK
        .pin_bit_mask = DIRECTION_MASK|STEPPERS_DISABLE_MASK|SPINDLE_MASK|COOLANT_MASK,
  #else
        .pin_bit_mask = DIRECTION_MASK|STEPPERS_DISABLE_MASK|SPINDLE_MASK,
  #endif
#endif
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };

    gpio_config(&gpioConfig);

   /****************************
    *  Software debounce init  *
    ****************************/

    if(hal.driver_cap.software_debounce)
        debounceTimer = xTimerCreate("msDelay", pdMS_TO_TICKS(32), pdFALSE, NULL, debounceTimerCallback);

    /******************************************
     *  Control, limit & probe pins dir init  *
     ******************************************/

    gpio_isr_register(gpio_isr, NULL, (int)ESP_INTR_FLAG_IRAM, NULL);

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

#if KEYPAD_ENABLE
//    keypad_init();
#endif

#if TRINAMIC_ENABLE
    trinamic_init();
#endif

#if WEBUI_ENABLE
    webui_init();
#endif

  // Set defaults

    IOInitDone = settings->version == 15;

    settings_changed(settings);

    hal.stepper_go_idle(true);
    hal.spindle_set_state((spindle_state_t){0}, 0.0f);
    hal.coolant_set_state((coolant_state_t){0});

    return IOInitDone;
}

#ifdef DRIVER_SETTINGS

static status_code_t driver_setting (uint_fast16_t param, float value, char *svalue)
{
    status_code_t status = Status_Unhandled;

#if BLUETOOTH_ENABLE
    status = bluetooth_setting(param, value, svalue);
#endif

#if WIFI_ENABLE
    if(status == Status_Unhandled)
        status = wifi_setting(param, value, svalue);
#endif

#if KEYPAD_ENABLE
    if(status == Status_Unhandled)
        status = keypad_setting(param, value, svalue);
#endif

#if TRINAMIC_ENABLE
    if(status == Status_Unhandled)
        status = trinamic_setting(param, value, svalue);
#endif

    if(status == Status_OK)
        hal.eeprom.memcpy_to_with_checksum(hal.eeprom.driver_area.address, (uint8_t *)&driver_settings, sizeof(driver_settings));

    return status;
}

static void driver_settings_report (setting_type_t setting)
{
#if KEYPAD_ENABLE
    keypad_settings_report(setting);
#endif

#if BLUETOOTH_ENABLE
    bluetooth_settings_report();
#endif

#if WIFI_ENABLE
    wifi_settings_report(setting);
#endif

#if TRINAMIC_ENABLE
    trinamic_settings_report(setting);
#endif
}

static void driver_settings_restore (void)
{
    memset(&driver_settings, 0, sizeof(driver_settings_t));

#if WIFI_ENABLE
    wifi_settings_restore();
#endif

#if BLUETOOTH_ENABLE
    bluetooth_settings_restore();
#endif

#if KEYPAD_ENABLE
    keypad_settings_restore();
#endif

#if TRINAMIC_ENABLE
        trinamic_settings_restore();
#endif

    hal.eeprom.memcpy_to_with_checksum(hal.eeprom.driver_area.address, (uint8_t *)&driver_settings, sizeof(driver_settings));
}

#endif

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{
    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

    uartInit();
  

#if MPG_MODE_ENABLE

    /***************************
    *   Mode ISR
    ***************************/


    uart2Init();
    hal.driver_cap.mpg_mode = On;
    // Drive MPG mode input pin low until setup complete
    // TODO 
    
    gpio_config_t io_conf;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = (gpio_pullup_t)1;
    io_conf.pin_bit_mask = 1ULL << MPG_ENABLE_PIN;
    gpio_config(&io_conf);
    gpio_set_intr_type(MPG_ENABLE_PIN, GPIO_INTR_ANYEDGE);
    xSemaphore = xSemaphoreCreateBinary();
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    gpio_isr_handler_add(MPG_ENABLE_PIN, mode_isr_handler, NULL);
    xTaskCreate(gpio_task_mode, "gpio_task_mode", 2048, NULL, 10, NULL);
#endif

#ifdef I2C_PORT
    I2CInit();
#endif

    hal.info = "ESP32";
    hal.driver_setup = driver_setup;
    hal.f_step_timer = rtc_clk_apb_freq_get() / STEPPER_DRIVER_PRESCALER; // 20 MHz
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

#if PROBE_ENABLE
    hal.probe_get_state = probeGetState;
    hal.probe_configure_invert_mask = probeConfigure;
#endif

    hal.spindle_set_state = spindleSetState;
    hal.spindle_get_state = spindleGetState;
#ifdef SPINDLE_PWM_DIRECT
    hal.spindle_get_pwm = spindleGetPWM;
    hal.spindle_update_pwm = spindle_set_speed;
#else
    hal.spindle_update_rpm = spindleUpdateRPM; // NOTE: fails in laser mode as ESP32 does not handle FPU access in ISRs!
#endif

    hal.system_control_get_state = systemGetState;

    selectStream(StreamType_Serial);

#if EEPROM_ENABLE
    hal.eeprom.type = EEPROM_Physical;
    hal.eeprom.get_byte = eepromGetByte;
    hal.eeprom.put_byte = eepromPutByte;
    hal.eeprom.memcpy_to_with_checksum = eepromWriteBlockWithChecksum;
    hal.eeprom.memcpy_from_with_checksum = eepromReadBlockWithChecksum;
#else
    if(nvsInit()) {
        hal.eeprom.type = EEPROM_Emulated;
        hal.eeprom.memcpy_from_flash = nvsRead;
        hal.eeprom.memcpy_to_flash = nvsWrite;
    } else
        hal.eeprom.type = EEPROM_None;
#endif

#ifdef DRIVER_SETTINGS

    if(hal.eeprom.type != EEPROM_None) {
        hal.eeprom.driver_area.address = GRBL_EEPROM_SIZE;
        hal.eeprom.driver_area.size = sizeof(driver_settings); // Add assert?
        hal.eeprom.size = GRBL_EEPROM_SIZE + sizeof(driver_settings) + 1;

        hal.driver_setting = driver_setting;
        hal.driver_settings_restore = driver_settings_restore;
        hal.driver_settings_report = driver_settings_report;
  #if TRINAMIC_ENABLE
        hal.driver_axis_settings_report = trinamic_axis_settings_report;
  #endif
    }
#endif

    hal.set_bits_atomic = bitsSetAtomic;
    hal.clear_bits_atomic = bitsClearAtomic;
    hal.set_value_atomic = valueSetAtomic;

    hal.show_message = showMessage;

#ifdef DEBUGOUT
    hal.debug_out = debug_out;
#endif

#if KEYPAD_ENABLE
    hal.execute_realtime = keypad_process_keypress;
#endif

#if TRINAMIC_ENABLE
    hal.user_mcode_check = trinamic_MCodeCheck;
    hal.user_mcode_validate = trinamic_MCodeValidate;
    hal.user_mcode_execute = trinamic_MCodeExecute;
    hal.driver_rt_report = trinamic_RTReport;
#endif

#if WIFI_ENABLE
    hal.report_options = reportIP;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

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
    hal.driver_cap.software_debounce = On;
    hal.driver_cap.step_pulse_delay = On;
    hal.driver_cap.amass_level = 3;
    hal.driver_cap.control_pull_up = On;
    hal.driver_cap.limits_pull_up = On;
    hal.driver_cap.probe_pull_up = On;
#if SDCARD_ENABLE
    hal.driver_cap.sd_card = On;
#endif
#if BLUETOOTH_ENABLE
    hal.driver_cap.bluetooth = On;
#endif
#if WIFI_ENABLE
    hal.driver_cap.wifi = On;
#endif

   // no need to move version check before init - compiler will fail any mismatch for existing entries
    return hal.version == 6;
}

/* interrupt handlers */

// Main stepper driver
IRAM_ATTR static void stepper_driver_isr (void *arg)
{
//  const int timer_idx = (int)arg;  // get the timer index

    TIMERG0.int_clr_timers.t0 = 1;
    TIMERG0.hw_timer[STEP_TIMER_INDEX].config.alarm_en = TIMER_ALARM_EN;

    /* Resume the suspended task. */

    /* A context switch should now be performed so the ISR returns directly to
    the resumed task. This is because the resumed task had a priority that
    was equal to or higher than the task that is currently in the Running state.
    NOTE: The syntax required to perform a context switch from an ISR varies
    from port to port and from compiler to compiler. Check the
    documentation and examples for the port being used to find the syntax required by your
    application. It is likely that this if() statement can be replaced by a
    single call to portYIELD_FROM_ISR() [or portEND_SWITCHING_ISR()]
    using xYieldRequired as the macro parameter: portYIELD_FROM_ISR( xYieldRequired );*/

    hal.stepper_interrupt_callback();

//    if(xTaskResumeFromISR(xStepperTask) == pdTRUE)
//         portYIELD_FROM_ISR();
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
      hal.limit_interrupt_callback(limitsGetState());

  if(grp & INPUT_GROUP_CONTROL)
      hal.control_interrupt_callback(systemGetState());

#if KEYPAD_ENABLE
  if(grp & INPUT_GROUP_KEYPAD)
      keypad_keyclick_handler(gpio_get_level(inputpin[INPUT_KEYPAD].pin));
#endif
}


// Mode ISR

#if MPG_MODE_ENABLE

static void gpio_task_mode(void* arg)
{
    for(;;) {
        if(xSemaphoreTake(xSemaphore,portMAX_DELAY) == pdTRUE) {
          // debounce a bit
          vTaskDelay(200);
          hal.stream.write_all("\t\tmode change\n");
          modeChange();
        }
    }
}

void IRAM_ATTR mode_isr_handler(void* arg) {
  if(xSemaphore == NULL){
    //hal.stream.write_all("ERROR null sem\n");
  }  
  xSemaphoreGive( xSemaphore );
}

#endif

