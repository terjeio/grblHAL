/*
  driver.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for ESP32

  Part of Grbl

  Copyright (c) 2018-2019 Terje Io

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

#include "GRBL/grbl.h"
#include "GRBL/report.h"

#include "driver.h"
#include "esp32-hal-uart.h"
#include "serial.h"
#include "nvs.h"
#include "esp_log.h"

#if WIFI_ENABLE
#include "wifi.h"
#include "TCPSTream.h"
#endif

#if BLUETOOTH_ENABLE
#include "bluetooth.h"
#endif

#if SDCARD_ENABLE
#include "sdcard.h"
#endif

#if KEYPAD_ENABLE
#include "keypad.h"
#endif

#if IOEXPAND_ENABLE
#include "ioexpand.h"
#endif

#if EEPROM_ENABLE
#include "eeprom.h"
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

driver_settings_t driver_settings;

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
    .type = StreamSetting_Serial,
    .read = uartRead,
    .write = uartWriteS,
    .write_all = uartWriteS,
    .get_rx_buffer_available = uartRXFree,
    .reset_read_buffer = uartFlush,
    .cancel_read_buffer = uartCancel,
    .suspend_read = uartSuspendInput,
    .enqueue_realtime_command = protocol_enqueue_realtime_command
};

#if WIFI_ENABLE

void wifiStreamWriteS (const char *data)
{
	TCPStreamWriteS(data);
	uartWriteS(data);
}

const io_stream_t wifi_stream = {
    .type = StreamSetting_WiFi,
    .read = TCPStreamGetC,
    .write = TCPStreamWriteS,
    .write_all = wifiStreamWriteS,
    .get_rx_buffer_available = TCPStreamRxFree,
    .reset_read_buffer = TCPStreamRxFlush,
    .cancel_read_buffer = TCPStreamRxCancel,
    .suspend_read = uartSuspendInput,
    .enqueue_realtime_command = protocol_enqueue_realtime_command
};

#endif
#if BLUETOOTH_ENABLE
void btStreamWriteS (const char *data)
{
	BTStreamWriteS(data);
	uartWriteS(data);
}

const io_stream_t bluetooth_stream = {
    .type = StreamSetting_Bluetooth,
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
#define INPUT_GROUP_KEYPAD  8

#define INPUT_RESET         0
#define INPUT_FEED_HOLD     1
#define INPUT_CYCLE_START   2
#define INPUT_SAFETY_DOOR   3
#define INPUT_PROBE         4
#define INPUT_LIMIT_X       5
#define INPUT_LIMIT_Y       6
#define INPUT_LIMIT_Z       7
#define INPUT_KEYPAD  		8

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
static bool pwmEnabled = false, IOInitDone = false;
static spindle_pwm_t spindle_pwm;

// Inverts the probe pin state depending on user settings and probing cycle mode.
static uint8_t probe_invert;

#if IOEXPAND_ENABLE
static ioexpand_t iopins = {0};
#endif

static uint_fast16_t spindle_set_speed (uint_fast16_t pwm_value);

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

static TimerHandle_t xDelayTimer = NULL, debounceTimer = NULL;
static TaskHandle_t xStepperTask = NULL;

QueueHandle_t i2cQueue = NULL;
SemaphoreHandle_t i2cBusy = NULL;


void selectStream (stream_setting_t stream)
{
	//
    switch(stream) {
#if BLUETOOTH_ENABLE
        case StreamSetting_Bluetooth:
            memcpy(&hal.stream, &bluetooth_stream, sizeof(io_stream_t));
            break;
#endif
#if WIFI_ENABLE
        case StreamSetting_WiFi:
            memcpy(&hal.stream, &wifi_stream, sizeof(io_stream_t));
        	hal.stream.write_all("[MSG:WIFI RUNNING]\r\n");
            break;
#endif
        case StreamSetting_Serial:
            memcpy(&hal.stream, &serial_stream, sizeof(io_stream_t));
            break;

        default:
        	break;
    }
}

void initRMT (settings_t *settings)
{
	rmt_item32_t rmtItem[2];

    rmt_config_t rmtConfig = {
        .rmt_mode = RMT_MODE_TX,
        .clk_div = 20,
        .mem_block_num = 2,
        .tx_config.loop_en = false,
		.tx_config.carrier_en = false,
		.tx_config.carrier_freq_hz = 0,
		.tx_config.carrier_duty_percent = 50,
		.tx_config.carrier_level = 0,
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
        rmt_fill_tx_items(rmtConfig.channel, &rmtItem[0], rmtConfig.mem_block_num, 0);
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

static void driver_delay_ms (uint32_t ms, void (*callback)(void))
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
#if IOEXPAND_ENABLE // TODO: read from expander?
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
//	timer_set_alarm_value(STEP_TIMER_GROUP, STEP_TIMER_INDEX, 5000ULL);
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
#else
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
static uint_fast16_t spindle_set_speed (uint_fast16_t pwm_value)
{
    if (pwm_value == spindle_pwm.off_value) {
        if(settings.spindle.disable_with_zero_speed)
            spindle_off();
#if PWM_RAMPED
        pwm_ramp.pwm_target = pwm_value;
        ledc_set_fade_step_and_start(ledConfig.speed_mode, ledConfig.channel, pwm_ramp.pwm_target, 1, 4, LEDC_FADE_NO_WAIT);
#else
        ledc_stop(ledConfig.speed_mode, ledConfig.channel, 0);
#endif
        pwmEnabled = false;
     } else {
#if PWM_RAMPED
    	 pwm_ramp.pwm_target = pwm_value;
    	 ledc_set_fade_step_and_start(ledConfig.speed_mode, ledConfig.channel, pwm_ramp.pwm_target, 1, 4, LEDC_FADE_NO_WAIT);
#else
    	 ledc_set_duty(ledConfig.speed_mode, ledConfig.channel, pwm_value);
    	 ledc_update_duty(ledConfig.speed_mode, ledConfig.channel);
#endif
        if(!pwmEnabled) {
            spindle_on();
            pwmEnabled = true;
        }
    }

    return pwm_value;
}

static void spindleUpdateRPM (float rpm)
{
    spindle_set_speed(spindle_compute_pwm_value(&spindle_pwm, rpm, false));
}

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

// Returns spindle state in a spindle_state_t variable
static spindle_state_t spindleGetState (void)
{
    spindle_state_t state = {0};

#if IOEXPAND_ENABLE // TODO: read from expander?
	state.on = iopins.spindle_on;
	state.ccw = hal.driver_cap.spindle_dir && iopins.spindle_dir;
#else
    state.on = gpio_get_level(SPINDLE_ENABLE_PIN) != 0;
    state.ccw = hal.driver_cap.spindle_dir && gpio_get_level(SPINDLE_DIRECTION_PIN) != 0;
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
    gpio_set_level(COOLANT_FLOOD_PIN, mode.flood ? 1 : 0);
    gpio_set_level(COOLANT_MIST_PIN, mode.mist ? 1 : 0);
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
    state.flood = gpio_get_level(COOLANT_FLOOD_PIN);
    state.mist  = gpio_get_level(COOLANT_MIST_PIN);
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

//	  printf("Debounce %d %d\n", grp, systemGetState().value);

	  if(grp & INPUT_GROUP_LIMIT)
		    hal.limit_interrupt_callback(limitsGetState());

	  if(grp & INPUT_GROUP_CONTROL)
		  hal.control_interrupt_callback(systemGetState());
}

#ifdef I2C_PORT

void I2CTask (void *queue)
{
	i2c_task_t task;

	while(xQueueReceive((QueueHandle_t)queue, &task, portMAX_DELAY) == pdPASS) {
#if KEYPAD_ENABLE
		if(task.action == 1) { // Read keypad character and add to input buffer
			if(i2cBusy != NULL && xSemaphoreTake(i2cBusy, 20 / portTICK_PERIOD_MS) == pdTRUE) {
				char keycode;
				i2c_cmd_handle_t cmd = i2c_cmd_link_create();
				i2c_master_start(cmd);
				i2c_master_write_byte(cmd, (KEYPAD_I2CADDR << 1) | I2C_MASTER_READ, true);
				i2c_master_read_byte(cmd, (uint8_t*)&keycode, I2C_MASTER_NACK);
				i2c_master_stop(cmd);
				if(i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS) == ESP_OK)
					keypad_enqueue_keycode(keycode);
				i2c_cmd_link_delete(cmd);
				xSemaphoreGive(i2cBusy);
			}
		}
#endif
	}
}

void i2c_init (void)
{
	static bool init_ok = false;

	if(!init_ok) {

		init_ok = true;

		i2c_config_t i2c_config = {
			.mode = I2C_MODE_MASTER,
			.sda_io_num = I2C_SDA,
			.scl_io_num = I2C_SCL,
			.sda_pullup_en = GPIO_PULLUP_DISABLE,
			.scl_pullup_en = GPIO_PULLUP_DISABLE,
			.master.clk_speed = I2C_CLOCK
		};

		i2c_param_config(I2C_PORT, &i2c_config);
		esp_err_t ret = i2c_driver_install(I2C_PORT, i2c_config.mode, 0, 0, 0);

		i2cQueue = xQueueCreate(5, sizeof(uint8_t));
		i2cBusy = xSemaphoreCreateBinary();

		TaskHandle_t I2CTaskHandle;

		xTaskCreatePinnedToCore(I2CTask, "I2C", 2048, (void *)i2cQueue, configMAX_PRIORITIES, &I2CTaskHandle, 1);

		xSemaphoreGive(i2cBusy);
	}
}

#endif

// Configures perhipherals when settings are initialized or changed
static void settings_changed (settings_t *settings)
{
    if((hal.driver_cap.variable_spindle = settings->spindle.rpm_max > settings->spindle.rpm_min)) {
		uint32_t pwm_max_value = (1UL << ledTimerConfig.duty_resolution) - 1;

		spindle_pwm.period = (uint32_t)(80000000UL / settings->spindle.pwm_freq);
		spindle_pwm.off_value = settings->spindle.invert.pwm ? spindle_pwm.period : 0;
		spindle_pwm.min_value = (uint32_t)(pwm_max_value * settings->spindle.pwm_min_value / 100.0f);
		spindle_pwm.max_value = (uint32_t)(pwm_max_value * settings->spindle.pwm_max_value / 100.0f);
		spindle_pwm.pwm_gradient = (float)(spindle_pwm.max_value - spindle_pwm.min_value) / (settings->spindle.rpm_max - settings->spindle.rpm_min);

		ledc_set_freq(ledTimerConfig.speed_mode, ledTimerConfig.timer_num, (uint32_t)settings->spindle.pwm_freq);
    }

    if(IOInitDone) {

        hal.spindle_set_state = hal.driver_cap.variable_spindle ? spindleSetStateVariable : spindleSetState;

    	// Validate stream setting, switch to serial if not supported
    	if((settings->stream == StreamSetting_WiFi && !hal.driver_cap.wifi) ||
        	(settings->stream == StreamSetting_Bluetooth && !hal.driver_cap.bluetooth))
    		settings->stream = StreamSetting_Serial;

    	// NOTE: some interfaces may defer actual stream switch until stack is operational
    	switch(settings->stream) {

    		case StreamSetting_Serial:
    			selectStream(StreamSetting_Serial);
    			break;

#if WIFI_ENABLE
    		case StreamSetting_WiFi:;
				static bool wifi_ok = false;
				if(!wifi_ok && driver_settings.wifi.ssid[0] != '\0')
					wifi_ok = wifi_init(&driver_settings.wifi);
					// else report error?
				break;
#endif

#if BLUETOOTH_ENABLE
    		case StreamSetting_Bluetooth:;
    			static bool bluetooth_ok = false;
    			if(!bluetooth_ok && driver_settings.bluetooth.device_name[0] != '\0')
					bluetooth_ok = bluetooth_init(&driver_settings.bluetooth);
				// else report error?
#endif
    		default:
    			break;
    	}

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

// Initializes MCU peripherals for Grbl use
static bool driver_setup (settings_t *settings)
{
	/********************************************************
	 * Read driver specific setting from persistent storage *
	 ********************************************************/

	if(hal.eeprom.type != EEPROM_None) {
		if(!hal.eeprom.memcpy_from_with_checksum((uint8_t *)&driver_settings, hal.eeprom.driver_area.address, sizeof(driver_settings)))
			hal.driver_settings_restore(SETTINGS_RESTORE_DRIVER_PARAMETERS);
	}

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
		.pin_bit_mask = DIRECTION_MASK|STEPPERS_DISABLE_MASK|SPINDLE_MASK|COOLANT_MASK,
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

#if SDCARD_ENABLE
    sdcard_init();
#endif

#if IOEXPAND_ENABLE
    ioexpand_init();
#endif

#if KEYPAD_ENABLE
    keypad_init();
#endif

  // Set defaults

    IOInitDone = settings->version == 14;

    settings_changed(settings);

    hal.stepper_go_idle(true);
    hal.spindle_set_state((spindle_state_t){0}, 0.0f);
    hal.coolant_set_state((coolant_state_t){0});

    return IOInitDone;
}

static bool driver_setting (uint_fast16_t param, float value, char *svalue)
{
	bool claimed = false;

	if(svalue) switch(param) {

		case 71:
			claimed = strlcpy(driver_settings.wifi.ssid, svalue, sizeof(driver_settings.wifi.ssid)) <= sizeof(driver_settings.wifi.ssid);
			break;

		case 72:
			claimed = strlcpy(driver_settings.wifi.password, svalue, sizeof(driver_settings.wifi.password)) <= sizeof(driver_settings.wifi.password);
			break;

		case 73:
			if((claimed = (value != NAN && value > 0.0f && value < 65536.0f)))
				driver_settings.wifi.port = (uint16_t)value;
			break;

		case 74:
			claimed = strlcpy(driver_settings.bluetooth.device_name, svalue, sizeof(driver_settings.bluetooth.device_name)) <= sizeof(driver_settings.bluetooth.device_name);
			break;

		case 75:
			claimed = strlcpy(driver_settings.bluetooth.service_name, svalue, sizeof(driver_settings.bluetooth.service_name)) <= sizeof(driver_settings.bluetooth.service_name);
			break;
	}

#if KEYPAD_ENABLE
	if(!claimed)
		claimed = keypad_setting(param, value, svalue);
#endif

	if(claimed)
		hal.eeprom.memcpy_to_with_checksum(hal.eeprom.driver_area.address, (uint8_t *)&driver_settings, sizeof(driver_settings));

    return claimed;
}

#if WIFI_ENABLE || BLUETOOTH_ENABLE
static void report_string_setting (uint8_t setting, char *value)
{
    hal.stream.write("$");
    hal.stream.write(uitoa((uint32_t)setting));
    hal.stream.write("=");
    hal.stream.write(value);
    hal.stream.write("\r\n");
}
#endif

static void driver_settings_report (bool axis_setting, axis_setting_type_t setting_type, uint8_t axis_idx)
{
    if(!axis_setting) {
#if KEYPAD_ENABLE
    	keypad_settings_report(axis_setting, setting_type, axis_idx);
#endif
#if WIFI_ENABLE
		report_string_setting(71, driver_settings.wifi.ssid);
		report_string_setting(72, driver_settings.wifi.password);
		report_uint_setting(73, driver_settings.wifi.port);
#endif
#if BLUETOOTH_ENABLE
		report_string_setting(74, driver_settings.bluetooth.device_name);
		report_string_setting(75, driver_settings.bluetooth.service_name);
#endif
    }
}

void driver_settings_restore (uint8_t restore_flag)
{
    if(restore_flag & SETTINGS_RESTORE_DRIVER_PARAMETERS) {
    	driver_settings.wifi.ssid[0] = '\0';
    	driver_settings.wifi.password[0] = '\0';
    	driver_settings.wifi.port = 23;
    	strcpy(driver_settings.bluetooth.device_name, "GRBL");
    	strcpy(driver_settings.bluetooth.service_name, "GRBL Serial Port");
#if KEYPAD_ENABLE
    	keypad_settings_restore(restore_flag);
#endif
    	hal.eeprom.memcpy_to_with_checksum(hal.eeprom.driver_area.address, (uint8_t *)&driver_settings, sizeof(driver_settings));
    }
}

// Initialize HAL pointers, setup serial comms and enable EEPROM
// NOTE: Grbl is not yet configured (from EEPROM data), driver_setup() will be called when done
bool driver_init (void)
{
    // Enable EEPROM and serial port here for Grbl to be able to configure itself and report any errors

    uartInit();

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
    hal.spindle_update_rpm = spindleUpdateRPM;

    hal.system_control_get_state = systemGetState;

    selectStream(StreamSetting_Serial);

#if EEPROM_ENABLE
    eeprom_init();
    hal.eeprom.type = EEPROM_Physical;
    hal.eeprom.get_byte = eepromGetByte;
    hal.eeprom.put_byte = eepromPutByte;
    hal.eeprom.memcpy_to_with_checksum = eepromWriteBlockWithChecksum;
    hal.eeprom.memcpy_from_with_checksum = eepromReadBlockWithChecksum;
	hal.eeprom.driver_area.address = GRBL_EEPROM_SIZE;
	hal.eeprom.driver_area.size = sizeof(driver_settings); // Add assert?
	hal.eeprom.size = GRBL_EEPROM_SIZE + sizeof(driver_settings) + 1;
#else
    if(nvsInit()) {
        hal.eeprom.type = EEPROM_Emulated;
        hal.eeprom.memcpy_from_flash = nvsRead;
        hal.eeprom.memcpy_to_flash = nvsWrite;
		hal.eeprom.driver_area.address = GRBL_EEPROM_SIZE;
		hal.eeprom.driver_area.size = sizeof(driver_settings); // Add assert?
		hal.eeprom.size = GRBL_EEPROM_SIZE + sizeof(driver_settings) + 1;
	} else
	    hal.eeprom.type = EEPROM_None;
#endif

    hal.driver_setting = driver_setting;
    hal.driver_settings_report = driver_settings_report;
    hal.driver_settings_restore = driver_settings_restore;

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

#if SDCARD_ENABLE
    hal.driver_reset = sdcard_reset;
#endif

  // driver capabilities, used for announcing and negotiating (with Grbl) driver functionality

    hal.driver_cap.spindle_dir = On;
    hal.driver_cap.variable_spindle = On;
#if PWM_RAMPED
    hal.driver_cap.spindle_at_speed = On;
#endif
    hal.driver_cap.mist_control = On;
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
    return hal.version == 5;
}

/* interrupt handlers */

// Main stepper driver
IRAM_ATTR static void stepper_driver_isr (void *arg)
{
//	const int timer_idx = (int)arg;  // get the timer index

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
  intr_status[0] = READ_PERI_REG(GPIO_STATUS_REG);			// get interrupt status for GPIO0-31
  intr_status[1] = READ_PERI_REG(GPIO_STATUS1_REG);			// get interrupt status for GPIO32-39
  SET_PERI_REG_MASK(GPIO_STATUS_W1TC_REG, intr_status[0]);  // clear intr for gpio0-gpio31
  SET_PERI_REG_MASK(GPIO_STATUS1_W1TC_REG, intr_status[1]);	// clear intr for gpio32-39

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
