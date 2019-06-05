/*
  keypad.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Texas Instruments Tiva C (TM4C123GH6PM) ARM processor

  Part of Grbl

  Copyright (c) 2017-2019 Terje Io

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

#include "keypad.h"

#define KEYBUF_SIZE 16
#define i2cIsBusy (i2cBusy || I2CMasterBusy(I2C1_BASE))

typedef struct {
    float fast_speed;
    float slow_speed;
    float step_speed;
    float fast_distance;
    float slow_distance;
    float step_distance;
} jog_config_t;

static bool jogging = false, keyreleased = true;
static volatile bool i2cBusy = false;
static char keybuf_buf[16];
static jogmode_t jogMode = JogMode_Fast;
static jog_config_t jog_config;
static volatile uint32_t keybuf_head = 0, keybuf_tail = 0;

static void I2C_interrupt_handler (void);
static void keyclick_int_handler (void);

static void claim_eeprom (void)
{
    if(hal.eeprom.driver_area.size == 0) {
        assert(EEPROM_ADDR_TOOL_TABLE - (sizeof(jog_config_t) + 2) > EEPROM_ADDR_GLOBAL + sizeof(settings_t) + 1);

        hal.eeprom.driver_area.address = EEPROM_ADDR_TOOL_TABLE - (sizeof(jog_config_t) + 2);
        hal.eeprom.driver_area.size = sizeof(jog_config_t);
    }
}

// Read selected coordinate data from persistent storage.
bool keypad_read_settings (void)
{
    return hal.eeprom.type != EEPROM_None && hal.eeprom.memcpy_from_with_checksum((uint8_t *)&jog_config, hal.eeprom.driver_area.address, hal.eeprom.driver_area.size);
}

// Read jog configuration data from persistent storage.
void keypad_write_settings (void)
{
    if (hal.eeprom.type != EEPROM_None) {
        claim_eeprom();
        hal.eeprom.memcpy_to_with_checksum(hal.eeprom.driver_area.address, (uint8_t *)&jog_config, hal.eeprom.driver_area.size);
    }
}

bool driver_setting (uint_fast16_t setting, float value, char *svalue)
{
    bool ok = false;

    switch(setting) {

        case Setting_JogStepSpeed:
            jog_config.step_speed = value;
            ok = true;
            break;

        case Setting_JogSlowSpeed:
            jog_config.slow_speed = value;
            ok = true;
            break;

        case Setting_JogFastSpeed:
            jog_config.fast_speed = value;
            ok = true;
            break;

        case Setting_JogStepDistance:
            jog_config.step_distance = value;
            ok = true;
            break;

        case Setting_JogSlowDistance:
            jog_config.slow_distance = value;
            ok = true;
            break;

        case Setting_JogFastDistance:
            jog_config.fast_distance = value;
            ok = true;
            break;
    }

    if(ok)
        keypad_write_settings();

    return ok;
}

void driver_settings_restore (uint8_t restore_flag)
{
    if(restore_flag & SETTINGS_RESTORE_DRIVER_PARAMETERS) {
        jog_config.step_speed    = 100.0f;
        jog_config.slow_speed    = 600.0f;
        jog_config.fast_speed    = 3000.0f;
        jog_config.step_distance = 0.25f;
        jog_config.slow_distance = 500.0f;
        jog_config.fast_distance = 3000.0f;

        keypad_write_settings();
    }
}

void driver_settings_report (bool axis_settings, axis_setting_type_t setting_type, uint8_t axis_idx)
{
    if(!axis_settings) {
        report_float_setting(Setting_JogStepSpeed, jog_config.step_speed, 0);
        report_float_setting(Setting_JogSlowSpeed, jog_config.slow_speed, 0);
        report_float_setting(Setting_JogFastSpeed, jog_config.fast_speed, 0);
        report_float_setting(Setting_JogStepDistance, jog_config.step_distance, N_DECIMAL_SETTINGVALUE);
        report_float_setting(Setting_JogSlowDistance, jog_config.slow_distance, N_DECIMAL_SETTINGVALUE);
        report_float_setting(Setting_JogFastDistance, jog_config.fast_distance, N_DECIMAL_SETTINGVALUE);
    }
}

void keypad_setup (void)
{
    claim_eeprom();

    if(!keypad_read_settings())
        driver_settings_restore(SETTINGS_RESTORE_DRIVER_PARAMETERS);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C1);
	SysCtlPeripheralReset(SYSCTL_PERIPH_I2C1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);

	GPIOPinConfigure(GPIO_PA6_I2C1SCL);
	GPIOPinConfigure(GPIO_PA7_I2C1SDA);
	GPIOPadConfigSet(GPIO_PORTA_BASE, GPIO_PIN_7, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_OD);

	GPIOPinTypeI2CSCL(GPIO_PORTA_BASE, GPIO_PIN_6);
	GPIOPinTypeI2C(GPIO_PORTA_BASE, GPIO_PIN_7);

	I2CMasterInitExpClk(I2C1_BASE, SysCtlClockGet(), false);
	I2CIntRegister(I2C1_BASE, I2C_interrupt_handler);

	GPIOPinTypeGPIOInput(KEYINTR_PORT, KEYINTR_PIN);
	GPIOPadConfigSet(KEYINTR_PORT, KEYINTR_PIN, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU); // -> WPU

	GPIOIntRegister(KEYINTR_PORT, keyclick_int_handler);
	GPIOIntTypeSet(KEYINTR_PORT, KEYINTR_PIN, GPIO_BOTH_EDGES);
	GPIOIntEnable(KEYINTR_PORT, KEYINTR_PIN);
}

// get single byte - via interrupt
static void I2C_GetKeycode (void)
{
   if(!i2cIsBusy) { // ignore if busy
   	i2cBusy = true;
       IntPendClear(INT_I2C1);
       I2CMasterIntClear(I2C1_BASE);
       I2CMasterSlaveAddrSet(I2C1_BASE, KEYPAD_I2CADDR, true);
       I2CMasterControl(I2C1_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);
       I2CMasterIntEnable(I2C1_BASE);
   }
}

static void enqueue_keycode (char cmd)
{
    uint32_t bptr = (keybuf_head + 1) & (KEYBUF_SIZE - 1);    // Get next head pointer

    if(bptr != keybuf_tail) {                       // If not buffer full
        keybuf_buf[keybuf_head] = cmd;              // add data to buffer
        keybuf_head = bptr;                         // and update pointer
    }
}

// Returns 0 if no keycode enqueued
static char keypad_get_keycode (void)
{
    uint32_t data = 0, bptr = keybuf_tail;

    if(bptr != keybuf_head) {
        data = keybuf_buf[bptr++];               // Get next character, increment tmp pointer
        keybuf_tail = bptr & (KEYBUF_SIZE - 1);  // and update pointer
    }

    return data;
}


// BE WARNED: this function may be dangerous to use...
static char *strrepl (char *str, int c, char *str3)
{
	char tmp[30];
	char *s = strrchr(str, c);

	while(s) {
		strcpy(tmp, str3);
		strcat(tmp, s + 1);
		strcpy(s, tmp);
		s = strrchr(str, c);
	}

	return str;
}

void process_keypress (uint_fast16_t state)
{
	bool addedGcode, jogCommand = false;
	char command[30] = "", keycode = keypad_get_keycode();

    if(keycode)
      switch(keycode) {

        case 'M':                                   // Mist override
            enqueue_accessory_override(CMD_OVERRIDE_COOLANT_MIST_TOGGLE);
            break;

        case 'C':                                   // Coolant override
            enqueue_accessory_override(CMD_OVERRIDE_COOLANT_FLOOD_TOGGLE);
            break;

        case CMD_FEED_HOLD_LEGACY:                  // Feed hold
            hal.stream.enqueue_realtime_command(CMD_FEED_HOLD);
            break;

        case CMD_CYCLE_START_LEGACY:                // Cycle start
            hal.stream.enqueue_realtime_command(CMD_CYCLE_START);
            break;

        case '0':
        case '1':
        case '2':                                   // Set jog mode
            jogMode = (jogmode_t)(keycode - '0');
            break;

        case 'h':                                   // "toggle" jog mode
            jogMode = jogMode == JogMode_Step ? JogMode_Fast : (jogMode == JogMode_Fast ? JogMode_Slow : JogMode_Step);
            break;

        case 'H':                                   // Home axes
            strcpy(command, "$H");
            break;

        case JOG_XR:                                // Jog X
            strcpy(command, "$J=G91X?F");
            break;

        case JOG_XL:                                // Jog -X
            strcpy(command, "$J=G91X-?F");
            break;

        case JOG_YF:                                // Jog Y
            strcpy(command, "$J=G91Y?F");
            break;

        case JOG_YB:                                // Jog -Y
            strcpy(command, "$J=G91Y-?F");
            break;

        case JOG_ZU:                                // Jog Z
            strcpy(command, "$J=G91Z?F");
            break;

        case JOG_ZD:                                // Jog -Z
            strcpy(command, "$J=G91Z-?F");
            break;

        case JOG_XRYF:                              // Jog XY
            strcpy(command, "$J=G91X?Y?F");
            break;

        case JOG_XRYB:                              // Jog X-Y
            strcpy(command, "$J=G91X?Y-?F");
            break;

        case JOG_XLYF:                              // Jog -XY
            strcpy(command, "$J=G91X-?Y?F");
            break;

        case JOG_XLYB:                              // Jog -X-Y
            strcpy(command, "$J=G91X-?Y-?F");
            break;

        case JOG_XRZU:                              // Jog XZ
            strcpy(command, "$J=G91X?Z?F");
            break;

        case JOG_XRZD:                              // Jog X-Z
            strcpy(command, "$J=G91X?Z-?F");
            break;

        case JOG_XLZU:                              // Jog -XZ
            strcpy(command, "$J=G91X-?Z?F");
            break;

        case JOG_XLZD:                              // Jog -X-Z
            strcpy(command, "$J=G91X-?Z-?F");
            break;
    }

	if(command[0] != '\0') {

		// add distance and speed to jog commands
		if((jogCommand = (command[0] == '$' && command[1] == 'J')))
			switch(jogMode) {

			case JogMode_Slow:
				strrepl(command, '?', ftoa(jog_config.slow_distance, 0));
				strcat(command, ftoa(jog_config.slow_speed, 0));
				break;

			case JogMode_Step:
				strrepl(command, '?', ftoa(jog_config.step_distance, 3));
				strcat(command, ftoa(jog_config.step_speed, 0));
				break;

			default:
				strrepl(command, '?', ftoa(jog_config.fast_distance, 0));
				strcat(command, ftoa(jog_config.fast_speed, 0));
				break;

		}

		if(!(jogCommand && keyreleased)) { // key still pressed? - do not execute jog command if released!
			addedGcode = hal.protocol_enqueue_gcode((char *)command);
			jogging = jogging || (jogCommand && addedGcode);
		}
	}
}

static void driver_keyclick_handler (bool keydown)
{
	keyreleased = !keydown;

	if(keydown)
		I2C_GetKeycode();

	else if(jogging) {
		jogging = false;
		hal.stream.enqueue_realtime_command(CMD_JOG_CANCEL);
		keybuf_tail = keybuf_head = 0; // flush keycode buffer
	}
}

static void I2C_interrupt_handler (void)
{
	char keycode = I2CMasterIntStatusEx(I2C1_BASE, false);

	I2CMasterIntClear(I2C1_BASE);

	keycode = (uint8_t)I2CMasterDataGet(I2C1_BASE);

	I2CMasterIntDisable(I2C1_BASE);

	i2cBusy = false;

	if(GPIOIntStatus(KEYINTR_PORT, KEYINTR_PIN) == 0) // only add keycode when key is still pressed
		enqueue_keycode(keycode);
}

static void keyclick_int_handler (void)
{
	uint32_t iflags = GPIOIntStatus(KEYINTR_PORT, KEYINTR_PIN);

    GPIOIntClear(KEYINTR_PORT, iflags);

	if(iflags & KEYINTR_PIN)
		driver_keyclick_handler(GPIOPinRead(KEYINTR_PORT, KEYINTR_PIN) != 0);
}
