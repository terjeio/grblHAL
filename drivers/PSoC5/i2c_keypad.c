/*
  keypad.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Driver code for Cypress PSoC 5 (CY8CKIT-059)

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
#include "i2c_keypad.h"
#include "grbl.h"

#define KEYBUF_SIZE 16

static bool jogging = false, keyreleased = true;
static uint8 keycode;
static char keybuf_buf[16];
static jogmode_t jogMode = JogMode_Fast;
static volatile uint32_t keybuf_head = 0, keybuf_tail = 0;

static void keyclick_int_handler (void);

void I2C_keypad_setup (void)
{
    I2C_Start();
    KeyPadInterrupt_StartEx(keyclick_int_handler); 
}

// get single byte - via interrupt
static void I2C_GetKeycode (void)
{
    if(I2C_MasterStatus() != I2C_MSTAT_XFER_INP) { // ignore if busy
        keycode = 0;
        I2C_MasterReadBuf(KEYPAD_I2CADDR, &keycode, 1, I2C_MODE_COMPLETE_XFER);
    }
}

static void enqueue_keycode (char cmd) {

    uint32_t bptr = (keybuf_head + 1) & (KEYBUF_SIZE - 1);    // Get next head pointer

    if(bptr != keybuf_tail) {                       // If not buffer full
        keybuf_buf[keybuf_head] = cmd;              // add data to buffer
        keybuf_head = bptr;                         // and update pointer
    }
}

// Returns 0 if no keycode enqueued
static char keypad_get_keycode (void) {

    uint32_t data = 0, bptr = keybuf_tail;

    if(bptr != keybuf_head) {
        data = keybuf_buf[bptr++];               // Get next character, increment tmp pointer
        keybuf_tail = bptr & (KEYBUF_SIZE - 1);  // and update pointer
    }

    return data;
}

// BE WARNED: this function may be dangerous to use...
static char *strrepl (char *str, int c, char *str3) {

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

void process_keypress (uint8_t state) {

	bool addedGcode, jogCommand = false;
	char command[30] = "", keycode = keypad_get_keycode();

    state = state;

	if(keycode)
	  switch(keycode) {

		case 'A':									// Mist override
			enqueue_accessory_ovr(CMD_COOLANT_MIST_OVR_TOGGLE);
			break;

		case 'E':									// Coolant override
			enqueue_accessory_ovr(CMD_COOLANT_FLOOD_OVR_TOGGLE);
			break;

		case 'h':									// "toggle" jog mode
			jogMode	= jogMode == JogMode_Step ? JogMode_Fast : (jogMode == JogMode_Fast ? JogMode_Slow : JogMode_Step);
			break;

		case 'H':									// Home axes
			strcpy(command, "$H");
			break;

		case 'R':									// Jog X-axis right
			strcpy(command, "$J=G91X?F");
			break;

		case 'L':									// Jog X-axis left
			strcpy(command, "$J=G91X-?F");
			break;

		case 'F':									// Jog Y-axis forward
			strcpy(command, "$J=G91Y?F");
			break;

		case 'B':									// Jog Y-axis back
			strcpy(command, "$J=G91Y-?F");
			break;

		case 'q':									// Jog XY-axes SE
			strcpy(command, "$J=G91X?Y-?F");
			break;

		case 'r':									// Jog XY-axes NE
			strcpy(command, "$J=G91X?Y?F");
			break;

		case 's':									// Jog XY-axes NW
			strcpy(command, "$J=G91X-?Y?F");
			break;

		case 't':									// Jog XY-axes SW
			strcpy(command, "$J=G91X-?Y-?F");
			break;

		case 'U':									// Jog Z-axis up
			strcpy(command, "$J=G91Z?F");
			break;

		case 'D':									// Jog Z-axis down
			strcpy(command, "$J=G91Z-?F");
			break;

	}

	if(command[0] != '\0') {

		// add distance and speed to jog commands
		if((jogCommand = (command[0] == '$' && command[1] == 'J')))
			switch(jogMode) {

			case JogMode_Slow:
				strrepl(command, '?', "500");
				strcat(command, "100");
				break;

			case JogMode_Step:
				strrepl(command, '?', "0.25");
				strcat(command, "600");
				break;

			default:
				strrepl(command, '?', "500");
				strcat(command, "3000");
				break;

		}

		if(!(jogCommand && keyreleased)) { // key still pressed? - do not execute jog command if released!
			addedGcode = hal.protocol_enqueue_gcode(command);
			jogging = jogging || (jogCommand && addedGcode);
		}
	}

}

static void driver_keyclick_handler (bool keydown) {

	keyreleased = !keydown;

	if(keydown)
		I2C_GetKeycode();

	else if(jogging) {
		jogging = false;
		hal.protocol_process_realtime(CMD_JOG_CANCEL);
		keybuf_tail = keybuf_head = 0; // flush keycode buffer
	}
}

void I2C_ISR_ExitCallback(void)
{    
    if(I2C_mstrStatus & I2C_MSTAT_RD_CMPLT) {
    	if(KeyPad_Read() == 0) // only add keycode when key is still pressed
    		enqueue_keycode(keycode);
    }
}

static void keyclick_int_handler (void) {

    KeyPad_ClearInterrupt();

	driver_keyclick_handler(KeyPad_Read() == 0);
}
