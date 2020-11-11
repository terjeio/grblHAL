/*
  i2c.c - An embedded CNC Controller with rs274/ngc (g-code) support

  I2C driver for Cypress PSoC 5 (CY8CKIT-059)

  Part of GrblHAL

  Copyright (c) 2020 Terje Io

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
#include "keypad/keypad.h"

static uint8 keycode;
keycode_callback_ptr on_keyclick;

static void keyclick_int_handler (void) {

    KeyPadIO_ClearInterrupt();

    on_keyclick(KeyPadIO_Read() != 0);
}

void I2C_Init (void)
{
    I2CPort_Start();
    KeyPadInterrupt_StartEx(keyclick_int_handler); 
}

void I2C_ISR_ExitCallback(void)
{    
    if(I2CPort_mstrStatus & I2CPort_MSTAT_RD_CMPLT) {
        if(KeyPadIO_Read() == 0) // only add keycode when key is still pressed
            keypad_enqueue_keycode(keycode);
    }
}

// get single byte - via interrupt
void I2C_GetKeycode (uint32_t i2cAddr, keycode_callback_ptr callback)
{
    if(I2CPort_MasterStatus() != I2CPort_MSTAT_XFER_INP) { // ignore if busy
        keycode = 0;
        on_keyclick = callback;
        I2CPort_MasterReadBuf(i2cAddr, &keycode, 1, I2CPort_MODE_COMPLETE_XFER);
    }
}