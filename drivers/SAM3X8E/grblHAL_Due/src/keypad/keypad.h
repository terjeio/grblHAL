/*
  keypad.h - I2C keypad plugin

  Part of grblHAL

  Copyright (c) 2017-2020 Terje Io

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

#ifndef _KEYPAD_H_
#define _KEYPAD_H_

#ifdef ARDUINO
#include "../grbl/gcode.h"
#include "../grbl/settings.h"
#else
#include "grbl/gcode.h"
#include "grbl/settings.h"
#endif

#define KEYBUF_SIZE 8 // must be a power of 2
#define KEYPAD_I2CADDR 0x49

#define JOG_XR   'R'
#define JOG_XL   'L'
#define JOG_YF   'F'
#define JOG_YB   'B'
#define JOG_ZU   'U'
#define JOG_ZD   'D'
#define JOG_XRYF 'r'
#define JOG_XRYB 'q'
#define JOG_XLYF 's'
#define JOG_XLYB 't'
#define JOG_XRZU 'w'
#define JOG_XRZD 'v'
#define JOG_XLZU 'u'
#define JOG_XLZD 'x'

typedef enum {
    JogMode_Fast = 0,
    JogMode_Slow,
    JogMode_Step
} jogmode_t;

typedef void (*keycode_callback_ptr)(const char c);
typedef bool (*on_keypress_preview_ptr)(const char c, uint_fast16_t state);
typedef void (*on_jogmode_changed_ptr)(jogmode_t jogmode);

typedef struct {
    on_keypress_preview_ptr on_keypress_preview;
    on_jogmode_changed_ptr on_jogmode_changed;
} keypad_t;

extern keypad_t keypad;

bool keypad_init (void);
void keypad_keyclick_handler (bool keydown);
void keypad_enqueue_keycode (char c);

#endif
