/*
  pico_cnc_map.h - driver code for RP2040 ARM processors

  Part of grblHAL

  Copyright (c) 2021 Terje Io

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

#if TRINAMIC_ENABLE
#error Trinamic plugin not supported!
#endif

#if N_AXIS > 4
#error Max number of axes is 4!
#endif

#define BOARD_NAME "PicoCNC"
#define HAS_BOARD_INIT

typedef union {
    uint32_t value;
    struct {
        uint32_t x_ena       :1,
                 y_ena       :1,
                 z_ena       :1,
                 a_ena       :1,
                 spindle_dir :1,
                 spindle_ena :1,
                 mist_ena    :1,
                 flood_ena   :1,
                 aux0_out    :1,
                 aux1_out    :1,
                 aux2_out    :1,
                 aux3_out    :1,
                 aux4_out    :1,
                 aux5_out    :1,
                 aux6_out    :1,
                 aux7_out    :1,
                 unused      :16;
    };
} output_sr_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t x_step :1,
                y_step :1,
                z_step :1,
                a_step :1,
                x_dir  :1,
                y_dir  :1,
                z_dir  :1,
                a_dir  :1;
    };
} step_dir_t;

typedef union {
    uint32_t value;
    struct {
        step_dir_t set;
        step_dir_t reset;
        uint16_t unused;
    };
} step_dir_sr_t;

// Define step pulse output pins.
#define SD_SHIFT_REGISTER  8
#define SD_SR_DATA_PIN     14
#define SD_SR_SCK_PIN      15 // includes next pin (16)

// Define output signals pins.
#define OUT_SHIFT_REGISTER  16
#define OUT_SR_DATA_PIN     26
#define OUT_SR_SCK_PIN      17 // includes next pin (13)

#define AUX_N_OUT 8
#define AUX_OUT_MASK 0xFF

// Define homing/hard limit switch input pins.
#define X_LIMIT_PIN     6
#define Y_LIMIT_PIN     5
#define Z_LIMIT_PIN     4
#define X_LIMIT_BIT     (1<<X_LIMIT_PIN)
#define Y_LIMIT_BIT     (1<<Y_LIMIT_PIN)
#define Z_LIMIT_BIT     (1<<Z_LIMIT_PIN)
#if N_AXIS > 3
#define A_LIMIT_PIN     3
#define A_LIMIT_BIT     (1<<A_LIMIT_PIN)
#define LIMIT_MASK      (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT|A_LIMIT_BIT) // All limit bits
#else
#define LIMIT_MASK      (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT) // All limit bits
#endif
#define LIMIT_INMODE    GPIO_MAP

// Define spindle PWM output pin.
#define SPINDLE_PWM_PIN             27
#define SPINDLE_PWM_BIT             (1<<SPINDLE_PWM_PIN)

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_RESET_PIN           22
#define CONTROL_RESET_BIT           (1<<CONTROL_RESET_PIN)
#define CONTROL_FEED_HOLD_PIN       7
#define CONTROL_FEED_HOLD_BIT       (1<<CONTROL_FEED_HOLD_PIN)
#define CONTROL_CYCLE_START_PIN     8
#define CONTROL_CYCLE_START_BIT     (1<<CONTROL_CYCLE_START_PIN)
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define CONTROL_SAFETY_DOOR_PIN     9
#define CONTROL_SAFETY_DOOR_BIT     (1<<CONTROL_SAFETY_DOOR_PIN)
#define CONTROL_MASK                (CONTROL_RESET_BIT|CONTROL_FEED_HOLD_BIT|CONTROL_CYCLE_START_BIT|CONTROL_SAFETY_DOOR_BIT)
#else
#define CONTROL_MASK                (CONTROL_RESET_BIT|CONTROL_FEED_HOLD_BIT|CONTROL_CYCLE_START_BIT)
#endif
#define CONTROL_INMODE              GPIO_MAP

// Define probe switch input pin.
#define PROBE_PIN                   28
#define PROBE_BIT                   (1<<PROBE_PIN)

#if KEYPAD_ENABLE
#define KEYPAD_STROBE_PIN           19
#define KEYPAD_STROBE_BIT           (1<<KEYPAD_STROBE_PIN)
#endif
