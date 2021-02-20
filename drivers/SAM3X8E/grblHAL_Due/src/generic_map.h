/*
  generic_map.h - river code for Atmel SAM3X8E ARM processor

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

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

// Define step pulse output pins.
#define X_STEP_PORT         PIOA
#define X_STEP_PIN          16
#define X_STEP_BIT          (1<<X_STEP_PIN)
#define Y_STEP_PORT         PIOA
#define Y_STEP_PIN          3
#define Y_STEP_BIT          (1<<Y_STEP_PIN)
#define Z_STEP_PORT         PIOC
#define Z_STEP_PIN          17
#define Z_STEP_BIT          (1<<Z_STEP_PIN)

// Define step direction output pins.
#define X_DIRECTION_PORT    PIOA
#define X_DIRECTION_PIN     24
#define X_DIRECTION_BIT     (1<<X_DIRECTION_PIN)
#define Y_DIRECTION_PORT    PIOA
#define Y_DIRECTION_PIN     2
#define Y_DIRECTION_BIT     (1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_PORT    PIOC
#define Z_DIRECTION_PIN     15
#define Z_DIRECTION_BIT     (1<<Z_DIRECTION_PIN)

// Define stepper driver enable/disable output pin(s).
#define X_DISABLE_PORT      PIOC
#define X_DISABLE_PIN       6
#define X_DISABLE_BIT       (1<<X_DISABLE_PIN)
#define Y_DISABLE_PORT      PIOA
#define Y_DISABLE_PIN       23
#define Y_DISABLE_BIT       (1<<Y_DISABLE_PIN)
#define Z_DISABLE_PORT      PIOB
#define Z_DISABLE_PIN       17
#define Z_DISABLE_BIT       (1<<Z_DISABLE_PIN)

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT        PIOB // C28
#define X_LIMIT_PIN         25
#define X_LIMIT_BIT         (1<<X_LIMIT_PIN)
#define Y_LIMIT_PORT        PIOD // D14
#define Y_LIMIT_PIN         25
#define Y_LIMIT_BIT         (1<<Y_LIMIT_PIN)
#define Z_LIMIT_PORT        PIOC // A11
#define Z_LIMIT_PIN         24
#define Z_LIMIT_BIT         (1<<Z_LIMIT_PIN)

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     PIOA
#define SPINDLE_ENABLE_PIN      15
#define SPINDLE_ENABLE_BIT      (1<<SPINDLE_ENABLE_PIN)
#define SPINDLE_DIRECTION_PORT  PIOD
#define SPINDLE_DIRECTION_PIN   3
#define SPINDLE_DIRECTION_BIT   (1<<SPINDLE_DIRECTION_PIN)

// Start of PWM & Stepper Enabled Spindle
#define SPINDLE_PWM_TIMER   (TC2->TC_CHANNEL[0])
#define SPINDLE_PWM_PORT    PIOC
#define SPINDLE_PWM_PIN     25  // TIOA6
#define SPINDLE_PWM_BIT     (1<<SPINDLE_PWM_PIN)

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT  PIOC
#define COOLANT_FLOOD_PIN   5
#define COOLANT_FLOOD_BIT   (1<<COOLANT_FLOOD_PIN)
#define COOLANT_MIST_PORT   PIOC
#define COOLANT_MIST_PIN    3
#define COOLANT_MIST_BIT    (1<<COOLANT_MIST_PIN)

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PORT          PIOC
#define RESET_PIN           12
#define RESET_BIT           (1<<RESET_PIN)

#define FEED_HOLD_PORT      PIOC
#define FEED_HOLD_PIN       14
#define FEED_HOLD_BIT       (1<<FEED_HOLD_PIN)

#define CYCLE_START_PORT    PIOC
#define CYCLE_START_PIN     16
#define CYCLE_START_BIT     (1<<CYCLE_START_PIN)

#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define SAFETY_DOOR_PORT    PIOC
#define SAFETY_DOOR_PIN     18
#define SAFETY_DOOR_BIT     (1<<SAFETY_DOOR_PIN)
#endif

// Define probe switch input pin.
#define PROBE_PORT          PIOC
#define PROBE_PIN           13
#define PROBE_BIT           (1<<PROBE_PIN)

#if KEYPAD_ENABLE
#define KEYPAD_PORT         PIOA
#define KEYPAD_PIN          5
#define KEYPAD_BIT          (1<<KEYPAD_PIN)
#endif

#if SDCARD_ENABLE
// Define SD card detect pin.
#define SD_CD_PORT          PIOA
#define SD_CD_PIN           30
#define SD_CD_BIT           (1<<SD_CD_PIN)
#endif

/**/

