/*
  mega_2560_map.h - driver code for Atmel SAM3X8E ARM processor, pin mappings compatible with Arduino Mega 2560

  Part of grblHAL

  Copyright (c) 2019-2021 Terje Io

  Mappings according to cpu_map.h for Arduino Mega 2560 : Working @EliteEng

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

#define BOARD_NAME "Mega 2560"

 // Define step pulse output pins.
#define X_STEP_PORT         PIOA
#define X_STEP_PIN          15  // Due Digital Pin 24
#define X_STEP_BIT          (1<<X_STEP_PIN)
#define Y_STEP_PORT         PIOD
#define Y_STEP_PIN          0   // Due Digital Pin 25
#define Y_STEP_BIT          (1<<Y_STEP_PIN)
#define Z_STEP_PORT         PIOD
#define Z_STEP_PIN          1   // Due Digital Pin 26
#define Z_STEP_BIT          (1<<Z_STEP_PIN)

// Define step direction output pins.
#define X_DIRECTION_PORT    PIOD
#define X_DIRECTION_PIN     9   // Due Digital Pin 30
#define X_DIRECTION_BIT     (1<<X_DIRECTION_PIN)
#define Y_DIRECTION_PORT    PIOA
#define Y_DIRECTION_PIN     7   // Due Digital Pin 31
#define Y_DIRECTION_BIT     (1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_PORT    PIOD
#define Z_DIRECTION_PIN     10  // Due Digital Pin 32
#define Z_DIRECTION_BIT     (1<<Z_DIRECTION_PIN)

// Define stepper driver enable/disable output pin(s).
#define X_DISABLE_PORT      PIOB
#define X_DISABLE_PIN       27  // Due Digital Pin 13
#define X_DISABLE_BIT       (1<<X_DISABLE_PIN)
/*
#define Y_DISABLE_PORT      PIOA
#define Y_DISABLE_PIN       23
#define Y_DISABLE_BIT       (1<<Y_DISABLE_PIN)
#define Z_DISABLE_PORT      PIOB
#define Z_DISABLE_PIN       17
#define Z_DISABLE_BIT       (1<<Z_DISABLE_PIN)
*/
// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT        PIOC
#define X_LIMIT_PIN         29  // Due Digital Pin 10
#define X_LIMIT_BIT         (1<<X_LIMIT_PIN)
#define Y_LIMIT_PORT        PIOD
#define Y_LIMIT_PIN         7   // Due Digital Pin 11
#define Y_LIMIT_BIT         (1<<Y_LIMIT_PIN)
#define Z_LIMIT_PORT        PIOD
#define Z_LIMIT_PIN         8   // Due Digital Pin 12
#define Z_LIMIT_BIT         (1<<Z_LIMIT_PIN)

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     PIOC
#define SPINDLE_ENABLE_PIN      24  // Due Digital Pin 6
#define SPINDLE_ENABLE_BIT      (1<<SPINDLE_ENABLE_PIN)
#define SPINDLE_DIRECTION_PORT  PIOC
#define SPINDLE_DIRECTION_PIN   25  // Due Digital Pin 5
#define SPINDLE_DIRECTION_BIT   (1<<SPINDLE_DIRECTION_PIN)

// Start of PWM & Stepper Enabled Spindle

#define SPINDLE_PWM_CHANNEL 6
#define SPINDLE_PWM_PORT    PIOC
#define SPINDLE_PWM_PIN     23 // Due Digital Pin 7 / PWML6 B
#define SPINDLE_PWM_BIT     (1<<SPINDLE_PWM_PIN)

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT  PIOC
#define COOLANT_FLOOD_PIN   22  // Due Digital Pin 8
#define COOLANT_FLOOD_BIT   (1<<COOLANT_FLOOD_PIN)
#define COOLANT_MIST_PORT   PIOC
#define COOLANT_MIST_PIN    21  // Due Digital Pin 9
#define COOLANT_MIST_BIT    (1<<COOLANT_MIST_PIN)

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PORT          PIOB
#define RESET_PIN           17  // DUE Analog Pin 8
#define RESET_BIT           (1<<RESET_PIN)

#define FEED_HOLD_PORT      PIOB
#define FEED_HOLD_PIN       18  // DUE Analog Pin 9
#define FEED_HOLD_BIT       (1<<FEED_HOLD_PIN)

#define CYCLE_START_PORT    PIOB
#define CYCLE_START_PIN     19  // DUE Analog Pin 10
#define CYCLE_START_BIT     (1<<CYCLE_START_PIN)

#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define SAFETY_DOOR_PORT    PIOB
#define SAFETY_DOOR_PIN     20  // DUE Analog Pin 11
#define SAFETY_DOOR_BIT     (1<<SAFETY_DOOR_PIN)
#endif

// Define probe switch input pin.
#define PROBE_PORT          PIOA
#define PROBE_PIN           0   // DUE Analog Pin CANTX
#define PROBE_BIT           (1<<PROBE_PIN)

/**/
