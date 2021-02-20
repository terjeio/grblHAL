/*
  st_morpho_map.h - driver code for STM32F4xx ARM processors

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io

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


#if TRINAMIC_ENABLE == 2130
#include "trinamic\tmc2130.h"
#endif

#if TRINAMIC_ENABLE == 2209
#include "trinamic\tmc2209.h"
#endif

#define BOARD_NAME "ST Nucleo-64"

#define HAS_BOARD_INIT
//#define SPINDLE_SYNC_ENABLE

void board_init (void);

// Define step pulse output pins.
#define STEP_PORT       GPIOC
#define X_STEP_PIN      0
#define Y_STEP_PIN      5
#define Z_STEP_PIN      9
#define X_STEP_BIT      (1<<X_STEP_PIN)
#define Y_STEP_BIT      (1<<Y_STEP_PIN)
#define Z_STEP_BIT      (1<<Z_STEP_PIN)
#if N_AXIS > 3
#define A_STEP_PIN      6
#define A_STEP_BIT      (1<<A_STEP_PIN)
#define STEP_MASK       (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT|A_STEP_BIT) // All step bits
#else
#define STEP_MASK       (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT) // All step bits
#endif
#define STEP_OUTMODE GPIO_MAP

// Define step direction output pins.
#define DIRECTION_PORT      GPIOA
#define X_DIRECTION_PIN     0
#define Y_DIRECTION_PIN     4
#define Z_DIRECTION_PIN     11
#define X_DIRECTION_BIT     (1<<X_DIRECTION_PIN)
#define Y_DIRECTION_BIT     (1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_BIT     (1<<Z_DIRECTION_PIN)
#if N_AXIS > 3
#define A_DIRECTION_PIN     12
#define A_DIRECTION_BIT     (1<<A_DIRECTION_PIN)
#define DIRECTION_MASK      (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT|A_DIRECTION_BIT) // All direction bits
#else
#define DIRECTION_MASK      (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT) // All direction bits
#endif
#define DIRECTION_OUTMODE   GPIO_MAP

// Define stepper driver enable/disable output pin.a2

#define X_STEPPERS_DISABLE_PORT   GPIOA
#define X_STEPPERS_DISABLE_PIN    1
#define Y_STEPPERS_DISABLE_PORT   GPIOB
#define Y_STEPPERS_DISABLE_PIN    12
#define Z_STEPPERS_DISABLE_PORT   GPIOB
#define Z_STEPPERS_DISABLE_PIN    1
#define X_STEPPERS_DISABLE_BIT    (1<<X_STEPPERS_DISABLE_PIN)
#define Y_STEPPERS_DISABLE_BIT    (1<<Y_STEPPERS_DISABLE_PIN)
#define Z_STEPPERS_DISABLE_BIT    (1<<Z_STEPPERS_DISABLE_PIN)
#if N_AXIS > 3
#define A_STEPPERS_DISABLE_PORT   GPIOA
#define A_STEPPERS_DISABLE_PIN    6
#define A_STEPPERS_DISABLE_BIT    (1<<A_STEPPERS_DISABLE_PIN)
#endif

// Define homing/hard limit switch input pins.
#define LIMIT_PORT       GPIOC
#define X_LIMIT_PIN      13
#define Y_LIMIT_PIN      12
#define Z_LIMIT_PIN      10
#define X_LIMIT_BIT      (1<<X_LIMIT_PIN)
#define Y_LIMIT_BIT      (1<<Y_LIMIT_PIN)
#define Z_LIMIT_BIT      (1<<Z_LIMIT_PIN)
#if N_AXIS > 3
#define A_LIMIT_PIN      11
#define A_LIMIT_BIT      (1<<A_LIMIT_PIN)
#define LIMIT_MASK       (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT|A_LIMIT_BIT) // All limit bits
#else
#define LIMIT_MASK       (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT) // All limit bits
#endif
#define LIMIT_INMODE GPIO_BITBAND

  // Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT         GPIOB
#define SPINDLE_ENABLE_PIN          3
#define SPINDLE_ENABLE_BIT          (1<<SPINDLE_ENABLE_PIN)
#define SPINDLE_DIRECTION_PORT      GPIOB
#define SPINDLE_DIRECTION_PIN       5
#define SPINDLE_DIRECTION_BIT       (1<<SPINDLE_DIRECTION_PIN)

// Define spindle PWM output pin.
#define SPINDLE_PWM_PORT            GPIOA
#define SPINDLE_PWM_PIN             8
#define SPINDLE_PWM_BIT             (1<<SPINDLE_PWM_PIN)

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT          GPIOB
#define COOLANT_FLOOD_PIN           10
#define COOLANT_FLOOD_BIT           (1<<COOLANT_FLOOD_PIN)
#define COOLANT_MIST_PORT           GPIOB
#define COOLANT_MIST_PIN            4
#define COOLANT_MIST_BIT            (1<<COOLANT_MIST_PIN)

// Define user-control controls (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT                GPIOC
#define CONTROL_RESET_PIN           2
#define CONTROL_RESET_BIT           (1<<CONTROL_RESET_PIN)
#define CONTROL_FEED_HOLD_PIN       3
#define CONTROL_FEED_HOLD_BIT       (1<<CONTROL_FEED_HOLD_PIN)
#define CONTROL_CYCLE_START_PIN     4
#define CONTROL_CYCLE_START_BIT     (1<<CONTROL_CYCLE_START_PIN)
#ifdef ENABLE_SAFETY_DOOR_INPUT_PIN
#define CONTROL_SAFETY_DOOR_PIN     1
#define CONTROL_SAFETY_DOOR_BIT     (1<<CONTROL_SAFETY_DOOR_PIN)
#define CONTROL_MASK                (CONTROL_RESET_BIT|CONTROL_FEED_HOLD_BIT|CONTROL_CYCLE_START_BIT|CONTROL_SAFETY_DOOR_BIT)
#else
#define CONTROL_MASK                (CONTROL_RESET_BIT|CONTROL_FEED_HOLD_BIT|CONTROL_CYCLE_START_BIT)
#endif
#define CONTROL_INMODE GPIO_MAP

// Define probe switch input pin.
#define PROBE_PORT                  GPIOC
#define PROBE_PIN                   7
#define PROBE_BIT                   (1<<PROBE_PIN)

// Spindle encoder pins.
#ifdef SPINDLE_SYNC_ENABLE

#define SPINDLE_INDEX_PORT  GPIOB
#define SPINDLE_INDEX_PIN   14
#define SPINDLE_INDEX_BIT   (1<<SPINDLE_INDEX_PIN)

#define SPINDLE_PULSE_PORT  GPIOD
#define SPINDLE_PULSE_PIN   2
#define SPINDLE_PULSE_BIT   (1<<SPINDLE_PULSE_PIN)

#endif

// Auxiliary I/O
#define AUXINPUT0_PORT  GPIOB
#define AUXINPUT0_PIN   14
#define AUXINPUT0_BIT   (1<<AUXINPUT0_PIN)
#define AUXINPUT1_PORT  GPIOA
#define AUXINPUT1_PIN   15
#define AUXINPUT1_BIT   (1<<AUXINPUT1_PIN)
#define AUX_N_IN 2
#define AUX_IN_MASK 0b11

#define AUXOUTPUT0_PORT GPIOB
#define AUXOUTPUT0_PIN  15
#define AUXOUTPUT0_BIT  (1<<AUXOUTPUT0_PIN)
#define AUXOUTPUT1_PORT GPIOB
#define AUXOUTPUT1_PIN  2
#define AUXOUTPUT1_BIT  (1<<AUXOUTPUT1_PIN)
#define AUX_N_OUT 2
#define AUX_OUT_MASK 0b11

#if KEYPAD_ENABLE
#define KEYPAD_PORT         GPIOB
#define KEYPAD_STROBE_PIN   0
#define KEYPAD_STROBE_BIT   (1<<KEYPAD_STROBE_PIN)
#endif

#if SDCARD_ENABLE
#define SD_CS_PORT  GPIOC
#define SD_CS_PIN   8
#define SD_CS_BIT   (1<<SD_CS_PIN)
#define SPI_PORT    1 // GPIOA, SCK_PIN = 5, MISO_PIN = 6, MOSI_PIN = 7
#endif

#if TRINAMIC_ENABLE
#define TRINAMIC_CS_PORT GPIOB
#define TRINAMIC_CS_PIN  7
#define TRINAMIC_CS_BIT  (1<<TRINAMIC_CS_PIN)
#define SPI_PORT         1 // GPIOA, SCK_PIN = 5, MISO_PIN = 6, MOSI_PIN = 7
#endif

// EOF
