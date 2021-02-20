/*
  uno_map.h - driver code for STM32F411 ARM processor on a Nucleo-F411RE board

  Part of grblHAL

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

#if TRINAMIC_ENABLE
#error Trinamic plugin not supported!
#endif

#define BOARD_NAME "Generic Uno"

#define VARIABLE_SPINDLE // Comment out to disable variable spindle

// Define step pulse output pins.
#define X_STEP_PORT         GPIOA // D2
#define X_STEP_PIN          10
#define X_STEP_BIT          (1<<X_STEP_PIN)
#define Y_STEP_PORT         GPIOB // D3
#define Y_STEP_PIN          3
#define Y_STEP_BIT          (1<<Y_STEP_PIN)
#define Z_STEP_PORT         GPIOB // D4
#define Z_STEP_PIN          5
#define Z_STEP_BIT          (1<<Z_STEP_PIN)
#define STEP_OUTMODE        GPIO_BITBAND

// Define step direction output pins.
#define X_DIRECTION_PORT    GPIOB // D5
#define X_DIRECTION_PIN     4
#define X_DIRECTION_BIT     (1<<X_DIRECTION_PIN)
#define Y_DIRECTION_PORT    GPIOB // D6
#define Y_DIRECTION_PIN     10
#define Y_DIRECTION_BIT     (1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_PORT    GPIOA // D7
#define Z_DIRECTION_PIN     8
#define Z_DIRECTION_BIT     (1<<Z_DIRECTION_PIN)
#define DIRECTION_OUTMODE   GPIO_BITBAND

// Define stepper driver enable/disable output pin.
#define STEPPERS_DISABLE_PORT   GPIOA // D8
#define STEPPERS_DISABLE_PIN    9
#define STEPPERS_DISABLE_BIT    (1<<STEPPERS_DISABLE_PIN)
#define STEPPERS_DISABLE_MASK   STEPPERS_DISABLE_BIT

// Define homing/hard limit switch input pins.
#define X_LIMIT_PORT        GPIOC // D9
#define X_LIMIT_PIN         7
#define X_LIMIT_BIT         (1<<X_LIMIT_PIN)
#define Y_LIMIT_PORT        GPIOB // D10
#define Y_LIMIT_PIN         6
#define Y_LIMIT_BIT         (1<<Y_LIMIT_PIN)
#ifdef VARIABLE_SPINDLE
  #define Z_LIMIT_PORT      GPIOA // D12
  #define Z_LIMIT_PIN       6
#else
  #define Z_LIMIT_PORT      GPIOA // D11
  #define Z_LIMIT_PIN       7
#endif
#define Z_LIMIT_BIT         (1<<Z_LIMIT_PIN)
#define LIMIT_MASK          (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT) // All limit bits
#define LIMIT_INMODE        GPIO_BITBAND

// Comment out if Z limit pin is not assigned to an interrupt enabled pin on a different port.

#define Z_LIMIT_POLL

// Define spindle enable and spindle direction output pins.
#ifdef VARIABLE_SPINDLE
  #define SPINDLE_ENABLE_PORT   GPIOB // on morpho header
  #define SPINDLE_ENABLE_PIN    7
#else
  #define SPINDLE_ENABLE_PORT   GPIOA // D12
  #define SPINDLE_ENABLE_PIN    6
#endif
#define SPINDLE_ENABLE_BIT      (1<<SPINDLE_ENABLE_PIN)
#define SPINDLE_DIRECTION_PORT  GPIOA // D13
#define SPINDLE_DIRECTION_PIN   5
#define SPINDLE_DIRECTION_BIT   (1<<SPINDLE_DIRECTION_PIN)

// Define spindle PWM output pin.
#ifdef VARIABLE_SPINDLE
#define SPINDLE_PWM_PORT        GPIOA // D11
#define SPINDLE_PWM_PIN         7
#define SPINDLE_PWM_BIT         (1<<SPINDLE_PWM_PIN)
#endif

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT  GPIOB // A3
#define COOLANT_FLOOD_PIN   0
#define COOLANT_FLOOD_BIT   (1<<COOLANT_FLOOD_PIN)
#define COOLANT_MIST_PORT   GPIOC // A4
#define COOLANT_MIST_PIN    1
#define COOLANT_MIST_BIT    (1<<COOLANT_MIST_PIN)

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define CONTROL_PORT            GPIOA
#define CONTROL_RESET_PIN       0 // A0
#define CONTROL_RESET_BIT       (1<<CONTROL_RESET_PIN)
#define CONTROL_FEED_HOLD_PIN   1 // A1
#define CONTROL_FEED_HOLD_BIT   (1<<CONTROL_FEED_HOLD_PIN)
#define CONTROL_CYCLE_START_PIN 4 // A2
#define CONTROL_CYCLE_START_BIT (1<<CONTROL_CYCLE_START_PIN)
#define CONTROL_MASK            (CONTROL_RESET_BIT|CONTROL_FEED_HOLD_BIT|CONTROL_CYCLE_START_BIT)
#define CONTROL_INMODE          GPIO_MAP

// Define probe switch input pin.
#define PROBE_PORT  GPIOC // A5
#define PROBE_PIN   0
#define PROBE_BIT   (1<<PROBE_PIN)

/**/
