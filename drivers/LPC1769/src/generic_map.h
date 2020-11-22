/*
  generic_map.h - driver code for NXP LPC176x ARM processors

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

#include "driver.h"

// Define step pulse output pins. NOTE: All step bit pins must be on the same port.

#define STEP_PN         2
#define STEP_PORT       port(STEP_PN)
#define X_STEP_PIN      1
#define Y_STEP_PIN      2
#define Z_STEP_PIN      3
#define X_STEP_BIT      (1<<X_STEP_PIN)
#define Y_STEP_BIT      (1<<Y_STEP_PIN)
#define Z_STEP_BIT      (1<<Z_STEP_PIN)
#define STEP_MASK (X_STEP_BIT|Y_STEP_BIT|Z_STEP_BIT) // All step bits
//#define STEP_OUTMODE GPIO_SHIFT3
//#define STEP_OUTMODE GPIO_BITBAND
#define STEP_OUTMODE GPIO_MAP

// Define step direction output pins. NOTE: All direction pins must be on the same port.

#define DIRECTION_PN      0
#define DIRECTION_PORT    port(DIRECTION_PN)
#define X_DIRECTION_PIN   11
#define Y_DIRECTION_PIN   20
#define Z_DIRECTION_PIN   22
#define X_DIRECTION_BIT   (1<<X_DIRECTION_PIN)
#define Y_DIRECTION_BIT   (1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_BIT   (1<<Z_DIRECTION_PIN)
#define DIRECTION_MASK (X_DIRECTION_BIT|Y_DIRECTION_BIT|Z_DIRECTION_BIT) // All direction bits
//#define DIRECTION_OUTMODE GPIO_MAP
#define DIRECTION_OUTMODE GPIO_BITBAND

// Define stepper driver enable/disable output pin.

#define STEPPERS_DISABLE_PN     0
#define STEPPERS_DISABLE_PORT   port(STEPPERS_DISABLE_PN)
#define STEPPERS_DISABLE_PIN    10
// 19 + 21
#define STEPPERS_DISABLE_BIT    (1<<STEPPERS_DISABLE_PIN)
#define STEPPERS_DISABLE_MASK   (STEPPERS_DISABLE_BIT)

// Define homing/hard limit switch input pins
// NOTE: All limit bit pins must be on the same port

#define LIMIT_PN      0
#define LIMIT_PORT    port(LIMIT_PN)

#define X_LIMIT_PIN 24
#define Y_LIMIT_PIN 26
#define Z_LIMIT_PIN 29
#define X_LIMIT_BIT (1<<X_LIMIT_PIN)
#define Y_LIMIT_BIT (1<<Y_LIMIT_PIN)
#define Z_LIMIT_BIT (1<<Z_LIMIT_PIN)
#define LIMIT_INMODE GPIO_BITBAND
#define LIMIT_MASK  (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT) // All limit bits
//#define LIMIT_SHIFT GPIO_SHIFT4 // Uncomment and set shift value if pins are consecutive and ordered

// Define flood and mist coolant output pins.

#define COOLANT_FLOOD_PN    2
#define COOLANT_FLOOD_PORT  port(COOLANT_FLOOD_PN)
#define COOLANT_FLOOD_PIN   4
#define COOLANT_FLOOD_BIT   (1<<COOLANT_FLOOD_PIN)

#define COOLANT_MIST_PN     2
#define COOLANT_MIST_PORT   port(COOLANT_MIST_PN)
#define COOLANT_MIST_PIN    6
#define COOLANT_MIST_BIT    (1<<COOLANT_MIST_PIN)

// Define user-control controls (cycle start, reset, feed hold) input pins.
// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).

#define CONTROL_PN       0
#define CONTROL_PORT     port(CONTROL_PN)

#define RESET_PIN           6
#define FEED_HOLD_PIN       7
#define CYCLE_START_PIN     8
#define SAFETY_DOOR_PORT    CONTROL_PORT
#define SAFETY_DOOR_PIN     9
#define RESET_BIT           (1<<RESET_PIN)
#define FEED_HOLD_BIT       (1<<FEED_HOLD_PIN)
#define CYCLE_START_BIT     (1<<CYCLE_START_PIN)
#define SAFETY_DOOR_BIT     (1<<SAFETY_DOOR_PIN)
#define CONTROL_MASK        (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT|SAFETY_DOOR_BIT)
//#define CONTROL_SHIFT       GPIO_SHIFT0 // Uncomment and set shift value if pins are consecutive and ordered
#define CONTROL_INMODE GPIO_BITBAND

// Define probe switch input pin.
#define PROBE_PN    4
#define PROBE_PORT  port(PROBE_PN)
#define PROBE_PIN   6
#define PROBE_BIT   (1<<PROBE_PIN)

// Define spindle enable, spindle direction and PWM output pins.

#define SPINDLE_ENABLE_PN     1
#define SPINDLE_ENABLE_PORT   port(SPINDLE_ENABLE_PN)
#define SPINDLE_ENABLE_PIN    18
#define SPINDLE_ENABLE_BIT    (1<<SPINDLE_ENABLE_PIN)

#define SPINDLE_DIRECTION_PN    1
#define SPINDLE_DIRECTION_PORT  port(SPINDLE_DIRECTION_PN)
#define SPINDLE_DIRECTION_PIN   19
#define SPINDLE_DIRECTION_BIT   (1<<SPINDLE_DIRECTION_PIN)

#ifdef SPINDLE_PWM_PIN_2_4
#define SPINDLE_PWM_CHANNEL     PWM1_CH5    // MOSFET3 (P2.4)
#else
#define SPINDLE_PWM_CHANNEL     PWM1_CH6    // BED MOSFET (P2.5)
#endif
#define SPINDLE_PWM_USE_PRIMARY_PIN   false
#define SPINDLE_PWM_USE_SECONDARY_PIN true

#if SDCARD_ENABLE
#define SD_SPI_PORT 0
#define SD_CS_PN    0
#define SD_CS_PORT  port(SD_CS_PN)
#define SD_CS_PIN   16
#endif

/**/
