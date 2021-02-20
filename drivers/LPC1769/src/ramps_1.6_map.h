/*
  ramps_1.6_map.h - driver code for LPC176x processor, pin mappings compatible with Ramps 1.6 board

  NOTE: board must be modified for 3.3V IO before use!

  Part of grblHAL

  Copyright (c) 2019-2020 Terje Io

  Mappings according to Re-ARM for NXP LCP1768

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

// NOTE:
// P0.27, P0.28 are dedicated I2C pins without pull up/down.
// P0.29, P0.30 must have same direction as used for USB operation.

#define BOARD_NAME "Ramps 1.6"

// Define step pulse output pins.

#define X_STEP_PN           2
#define X_STEP_PORT         port(X_STEP_PN)
#define X_STEP_PIN          1   // Due Analog Pin 0
#define X_STEP_BIT          (1<<X_STEP_PIN)
#define Y_STEP_PN           2
#define Y_STEP_PORT         port(Y_STEP_PN)
#define Y_STEP_PIN          2   // Due Analog Pin 6
#define Y_STEP_BIT          (1<<Y_STEP_PIN)
#define Z_STEP_PN           2
#define Z_STEP_PORT         port(Z_STEP_PN)
#define Z_STEP_PIN          3   // Due Digital Pin 46
#define Z_STEP_BIT          (1<<Z_STEP_PIN)
#ifdef A_AXIS
#define A_STEP_PN           2
#define A_STEP_PORT         port(A_STEP_PN)
#define A_STEP_PIN          0   // Due Digital Pin 26
#define A_STEP_BIT          (1<<A_STEP_PIN)
#endif
#ifdef B_AXIS
#define B_STEP_PN           2
#define B_STEP_PORT         port(B_STEP_PN)
#define B_STEP_PIN          8   // Due Digital Pin 36
#define B_STEP_BIT          (1<<B_STEP_PIN)
#endif
#define STEP_OUTMODE GPIO_BITBAND

// Define step direction output pins.
#define X_DIRECTION_PN      0
#define X_DIRECTION_PORT    port(X_DIRECTION_PN)
#define X_DIRECTION_PIN     11  // Due Analog Pin 1
#define X_DIRECTION_BIT     (1<<X_DIRECTION_PIN)
#define Y_DIRECTION_PN      0
#define Y_DIRECTION_PORT    port(Y_DIRECTION_PN)
#define Y_DIRECTION_PIN     20  // Due Analog Pin 7
#define Y_DIRECTION_BIT     (1<<Y_DIRECTION_PIN)
#define Z_DIRECTION_PN      0
#define Z_DIRECTION_PORT    port(Z_DIRECTION_PN)
#define Z_DIRECTION_PIN     22  // Due Digital Pin 48
#define Z_DIRECTION_BIT     (1<<Z_DIRECTION_PIN)
#ifdef A_AXIS
#define A_DIRECTION_PN      0
#define A_DIRECTION_PORT    port(A_DIRECTION_PN)
#define A_DIRECTION_PIN     5   // Due Digital Pin 28
#define A_DIRECTION_BIT     (1<<A_DIRECTION_PIN)
#endif
#ifdef B_AXIS
#define B_DIRECTION_PN      2
#define B_DIRECTION_PORT    port(B_DIRECTION_PN)
#define B_DIRECTION_PIN     13  // Due Digital Pin 34
#define B_DIRECTION_BIT     (1<<B_DIRECTION_PIN)
#endif
#define DIRECTION_OUTMODE GPIO_BITBAND

// Define stepper driver enable/disable output pin(s).
#define X_DISABLE_PN        0
#define X_DISABLE_PORT      port(X_DISABLE_PN)
#define X_DISABLE_PIN       10  // Due Digital Pin 38
#define X_DISABLE_BIT       (1<<X_DISABLE_PIN)
#define Y_DISABLE_PN        0
#define Y_DISABLE_PORT      port(Y_DISABLE_PN)
#define Y_DISABLE_PIN       19  // Due Analog Pin 2
#define Y_DISABLE_BIT       (1<<Y_DISABLE_PIN)
#define Z_DISABLE_PN        0
#define Z_DISABLE_PORT      port(Z_DISABLE_PN)
#define Z_DISABLE_PIN       21  // Due Analog Pin 8
#define Z_DISABLE_BIT       (1<<Z_DISABLE_PIN)
#ifdef A_AXIS
#define A_DISABLE_PN        0
#define A_DISABLE_PORT      port(A_DISABLE_PN)
#define A_DISABLE_PIN       4   // Due Digital Pin 24
#define A_DISABLE_BIT       (1<<A_DISABLE_PIN)
#endif
#ifdef B_AXIS
#define B_DISABLE_PN        4
#define B_DISABLE_PORT      port(B_DISABLE_PN)
#define B_DISABLE_PIN       29  // Due Digital Pin 30
#define B_DISABLE_BIT       (1<<B_DISABLE_PIN)
#endif
#define DISABLE_OUTMODE GPIO_BITBAND

// Define homing/hard limit switch input pins.
// NOTE: All limit bits (needs to be on same port)
#define X_LIMIT_PN          1
#define X_LIMIT_PORT        port(X_LIMIT_PN)
#define X_LIMIT_PIN         24  // Due Digital Pin 3
#define X_LIMIT_BIT         (1<<X_LIMIT_PIN)
#define Y_LIMIT_PN          1
#define Y_LIMIT_PORT        port(Y_LIMIT_PN)
#define Y_LIMIT_PIN         26  // Due Digital Pin 14
#define Y_LIMIT_BIT         (1<<Y_LIMIT_PIN)
#define Z_LIMIT_PN          1
#define Z_LIMIT_PORT        port(Z_LIMIT_PN)
#define Z_LIMIT_PIN         29  // Due Digital Pin 18
#define Z_LIMIT_BIT         (1<<Z_LIMIT_PIN)

// Define max homing/hard limit switch input pins.
#if LIMIT_MAX_ENABLE
#define X_LIMIT_PN_MAX      1
#define X_LIMIT_PORT_MAX    port(X_LIMIT_PN_MAX)
#define X_LIMIT_PIN_MAX     25  // Due Digital Pin 2
#define X_LIMIT_BIT_MAX     (1<<X_LIMIT_PIN_MAX)
#define Y_LIMIT_PN_MAX      1
#define Y_LIMIT_PORT_MAX    port(Y_LIMIT_PN_MAX)
#define Y_LIMIT_PIN_MAX     27  // Due Digital Pin 15
#define Y_LIMIT_BIT_MAX     (1<<Y_LIMIT_PIN_MAX)
#define Z_LIMIT_PN_MAX      1
#define Z_LIMIT_PORT_MAX    port(Z_LIMIT_PN_MAX)
#define Z_LIMIT_PIN_MAX     28  // Due Digital Pin 19
#define Z_LIMIT_BIT_MAX     (1<<Z_LIMIT_PIN_MAX)
#define LIMIT_MASK (X_LIMIT_BIT|X_LIMIT_BIT_MAX|Y_LIMIT_BIT|Y_LIMIT_BIT_MAX|Z_LIMIT_BIT|Z_LIMIT_BIT_MAX)
#else
#define LIMIT_MASK (X_LIMIT_BIT|Y_LIMIT_BIT|Z_LIMIT_BIT)
#endif

#define LIMITS_POLL_PORT port(1) // NOTE: Port 1 is not interrupt capable, use polling instead!
#define LIMIT_INMODE GPIO_BITBAND

// Define probe switch input pin.
#define PROBE_PN    4
#define PROBE_PORT  port(PROBE_PN)
#define PROBE_PIN   6
#define PROBE_BIT   (1<<PROBE_PIN)

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PN       1
#define SPINDLE_ENABLE_PORT     port(SPINDLE_ENABLE_PN)
#define SPINDLE_ENABLE_PIN      18  // Due Digital Pin 4
#define SPINDLE_ENABLE_BIT      (1<<SPINDLE_ENABLE_PIN)
#define SPINDLE_DIRECTION_PN    1
#define SPINDLE_DIRECTION_PORT  port(SPINDLE_DIRECTION_PN)
#define SPINDLE_DIRECTION_PIN   19  // Due Digital Pin 5
#define SPINDLE_DIRECTION_BIT   (1<<SPINDLE_DIRECTION_PIN)

// Start of PWM & Stepper Enabled Spindle

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PN    0
#define COOLANT_FLOOD_PORT  port(COOLANT_FLOOD_PN)
#define COOLANT_FLOOD_PIN   26  // Due Analog port 9
#define COOLANT_FLOOD_BIT   (1<<COOLANT_FLOOD_PIN)

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
#define RESET_PORT_PN       0
#define RESET_PORT          port(RESET_PORT_PN)
#define RESET_PIN           27  // DUE Analog Pin 3
#define RESET_BIT           (1<<RESET_PIN)

#define FEED_HOLD_PN        0
#define FEED_HOLD_PORT      port(FEED_HOLD_PN)
#define FEED_HOLD_PIN       28  // DUE Analog Pin 4
#define FEED_HOLD_BIT       (1<<FEED_HOLD_PIN)

#define CYCLE_START_PN      2
#define CYCLE_START_PORT    port(CYCLE_START_PN)
#define CYCLE_START_PIN     6   // DUE Analog Pin 5
#define CYCLE_START_BIT     (1<<CYCLE_START_PIN)

#define CONTROL_INMODE GPIO_BITBAND

#ifdef SPINDLE_PWM_PIN_2_4
#define SPINDLE_PWM_CHANNEL         PWM1_CH5    // MOSFET3 (P2.4)
#else
#define SPINDLE_PWM_CHANNEL         PWM1_CH6    // BED MOSFET (P2.5)
#endif
#define SPINDLE_PWM_USE_PRIMARY_PIN   false
#define SPINDLE_PWM_USE_SECONDARY_PIN true

#define SD_SPI_PORT 1
#define SD_CS_PN    0
#define SD_CS_PORT  port(SD_CS_PN)
#define SD_CS_PIN   6

/**/
