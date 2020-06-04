/*
  driver.h - driver code for simulator MCU

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

#define portINT(p) portQ(p)
#define portQ(p) GPIO ## p ## _IRQ

#define STEPPER_TIMER 0

#define LIMITS_PORT0 0
#define LIMITS_IRQ0  portINT(LIMITS_PORT0)
#define LIMITS_PORT1 1
#define LIMITS_IRQ1  portINT(LIMITS_PORT1)

#define SPINDLE_PORT 3
#define COOLANT_PORT 4
#define STEP_PORT0   6
#define STEP_PORT1   7
#define DIR_PORT     8
#define STEPPER_ENABLE_PORT 9

#define SPINDLE_MASK 0x03
#define COOLANT_MASK 0x03

#define CONTROL_PORT        2
#define CONTROL_IRQ         portINT(CONTROL_PORT)
#define RESET_PIN           0
#define FEED_HOLD_PIN       1
#define CYCLE_START_PIN     2
#define SAFETY_DOOR_PIN     3
#define ESTOP_PIN           4
#define RESET_BIT           (1<<RESET_PIN)
#define FEED_HOLD_BIT       (1<<FEED_HOLD_PIN)
#define CYCLE_START_BIT     (1<<CYCLE_START_PIN)
#define SAFETY_DOOR_BIT     (1<<SAFETY_DOOR_PIN)
#define ESTOP_BIT           (1<<ESTOP_PIN)
#define CONTROL_MASK        (RESET_BIT|FEED_HOLD_BIT|CYCLE_START_BIT|SAFETY_DOOR_BIT)

#define PROBE_PORT          5
#define PROBE_PIN           0
#define PROBE_CONNECTED_PIN 1
#define PROBE_BIT           (1<<PROBE_PIN)
#define PROBE_CONNECTED_BIT (1<<PROBE_CONNECTED_PIN)
#define PROBE_MASK          (PROBE_BIT|PROBE_CONNECTED_BIT)
