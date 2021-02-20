/*
  cnc_boosterpack_map.h - pin mapping configuration file for CNC BoosterPack

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

#define BOARD_NAME "CNC BoosterPack"

#ifdef EEPROM_ENABLE
#undef EEPROM_ENABLE
#endif
#define EEPROM_ENABLE   1 // CNC BoosterPack has on-board EEPROM

/// Define step pulse output pins
#define STEP_PORT       6
#define X_STEP_PIN      BIT1
#define Y_STEP_PIN      BIT2
#define Z_STEP_PIN      BIT3
#define STEP_PORT_OUT   portOut(STEP_PORT)
#define STEP_PORT_DIR   portDir(STEP_PORT)
#define HWSTEP_MASK     (X_STEP_PIN|Y_STEP_PIN|Z_STEP_PIN) // All direction bits

// Define step direction output pins. NOTE: All direction pins must be on the same port.
#define DIRECTION_PORT      3
#define X_DIRECTION_PIN     BIT0
#define Y_DIRECTION_PIN     BIT1
#define Z_DIRECTION_PIN     BIT2
#define DIRECTION_PORT_OUT  portOut(DIRECTION_PORT)
#define DIRECTION_PORT_DIR  portDir(DIRECTION_PORT)
#define HWDIRECTION_MASK    (X_DIRECTION_PIN|Y_DIRECTION_PIN|Z_DIRECTION_PIN) // All direction bits

// Define stepper driver enable/disable output pin.
#define STEPPERS_DISABLE_PORT_XY    6
#define STEPPERS_DISABLE_PIN_XY     BIT4
#define STEPPERS_DISABLE_OUT_XY     portOut(STEPPERS_DISABLE_PORT_XY)
#define STEPPERS_DISABLE_DIR_XY     portDir(STEPPERS_DISABLE_PORT_XY)
#define STEPPERS_DISABLE_PORT_Z     7
#define STEPPERS_DISABLE_PIN_Z      BIT4
#define STEPPERS_DISABLE_OUT_Z      portOut(STEPPERS_DISABLE_PORT_Z)
#define STEPPERS_DISABLE_DIR_Z      portDir(STEPPERS_DISABLE_PORT_Z)

#ifdef CNC_BOOSTERPACK_A4998

// Stepper driver VDD supply

#define STEPPERS_VDD_PORT 6
#define STEPPERS_VDD_OUT  portOut(STEPPERS_VDD_PORT)
#define STEPPERS_VDD_DIR  portDir(STEPPERS_VDD_PORT)
#define STEPPERS_VDD_PIN  6
#define STEPPERS_VDD_BIT  (1<<STEPPERS_VDD_PIN)

#endif

// Define homing/hard limit switch input pins and limit interrupt vectors.
// NOTE: All limit bit pins must be on the same port
#define LIMIT_PORT          2
#define X_LIMIT_PIN         BIT2
#define Y_LIMIT_PIN         BIT4
#define Z_LIMIT_PIN         BIT5
#define LIMIT_PORT_IN       portIn(LIMIT_PORT)
#define LIMIT_PORT_OUT      portOut(LIMIT_PORT)
#define LIMIT_PORT_DIR      portDir(LIMIT_PORT)
#define LIMIT_PORT_REN      portRen(LIMIT_PORT)
#define LIMIT_PORT_IE       portIe(LIMIT_PORT)
#define LIMIT_PORT_IES      portEs(LIMIT_PORT)
#define LIMIT_PORT_IFG      portIfg(LIMIT_PORT)
#define LIMIT_PORT_VECTOR   portInt(LIMIT_PORT)
#define HWLIMIT_MASK        (X_LIMIT_PIN|Y_LIMIT_PIN|Z_LIMIT_PIN) // All limit bits

// Define spindle enable and spindle direction output pins.
#define SPINDLE_ENABLE_PORT     7
#define SPINDLE_ENABLE_PIN      BIT0
#define SPINDLE_ENABLE_In       portIn(SPINDLE_ENABLE_PORT)
#define SPINDLE_ENABLE_OUT      portOut(SPINDLE_ENABLE_PORT)
#define SPINDLE_ENABLE_DIR      portDir(SPINDLE_ENABLE_PORT)

#define SPINDLE_DIRECTION_PORT  3
#define SPINDLE_DIRECTION_PIN   BIT6
#define SPINDLE_DIRECTION_IN    portIn(SPINDLE_DIRECTION_PORT)
#define SPINDLE_DIRECTION_OUT   portOut(SPINDLE_DIRECTION_PORT)
#define SPINDLE_DIRECTION_DIR   portDir(SPINDLE_DIRECTION_PORT)

// Define flood and mist coolant enable output pins.
#define COOLANT_FLOOD_PORT  4
#define COOLANT_FLOOD_PIN   BIT0
#define COOLANT_FLOOD_IN    portIn(COOLANT_FLOOD_PORT)
#define COOLANT_FLOOD_OUT   portOut(COOLANT_FLOOD_PORT)
#define COOLANT_FLOOD_DIR   portDir(COOLANT_FLOOD_PORT)

#define COOLANT_MIST_PORT   3
#define COOLANT_MIST_PIN    BIT7
#define COOLANT_MIST_IN     portIn(COOLANT_MIST_PORT)
#define COOLANT_MIST_OUT    portOut(COOLANT_MIST_PORT)
#define COOLANT_MIST_DIR    portDir(COOLANT_MIST_PORT)

// Define user-control CONTROLs (cycle start, reset, feed hold) input pins.
// NOTE: All CONTROLs pins must be on the same port and not on a port with other input pins (limits).
#define CONTROL_PORT        1
#define RESET_PIN           BIT5
#define FEED_HOLD_PIN       BIT3
#define CYCLE_START_PIN     BIT2
#define SAFETY_DOOR_PIN     BIT4
#define CONTROL_PORT_OUT    portOut(CONTROL_PORT)
#define CONTROL_PORT_IN     portIn(CONTROL_PORT)
#define CONTROL_PORT_DIR    portDir(CONTROL_PORT)
#define CONTROL_PORT_REN    portRen(CONTROL_PORT)
#define CONTROL_PORT_IE     portIe(CONTROL_PORT)
#define CONTROL_PORT_IES    portEs(CONTROL_PORT)
#define CONTROL_PORT_IFG    portIfg(CONTROL_PORT)
#define CONTROL_PORT_VECTOR portInt(CONTROL_PORT)
#define HWCONTROL_MASK      (RESET_PIN|FEED_HOLD_PIN|CYCLE_START_PIN|SAFETY_DOOR_PIN)

// Define probe switch input pin.
#define PROBE_PORT      2
#define PROBE_PIN       BIT7
#define PROBE_PORT_OUT  portOut(PROBE_PORT)
#define PROBE_PORT_IN   portIn(PROBE_PORT)
#define PROBE_PORT_DIR  portDir(PROBE_PORT)
#define PROBE_PORT_REN  portRen(PROBE_PORT)
#define PROBE_PORT_IE   portIe(PROBE_PORT)

// Start of PWM & Stepper Enabled Spindle

#define PWM_PORT     2
#define PWM_PIN      BIT0
#define PWM_PORT_DIR portDir(PROBE_PORT)
#define PWM_SEL      portSel(PWM_PORT)

#define PWM_TIMER A
#define PWM_TIMER_INSTANCE 1
#define PWM_TIMER_CTL     timerCtl(PWM_TIMER, PWM_TIMER_INSTANCE)
#define PWM_TIMER_CCTL0   timerCCtl(PWM_TIMER, PWM_TIMER_INSTANCE, 0)
#define PWM_TIMER_CCTL1   timerCCtl(PWM_TIMER, PWM_TIMER_INSTANCE, 1)
#define PWM_TIMER_CCR0    timerCcr(PWM_TIMER, PWM_TIMER_INSTANCE, 0)
#define PWM_TIMER_CCR1    timerCcr(PWM_TIMER, PWM_TIMER_INSTANCE, 1)
#define PWM_TIMER_IV      timerIv(PWM_TIMER, PWM_TIMER_INSTANCE)
#define PWM_TIMER_EX0     timerEx(PWM_TIMER, PWM_TIMER_INSTANCE)

#define PULSE_TIMER A
#define PULSE_TIMER_INSTANCE 0
#define PULSE_TIMER_R       timerR(PULSE_TIMER, PULSE_TIMER_INSTANCE)
#define PULSE_TIMER_CTL     timerCtl(PULSE_TIMER, PULSE_TIMER_INSTANCE)
#define PULSE_TIMER_CCTL0   timerCCtl(PULSE_TIMER, PULSE_TIMER_INSTANCE, 0)
#define PULSE_TIMER_CCTL1   timerCCtl(PULSE_TIMER, PULSE_TIMER_INSTANCE, 1)
#define PULSE_TIMER_CCR0    timerCcr(PULSE_TIMER, PULSE_TIMER_INSTANCE, 0)
#define PULSE_TIMER_CCR1    timerCcr(PULSE_TIMER, PULSE_TIMER_INSTANCE, 1)
#define PULSE_TIMER_IV      timerIv(PULSE_TIMER, PULSE_TIMER_INSTANCE)
#define PULSE_TIMER_EX0     timerEx(PULSE_TIMER, PULSE_TIMER_INSTANCE)
#define PULSE_TIMER0_VECTOR timerInt(PULSE_TIMER, PULSE_TIMER_INSTANCE, 0)
#define PULSE_TIMER1_VECTOR timerInt(PULSE_TIMER, PULSE_TIMER_INSTANCE, 1)

#define STEPPER_TIMER A
#define STEPPER_TIMER_INSTANCE 2
#define STEPPER_TIMER_CTL     timerCtl(STEPPER_TIMER, STEPPER_TIMER_INSTANCE)
#define STEPPER_TIMER_CCTL0   timerCCtl(STEPPER_TIMER, STEPPER_TIMER_INSTANCE, 0)
#define STEPPER_TIMER_CCR0    timerCcr(STEPPER_TIMER, STEPPER_TIMER_INSTANCE, 0)
#define STEPPER_TIMER_EX0     timerEx(STEPPER_TIMER, STEPPER_TIMER_INSTANCE)
#define STEPPER_TIMER0_VECTOR timerInt(STEPPER_TIMER, STEPPER_TIMER_INSTANCE, 0)

#define DEBOUNCE_TIMER A
#define DEBOUNCE_TIMER_INSTANCE 2
#define DEBOUNCE_TIMER_CTL      timerCtl(DEBOUNCE_TIMER, DEBOUNCE_TIMER_INSTANCE)
#define DEBOUNCE_TIMER_CCTL0    timerCCtl(DEBOUNCE_TIMER, DEBOUNCE_TIMER_INSTANCE, 0)
#define DEBOUNCE_TIMER_EX0      timerEx(DEBOUNCE_TIMER, DEBOUNCE_TIMER_INSTANCE)
#define DEBOUNCE_TIMER_CCR0     timerCcr(DEBOUNCE_TIMER, DEBOUNCE_TIMER_INSTANCE, 0)
#define DEBOUNCE_TIMER0_VECTOR  timerInt(DEBOUNCE_TIMER, DEBOUNCE_TIMER_INSTANCE, 0)

#define SYSTICK_TIMER B
#define SYSTICK_TIMER_INSTANCE  0
#define SYSTICK_TIMER_CTL       timerCtl(SYSTICK_TIMER, SYSTICK_TIMER_INSTANCE)
#define SYSTICK_TIMER_CCTL0     timerCCtl(SYSTICK_TIMER, SYSTICK_TIMER_INSTANCE, 0)
#define SYSTICK_TIMER_EX0       timerEx(SYSTICK_TIMER, SYSTICK_TIMER_INSTANCE)
#define SYSTICK_TIMER_CCR0      timerCcr(SYSTICK_TIMER, SYSTICK_TIMER_INSTANCE, 0)
#define SYSTICK_TIMER0_VECTOR   timerInt(SYSTICK_TIMER, SYSTICK_TIMER_INSTANCE, 0)

/* EOF */
