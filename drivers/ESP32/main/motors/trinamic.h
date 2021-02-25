/*
  motors/trinamic.h - Trinamic stepper driver plugin

  Part of grblHAL

  Copyright (c) 2018-2021 Terje Io

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

#ifndef _TRINAMIC_H_
#define _TRINAMIC_H_

#ifdef ARDUINO
#include "../driver.h"
#else
#include "driver.h"
#endif

#if TRINAMIC_ENABLE

#if TRINAMIC_ENABLE == 2130
#define R_SENSE 110
#include "../trinamic/tmc2130hal.h"
#endif
#if TRINAMIC_ENABLE == 2209
#define R_SENSE 110
#include "../trinamic/tmc2209hal.h"
#endif
#if TRINAMIC_ENABLE == 5160
#define R_SENSE 75
#include "../trinamic/tmc5160hal.h"
#endif

// General
#if TRINAMIC_MIXED_DRIVERS
#define TMC_X_ENABLE 0
#else
#define TMC_X_ENABLE 1 // Do not change
#endif
#define TMC_X_MONITOR 1
#define TMC_X_MICROSTEPS 16
#define TMC_X_R_SENSE R_SENSE   // mOhm
#define TMC_X_CURRENT 500       // mA RMS
#define TMC_X_HOLD_CURRENT_PCT 50
#define TMC_X_SGT 22

#define TMC_X_ADVANCED \
stepper[X_AXIS]->stealthChop(X_AXIS, 0); \
stepper[X_AXIS]->sg_filter(X_AXIS, 1); \
stepper[X_AXIS]->sg_stall_value(X_AXIS, 33); \
stepper[X_AXIS]->sedn(X_AXIS, 1); \
stepper[X_AXIS]->semin(X_AXIS, 5); \
stepper[X_AXIS]->semax(X_AXIS, 2); \
stepper[X_AXIS]->toff(X_AXIS, 3); \
stepper[X_AXIS]->tbl(X_AXIS, 1); \
stepper[X_AXIS]->chopper_mode(X_AXIS, 0); \
stepper[X_AXIS]->hysteresis_start(X_AXIS, 4); \
stepper[X_AXIS]->hysteresis_end(X_AXIS, -2);

#if TRINAMIC_MIXED_DRIVERS
#define TMC_Y_ENABLE 0
#else
#define TMC_Y_ENABLE 1 // Do not change
#endif
#define TMC_Y_MONITOR 1
#define TMC_Y_MICROSTEPS 16
#define TMC_Y_R_SENSE R_SENSE   // mOhm
#define TMC_Y_CURRENT 500       // mA RMS
#define TMC_Y_HOLD_CURRENT_PCT 50
#define TMC_Y_SGT 22

#define TMC_Y_ADVANCED \
stepper[Y_AXIS]->stealthChop(Y_AXIS, 0); \
stepper[Y_AXIS]->sg_filter(Y_AXIS, 1); \
stepper[Y_AXIS]->sg_stall_value(Y_AXIS, 33); \
stepper[Y_AXIS]->sedn(Y_AXIS, 1); \
stepper[Y_AXIS]->semin(Y_AXIS, 5); \
stepper[Y_AXIS]->semax(Y_AXIS, 2); \
stepper[Y_AXIS]->toff(Y_AXIS, 3); \
stepper[Y_AXIS]->tbl(Y_AXIS, 1); \
stepper[Y_AXIS]->chopper_mode(Y_AXIS, 0); \
stepper[Y_AXIS]->hysteresis_start(Y_AXIS, 4); \
stepper[Y_AXIS]->hysteresis_end(Y_AXIS, -2);


#if TRINAMIC_MIXED_DRIVERS
#define TMC_Z_ENABLE 0
#else
#define TMC_Z_ENABLE 1 // Do not change
#endif
#define TMC_Z_MONITOR 1
#define TMC_Z_MICROSTEPS 16
#define TMC_Z_R_SENSE R_SENSE   // mOhm
#define TMC_Z_CURRENT 500       // mA RMS
#define TMC_Z_HOLD_CURRENT_PCT 50
#define TMC_Z_SGT 22

#define TMC_Z_ADVANCED \
stepper[Z_AXIS]->stealthChop(Z_AXIS, 0); \
stepper[Z_AXIS]->sg_filter(Z_AXIS, 1); \
stepper[Z_AXIS]->sg_stall_value(Z_AXIS, 33); \
stepper[Z_AXIS]->sedn(Z_AXIS, 1); \
stepper[Z_AXIS]->semin(Z_AXIS, 5); \
stepper[Z_AXIS]->semax(Z_AXIS, 2); \
stepper[Z_AXIS]->toff(Z_AXIS, 3); \
stepper[Z_AXIS]->tbl(Z_AXIS, 1); \
stepper[Z_AXIS]->chopper_mode(Z_AXIS, 0); \
stepper[Z_AXIS]->hysteresis_start(Z_AXIS, 4); \
stepper[Z_AXIS]->hysteresis_end(Z_AXIS, -2);


#ifdef A_AXIS

#if TRINAMIC_MIXED_DRIVERS
#define TMC_A_ENABLE 0
#else
#define TMC_A_ENABLE 1 // Do not change
#endif
#define TMC_A_MONITOR 1
#define TMC_A_MICROSTEPS 16
#define TMC_A_R_SENSE R_SENSE   // mOhm
#define TMC_A_CURRENT 500       // mA RMS
#define TMC_A_HOLD_CURRENT_PCT 50
#define TMC_A_SGT 22

#define TMC_A_ADVANCED \
stepper[A_AXIS]->stealthChop(A_AXIS, 0); \
stepper[A_AXIS]->sg_filter(A_AXIS, 1); \
stepper[A_AXIS]->sg_stall_value(A_AXIS, 33); \
stepper[A_AXIS]->sedn(A_AXIS, 1); \
stepper[A_AXIS]->semin(A_AXIS, 5); \
stepper[A_AXIS]->semax(A_AXIS, 2); \
stepper[A_AXIS]->toff(A_AXIS, 3); \
stepper[A_AXIS]->tbl(A_AXIS, 1); \
stepper[A_AXIS]->chopper_mode(A_AXIS, 0); \
stepper[A_AXIS]->hysteresis_start(A_AXIS, 4); \
stepper[A_AXIS]->hysteresis_end(A_AXIS, -2);

#endif

#ifdef B_AXIS

#if TRINAMIC_MIXED_DRIVERS
#define TMC_B_ENABLE 0
#else
#define TMC_B_ENABLE 1 // Do not change
#endif
#define TMC_B_MONITOR 1
#define TMC_B_MICROSTEPS 16
#define TMC_B_R_SENSE R_SENSE   // mOhm
#define TMC_B_CURRENT 500       // mA RMS
#define TMC_B_HOLD_CURRENT_PCT 50
#define TMC_B_SGT 22

#define TMC_B_ADVANCED \
stepper[B_AXIS]->stealthChop(B_AXIS, 0); \
stepper[B_AXIS]->sg_filter(B_AXIS, 1); \
stepper[B_AXIS]->sg_stall_value(B_AXIS, 33); \
stepper[B_AXIS]->sedn(B_AXIS, 1); \
stepper[B_AXIS]->semin(B_AXIS, 5); \
stepper[B_AXIS]->semax(B_AXIS, 2); \
stepper[B_AXIS]->toff(B_AXIS, 3); \
stepper[B_AXIS]->tbl(B_AXIS, 1); \
stepper[B_AXIS]->chopper_mode(B_AXIS, 0); \
stepper[B_AXIS]->hysteresis_start(B_AXIS, 4); \
stepper[B_AXIS]->hysteresis_end(B_AXIS, -2);

#endif

#ifdef C_AXIS

#if TRINAMIC_MIXED_DRIVERS
#define TMC_C_ENABLE 0
#else
#define TMC_C_ENABLE 1 // Do not change
#endif
#define TMC_C_MONITOR 1
#define TMC_C_MICROSTEPS 16
#define TMC_C_R_SENSE R_SENSE   // mOhm
#define TMC_C_CURRENT 500       // mA RMS
#define TMC_C_HOLD_CURRENT_PCT 50
#define TMC_C_SGT 22

#define TMC_C_ADVANCED \
stepper[C_AXIS]->stealthChop(C_AXIS, 0); \
stepper[C_AXIS]->sg_filter(C_AXIS, 1); \
stepper[C_AXIS]->sg_stall_value(C_AXIS, 33); \
stepper[C_AXIS]->sedn(C_AXIS, 1); \
stepper[C_AXIS]->semin(C_AXIS, 5); \
stepper[C_AXIS]->semax(C_AXIS, 2); \
stepper[C_AXIS]->toff(C_AXIS, 3); \
stepper[C_AXIS]->tbl(C_AXIS, 1); \
stepper[C_AXIS]->chopper_mode(C_AXIS, 0); \
stepper[C_AXIS]->hysteresis_start(C_AXIS, 4); \
stepper[C_AXIS]->hysteresis_end(C_AXIS, -2);

#endif

//

typedef enum {
   TMCMode_StealthChop = 0,
   TMCMode_CoolStep,
   TMCMode_StallGuard,
} trinamic_mode_t;

typedef struct {
    uint16_t current; // mA
    uint8_t hold_current_pct;
    uint16_t r_sense; // mOhm
    uint8_t microsteps;
    trinamic_mode_t mode;
    int8_t homing_sensitivity;
} motor_settings_t;

typedef struct {
    axes_signals_t driver_enable;
    axes_signals_t homing_enable;
    motor_settings_t driver[N_AXIS];
} trinamic_settings_t;

typedef void (*trinamic_on_drivers_init_ptr)(axes_signals_t enabled);

typedef struct {
    trinamic_on_drivers_init_ptr on_drivers_init;
} trinamic_driver_if_t;

bool trinamic_init (void);
void trinamic_homing (bool enable);
axes_signals_t trinamic_stepper_enable (axes_signals_t enable);
void trinamic_fault_handler (void);
void trinamic_warn_handler (void);
void trinamic_if_init (trinamic_driver_if_t *driver);

#endif // TRINAMIC_ENABLE

#endif
