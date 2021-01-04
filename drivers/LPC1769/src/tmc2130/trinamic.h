/*
  trinamic.h - Trinamic TMC2130 plugin

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

#ifndef _TMC_2130_H_
#define _TMC_2130_H_

#ifdef ARDUINO_SAMD_MKRZERO
#include "../driver.h"
#include "../trinamic/tmc2130.h"
#if TRINAMIC_I2C
#include "../trinamic/tmc2130_i2c_map.h"
#endif
#else
#include "driver.h"
#include "trinamic/tmc2130.h"
#if TRINAMIC_I2C
#include "trinamic/tmc2130_i2c_map.h"
#endif
#endif

#if TRINAMIC_ENABLE == 2130

#define tmc_write_register(axis, reg, val) { TMC2130_datagram_t *p = TMC2130_GetRegPtr(&stepper[axis], reg); p->payload.value = val; TMC2130_WriteRegister(&stepper[ axis], p); }

#define tmc_stealthChop(axis, val)      { stepper[axis].gconf.reg.en_pwm_mode = val; TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].gconf); }
#define tmc_sg_filter(axis, val)        { stepper[axis].coolconf.reg.sfilt = val; TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].coolconf); }
#define tmc_sg_stall_value(axis, val)   { stepper[axis].coolconf.reg.sgt = val; TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].coolconf); }
#define tmc_sedn(axis, val)             { stepper[axis].coolconf.reg.sedn = val; TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].coolconf); }
#define tmc_semin(axis, val)            { stepper[axis].coolconf.reg.semin = val; TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].coolconf); }
#define tmc_semax(axis, val)            { stepper[axis].coolconf.reg.semax = val; TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].coolconf); }
#define tmc_toff(axis, val)             { stepper[axis].chopconf.reg.toff = val; TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].chopconf); }
#define tmc_tbl(axis, val)              { stepper[axis].chopconf.reg.tbl = val; TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].chopconf); }
#define tmc_chopper_mode(axis, val)     { stepper[axis].chopconf.reg.chm = val; TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].chopconf); }
#define tmc_hysteresis_start(axis, val) { stepper[axis].chopconf.reg.hstrt = (uint8_t)(val - 1) & 0x07; TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].chopconf); }
#define tmc_hysteresis_end(axis, val)   { stepper[axis].chopconf.reg.hend = (uint8_t)(val + 3) & 0x0F; TMC2130_WriteRegister(&stepper[axis], (TMC2130_datagram_t *)&stepper[axis].chopconf); }

// General
#if TRINAMIC_MIXED_DRIVERS
#define TMC_X_ENABLE 0
#else
#define TMC_X_ENABLE 1 // Do not change
#endif
#define TMC_X_MONITOR 1
#define TMC_X_MICROSTEPS TMC2130_Microsteps_16
#define TMC_X_R_SENSE TMC2130_R_SENSE         // mOhm
#define TMC_X_CURRENT 500         // mA RMS
#define TMC_X_HOLD_CURRENT_PCT 50
#define TMC_X_SGT 22

#define TMC_X_ADVANCED \
tmc_stealthChop(X_AXIS, 1); \
tmc_sg_filter(X_AXIS, 1); \
tmc_sg_stall_value(X_AXIS, TMC_X_SGT); \
tmc_sedn(X_AXIS, 1); \
tmc_semin(X_AXIS, 5); \
tmc_semax(X_AXIS, 2); \
tmc_toff(X_AXIS, 3); \
tmc_tbl(X_AXIS, 1); \
tmc_chopper_mode(X_AXIS, 0); \
tmc_hysteresis_start(X_AXIS, 4); \
tmc_hysteresis_end(X_AXIS, -2);

#if TRINAMIC_MIXED_DRIVERS
#define TMC_Y_ENABLE 0
#else
#define TMC_Y_ENABLE 1 // Do not change
#endif
#define TMC_Y_MONITOR 1
#define TMC_Y_MICROSTEPS TMC2130_Microsteps_16
#define TMC_Y_R_SENSE TMC2130_R_SENSE         // mOhm
#define TMC_Y_CURRENT 500         // mA RMS
#define TMC_Y_HOLD_CURRENT_PCT 50
#define TMC_Y_SGT 22

#define TMC_Y_ADVANCED \
tmc_stealthChop(Y_AXIS, 1); \
tmc_sg_filter(Y_AXIS, 1); \
tmc_sg_stall_value(Y_AXIS, TMC_Y_SGT); \
tmc_sedn(Y_AXIS, 1); \
tmc_semin(Y_AXIS, 5); \
tmc_semax(Y_AXIS, 2); \
tmc_toff(Y_AXIS, 3); \
tmc_tbl(Y_AXIS, 1); \
tmc_chopper_mode(Y_AXIS, 0); \
tmc_hysteresis_start(Y_AXIS, 5); \
tmc_hysteresis_end(Y_AXIS, 1);

#if TRINAMIC_MIXED_DRIVERS
#define TMC_Z_ENABLE 0
#else
#define TMC_Z_ENABLE 1 // Do not change
#endif
#define TMC_Z_MONITOR 1
#define TMC_Z_MICROSTEPS TMC2130_Microsteps_16
#define TMC_Z_R_SENSE TMC2130_R_SENSE         // mOhm
#define TMC_Z_CURRENT 500         // mA RMS
#define TMC_Z_HOLD_CURRENT_PCT 50
#define TMC_Z_SGT 22

#define TMC_Z_ADVANCED \
tmc_stealthChop(Z_AXIS, 1); \
tmc_sg_filter(Z_AXIS, 1); \
tmc_sg_stall_value(Z_AXIS, TMC_Z_SGT); \
tmc_sedn(Z_AXIS, 1); \
tmc_semin(Z_AXIS, 5); \
tmc_semax(Z_AXIS, 2); \
tmc_toff(Z_AXIS, 3); \
tmc_tbl(Z_AXIS, 1); \
tmc_chopper_mode(Z_AXIS, 0); \
tmc_hysteresis_start(Z_AXIS, 5); \
tmc_hysteresis_end(Z_AXIS, 1);

#ifdef A_AXIS

#if TRINAMIC_MIXED_DRIVERS
#define TMC_A_ENABLE 0
#else
#define TMC_A_ENABLE 1 // Do not change
#endif
#define TMC_A_MONITOR 1
#define TMC_A_MICROSTEPS TMC2130_Microsteps_16
#define TMC_A_R_SENSE TMC2130_R_SENSE         // mOhm
#define TMC_A_CURRENT 500         // mA RMS
#define TMC_A_HOLD_CURRENT_PCT 50
#define TMC_A_SGT 22

#define TMC_A_ADVANCED \
tmc_stealthChop(A_AXIS, 1); \
tmc_sg_filter(A_AXIS, 1); \
tmc_sg_stall_value(A_AXIS, TMC_A_SGT); \
tmc_sedn(A_AXIS, 1); \
tmc_semin(A_AXIS, 5); \
tmc_semax(A_AXIS, 2); \
tmc_toff(A_AXIS, 3); \
tmc_tbl(A_AXIS, 1); \
tmc_chopper_mode(A_AXIS, 0); \
tmc_hysteresis_start(A_AXIS, 5); \
tmc_hysteresis_end(A_AXIS, 1);

#endif

#ifdef B_AXIS

#if TRINAMIC_MIXED_DRIVERS
#define TMC_B_ENABLE 0
#else
#define TMC_B_ENABLE 1 // Do not change
#endif
#define TMC_B_MONITOR 1
#define TMC_B_MICROSTEPS TMC2130_Microsteps_16
#define TMC_B_R_SENSE TMC2130_R_SENSE         // mOhm
#define TMC_B_CURRENT 500         // mA RMS
#define TMC_B_HOLD_CURRENT_PCT 50
#define TMC_B_SGT 22

#define TMC_B_ADVANCED \
tmc_stealthChop(B_AXIS, 1); \
tmc_sg_filter(B_AXIS, 1); \
tmc_sg_stall_value(B_AXIS, TMC_B_SGT); \
tmc_sedn(B_AXIS, 1); \
tmc_semin(B_AXIS, 5); \
tmc_semax(B_AXIS, 2); \
tmc_toff(B_AXIS, 3); \
tmc_tbl(B_AXIS, 1); \
tmc_chopper_mode(B_AXIS, 0); \
tmc_hysteresis_start(B_AXIS, 5); \
tmc_hysteresis_end(B_AXIS, 1);

#endif

#ifdef C_AXIS

#if TRINAMIC_MIXED_DRIVERS
#define TMC_C_ENABLE 0
#else
#define TMC_C_ENABLE 1 // Do not change
#endif
#define TMC_C_MONITOR 1
#define TMC_C_MICROSTEPS TMC2130_Microsteps_16
#define TMC_C_R_SENSE TMC2130_R_SENSE         // mOhm
#define TMC_C_CURRENT 500         // mA RMS
#define TMC_C_HOLD_CURRENT_PCT 50
#define TMC_C_SGT 22

#define TMC_C_ADVANCED \
tmc_stealthChop(C_AXIS, 1); \
tmc_sg_filter(C_AXIS, 1); \
tmc_sg_stall_value(C_AXIS, TMC_C_SGT); \
tmc_sedn(C_AXIS, 1); \
tmc_semin(C_AXIS, 5); \
tmc_semax(C_AXIS, 2); \
tmc_toff(C_AXIS, 3); \
tmc_tbl(C_AXIS, 1); \
tmc_chopper_mode(C_AXIS, 0); \
tmc_hysteresis_start(C_AXIS, 5); \
tmc_hysteresis_end(C_AXIS, 1);

#endif

//

typedef struct {
    uint16_t current; // mA
    uint16_t r_sense; // mOhm
    tmc2130_microsteps_t microsteps;
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
    TMC2130_interface_t interface;
} trinamic_driver_if_t;

bool trinamic_init (void);
void trinamic_homing (bool enable);
axes_signals_t trinamic_stepper_enable (axes_signals_t enable);
void trinamic_fault_handler (void);
void trinamic_warn_handler (void);
void trinamic_if_init (trinamic_driver_if_t *driver);

#endif // TRINAMIC_ENABLE

#endif
