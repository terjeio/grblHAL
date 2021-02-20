/*
 * tmc2130.h - register and message (datagram) descriptors for Trinamic TMC2130 stepper driver
 *
 * v0.0.5 / 2020-01-05 / (c) Io Engineering / Terje
 */

/*

Copyright (c) 2018-2021, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its contributors may
be used to endorse or promote products derived from this software without
specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#ifndef _TRINAMIC2130_H_
#define _TRINAMIC2130_H_

#include <stdint.h>
#include <stdbool.h>

#include "common.h"

//#define TMC2130_COMPLETE // comment out for minimum set of registers

#pragma pack(push, 1)

typedef enum {
    TMC2130_Microsteps_1 = 1,
    TMC2130_Microsteps_2 = 2,
    TMC2130_Microsteps_4 = 4,
    TMC2130_Microsteps_8 = 8,
    TMC2130_Microsteps_16 = 16,
    TMC2130_Microsteps_32 = 32,
    TMC2130_Microsteps_64 = 64,
    TMC2130_Microsteps_128 = 128,
    TMC2130_Microsteps_256 = 256
} tmc2130_microsteps_t;

// default values

// General
#define TMC2130_F_CLK 13200000UL    // typical value @ 50C for internal osc - see datasheet for calibration procedure if required
#define TMC2130_MICROSTEPS TMC2130_Microsteps_4
#define TMC2130_R_SENSE 110         // mOhm
#define TMC2130_CURRENT 500         // mA RMS
#define TMC2130_HOLD_CURRENT_PCT 50

// CHOPCONF
#define TMC2130_INTERPOLATE 1       // intpol: 0 = off, 1 = on
#define TMC2130_CONSTANT_OFF_TIME 5 // toff: 1 - 15
#define TMC2130_BLANK_TIME 1        // tbl: 0 = 16, 1 = 24, 2 = 36, 3 = 54 clocks
#define TMC2130_RANDOM_TOFF 1       // rndtf: 0 = fixed, 1 = random
#define TMC2130_CHOPPER_MODE 0      // chm: 0 = spreadCycle, 1 = constant off time
// TMC2130_CHOPPER_MODE 0 defaults
#define TMC2130_HSTRT 3             // hstrt: 0 � 7
#define TMC2130_HEND 2              // hend: -3 � 12
// TMC2130_CHOPPER_MODE 1 defaults
#define TMC2130_FAST_DECAY_TIME 13  // fd3 & hstrt: 0 - 15
#define TMC2130_SINE_WAVE_OFFSET 2  // hend: -3 � 12

// IHOLD_IRUN
#define TMC2130_IRUN 31             // max. current
#define TMC2130_IHOLD ((TMC2130_IRUN * TMC2130_HOLD_CURRENT_PCT) / 100)
#define TMC2130_IHOLDDELAY 6

// TPOWERDOWN
#define TMC2130_TPOWERDOWN 128      // 0 � ((2^8)-1) * 2^18 tCLK

// EN_PWM_MODE
#define TMC2130_EN_PWM_MODE 1       // en_pwm_ mode: 0 = stealthChop off, 1 = stealthChop on

// TPWMTHRS
#define TMC2130_TPWM_THRS 0         // tpwmthrs: 0 � 2^20 - 1 (20 bits)

// PWM_CONF
#define TMC2130_PWM_AUTOSCALE 1     // pwm_autoscale: 0 = forward controlled mode, 1 = automatic scaling
#define TMC2130_PWM_FREQ 1          // pwm_freq: 0 = 1/1024, 1 = 2/683, 2 = 2/512, 3 = 2/410 fCLK
#define TMC2130_PWM_AMPL 255        // pwm_ampl: 0 � 255
#define TMC2130_PWM_GRAD 5          // pwm_autoscale = 1: 1 � 15, pwm_autoscale = 0: 0 � 255

// COOLCONF
#define TMC2130_COOLSTEP_ENABLE 0
// TMC2130_COOLSTEP_ENABLE = 1 defaults
#define TMC2130_COOLSTEP_SEMIN 1    // semin: 0 = coolStep off, 1 � 15 = coolStep on
#define TMC2130_COOLSTEP_SEMAX 1    // semax: 0 � 15

// end of default values

typedef enum {
    TMC2130Reg_GCONF = 0x00,
    TMC2130Reg_GSTAT = 0x01,
    TMC2130Reg_IOIN = 0x04,
    TMC2130Reg_IHOLD_IRUN = 0x10,
    TMC2130Reg_TPOWERDOWN = 0x11,
    TMC2130Reg_TSTEP = 0x12,
    TMC2130Reg_TPWMTHRS = 0x13,
    TMC2130Reg_TCOOLTHRS = 0x14,
    TMC2130Reg_THIGH = 0x15,
    TMC2130Reg_XDIRECT = 0x2D,
    TMC2130Reg_VDCMIN = 0x33,
    TMC2130Reg_MSLUT_BASE = 0x60,
    TMC2130Reg_MSLUTSEL = 0x68,
    TMC2130Reg_MSLUTSTART = 0x69,
    TMC2130Reg_MSCNT = 0x6A,
    TMC2130Reg_MSCURACT = 0x6B,
    TMC2130Reg_CHOPCONF = 0x6C,
    TMC2130Reg_COOLCONF = 0x6D,
    TMC2130Reg_DCCTRL = 0x6E,
    TMC2130Reg_DRV_STATUS = 0x6F,
    TMC2130Reg_PWMCONF = 0x70,
    TMC2130Reg_PWM_SCALE = 0x71,
    TMC2130Reg_ENCM_CTRL = 0x72,
    TMC2130Reg_LOST_STEPS = 0x73,
} tmc2130_regaddr_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t
        reset_flag   :1,
        driver_error :1,
        sg2          :1,
        standstill   :1,
        unused       :4;
    };
} TMC2130_status_t;

// --- register definitions ---

// GCONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        I_scale_analog      :1,
        internal_Rsense     :1,
        en_pwm_mode         :1,
        enc_commutation     :1,
        shaft               :1,
        diag0_error         :1,
        diag0_otpw          :1,
        diag0_stall         :1,
        diag1_stall         :1,
        diag1_index         :1,
        diag1_onstate       :1,
        diag1_steps_skipped :1,
        diag0_int_pushpull  :1,
        diag1_pushpull      :1,
        small_hysteresis    :1,
        stop_enable         :1,
        direct_mode         :1,
        test_mode           :1,
        reserved            :14;
    };
} TMC2130_gconf_reg_t;

// GSTAT : R+C
typedef union {
    uint32_t value;
    struct {
        uint32_t
        reset    :1,
        drv_err  :1,
        uv_cp    :1,
        reserved :29;
    };
} TMC2130_gstat_reg_t;

// IOIN : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        step         :1,
        dir          :1,
        dcen_cfg4    :1,
        dcen_cfg5    :1,
        drv_enn_cfg6 :1,
        dco          :1,
        always_1     :1,
        dont_care    :1,
        reserved     :16,
        version      :8;
    };
} TMC2130_ioin_reg_t;

// IHOLD_IRUN : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        ihold      :5,
        reserved1  :3,
        irun       :5,
        reserved2  :3,
        iholddelay :4,
        reserved3  :12;
    };
} TMC2130_ihold_irun_reg_t;

// TPOWERDOWN : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tpowerdown :8,
        reserved   :24;
    };
} TMC2130_tpowerdown_reg_t;

// TSTEP : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tstep    :20,
        reserved :12;
    };
} TMC2130_tstep_reg_t;

// TPWMTHRS : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tpwmthrs :20,
        reserved :12;
    };
} TMC2130_tpwmthrs_reg_t;

// TCOOLTHRS : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tcoolthrs :20,
        reserved  :12;
    };
} TMC2130_tcoolthrs_reg_t;

// THIGH : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        thigh    :20,
        reserved :12;
    };
} TMC2130_thigh_reg_t;

// XDIRECT : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        direct_mode :1,
        reserved    :31;
    };
    struct {
        uint32_t
        coil_a_current :8,
        reserved1      :8,
        coil_b_current :8,
        reserved2      :8;
    };
} TMC2130_xdirect_reg_t;

// VDCMIN : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        vdcmin   :23,
        reserved :9;
    };
} TMC2130_vdcmin_reg_t;

// MSLUTn : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        mte :32;
    };
} TMC2130_mslut_n_reg_t;

// MSLUTSEL : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        w0 :2,
        w1 :2,
        w2 :2,
        w3 :2,
        x1 :8,
        x2 :8,
        x3 :8;
    };
} TMC2130_mslutsel_reg_t;

// MSLUTSTART : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        start_sin   :8,
        reserved1   :8,
        start_sin90 :8,
        reserved2   :8;
    };
} TMC2130_mslutstart_reg_t;

// MSCNT : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        mscnt    :10,
        reserved :22;
    };
} TMC2130_mscnt_reg_t;

// MSCURACT : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        cur_a     :9,
        reserved1 :7,
        cur_b     :9,
        reserved2 :7;
    };
} TMC2130_mscuract_reg_t;

// DCCTRL : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        dc_time   :9,
        reserved1 :7,
        dc_sg     :8,
        reserved2 :8;
    };
} TMC2130_dcctrl_reg_t;

// CHOPCONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        toff     :4,
        hstrt    :3,
        hend     :4,
        fd3      :1,
        disfdcc  :1,
        rndtf    :1,
        chm      :1,
        tbl      :2,
        vsense   :1,
        vhighfs  :1,
        vhighchm :1,
        sync     :4,
        mres     :4,
        intpol   :1,
        dedge    :1,
        diss2g   :1,
        reserved :1;
    };
} TMC2130_chopconf_reg_t;

// DRV_STATUS : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        sg_result  :10,
        reserved1  :5,
        fsactive   :1,
        cs_actual  :5,
        reserved   :3,
        stallGuard :1,
        ot         :1,
        otpw       :1,
        s2ga       :1,
        s2gb       :1,
        ola        :1,
        olb        :1,
        stst       :1;
    };
} TMC2130_drv_status_reg_t;

// COOLCONF : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        semin     :4,
        reserved1 :1,
        seup      :2,
        reserved2 :1,
        semax     :4,
        reserved3 :1,
        sedn      :2,
        seimin    :1,
        sgt       :7,
        reserved4 :1,
        sfilt     :1,
        reserved5 :7;
    };
} TMC2130_coolconf_reg_t;

// PWMCONF : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_ampl      :8,
        pwm_grad      :8,
        pwm_freq      :2,
        pwm_autoscale :1,
        pwm_symmetric :1,
        freewheel     :2,
        reserved      :10;
    };
} TMC2130_pwmconf_reg_t;

// PWM_SCALE : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_scale :8,
        reserved  :24;
    };
} TMC2130_pwm_scale_reg_t;

// ENCM_CTRL : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        inv :1,
        maxspeed :1,
        reserved :30;
    };
} TMC2130_encm_ctrl_reg_t;

// LOST_STEPS : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        lost_steps :20,
        reserved   :12;
    };
} TMC2130_lost_steps_reg_t;

// --- end of register definitions ---

typedef union {
    tmc2130_regaddr_t reg;
    uint8_t value;
    struct {
        uint8_t
        idx   :7,
        write :1;
    };
} TMC2130_addr_t;

// --- datagrams ---

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_gconf_reg_t reg;
} TMC2130_gconf_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_gstat_reg_t reg;
} TMC2130_stat_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_ioin_reg_t reg;
} TMC2130_ioin_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_ihold_irun_reg_t reg;
} TMC2130_ihold_irun_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_tpowerdown_reg_t reg;
} TMC2130_tpowerdown_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_tcoolthrs_reg_t reg;
} TMC2130_tcoolthrs_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_mslutstart_reg_t reg;
} TMC2130_mslutstart_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_tstep_reg_t reg;
} TMC2130_tstep_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_tpwmthrs_reg_t reg;
} TMC2130_tpwmthrs_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_thigh_reg_t reg;
} TMC2130_thigh_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_xdirect_reg_t reg;
} TMC2130_xdirect_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_vdcmin_reg_t reg;
} TMC2130_vdcmin_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_mslut_n_reg_t reg;
} TMC2130_mslut_n_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_mslutsel_reg_t reg;
} TMC2130_mslutsel_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_mscnt_reg_t reg;
} TMC2130_mscnt_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_mscuract_reg_t reg;
} TMC2130_mscuract_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_dcctrl_reg_t reg;
} TMC2130_dcctrl_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_chopconf_reg_t reg;
} TMC2130_chopconf_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_drv_status_reg_t reg;
} TMC2130_drv_status_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_coolconf_reg_t reg;
} TMC2130_coolconf_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_pwmconf_reg_t reg;
} TMC2130_pwmconf_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_pwm_scale_reg_t reg;
} TMC2130_pwm_scale_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_encm_ctrl_reg_t reg;
} TMC2130_encm_ctrl_dgr_t;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_lost_steps_reg_t reg;
} TMC2130_lost_steps_dgr_t;

// -- end of datagrams

typedef union {
    uint32_t value;
    uint8_t data[4];
    TMC2130_gconf_reg_t gconf;
    TMC2130_gstat_reg_t gstat;
    TMC2130_ioin_reg_t ioin;
    TMC2130_ihold_irun_reg_t ihold_irun;
    TMC2130_tpowerdown_reg_t tpowerdown;
    TMC2130_tstep_reg_t tstep;
    TMC2130_tpwmthrs_reg_t tpwmthrs;
    TMC2130_tcoolthrs_reg_t tcoolthrs;
    TMC2130_thigh_reg_t thigh;
    TMC2130_xdirect_reg_t xdirect;
    TMC2130_vdcmin_reg_t vdcmin;
    TMC2130_mslut_n_reg_t mslut;
    TMC2130_mslutsel_reg_t mslutsel;
    TMC2130_mslutstart_reg_t mslutstart;
    TMC2130_mscnt_reg_t mscnt;
    TMC2130_mscuract_reg_t mscuract;
    TMC2130_dcctrl_reg_t dcctrl;
    TMC2130_drv_status_reg_t drv_status;
    TMC2130_chopconf_reg_t chopconf;
    TMC2130_coolconf_reg_t coolconf;
    TMC2130_pwmconf_reg_t pwmconf;
    TMC2130_pwm_scale_reg_t pwm_scale;
    TMC2130_encm_ctrl_reg_t encm_ctrl;
    TMC2130_lost_steps_reg_t lost_steps;
} TMC2130_payload;

typedef struct {
    TMC2130_addr_t addr;
    TMC2130_payload payload;
} TMC2130_datagram_t;

typedef struct {
    // driver registers
    TMC2130_gconf_dgr_t gconf;
    TMC2130_stat_dgr_t gstat;
    TMC2130_ioin_dgr_t ioin;
    TMC2130_ihold_irun_dgr_t ihold_irun;
    TMC2130_tpowerdown_dgr_t tpowerdown;
    TMC2130_tstep_dgr_t tstep;
    TMC2130_tpwmthrs_dgr_t tpwmthrs;
    TMC2130_tcoolthrs_dgr_t tcoolthrs;
    TMC2130_thigh_dgr_t thigh;
    TMC2130_vdcmin_dgr_t vdcmin;
#ifdef TMC2130_COMPLETE
    TMC2130_xdirect_dgr_t xdirect;
    TMC2130_mslut_n_dgr_t mslut[8];
    TMC2130_mslutsel_dgr_t mslutsel;
    TMC2130_mslutstart_dgr_t mslutstart;
    TMC2130_encm_ctrl_dgr_t encm_ctrl;
#endif
    TMC2130_mscnt_dgr_t mscnt;
    TMC2130_mscuract_dgr_t mscuract;
    TMC2130_dcctrl_dgr_t dcctrl;
    TMC2130_drv_status_dgr_t drv_status;
    TMC2130_chopconf_dgr_t chopconf;
    TMC2130_coolconf_dgr_t coolconf;
    TMC2130_pwmconf_dgr_t pwmconf;
    TMC2130_pwm_scale_dgr_t pwm_scale;
    TMC2130_lost_steps_dgr_t lost_steps;
    TMC2130_status_t driver_status;

    trinamic_motor_t motor;
    trinamic_config_t config;
} TMC2130_t;

#pragma pack(pop)

bool TMC2130_Init(TMC2130_t *driver);
void TMC2130_SetDefaults (TMC2130_t *driver);
void TMC2130_SetCurrent (TMC2130_t *driver, uint16_t mA, uint8_t hold_pct);
uint16_t TMC2130_GetCurrent (TMC2130_t *driver);
uint32_t TMC2130_GetTPWMTHRS (TMC2130_t *driver, float stpmm);
bool TMC2130_MicrostepsIsValid (uint16_t usteps);
void TMC2130_SetMicrosteps(TMC2130_t *driver, tmc2130_microsteps_t usteps);
void TMC2130_SetHybridThreshold (TMC2130_t *driver, uint32_t threshold, float steps_mm);
void TMC2130_SetTHIGH (TMC2130_t *driver, float mm_sec, float steps_mm);
void TMC2130_SetTCOOLTHRS (TMC2130_t *driver, float mm_sec, float steps_mm);
void TMC2130_SetConstantOffTimeChopper(TMC2130_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator);
TMC2130_datagram_t *TMC2130_GetRegPtr (TMC2130_t *driver, tmc2130_regaddr_t reg);
TMC2130_status_t TMC2130_WriteRegister (TMC2130_t *driver, TMC2130_datagram_t *reg);
TMC2130_status_t TMC2130_ReadRegister (TMC2130_t *driver, TMC2130_datagram_t *reg);

#endif
