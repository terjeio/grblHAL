/*
 * tmc5160.h - register and message (datagram) descriptors for Trinamic TMC5160 stepper driver
 *
 * v0.0.1 / 2021-01-04 / (c) Io Engineering / Terje
 */

/*

Copyright (c) 2021, Terje Io
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

#ifndef _TRINAMIC5160_H_
#define _TRINAMIC5160_H_

#include "common.h"

//#define TMC5160_COMPLETE // comment out for minimum set of registers

#pragma pack(push, 1)

typedef enum {
    TMC5160_Microsteps_1 = 1,
    TMC5160_Microsteps_2 = 2,
    TMC5160_Microsteps_4 = 4,
    TMC5160_Microsteps_8 = 8,
    TMC5160_Microsteps_16 = 16,
    TMC5160_Microsteps_32 = 32,
    TMC5160_Microsteps_64 = 64,
    TMC5160_Microsteps_128 = 128,
    TMC5160_Microsteps_256 = 256
} tmc5160_microsteps_t;

// default values

// General
#define TMC5160_F_CLK 12000000UL    // typical value @ 50C for internal osc - see datasheet for calibration procedure if required
#define TMC5160_MICROSTEPS TMC5160_Microsteps_4
#define TMC5160_R_SENSE 75          // mOhm
#define TMC5160_CURRENT 500         // mA RMS
#define TMC5160_HOLD_CURRENT_PCT 50

// CHOPCONF
#define TMC5160_INTERPOLATE 1       // intpol: 0 = off, 1 = on
#define TMC5160_CONSTANT_OFF_TIME 5 // toff: 1 - 15
#define TMC5160_BLANK_TIME 1        // tbl: 0 = 16, 1 = 24, 2 = 36, 3 = 54 clocks
#define TMC5160_CHOPPER_MODE 0      // chm: 0 = spreadCycle, 1 = constant off time
// TMC5160_CHOPPER_MODE 0 defaults
#define TMC5160_HSTRT 3             // hstrt: 0 � 7
#define TMC5160_HEND 2              // hend: -3 � 12
// TMC5160_CHOPPER_MODE 1 defaults
#define TMC5160_FAST_DECAY_TIME 13  // fd3 & hstrt: 0 - 15
#define TMC5160_SINE_WAVE_OFFSET 2  // hend: -3 � 12

// IHOLD_IRUN
#define TMC5160_IRUN 31             // max. current
#define TMC5160_IHOLD ((TMC5160_IRUN * TMC5160_HOLD_CURRENT_PCT) / 100)
#define TMC5160_IHOLDDELAY 6

// TPOWERDOWN
#define TMC5160_TPOWERDOWN 128      // 0 � ((2^8)-1) * 2^18 tCLK

// EN_PWM_MODE
#define TMC5160_EN_PWM_MODE 1       // en_pwm_ mode: 0 = stealthChop off, 1 = stealthChop on

// TPWMTHRS
#define TMC5160_TPWM_THRS 0         // tpwmthrs: 0 � 2^20 - 1 (20 bits)

// COOLCONF
#define TMC5160_COOLSTEP_ENABLE 0
// TMC5160_COOLSTEP_ENABLE = 1 defaults
#define TMC5160_COOLSTEP_SEMIN 1    // semin: 0 = coolStep off, 1 � 15 = coolStep on
#define TMC5160_COOLSTEP_SEMAX 1    // semax: 0 � 15

// end of default values

typedef enum {
    TMC5160Reg_GCONF = 0x00,
    TMC5160Reg_GSTAT = 0x01,
    TMC5160Reg_IFCNT = 0x02,
    TMC5160Reg_SLAVECONF = 0x03,
    TMC5160Reg_IOIN = 0x04,

    TMC5160Reg_OUTPUT = 0x05,
    TMC5160Reg_X_COMPARE = 0x06,
    TMC5160Reg_OTP_READ = 0x07,
    TMC5160Reg_FACTORY_CONF = 0x08,
    TMC5160Reg_SHORT_CONF = 0x09,
    TMC5160Reg_DRV_CONF = 0x0A,
    TMC5160Reg_GLOBAL_SCALER = 0x0B,
    TMC5160Reg_OFFSET_READ = 0x0C,

    TMC5160Reg_IHOLD_IRUN = 0x10,
    TMC5160Reg_TPOWERDOWN = 0x11,
    TMC5160Reg_TSTEP = 0x12,
    TMC5160Reg_TPWMTHRS = 0x13,
    TMC5160Reg_TCOOLTHRS = 0x14,
    TMC5160Reg_THIGH = 0x15,

    TMC5160Reg_XDIRECT = 0x2D,

    TMC5160Reg_VDCMIN = 0x33,
    TMC5160Reg_SW_MODE = 0x34,
    TMC5160Reg_RAMP_STAT = 0x35,
    TMC5160Reg_XLATCH = 0x36,


    TMC5160Reg_MSLUT_BASE = 0x60,
    TMC5160Reg_MSLUTSEL = 0x68,
    TMC5160Reg_MSLUTSTART = 0x69,
    TMC5160Reg_MSCNT = 0x6A,
    TMC5160Reg_MSCURACT = 0x6B,
    TMC5160Reg_CHOPCONF = 0x6C,
    TMC5160Reg_COOLCONF = 0x6D,
    TMC5160Reg_DCCTRL = 0x6E,
    TMC5160Reg_DRV_STATUS = 0x6F,
    TMC5160Reg_PWMCONF = 0x70,
    TMC5160Reg_PWM_SCALE = 0x71,
    TMC5160Reg_PWM_AUTO = 0x72,
    TMC5160Reg_LOST_STEPS = 0x73,
} tmc5160_regaddr_t;

typedef union {
    uint8_t value;
    struct {
        uint8_t
        reset_flag       :1,
        driver_error     :1,
        sg2              :1,
        standstill       :1,
        velocity_reached :1,
        position_reached :1,
        status_stop_l    :1,
        status_stop_r    :1;
    };
} TMC5160_status_t;

// --- register definitions ---

// GCONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        recalibrate            :1,
        faststandstill         :1,
        en_pwm_mode            :1,
        multistep_filt         :1,
        shaft                  :1,
        diag0_error            :1,
        diag0_otpw             :1,
        diag0_stall            :1,
        diag1_stall            :1,
        diag1_index            :1,
        diag1_onstate          :1,
        diag1_steps_skipped    :1,
        diag0_int_pushpull     :1,
        diag1_poscomp_pushpull :1,
        small_hysteresis       :1,
        stop_enable            :1,
        direct_mode            :1,
        test_mode              :1,
        reserved               :14;
    };
} TMC5160_gconf_reg_t;

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
} TMC5160_gstat_reg_t;

// IFCNT : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        count    :8,
        reserved :24;
    };
} TMC5160_ifcnt_reg_t;

// SLAVECONF : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        slaveaddr :8,
        senddelay :4,
        reserved1 :20;
    };
} TMC5160_slaveconf_reg_t;

// IOIN : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        refl_step          :1,
        refl_dir           :1,
        encb_dcen_cfg4     :1,
        encb_dcen_cfg5     :1,
        drv_enn            :1,
        enc_n_dco_dco_cfg6 :1,
        sd_mode            :1,
        swcomp_in          :1,
        reserved           :16,
        version            :8;
    };
} TMC5160_ioin_reg_t;

// OUTPUT : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        io_polarity :1,
        version     :31;
    };
} TMC5160_output_reg_t;

// X_COMPARE : W
typedef union {
    uint32_t value;
    struct {
        uint32_t x_compare;
    };
} TMC5160_x_compare_reg_t;

// OTP_PROG : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        otpbit    :3,
        otpbyte   :2,
        reserved1 :3,
        otpmagic  :8,
        reserved2 :16;
    };
} TMC5160_otp_prog_reg_t;

// OTP_READ : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        otp0_0_4 :5,
        otp0_5   :1,
        otp0_6   :1,
        otp0_7   :1,
        reserved :24;
    };
} TMC5160_otp_read_reg_t;

// FACTORY_CONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        fclktrim :5,
        reserved :27;
    };
} TMC5160_factory_conf_reg_t;

// SHORT_CONF : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        s2vs_level  :4,
        reserved1   :4,
        s2g_level   :4,
        reserved2   :4,
        shortfilter :2,
        shortdelay  :1,
        reserved3   :13;
    };
} TMC5160_short_conf_reg_t;

// DRV_CONF : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        bbmtime     :5,
        reserved1   :3,
        bbmclks     :4,
        reserved2   :4,
        otselect    :2,
        drvstrenght :2,
        filt_isense :2,
        reserved3   :10;
    };
} TMC5160_drv_conf_reg_t;

// GLOBAL_SCALER : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        scaler   :8,
        reserved :24;
    };
} TMC5160_global_scaler_reg_t;

// OFFSET_READ : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        phase_a  :8,
        phase_b  :8,
        reserved :20;
    };
} TMC5160_offset_read_reg_t;

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
} TMC5160_ihold_irun_reg_t;

// TPOWERDOWN : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tpowerdown :8,
        reserved   :24;
    };
} TMC5160_tpowerdown_reg_t;

// TSTEP : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tstep    :20,
        reserved :12;
    };
} TMC5160_tstep_reg_t;

// TPWMTHRS : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tpwmthrs :20,
        reserved :12;
    };
} TMC5160_tpwmthrs_reg_t;

// TCOOLTHRS : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tcoolthrs :20,
        reserved  :12;
    };
} TMC5160_tcoolthrs_reg_t;

// THIGH : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        thigh    :20,
        reserved :12;
    };
} TMC5160_thigh_reg_t;

// VDCMIN : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        vdcmin   :23,
        reserved :9;
    };
} TMC5160_vdcmin_reg_t;

// MSLUTn : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        mte :32;
    };
} TMC5160_mslut_n_reg_t;

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
} TMC5160_mslutsel_reg_t;

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
} TMC5160_mslutstart_reg_t;

//??MSLUTSEL

// MSCNT : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        mscnt    :10,
        reserved :22;
    };
} TMC5160_mscnt_reg_t;

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
} TMC5160_mscuract_reg_t;

// CHOPCONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        toff      :4,
        hstrt     :3,
        hend      :4,
        fd3       :1,
        disfdcc   :1,
        reserved1 :1,
        chm       :1,
        tbl       :2,
        reserved2 :1,
        vhighfs   :1,
        vhighchm  :1,
        tpfd      :4,
        mres      :4,
        intpol    :1,
        dedge     :1,
        diss2g    :1,
        diss2vs   :1;
    };
} TMC5160_chopconf_reg_t;

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
} TMC5160_coolconf_reg_t;

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
} TMC5160_dcctrl_reg_t;

// DRV_STATUS : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        sg_result  :10,
        reserved1  :2,
        s2vsa      :1,
        s2vsb      :1,
        stealth    :1,
        fsactive   :1,
        cs_actual  :5,
        reserved2  :3,
        stallguard :1,
        ot         :1,
        otpw       :1,
        s2ga       :1,
        s2gb       :1,
        ola        :1,
        olb        :1,
        stst       :1;
    };
} TMC5160_drv_status_reg_t;

// PWMCONF : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_ofs       :8,
        pwm_grad      :8,
        pwm_freq      :2,
        pwm_autoscale :1,
        pwm_autograd  :1,
        freewheel     :2,
        reserved      :2,
        pwm_reg       :4,
        pwm_lim       :4;
    };
} TMC5160_pwmconf_reg_t;

// PWM_SCALE : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_scale_sum  :8,
        reserved1      :8,
        pwm_scale_auto :9,
        reserved2      :7;
    };
} TMC5160_pwm_scale_reg_t;

// PWM_AUTO : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_ofs_auto   :8,
        reserved1      :8,
        pwm_grad_auto  :8,
        reserved2      :8;
    };
} TMC5160_pwm_auto_reg_t;

// LOST_STEPS : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        lost_steps :20,
        reserved   :12;
    };
} TMC5160_lost_steps_reg_t;

// --- end of register definitions ---

typedef union {
    tmc5160_regaddr_t reg;
    uint8_t value;
    struct {
        uint8_t
        idx   :7,
        write :1;
    };
} TMC5160_addr_t;

// --- datagrams ---

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_gconf_reg_t reg;
} TMC5160_gconf_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_gstat_reg_t reg;
} TMC5160_stat_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_ioin_reg_t reg;
} TMC5160_ioin_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_global_scaler_reg_t reg;
} TMC5160_global_scaler_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_ihold_irun_reg_t reg;
} TMC5160_ihold_irun_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_tpowerdown_reg_t reg;
} TMC5160_tpowerdown_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_tcoolthrs_reg_t reg;
} TMC5160_tcoolthrs_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_mslutstart_reg_t reg;
} TMC5160_mslutstart_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_tstep_reg_t reg;
} TMC5160_tstep_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_tpwmthrs_reg_t reg;
} TMC5160_tpwmthrs_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_thigh_reg_t reg;
} TMC5160_thigh_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_vdcmin_reg_t reg;
} TMC5160_vdcmin_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_mslut_n_reg_t reg;
} TMC5160_mslut_n_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_mslutsel_reg_t reg;
} TMC5160_mslutsel_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_mscnt_reg_t reg;
} TMC5160_mscnt_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_mscuract_reg_t reg;
} TMC5160_mscuract_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_dcctrl_reg_t reg;
} TMC5160_dcctrl_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_chopconf_reg_t reg;
} TMC5160_chopconf_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_drv_status_reg_t reg;
} TMC5160_drv_status_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_coolconf_reg_t reg;
} TMC5160_coolconf_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_pwmconf_reg_t reg;
} TMC5160_pwmconf_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_pwm_scale_reg_t reg;
} TMC5160_pwm_scale_dgr_t;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_lost_steps_reg_t reg;
} TMC5160_lost_steps_dgr_t;

// -- end of datagrams

typedef union {
//    uint32_t value;
    uint8_t data[4];
    TMC5160_gconf_reg_t gconf;
    TMC5160_gstat_reg_t gstat;
    TMC5160_ioin_reg_t ioin;
    TMC5160_global_scaler_dgr_t global_scaler;
    TMC5160_ihold_irun_reg_t ihold_irun;
    TMC5160_tpowerdown_reg_t tpowerdown;
    TMC5160_tstep_reg_t tstep;
    TMC5160_tpwmthrs_reg_t tpwmthrs;
    TMC5160_tcoolthrs_reg_t tcoolthrs;
    TMC5160_thigh_reg_t thigh;
    TMC5160_vdcmin_reg_t vdcmin;
    TMC5160_mslut_n_reg_t mslut;
    TMC5160_mslutsel_reg_t mslutsel;
    TMC5160_mslutstart_reg_t mslutstart;
    TMC5160_mscnt_reg_t mscnt;
    TMC5160_mscuract_reg_t mscuract;
    TMC5160_dcctrl_reg_t dcctrl;
    TMC5160_drv_status_reg_t drv_status;
    TMC5160_chopconf_reg_t chopconf;
    TMC5160_coolconf_reg_t coolconf;
    TMC5160_pwmconf_reg_t pwmconf;
    TMC5160_pwm_scale_reg_t pwm_scale;
    TMC5160_lost_steps_reg_t lost_steps;
} TMC5160_payload;

typedef struct {
    TMC5160_addr_t addr;
    TMC5160_payload payload;
} TMC5160_datagram_t;

typedef struct {
    // driver registers
    TMC5160_gconf_dgr_t gconf;
    TMC5160_stat_dgr_t gstat;
    TMC5160_ioin_dgr_t ioin;
    TMC5160_global_scaler_dgr_t global_scaler;
    TMC5160_ihold_irun_dgr_t ihold_irun;
    TMC5160_tpowerdown_dgr_t tpowerdown;
    TMC5160_tstep_dgr_t tstep;
    TMC5160_tpwmthrs_dgr_t tpwmthrs;
    TMC5160_tcoolthrs_dgr_t tcoolthrs;
    TMC5160_thigh_dgr_t thigh;
    TMC5160_vdcmin_dgr_t vdcmin;
#ifdef TMC5160_COMPLETE
    TMC5160_xdirect_dgr_t xdirect;
    TMC5160_mslut_n_dgr_t mslut[8];
    TMC5160_mslutsel_dgr_t mslutsel;
    TMC5160_mslutstart_dgr_t mslutstart;
    TMC5160_encm_ctrl_dgr_t encm_ctrl;
#endif
    TMC5160_mscnt_dgr_t mscnt;
    TMC5160_mscuract_dgr_t mscuract;
    TMC5160_dcctrl_dgr_t dcctrl;
    TMC5160_drv_status_dgr_t drv_status;
    TMC5160_chopconf_dgr_t chopconf;
    TMC5160_coolconf_dgr_t coolconf;
    TMC5160_pwmconf_dgr_t pwmconf;
    TMC5160_pwm_scale_dgr_t pwm_scale;
    TMC5160_lost_steps_dgr_t lost_steps;
    TMC5160_status_t driver_status;

    trinamic_motor_t motor;
    trinamic_config_t config;
} TMC5160_t;

#pragma pack(pop)

bool TMC5160_Init(TMC5160_t *driver);
void TMC5160_SetDefaults (TMC5160_t *driver);
void TMC5160_SetCurrent (TMC5160_t *driver, uint16_t mA, uint8_t hold_pct);
uint16_t TMC5160_GetCurrent (TMC5160_t *driver);
uint32_t TMC5160_GetTPWMTHRS (TMC5160_t *driver, float steps_mm);
bool TMC5160_MicrostepsIsValid (uint16_t usteps);
void TMC5160_SetMicrosteps(TMC5160_t *driver, tmc5160_microsteps_t usteps);
void TMC5160_SetHybridThreshold (TMC5160_t *driver, float mm_sec, float steps_mm);
void TMC5160_SetTHIGH (TMC5160_t *driver, float mm_sec, float steps_mm);
void TMC5160_SetTCOOLTHRS (TMC5160_t *driver, float mm_sec, float steps_mm);

void TMC5160_SetConstantOffTimeChopper(TMC5160_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator);
TMC5160_datagram_t *TMC5160_GetRegPtr (TMC5160_t *driver, tmc5160_regaddr_t reg);
TMC5160_status_t TMC5160_WriteRegister (TMC5160_t *driver, TMC5160_datagram_t *reg);
TMC5160_status_t TMC5160_ReadRegister (TMC5160_t *driver, TMC5160_datagram_t *reg);

#endif
