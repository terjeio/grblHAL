/*
 * tmc2209.h - register and message (datagram) descriptors for Trinamic TMC2209 stepper driver
 *
 * v0.0.2 / 2020-12-26 / (c) Io Engineering / Terje
 */

/*

Copyright (c) 2020, Terje Io
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

#ifndef _TRINAMIC2209_H_
#define _TRINAMIC2209_H_

#include <stdint.h>
#include <stdbool.h>

#include "common.h"

//#define TMC2209_COMPLETE // comment out for minimum set of registers

#pragma pack(push, 1)

typedef enum {
    TMC2209_Microsteps_1 = 1,
    TMC2209_Microsteps_2 = 2,
    TMC2209_Microsteps_4 = 4,
    TMC2209_Microsteps_8 = 8,
    TMC2209_Microsteps_16 = 16,
    TMC2209_Microsteps_32 = 32,
    TMC2209_Microsteps_64 = 64,
    TMC2209_Microsteps_128 = 128,
    TMC2209_Microsteps_256 = 256
} tmc2209_microsteps_t;

// default values

// General
#define TMC2209_F_CLK 13200000UL    // typical value @ 50C for internal osc - see datasheet for calibration procedure if required
#define TMC2209_MICROSTEPS TMC2209_Microsteps_4
#define TMC2209_R_SENSE 110         // mOhm
#define TMC2209_CURRENT 500         // mA RMS
#define TMC2209_HOLD_CURRENT_PCT 50

// CHOPCONF
#define TMC2209_INTERPOLATE 1       // intpol: 0 = off, 1 = on
#define TMC2209_CONSTANT_OFF_TIME 5 // toff: 1 - 15
#define TMC2209_BLANK_TIME 1        // tbl: 0 = 16, 1 = 24, 2 = 36, 3 = 54 clocks
#define TMC2209_RANDOM_TOFF 1       // rndtf: 0 = fixed, 1 = random
#define TMC2209_CHOPPER_MODE 0      // chm: 0 = spreadCycle, 1 = constant off time
// TMC2209_CHOPPER_MODE 0 defaults
#define TMC2209_HSTRT 3             // hstrt: 0 � 7
#define TMC2209_HEND 2              // hend: -3 � 12
// TMC2209_CHOPPER_MODE 1 defaults
#define TMC2209_FAST_DECAY_TIME 13  // fd3 & hstrt: 0 - 15
#define TMC2209_SINE_WAVE_OFFSET 2  // hend: -3 � 12

// IHOLD_IRUN
#define TMC2209_IRUN 31             // max. current
#define TMC2209_IHOLD ((TMC2209_IRUN * TMC2209_HOLD_CURRENT_PCT) / 100)
#define TMC2209_IHOLDDELAY 6

// TPOWERDOWN
#define TMC2209_TPOWERDOWN 128      // 0 � ((2^8)-1) * 2^18 tCLK

// EN_PWM_MODE
#define TMC2209_EN_PWM_MODE 1       // en_pwm_ mode: 0 = stealthChop off, 1 = stealthChop on

// TPWMTHRS
#define TMC2209_TPWM_THRS 0         // tpwmthrs: 0 � 2^20 - 1 (20 bits)

// PWM_CONF
#define TMC2209_PWM_AUTOSCALE 1     // pwm_autoscale: 0 = forward controlled mode, 1 = automatic scaling
#define TMC2209_PWM_FREQ 1          // pwm_freq: 0 = 1/1024, 1 = 2/683, 2 = 2/512, 3 = 2/410 fCLK
#define TMC2209_PWM_AMPL 255        // pwm_ampl: 0 � 255
#define TMC2209_PWM_GRAD 5          // pwm_autoscale = 1: 1 � 15, pwm_autoscale = 0: 0 � 255

// COOLCONF
#define TMC2209_COOLSTEP_ENABLE 0
// TMC2209_COOLSTEP_ENABLE = 1 defaults
#define TMC2209_COOLSTEP_SEMIN 1    // semin: 0 = coolStep off, 1 � 15 = coolStep on
#define TMC2209_COOLSTEP_SEMAX 1    // semax: 0 � 15

// end of default values

typedef enum {
    TMC2209Reg_GCONF = 0x00,
    TMC2209Reg_GSTAT = 0x01,
    TMC2209Reg_IFCNT = 0x02,
    TMC2209Reg_SLAVECONF = 0x03,
    TMC2209Reg_OTP_PROG = 0x04,
    TMC2209Reg_OTP_READ = 0x05,
    TMC2209Reg_IOIN = 0x06,
    TMC2209Reg_FACTORY_CONF = 0x07,

    TMC2209Reg_IHOLD_IRUN = 0x10,
    TMC2209Reg_TPOWERDOWN = 0x11,
    TMC2209Reg_TSTEP = 0x12,
    TMC2209Reg_TPWMTHRS = 0x13,
    TMC2209Reg_VACTUAL = 0x22,
    TMC2209Reg_TCOOLTHRS = 0x14,
    TMC2209Reg_SGTHRS = 0x40,
    TMC2209Reg_SG_RESULT = 0x41,
    TMC2209Reg_COOLCONF = 0x42,

    TMC2209Reg_MSCNT = 0x6A,
    TMC2209Reg_MSCURACT = 0x6B,
    TMC2209Reg_CHOPCONF = 0x6C,
    TMC2209Reg_DRV_STATUS = 0x6F,
    TMC2209Reg_PWMCONF = 0x70,
    TMC2209Reg_PWM_SCALE = 0x71,
    TMC2209Reg_PWM_AUTO = 0x72,
    TMC2209Reg_LAST_ADDR = TMC2209Reg_PWM_AUTO
} tmc2209_regaddr_t;

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
} TMC2209_status_t;

// --- register definitions ---

// GCONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        I_scale_analog      :1,
        internal_Rsense     :1,
        en_spreadcycle      :1,
        shaft               :1,
        index_otpw          :1,
        index_step          :1,
        pdn_disable         :1,
        mstep_reg_select    :1,
        multistep_filt      :1,
        test_mode           :1,
        reserved            :22;
    };
} TMC2209_gconf_reg_t;

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
} TMC2209_gstat_reg_t;

// IFCNT : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        count    :8,
        reserved :24;
    };
} TMC2209_ifcnt_reg_t;

// SLAVECONF : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        reserved0 :8,
        conf      :4,
        reserved1 :20;
    };
} TMC2209_slaveconf_reg_t;

// OTP_PROG : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        otpbit   :2,
        otpbyte  :2,
        otpmagic :28;
    };
} TMC2209_otp_prog_reg_t;

// OTP_READ : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        otp0_0_4 :5,
        otp0_5   :1,
        otp0_6   :1,
        otp0_7   :1,
        otp1_0_3 :4,
        otp1_4   :1,
        otp1_5_7 :3,
        otp2_0   :1,
        otp2_1   :1,
        otp2_2   :1,
        otp2_3_4 :2,
        otp2_5_6 :2,
        otp2_7   :1,
        reserved :8;
    };
} TMC2209_otp_read_reg_t;

// IOIN : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        enn       :1,
        unused0   :1,
        ms1       :1,
        ms2       :1,
        diag      :1,
        unused1   :1,
        pdn_uart  :1,
        step      :1,
        spread_en :1,
        dir       :1,
        reserved  :14,
        version   :8;
    };
} TMC2209_ioin_reg_t;

// FACTORY_CONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        fclktrim  :4,
        reserved1 :3,
        ottrim    :2,
        reserved :23;
    };
} TMC2209_factory_conf_reg_t;

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
} TMC2209_ihold_irun_reg_t;

// TPOWERDOWN : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tpowerdown :8,
        reserved   :24;
    };
} TMC2209_tpowerdown_reg_t;

// TSTEP : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tstep    :20,
        reserved :12;
    };
} TMC2209_tstep_reg_t;

// TPWMTHRS : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tpwmthrs :20,
        reserved :12;
    };
} TMC2209_tpwmthrs_reg_t;

// TCOOLTHRS : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        tcoolthrs :20,
        reserved  :12;
    };
} TMC2209_tcoolthrs_reg_t;

// VACTUAL : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        actual   :24,
        reserved :8;
    };
} TMC2209_vactual_reg_t;

// SGTHRS : W
typedef union {
    uint32_t value;
    struct {
        uint32_t
        threshold :8,
        reserved  :24;
    };
} TMC2209_sgthrs_reg_t;

// SG_RESULT : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        result   :10,
        reserved :22;
    };
} TMC2209_sg_result_reg_t;

// MSCNT : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        mscnt    :10,
        reserved :22;
    };
} TMC2209_mscnt_reg_t;

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
} TMC2209_mscuract_reg_t;

// CHOPCONF : RW
typedef union {
    uint32_t value;
    struct {
        uint32_t
        toff      :4,
        hstrt     :3,
        hend      :4,
        reserved0 :4,
        tbl       :2,
        vsense    :1,
        reserved1 :6,
        mres      :4,
        intpol    :1,
        dedge     :1,
        diss2g    :1,
        diss2vs   :1;
    };
} TMC2209_chopconf_reg_t;

// DRV_STATUS : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        otpw       :1,
        ot         :1,
        s2ga       :1,
        s2gb       :1,
        s2vsa      :1,
        s2vsb      :1,
        ola        :1,
        olb        :1,
        t120       :1,
        t143       :1,
        t150       :1,
        t157       :1,
        reserved1  :4,
        cs_actual  :5,
        reserved2  :3,
        reserved3  :6,
        stealth    :1,
        stst       :1;
    };
} TMC2209_drv_status_reg_t;

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
        reserved5 :16;
    };
} TMC2209_coolconf_reg_t;

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
} TMC2209_pwmconf_reg_t;

// PWM_SCALE : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_scale :8,
        reserved  :24;
    };
} TMC2209_pwm_scale_reg_t;

// PWM_AUTO : R
typedef union {
    uint32_t value;
    struct {
        uint32_t
        pwm_ofs_auto  :8,
        unused0       :8,
        pwm_grad_auto :8,
        unused1       :8;
    };
} TMC2209_pwm_auto_ctrl_reg_t;

// --- end of register definitions ---

typedef union {
    tmc2209_regaddr_t reg;
    uint8_t value;
    struct {
        uint8_t
        idx   :7,
        write :1;
    };
} TMC2209_addr_t;

// --- datagrams ---

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_gconf_reg_t reg;
} TMC2209_gconf_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_gstat_reg_t reg;
} TMC2209_stat_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_tpowerdown_reg_t reg;
} TMC2209_tpowerdown_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_ifcnt_reg_t reg;
} TMC2209_ifcnt_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_slaveconf_reg_t reg;
} TMC2209_slaveconf_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_otp_prog_reg_t reg;
} TMC2209_otp_prog_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_otp_read_reg_t reg;
} TMC2209_otp_read_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_ioin_reg_t reg;
} TMC2209_ioin_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_factory_conf_reg_t reg;
} TMC2209_factory_conf_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_ihold_irun_reg_t reg;
} TMC2209_ihold_irun_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_tstep_reg_t reg;
} TMC2209_tstep_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_tpwmthrs_reg_t reg;
} TMC2209_tpwmthrs_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_tcoolthrs_reg_t reg;
} TMC2209_tcoolthrs_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_vactual_reg_t reg;
} TMC2209_vactual_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_sgthrs_reg_t reg;
} TMC2209_sgthrs_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_sg_result_reg_t reg;
} TMC2209_sg_result_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_mscnt_reg_t reg;
} TMC2209_mscnt_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_mscuract_reg_t reg;
} TMC2209_mscuract_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_chopconf_reg_t reg;
} TMC2209_chopconf_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_drv_status_reg_t reg;
} TMC2209_drv_status_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_coolconf_reg_t reg;
} TMC2209_coolconf_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_pwmconf_reg_t reg;
} TMC2209_pwmconf_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_pwm_scale_reg_t reg;
} TMC2209_pwm_scale_dgr_t;

typedef struct {
    TMC2209_addr_t addr;
    TMC2209_pwm_auto_ctrl_reg_t reg;
} TMC2209_pwm_auto_ctrl_dgr_t;

// -- end of datagrams

typedef union {
    uint32_t value;
    uint8_t data[4];
    TMC2209_gconf_reg_t gconf;
    TMC2209_gstat_reg_t gstat;
    TMC2209_ifcnt_reg_t ifcnt;
    TMC2209_slaveconf_reg_t slaveconf;
    TMC2209_otp_prog_reg_t otp_prog;
    TMC2209_otp_read_reg_t otp_read;
    TMC2209_ioin_reg_t ioin;
    TMC2209_factory_conf_reg_t factory_conf;
    TMC2209_ihold_irun_reg_t ihold_irun;
    TMC2209_tpowerdown_reg_t tpowerdown;
    TMC2209_tstep_reg_t tstep;
    TMC2209_tpwmthrs_reg_t tpwmthrs;
    TMC2209_tcoolthrs_reg_t tcoolthrs;
    TMC2209_vactual_reg_t vactual;
    TMC2209_sgthrs_reg_t sgthrs;
    TMC2209_sg_result_reg_t sg_result;
    TMC2209_coolconf_reg_t coolconf;
    TMC2209_mscnt_reg_t mscnt;
    TMC2209_mscuract_reg_t mscuract;
    TMC2209_chopconf_reg_t chopconf;
    TMC2209_drv_status_reg_t drv_status;
    TMC2209_pwmconf_reg_t pwmconf;
    TMC2209_pwm_scale_reg_t pwm_scale;
    TMC2209_pwm_auto_ctrl_reg_t pwm_auto_ctrl;
} TMC2209_payload;

typedef struct {
     TMC2209_addr_t addr;
     TMC2209_payload payload;
} TMC2209_datagram_t;

typedef union {
    uint8_t data[8];
    struct {
        uint8_t sync;
        uint8_t slave;
        TMC2209_addr_t addr;
        TMC2209_payload payload;
        uint8_t crc;
    } msg;
} TMC2209_write_datagram_t;

typedef union {
    uint8_t data[4];
    struct {
        uint8_t sync;
        uint8_t slave;
        TMC2209_addr_t addr;
        uint8_t crc;
    } msg;
} TMC2209_read_datagram_t;

typedef struct {
    // driver registers
    TMC2209_gconf_dgr_t gconf;
    TMC2209_stat_dgr_t stat;
    TMC2209_ifcnt_dgr_t ifcnt;
    TMC2209_slaveconf_dgr_t slaveconf;
    TMC2209_otp_prog_dgr_t otp_prog;
    TMC2209_otp_read_dgr_t otp_read;
    TMC2209_ioin_dgr_t ioin;
    TMC2209_factory_conf_dgr_t factory_conf;
    TMC2209_ihold_irun_dgr_t ihold_irun;
    TMC2209_tpowerdown_dgr_t tpowerdown;
    TMC2209_tstep_dgr_t tstep;
    TMC2209_tpwmthrs_dgr_t tpwmthrs;
    TMC2209_tcoolthrs_dgr_t tcoolthrs;
    TMC2209_vactual_dgr_t vactual;
    TMC2209_sgthrs_dgr_t sgthrs;
    TMC2209_sg_result_dgr_t sg_result;
    TMC2209_coolconf_dgr_t coolconf;
    TMC2209_mscnt_dgr_t mscnt;
    TMC2209_mscuract_dgr_t mscuract;
    TMC2209_chopconf_dgr_t chopconf;
    TMC2209_drv_status_dgr_t drv_status;
    TMC2209_pwmconf_dgr_t pwmconf;
    TMC2209_pwm_scale_dgr_t pwm_scale;
    TMC2209_pwm_auto_ctrl_dgr_t pwm_auto;

    TMC2209_status_t driver_status;

    trinamic_motor_t motor;
    trinamic_config_t config;
} TMC2209_t;

#pragma pack(pop)

bool TMC2209_Init(TMC2209_t *driver);
void TMC2209_SetDefaults (TMC2209_t *driver);
void TMC2209_SetCurrent (TMC2209_t *driver, uint16_t mA, uint8_t hold_pct);
uint16_t TMC2209_GetCurrent (TMC2209_t *driver);
uint32_t TMC2209_GetTPWMTHRS (TMC2209_t *driver, float stpmm);
bool TMC2209_MicrostepsIsValid (uint16_t usteps);
void TMC2209_SetMicrosteps(TMC2209_t *driver, tmc2209_microsteps_t usteps);
void TMC2209_SetHybridThreshold (TMC2209_t *driver, uint32_t threshold, float steps_mm);
void TMC2209_SetTCOOLTHRS (TMC2209_t *driver, float mm_sec, float steps_mm);
void TMC2209_SetConstantOffTimeChopper(TMC2209_t *driver, uint8_t constant_off_time, uint8_t blank_time, uint8_t fast_decay_time, int8_t sine_wave_offset, bool use_current_comparator);
TMC2209_datagram_t *TMC2209_GetRegPtr (TMC2209_t *driver, tmc2209_regaddr_t reg);
bool TMC2209_WriteRegister (TMC2209_t *driver, TMC2209_datagram_t *reg);
bool TMC2209_ReadRegister (TMC2209_t *driver, TMC2209_datagram_t *reg);

#endif
