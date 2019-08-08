// PWM Driver - LPC176x
//
// Copyright 2017 Brett Fleming
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
// IN THE SOFTWARE.

#include <stdint.h>
#include <stdbool.h>

#include "chip.h"
#include "core_cm3.h"

/*------------- Pulse-Width Modulation (PWM) ---------------------------------*/
typedef struct
{
  __IO uint32_t IR;
  __IO uint32_t TCR;
  __IO uint32_t TC;
  __IO uint32_t PR;
  __IO uint32_t PC;
  __IO uint32_t MCR;
  __IO uint32_t MR0;
  __IO uint32_t MR1;
  __IO uint32_t MR2;
  __IO uint32_t MR3;
  __IO uint32_t CCR;
  __I  uint32_t CR0;
  __I  uint32_t CR1;
  __I  uint32_t CR2;
  __I  uint32_t CR3;
       uint32_t RESERVED0;
  __IO uint32_t MR4;
  __IO uint32_t MR5;
  __IO uint32_t MR6;
  __IO uint32_t PCR;
  __IO uint32_t LER;
       uint32_t RESERVED1[7];
  __IO uint32_t CTCR;
} LPC_PWM_TypeDef;

#define LPC_PWM1              ((LPC_PWM_TypeDef       *) LPC_PWM1_BASE     )

typedef struct _PWM_Channel_Config {
    volatile uint32_t* MRn; //LPC_PWM1->MR1 through MR6
    uint32_t PCR_Enable_Mask;
    uint32_t LER_Enable_Mask;
                                  //PWM Output Mapping  PWM1   PWM2   PWM3   PWM4   PWM5   PWM6
    uint32_t PINSEL3_Enable_Mask; //For PWM support on (P1.18, P1.20, P1.21, P1.23, P1.24, P1.26)
    uint32_t PINSEL4_Enable_Mask; //For PWM support on (P2.0,  P2.1,  P2.2,  P2.3,  P2.4,  P2.5)
} const PWM_Channel_Config;

extern PWM_Channel_Config PWM1_CH1;
extern PWM_Channel_Config PWM1_CH2;
extern PWM_Channel_Config PWM1_CH3;
extern PWM_Channel_Config PWM1_CH4;
extern PWM_Channel_Config PWM1_CH5;
extern PWM_Channel_Config PWM1_CH6;

void pwm_init(PWM_Channel_Config* channel, bool primaryPin, bool secondaryPin, uint32_t period, uint32_t width);
void pwm_set_period(uint32_t period);
void pwm_set_width(PWM_Channel_Config* channel, uint32_t width);
void pwm_enable(PWM_Channel_Config* channel);
void pwm_disable(PWM_Channel_Config* channel);

