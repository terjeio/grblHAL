//*****************************************************************************
//
// Copyright (C) 2012 - 2014 Texas Instruments Incorporated - http://www.ti.com/
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  Redistributions of source code must retain the above copyright
//  notice, this list of conditions and the following disclaimer.
//
//  Redistributions in binary form must reproduce the above copyright
//  notice, this list of conditions and the following disclaimer in the
//  documentation and/or other materials provided with the
//  distribution.
//
//  Neither the name of Texas Instruments Incorporated nor the names of
//  its contributors may be used to endorse or promote products derived
//  from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// MSP432 Family Interrupt Vector Table for CGT
//
//****************************************************************************

// Modified for grbl

#include <stdint.h>

#include "driver.h"

/* Forward declaration of the default fault handlers. */
static void resetISR(void);
static void nmiISR(void);
static void faultISR(void);
static void defaultISR(void);


/* External declaration for the reset handler that is to be called when the */
/* processor is started                                                     */
extern void _c_int00(void);


/* Linker variable that marks the top of the stack. */
extern unsigned long __STACK_END;


/* External declarations for the interrupt handlers used by the application. */
extern void SERIAL_IRQHandler (void);
extern void SERIAL2_IRQHandler (void);
extern void SYSTICK_IRQHandler (void);
extern void STEPPULSE_0_IRQHandler (void);
extern void STEPPULSE_N_IRQHandler (void);
extern void STEPPER_IRQHandler (void);
extern void DEBOUNCE_IRQHandler (void);
extern void RPMCOUNTER_IRQHandler (void);
//extern void EUSCIB1_IRQHandler (void);
#define EUSCIB1_IRQHandler defaultISR // for now...

#ifdef CNC_BOOSTERPACK_SHORTS
extern void LIMIT_IRQHandler (void);
extern void MODE_IRQHandler (void);
extern void CONTROL_IRQHandler (void);
//extern void RPMINDEX_IRQHandler (void);
#define PORT2_HANDLER LIMIT_IRQHandler
#define PORT5_HANDLER MODE_IRQHandler
#define PORT6_HANDLER CONTROL_IRQHandler
#else
extern void CONTROL_SD_RST_Handler (void);
extern void CONTROL_FH_IRQHandler (void);
extern void LIMIT_X_IRQHandler (void);
extern void LIMIT_YZ_IRQHandler (void);
#endif

extern void KEYPAD_IRQHandler (void);
#define PORT4_HANDLER KEYPAD_IRQHandler

#ifndef PORT1_HANDLER
#define PORT1_HANDLER defaultISR
#endif

#ifndef PORT2_HANDLER
#define PORT2_HANDLER defaultISR
#endif

#ifndef PORT3_HANDLER
#define PORT3_HANDLER defaultISR
#endif

#ifndef PORT4_HANDLER
#define PORT4_HANDLER defaultISR
#endif

#ifndef PORT5_HANDLER
#define PORT5_HANDLER defaultISR
#endif

#ifndef PORT6_HANDLER
#define PORT6_HANDLER defaultISR
#endif


/* To be added by user */


/* Intrrupt vector table.  Note that the proper constructs must be placed on this to  */
/* ensure that it ends up at physical address 0x0000.0000 or at the start of          */
/* the program if located at a start address other than 0.                            */
#pragma DATA_SECTION(interruptVectors, ".intvecs")
void (* const interruptVectors[])(void) =
{
    (void (*)(void))((uint32_t)&__STACK_END),
                                            /* The initial stack pointer */
    resetISR,                               /* The reset handler         */
    nmiISR,                                 /* The NMI handler           */
    faultISR,                               /* The hard fault handler    */
    defaultISR,                             /* The MPU fault handler     */
    defaultISR,                             /* The bus fault handler     */
    defaultISR,                             /* The usage fault handler   */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    0,                                      /* Reserved                  */
    defaultISR,                             /* SVCall handler            */
    defaultISR,                             /* Debug monitor handler     */
    0,                                      /* Reserved                  */
    defaultISR,                             /* The PendSV handler        */
    SYSTICK_IRQHandler,                     /* The SysTick handler       */
    defaultISR,                             /* PSS ISR                   */
    defaultISR,                             /* CS ISR                    */
    defaultISR,                             /* PCM ISR                   */
    defaultISR,                             /* WDT ISR                   */
    defaultISR,                             /* FPU ISR                   */
    defaultISR,                             /* FLCTL ISR                 */
    defaultISR,                             /* COMP0 ISR                 */
    defaultISR,                             /* COMP1 ISR                 */
    defaultISR,                             /* TA0_0 ISR                 */
    defaultISR,                             /* TA0_N ISR                 */
    RPMCOUNTER_IRQHandler,                  /* TA1_0 ISR                 */
    defaultISR,                             /* TA1_N ISR                 */
    STEPPULSE_0_IRQHandler,                 /* TA2_0 ISR                 */
    STEPPULSE_N_IRQHandler,                 /* TA2_N ISR                 */
    DEBOUNCE_IRQHandler,                    /* TA3_0 ISR                 */
    defaultISR,                             /* TA3_N ISR                 */
    SERIAL_IRQHandler,                      /* EUSCIA0 ISR               */
    defaultISR,                             /* EUSCIA1 ISR               */
    SERIAL2_IRQHandler,                     /* EUSCIA2 ISR               */
    defaultISR,                             /* EUSCIA3 ISR               */
    defaultISR,                             /* EUSCIB0 ISR               */
    EUSCIB1_IRQHandler,                     /* EUSCIB1 ISR               */
    defaultISR,                             /* EUSCIB2 ISR               */
    defaultISR,                             /* EUSCIB3 ISR               */
    defaultISR,                             /* ADC14 ISR                 */
    STEPPER_IRQHandler,                     /* T32_INT1 ISR              */
    defaultISR,                             /* T32_INT2 ISR              */
    defaultISR,                             /* T32_INTC ISR              */
    defaultISR,                             /* AES ISR                   */
    defaultISR,                             /* RTC ISR                   */
    defaultISR,                             /* DMA_ERR ISR               */
    defaultISR,                             /* DMA_INT3 ISR              */
    defaultISR,                             /* DMA_INT2 ISR              */
    defaultISR,                             /* DMA_INT1 ISR              */
    defaultISR,                             /* DMA_INT0 ISR              */
    PORT1_HANDLER,                          /* PORT1 ISR                 */
    PORT2_HANDLER,                          /* PORT2 ISR                 */
    PORT3_HANDLER,                          /* PORT3 ISR                 */
    PORT4_HANDLER,                          /* PORT4 ISR                 */
    PORT5_HANDLER,                          /* PORT5 ISR                 */
    PORT6_HANDLER,                          /* PORT6 ISR                 */
    defaultISR,                             /* Reserved 41               */
    defaultISR,                             /* Reserved 42               */
    defaultISR,                             /* Reserved 43               */
    defaultISR,                             /* Reserved 44               */
    defaultISR,                             /* Reserved 45               */
    defaultISR,                             /* Reserved 46               */
    defaultISR,                             /* Reserved 47               */
    defaultISR,                             /* Reserved 48               */
    defaultISR,                             /* Reserved 49               */
    defaultISR,                             /* Reserved 50               */
    defaultISR,                             /* Reserved 51               */
    defaultISR,                             /* Reserved 52               */
    defaultISR,                             /* Reserved 53               */
    defaultISR,                             /* Reserved 54               */
    defaultISR,                             /* Reserved 55               */
    defaultISR,                             /* Reserved 56               */
    defaultISR,                             /* Reserved 57               */
    defaultISR,                             /* Reserved 58               */
    defaultISR,                             /* Reserved 59               */
    defaultISR,                             /* Reserved 60               */
    defaultISR,                             /* Reserved 61               */
    defaultISR,                             /* Reserved 62               */
    defaultISR,                             /* Reserved 63               */
    defaultISR                              /* Reserved 64               */
};


/* This is the code that gets called when the processor first starts execution */
/* following a reset event.  Only the absolutely necessary set is performed,   */
/* after which the application supplied entry() routine is called.  Any fancy  */
/* actions (such as making decisions based on the reset cause register, and    */
/* resetting the bits in that register) are left solely in the hands of the    */
/* application.                                                                */
void resetISR(void)
{
    /* Jump to the CCS C Initialization Routine. */
    __asm("    .global _c_int00\n"
          "    b.w     _c_int00");
}


/* This is the code that gets called when the processor receives a NMI.  This  */
/* simply enters an infinite loop, preserving the system state for examination */
/* by a debugger.                                                              */
static void nmiISR(void)
{
    /* Fault trap exempt from ULP advisor */
    #pragma diag_push
    #pragma CHECK_ULP("-2.1")

    /* Enter an infinite loop. */
    while(1)
    {
    }

    #pragma diag_pop
}


/* This is the code that gets called when the processor receives a fault        */
/* interrupt.  This simply enters an infinite loop, preserving the system state */
/* for examination by a debugger.                                               */
static void faultISR(void)
{
    /* Fault trap exempt from ULP advisor */
    #pragma diag_push
    #pragma CHECK_ULP("-2.1")

    /* Enter an infinite loop. */
    while(1)
    {
    }

    #pragma diag_pop
}


/* This is the code that gets called when the processor receives an unexpected  */
/* interrupt.  This simply enters an infinite loop, preserving the system state */
/* for examination by a debugger.                                               */
static void defaultISR(void)
{
    /* Fault trap exempt from ULP advisor */
    #pragma diag_push
    #pragma CHECK_ULP("-2.1")

    /* Enter an infinite loop. */
    while(1)
    {
    }

    #pragma diag_pop
}
