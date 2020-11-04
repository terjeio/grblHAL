//
// main.c - Spindle Simulator for voltage controlled (PWM) spindles with encoder
//
// For Texas Instruments MSP430 Value Line Launchpad
//
// v1.1 / 2020-11-04 / Io Engineering / Terje
//

/*

Copyright (c) 2019-2020, Terje Io
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

· Redistributions of source code must retain the above copyright notice, this
list of conditions and the following disclaimer.

· Redistributions in binary form must reproduce the above copyright notice, this
list of conditions and the following disclaimer in the documentation and/or
other materials provided with the distribution.

· Neither the name of the copyright holder nor the names of its contributors may
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

#include <msp430.h> 

#include <string.h>
#include <stdbool.h>
#include <stdint.h>

#include "serial.h"

#define MSG_ABOUT 0U
#define MSG_PARAM 1U
#define MSG_OK    2U
#define MSG_FAIL  3U
#define MSG_BADC  4U

char const *const message[] = {
    "\r\nSpindle Simulator 1.1\0",
    "Error: missing parameters",
    "OK",
    "FAILED",
    "Bad command"
};

typedef struct {
    char const *const command;
    bool (*const handler)(char *);
    const bool report;
} command_t;

char cmdbuf[16];
uint16_t ppr = 120, rpm = 400, max_rpm = 1000;
uint32_t pulse_period = 0;
bool update_rpm = true;
int16_t step = 0;

bool manual = false, lock = false, invert = false;

void trigOut (void)
{
    SPINDLE_TRIG_PORT_OUT |= SPINDLE_TRIG_BIT;
    __delay_cycles(1600); // 100uS
    SPINDLE_TRIG_PORT_OUT &= ~SPINDLE_TRIG_BIT;
}

int parseInt (char *s)
{
    int c, res = 0, negative = 0;

    if(*s == '-') {
        negative = 1;
        s++;
    } else if(*s == '+')
        s++;

    while((c = *s++) != '\0') {
        if(c >= 48 && c <= 57)
            res = (res * 10) + c - 48;
    }

    return negative ? -res : res;
}

uint16_t read_adc (void)
{
    uint16_t adc;

    ADC10CTL1 = INCH_3|ADC10DIV_3|ADC10SSEL_3;
    ADC10CTL0 = ADC10SHT_3|ADC10ON|ENC|ADC10SC; //|REFON|SREF_1;

    while (ADC10CTL1 & ADC10BUSY) ;

    adc = ADC10MEM;
    ADC10CTL0 &= ~ENC;

    return adc;
}

void setRPM (int value, int16_t offset)
{
    uint32_t new_period = ((2000000UL * 60UL) / (value + (lock ? 0 : offset)) / ppr);

    rpm = value;
    pulse_period = pulse_period > (1UL << 16) ? (1UL << 16) - 1 : pulse_period;

    if((update_rpm = new_period != pulse_period))
        pulse_period = new_period;
}

bool setPPR (int value)
{
    ppr = value;
    pulse_period = step = 0;

    setRPM(rpm, 0);

    return true;
}

bool cmdSetPPR (char *params)
{
    return setPPR(parseInt(params));
}

bool cmdSetRPM (char *params)
{
    setRPM(parseInt(params), 0);

    return true;
}

void SpindleOn (bool on)
{
    if(on) {
        SPINDLE_PULSE_CTL |= MC0;       // Start timer in up mode and
        trigOut();                      // output trigger pulse
    } else
        SPINDLE_PULSE_CTL &= ~MC0;      // Stop timer
}

bool cmdSpindle (char *params)
{
    if(manual)
        SpindleOn(parseInt(params));

    return manual;
}

bool cmdAuto (char *params)
{
    manual = !parseInt(params);

    return true;
}

bool cmdInvert (char *params)
{
    invert = parseInt(params);

    return true;
}

bool cmdLock (char *params)
{
    lock = parseInt(params);

    return true;
}

bool cmdStep (char *params)
{
    step = parseInt(params);

    trigOut();

    return true;
}

void exeCommand (char *cmdline)
{
    static const command_t commands[] = {
        "RPM:",     cmdSetRPM, true,
        "PPR:",     cmdSetPPR, true,
        "SPINDLE:", cmdSpindle, true,
        "AUTO:",    cmdAuto, true,
        "LOCK:",    cmdLock, true,
        "STEP:",    cmdStep, true,
        "INVERT:",  cmdInvert, true
    };

    static const uint16_t numcmds = sizeof(commands) / sizeof(command_t);

    bool ok = false;
    uint16_t i = 0, cmdlen;

    while(!ok && i < numcmds) {

        cmdlen = strlen(commands[i].command);

        if(!(ok = !strncmp(commands[i].command, cmdline, cmdlen)))
            i++;

    }

    if(ok) {
        ok = commands[i].handler(cmdline + cmdlen);
        if(commands[i].report)
            serialWriteLn(message[ok ? MSG_OK : MSG_FAIL]);
    } else
        serialWriteLn(message[MSG_BADC]);
}

uint16_t read_rpm (void)
{
    uint16_t i = 64, adc = 0;

    static p = 0;

    p = 0;

    while(i--) {
        p++;
        adc += read_adc();
        __delay_cycles(100);
    }

    adc = (adc >> 6) * 36 / 33; // MSP430G2553 has 3.6V supply, ADC source 3.3V
    adc = invert ? 1023 - adc : adc;

    return (uint16_t)(((uint32_t)adc * max_rpm) / 1024UL);
}

void main (void)
{
    char c;
    uint16_t cmdptr = 0;

    WDTCTL = WDTPW | WDTHOLD;	        // Stop watchdog timer

    DCOCTL = CALDCO_16MHZ;              // Set DCO for 16MHz using
    BCSCTL1 = CALBC1_16MHZ;             // calibration registers

    INDEX_CCR0 = 40000;                 // Set inital index pulse time and
    INDEX_CCR1 = 20000;                 // pulse width
    INDEX_CCTL1 = OUTMOD_3;             // Set output mode to PWM,
    INDEX_CTL = TASSEL1|ID0|ID1|TACLR;  // bind to SMCLK and clear TA
    INDEX_PORT_SEL |= INDEX_BIT;        // Enable TA0.1 on INDEX pin
    INDEX_PORT_DIR |= INDEX_BIT;        // Set index pulse pin as output and
    SPINDLE_PULSE_CCTL0 = CCIE;         // enable timer interrupt

    SPINDLE_PULSE_CCR0 = 40000;                     // Set initial pulse time and
    SPINDLE_PULSE_CCR1 = 20000;                     // pulse width
    SPINDLE_PULSE_CCTL1 = OUTMOD_3;                 // Set output mode to PWM,
    SPINDLE_PULSE_CTL = TASSEL1|ID0|ID1|TACLR;      // bind to SMCLK and clear TA
    SPINDLE_PULSE_PORT_SEL |= SPINDLE_PULSE_BIT;    // Enable TA1.0 on pulse pin
    SPINDLE_PULSE_PORT_DIR |= SPINDLE_PULSE_BIT;    // Set pulse pin as output and
    INDEX_CCTL0 = CCIE;                             // enable timer interrupt

    setRPM(rpm, 0);

    SPINDLE_ON_PORT_OUT |= SPINDLE_ON_BIT;
    SPINDLE_ON_PORT_REN |= SPINDLE_ON_BIT;

    SPINDLE_TRIG_PORT_DIR |= SPINDLE_TRIG_BIT;  // Set trig bit as output and
    SPINDLE_TRIG_PORT_OUT &= ~SPINDLE_TRIG_BIT; // clear it

    P1SEL |= VPWMIN;                        // Temp sensor input to ADC
    ADC10CTL0 = 0;
    ADC10CTL1 = ADC10DIV_3|INCH_3;          // A3, input channel 5, trigger on ADC10SC bit, no clock division,
    ADC10CTL0 = SREF_0|ADC10ON|ADC10SHT_3;  // 1.5V ref,Ref on,64 clocks for sample
    ADC10AE0 |= VPWMIN;

    serialInit();

    _EINT();                                // Enable interrupts

    serialRxFlush();
    serialWriteLn(message[MSG_ABOUT]);

    while(1) {

        if(!manual) {
            SpindleOn((SPINDLE_ON_PORT_IN & SPINDLE_ON_BIT) != 0);
            setRPM(read_rpm(), step);
        } else
            setRPM(rpm, (int16_t)(read_rpm() / 8) - 64);

        if(serialRxCount()) { // bytes waiting, process them

            c = serialRead();

            serialPutC(c);

            if(c == CR && cmdptr > 0) {

                cmdbuf[cmdptr] = 0;

                serialPutC(LF);

                exeCommand(cmdbuf);
                cmdptr = 0;
                cmdbuf[0] = 0;

            } else if(c == DEL) {
                if(cmdptr > 0)
                    cmdptr--;
            } else if(c >= ' ' && cmdptr < sizeof(cmdbuf))
                cmdbuf[cmdptr++] = c & (c >= 'a' && c <= 'z' ? 0x5F : 0xFF);
        }

    }
}

#pragma vector=SPINDLE_PULSE_IRQH
__interrupt void TIMER_SP_ISR(void)
{
    static unsigned int index = 0;

    if(index == 0) {
        index = ppr - 1;
        INDEX_CTL |= MC0;   // Start index timer in up mode
    } else {
        index--;
        if(update_rpm) {
            update_rpm = false;
            SPINDLE_PULSE_CCR0 = (uint16_t)pulse_period;
            SPINDLE_PULSE_CCR1 = (uint16_t)pulse_period >> 1;

            INDEX_CCR0 = SPINDLE_PULSE_CCR0;
            INDEX_CCR1 = SPINDLE_PULSE_CCR1;
        }
    }
}

#pragma vector=INDEX_IRQH
__interrupt void TIMER_INDEX_ISR(void)
{
    INDEX_CTL &= ~MC0;     // Stop index timer
}
