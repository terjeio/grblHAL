//
// config.h - port assignments etc. for Spindle Simulator
//
// v1.0 / 2019-03-21 / Io Engineering / Terje
//

/*

Copyright (c) 2019, Terje Io
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

#include "portmacros.h"

#define SERIAL_PORT   1
#define RXD           BIT1  // P1.1
#define TXD           BIT2  // P1.2
#define RTS_PORT      1
#define RTS_BIT       BIT3  // P1.3
#define SERIAL_MODULE A0

#define SERIAL_TXIE UCA0TXIE
#define SERIAL_RXIE UCA0RXIE
#define SERIAL_TXIFG UCA0TXIFG
#define SERIAL_RXIFG UCA0RXIFG

#define VPWMIN BIT3 // P1.3 - PWM voltage input (LP filtered)

#define INDEX_TIMER  A
#define INDEX_TIMERI 0
#define INDEX_PORT   1
#define INDEX_BIT    BIT6  // P1.6

#define INDEX_CTL timerCtl(INDEX_TIMER, INDEX_TIMERI)
#define INDEX_CCR0 timerCcr(INDEX_TIMER, INDEX_TIMERI, 0)
#define INDEX_CCR1 timerCcr(INDEX_TIMER, INDEX_TIMERI, 1)
#define INDEX_CCTL0 timerCCtl(INDEX_TIMER, INDEX_TIMERI, 0)
#define INDEX_CCTL1 timerCCtl(INDEX_TIMER, INDEX_TIMERI, 1)
#define INDEX_IRQH timerInt(INDEX_TIMER, INDEX_TIMERI, 0)

#define INDEX_PORT_DIR  portDir(INDEX_PORT)
#define INDEX_PORT_SEL  portSel(INDEX_PORT)

#define SPINDLE_TRIG_PORT     2
#define SPINDLE_TRIG_BIT      BIT0 // P2.0
#define SPINDLE_TRIG_PORT_DIR portDir(SPINDLE_ON_PORT)
#define SPINDLE_TRIG_PORT_OUT portOut(SPINDLE_ON_PORT)
#define SPINDLE_TRIG_PORT_REN portRen(SPINDLE_ON_PORT)

#define SPINDLE_PULSE_TIMER  A
#define SPINDLE_PULSE_TIMERI 1
#define SPINDLE_PULSE_PORT   2
#define SPINDLE_PULSE_BIT    BIT1 // P2.1

#define SPINDLE_PULSE_CTL timerCtl(SPINDLE_PULSE_TIMER, SPINDLE_PULSE_TIMERI)
#define SPINDLE_PULSE_CCR0 timerCcr(SPINDLE_PULSE_TIMER, SPINDLE_PULSE_TIMERI, 0)
#define SPINDLE_PULSE_CCR1 timerCcr(SPINDLE_PULSE_TIMER, SPINDLE_PULSE_TIMERI, 1)
#define SPINDLE_PULSE_CCTL0 timerCCtl(SPINDLE_PULSE_TIMER, SPINDLE_PULSE_TIMERI, 0)
#define SPINDLE_PULSE_CCTL1 timerCCtl(SPINDLE_PULSE_TIMER, SPINDLE_PULSE_TIMERI, 1)
#define SPINDLE_PULSE_IRQH timerInt(SPINDLE_PULSE_TIMER, SPINDLE_PULSE_TIMERI, 0)

#define SPINDLE_PULSE_PORT_DIR portDir(SPINDLE_PULSE_PORT)
#define SPINDLE_PULSE_PORT_SEL portSel(SPINDLE_PULSE_PORT)

#define SPINDLE_ON_PORT     2
#define SPINDLE_ON_BIT      BIT2 // P2.2
#define SPINDLE_ON_PORT_IN  portIn(SPINDLE_ON_PORT)
#define SPINDLE_ON_PORT_OUT portOut(SPINDLE_ON_PORT)
#define SPINDLE_ON_PORT_REN portRen(SPINDLE_ON_PORT)
