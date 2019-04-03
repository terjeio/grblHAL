//
// serial.h - serial (UART) port library including MCP4725 DAC support, for PCB laser
//
// v1.5 / 2018-06-18 / Io Engineering / Terje
//
//

/*

Copyright (c) 2015-2018, Terje Io
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

#include <stdbool.h>
#include <stdint.h>

#include "config.h"

#define XON  0x11
#define XOFF 0x13
#define EOF  0x1A
#define BS   0x08
#define LF   0x0A
#define CR   0x0D
#define CAN  0x18
#define DEL  0x7F

#define XONOK (XON|0x80)
#define XOFFOK (XOFF|0x80)

#define TX_BUFFER_SIZE 16

#ifdef __MSP430F5310__
#define RX_BUFFER_SIZE 1024
#define RX_BUFFER_HWM 900
#define RX_BUFFER_LWM 400
#else
#define RX_BUFFER_SIZE 192
#define RX_BUFFER_HWM 188
#define RX_BUFFER_LWM 150
#endif

#define RTS_PORT_IN   portIn(RTS_PORT)
#define RTS_PORT_OUT  portOut(RTS_PORT)
#define RTS_PORT_DIR  portDir(RTS_PORT)

#define SERIAL_SEL portSel(SERIAL_PORT)
#define SERIAL_SEL2 portSel2(SERIAL_PORT)
#define SERIAL_MCTL uartMCTL(SERIAL_MODULE)
#define SERIAL_CTL0 uartCTL(SERIAL_MODULE, 0)
#define SERIAL_CTL1 uartCTL(SERIAL_MODULE, 1)
#define SERIAL_BR0 uartBR(SERIAL_MODULE, 0)
#define SERIAL_BR1 uartBR(SERIAL_MODULE, 1)
#define SERIAL_IE uartIE(SERIAL_MODULE)
#define SERIAL_IFG uartIFG(SERIAL_MODULE)
#define SERIAL_RXD uartRXD(SERIAL_MODULE)
#define SERIAL_TXD uartTXD(SERIAL_MODULE)

/* UART */

void serialInit (void);
uint16_t serialTxCount(void);
uint16_t serialRxCount(void);
void serialRxFlush (void);
char serialRead (void);
void serialPutC (const char data);
void serialWriteS (const char *data);
void serialWriteLn (const char *data);
void serialWrite (const char *data, uint16_t length);
