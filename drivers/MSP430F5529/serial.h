//
// serial.h - serial (UART) port library including MCP4725 DAC support, for PCB laser
//
// v1.0 / 2015-01-26 / Io Engineering / Terje
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

#include "portmacros.h"

#define ASCII_ETX  0x03
#define ASCII_ACK  0x06
#define ASCII_BS   0x08
#define ASCII_LF   0x0A
#define ASCII_CR   0x0D
#define ASCII_XON  0x11
#define ASCII_XOFF 0x13
#define ASCII_NAK  0x15
#define ASCII_EOF  0x1A
#define ASCII_CAN  0x18
#define ASCII_EM   0x19
#define ASCII_DEL  0x7F
#define ASCII_EOL  "\r\n"

#define XONOK (ASCII_XON|0x80)
#define XOFFOK (ASCII_XOFF|0x80)
#define TX_BUFFER_SIZE 128
#define RX_BUFFER_SIZE 256
#define RX_BUFFER_HWM 188
#define RX_BUFFER_LWM 150

#define SERIAL_PORT 4
#define SERIAL_SEL portSel(SERIAL_PORT)
#define SERIAL_RXD BIT4
#define SERIAL_TXD BIT5
#define SERIAL_RTS_PORT     8
#define SERIAL_RTS_BIT      BIT2
#define SERIAL_RTS_PORT_OUT portOut(SERIAL_RTS_PORT)
#define SERIAL_RTS_PORT_IN  portIn(SERIAL_RTS_PORT)
#define SERIAL_RTS_PORT_DIR portDir(SERIAL_RTS_PORT)

/* UART */

void serialInit (void);
uint16_t serialTxCount (void);
uint16_t serialRxCount (void);
uint16_t serialRxFree (void);
void serialRxFlush (void);
void serialRxCancel (void);
int16_t serialGetC (void);
bool serialPutC (const char data);
void serialWriteS (const char *data);
void serialWriteLn (const char *data);
void serialWrite (const char *data, uint16_t length);

