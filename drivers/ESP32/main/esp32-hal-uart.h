// Copyright 2015-2016 Espressif Systems (Shanghai) PTE LTD
// Copyright 2018 Terje Io : Modifications for grbl
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef MAIN_ESP32_HAL_UART_H_
#define MAIN_ESP32_HAL_UART_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "driver.h"

#define SERIAL_5N1 0x8000010
#define SERIAL_6N1 0x8000014
#define SERIAL_7N1 0x8000018
#define SERIAL_8N1 0x800001c
#define SERIAL_5N2 0x8000030
#define SERIAL_6N2 0x8000034
#define SERIAL_7N2 0x8000038
#define SERIAL_8N2 0x800003c
#define SERIAL_5E1 0x8000012
#define SERIAL_6E1 0x8000016
#define SERIAL_7E1 0x800001a
#define SERIAL_8E1 0x800001e
#define SERIAL_5E2 0x8000032
#define SERIAL_6E2 0x8000036
#define SERIAL_7E2 0x800003a
#define SERIAL_8E2 0x800003e
#define SERIAL_5O1 0x8000013
#define SERIAL_6O1 0x8000017
#define SERIAL_7O1 0x800001b
#define SERIAL_8O1 0x800001f
#define SERIAL_5O2 0x8000033
#define SERIAL_6O2 0x8000037
#define SERIAL_7O2 0x800003b
#define SERIAL_8O2 0x800003f

#define ESP_REG(addr) *((volatile uint32_t *)(addr))

#define DEBUG_PRINT(string) uartWriteS(string)

void serialInit (void);
uint32_t serialAvailable (void);
uint16_t serialRXFree (void);
uint32_t serialAvailableForWrite (void);
int16_t serialRead (void);
bool serialSuspendInput (bool suspend);

bool serialPutC (const char c);
void serialWriteS (const char *data);

void serialFlush (void);
void serialCancel (void);

#if SERIAL2_ENABLE

void serial2Init (uint32_t baud_rate);
bool serial2SetBaudRate (uint32_t baud_rate);
void serial2Stop (void);
void serial2Start (void);
uint16_t serial2txCount (void);
uint16_t serial2Available (void);
uint16_t serial2RXFree (void);
int16_t serial2Read (void);
bool serial2SuspendInput (bool suspend);

bool serial2PutC (const char c);
void serial2Write (const char *s, uint16_t length);

void serial2Flush (void);
void serial2Cancel (void);
void serial2Direction (bool tx);

void serialSelect (bool mpg_mode);

#endif

#ifdef __cplusplus
}
#endif

#endif /* MAIN_ESP32_HAL_UART_H_ */
