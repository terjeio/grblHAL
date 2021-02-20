/*

  modbus.h - a lightweigth ModBus implementation

  Part of grblHAL

  Copyright (c) 2020 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#ifndef _MODBUS_H_
#define _MODBUS_H_

#define MODBUS_ENABLE 1
#define MODBUS_MAX_ADU_SIZE 10
#define MODBUS_QUEUE_LENGTH 8

typedef enum {
    ModBus_Idle,
    ModBus_Silent,
    ModBus_TX,
    ModBus_AwaitReply,
    ModBus_Timeout,
    ModBus_GotReply,
    ModBus_Exception
} modbus_state_t;

typedef enum {
    ModBus_ReadCoils = 1,
    ModBus_ReadDiscreteInputs = 2,
    ModBus_ReadHoldingRegisters = 3,
    ModBus_ReadInputRegisters = 4,
    ModBus_WriteCoil = 5,
    ModBus_WriteRegister = 6,
    ModBus_ReadExceptionStatus = 7,
    ModBus_Diagnostics = 8
} modbus_function_t;

typedef struct {
    uint8_t tx_length;
    uint8_t rx_length;
    void *xx;
    char adu[MODBUS_MAX_ADU_SIZE];
} modbus_message_t;

typedef struct {
    bool (*set_baud_rate)(uint32_t baud);
    void (*set_direction)(bool tx); // NULL if auto direction
    uint16_t (*get_tx_buffer_count)(void);
    uint16_t (*get_rx_buffer_count)(void);
    void (*write)(const char *c, uint16_t len);
    int16_t (*read)(void);
    void (*flush_tx_buffer)(void);
    void (*flush_rx_buffer)(void);
    // Callbacks
    void (*on_rx_packet)(modbus_message_t *msg);
    void (*on_rx_exception)(uint8_t code);
} modbus_stream_t;

bool modbus_init (modbus_stream_t *stream);
bool modbus_send (modbus_message_t *msg, bool block);
modbus_state_t modbus_get_state (void);

#endif
