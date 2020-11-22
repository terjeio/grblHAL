/*

  modbus.c - a lightweigth ModBus implementation

  Part of GrblHAL

  Copyright (c) 2020 Terje Io
  except modbus_CRC16x which is Copyright (c) 2006 Christian Walter <wolti@sil.at>
  Lifted from his FreeModbus Libary

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

#include <string.h>

#ifdef ARDUINO
#include "../grbl/hal.h"
#include "../grbl/protocol.h"
#else
#include "grbl/hal.h"
#include "grbl/protocol.h"
#endif

#include "modbus.h"

typedef struct queue_entry {
    bool async;
    bool sent;
    modbus_message_t msg;
    struct queue_entry *next;
} queue_entry_t;

static modbus_stream_t *stream;
static uint16_t rx_timeout = 0;
static int16_t exception_code = 0;
static queue_entry_t queue[MODBUS_QUEUE_LENGTH];
static volatile bool spin_lock = false;
static volatile queue_entry_t *tail, *head, *packet = NULL;
static volatile modbus_state_t state = ModBus_Idle;
static driver_reset_ptr driver_reset;
static on_execute_realtime_ptr on_execute_realtime;
static on_report_options_ptr on_report_options;

// Compute the MODBUS RTU CRC
static uint16_t modbus_CRC16x (char *buf, uint_fast16_t len)
{
    uint16_t crc = 0xFFFF;
    uint_fast8_t pos, i;

    for (pos = 0; pos < len; pos++) {
        crc ^= (uint16_t)buf[pos];          // XOR byte into least sig. byte of crc
        for (i = 8; i != 0; i--) {          // Loop over each bit
            if ((crc & 0x0001) != 0) {      // If the LSB is set
                crc >>= 1;                  // Shift right and XOR 0xA001
                crc ^= 0xA001;
            } else                          // Else LSB is not set
                crc >>= 1;                  // Just shift right
        }
    }

    // Note, this number has low and high bytes swapped, so use it accordingly (or swap bytes)
    return crc;
}

static bool valid_crc (char *buf, uint_fast16_t len)
{
    uint16_t crc = modbus_CRC16x(buf, len - 2);

    return buf[len - 1] == (crc >> 8) && buf[len - 2] == (crc & 0xFF);
}

void modbus_poll (uint_fast16_t grbl_state)
{
    static uint32_t last_ms;

    UNUSED(grbl_state);

    on_execute_realtime(grbl_state);

    uint32_t ms = hal.get_elapsed_ticks();

    if(ms == last_ms) // check once every ms
        return;

    spin_lock = true;

    switch(state) {

        case ModBus_Idle:
            if(tail != head && !packet) {

                packet = tail;
                tail = tail->next;
                state = ModBus_TX;
                rx_timeout = stream->rx_timeout;

                if(stream->set_direction)
                    stream->set_direction(true);

                packet->sent = true;
                stream->flush_rx_buffer();
                stream->write(((queue_entry_t *)packet)->msg.adu, ((queue_entry_t *)packet)->msg.tx_length);
            }
            break;

        case ModBus_TX:
            if(!stream->get_tx_buffer_count()) {

                state = ModBus_AwaitReply;

                if(stream->set_direction)
                    stream->set_direction(false);
            }
            break;

        case ModBus_AwaitReply:
            if(rx_timeout && --rx_timeout == 0) {
                if(packet->async)
                    state = ModBus_Idle;
                else if(stream->read() == 1 && (stream->read() & 0x80)) {
                    exception_code = stream->read();
                    state = ModBus_Exception;
                } else
                    state = ModBus_Timeout;
                packet = NULL;
                spin_lock = false;
                return;
            }

            if(stream->get_rx_buffer_count() >= packet->msg.rx_length) {

                char *buf = ((queue_entry_t *)packet)->msg.adu;

                do {
                    *buf++ = stream->read();
                } while(--packet->msg.rx_length);

                if((state = packet->async ? ModBus_Idle : ModBus_GotReply) == ModBus_Idle)
                    stream->on_rx_packet(&((queue_entry_t *)packet)->msg);

                packet = NULL;
            }
            break;

        default:
            break;
    }

    last_ms = ms;
    spin_lock = false;
}

bool modbus_send (modbus_message_t *msg, bool block)
{
    static queue_entry_t sync_msg = {0};

    uint_fast16_t crc = modbus_CRC16x(msg->adu, msg->tx_length - 2);

    msg->adu[msg->tx_length - 1] = crc >> 8;
    msg->adu[msg->tx_length - 2] = crc & 0xFF;

    while(spin_lock);

    if(block) {

        bool poll = true;

        while(state != ModBus_Idle) {
            if(!protocol_execute_realtime())
                return false;
        }

        if(stream->set_direction)
            stream->set_direction(true);

        state = ModBus_TX;
        rx_timeout = stream->rx_timeout;

        memcpy(&sync_msg.msg, msg, sizeof(modbus_message_t));

        sync_msg.async = false;
        stream->flush_rx_buffer();
        stream->write(sync_msg.msg.adu, sync_msg.msg.tx_length);

        packet = &sync_msg;

        while(poll) {

            if(!protocol_execute_realtime())
                poll = false;

            else switch(state) {

                case ModBus_Timeout:
                    stream->on_rx_exception(0);
                    poll = false;
                    break;

                case ModBus_Exception:
                    stream->on_rx_exception(exception_code == -1 ? 0 : (uint8_t)(exception_code & 0xFF));
                    poll = false;
                    break;

                case ModBus_GotReply:
                    if(packet)
                        stream->on_rx_packet(&((queue_entry_t *)packet)->msg);
                    poll = block = false;
                    break;

                default:
                    break;
            }
        }

        state = ModBus_Idle;

    } else if(packet != &sync_msg) {
        if(head->next != tail) {
            head->async = true;
            head->sent = false;
            memcpy((void *)&(head->msg), msg, sizeof(modbus_message_t));
            head = head->next;
        }
    }

    return !block;
}

modbus_state_t modbus_get_state (void)
{
    return state;
}

static void modbus_reset (void)
{
    while(spin_lock);

    packet = NULL;
    tail = head;
    state = ModBus_Idle;

    stream->flush_tx_buffer();
    stream->flush_rx_buffer();

    driver_reset();
}

static void onReportOptions (void)
{
    on_report_options();
    hal.stream.write("[PLUGIN:MODBUS v0.01]" ASCII_EOL);
}

void modbus_init (modbus_stream_t *mstream)
{
    uint_fast8_t idx;

    stream = mstream;

    if(driver_reset == NULL) {

        driver_reset = hal.driver_reset;
        hal.driver_reset = modbus_reset;

        on_execute_realtime = grbl.on_execute_realtime;
        grbl.on_execute_realtime = modbus_poll;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = onReportOptions;
    }

    head = tail = &queue[0];

    for(idx = 0; idx < MODBUS_QUEUE_LENGTH; idx++)
        queue[idx].next = idx == MODBUS_QUEUE_LENGTH - 1 ? &queue[0] : &queue[idx + 1];
}
