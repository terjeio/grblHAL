/*

  modbus.c - a lightweigth ModBus implementation

  Part of grblHAL

  Copyright (c) 2020-2021 Terje Io
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
#include "../grbl/nvs_buffer.h"
#include "../grbl/state_machine.h"
#else
#include "grbl/hal.h"
#include "grbl/protocol.h"
#include "grbl/settings.h"
#include "grbl/nvs_buffer.h"
#include "grbl/state_machine.h"
#endif

#include "modbus.h"

#define DEFAULT_BAUDRATE 3 // 19200

typedef struct queue_entry {
    bool async;
    bool sent;
    modbus_message_t msg;
    struct queue_entry *next;
} queue_entry_t;

static const uint32_t baud[]    = { 2400, 4800, 9600, 19200, 38400, 115200 };
static const uint16_t silence[] = {   16,    8,    4,     2,     2,      2 };

static modbus_stream_t *stream;
static uint16_t rx_timeout = 0, silence_until = 0, silence_timeout;
static int16_t exception_code = 0;
static queue_entry_t queue[MODBUS_QUEUE_LENGTH];
static modbus_settings_t modbus;
static volatile bool spin_lock = false;
static volatile queue_entry_t *tail, *head, *packet = NULL;
static volatile modbus_state_t state = ModBus_Idle;

static driver_reset_ptr driver_reset;
static on_execute_realtime_ptr on_execute_realtime;
static on_report_options_ptr on_report_options;
static nvs_address_t nvs_address;

static status_code_t modbus_set_baud (setting_id_t id, uint_fast16_t value);
static uint32_t modbus_get_baud (setting_id_t setting);
static void modbus_settings_restore (void);
static void modbus_settings_load (void);

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
/*
static bool valid_crc (char *buf, uint_fast16_t len)
{
    uint16_t crc = modbus_CRC16x(buf, len - 2);

    return buf[len - 1] == (crc >> 8) && buf[len - 2] == (crc & 0xFF);
}
*/
void modbus_poll (sys_state_t grbl_state)
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
                rx_timeout = modbus.rx_timeout;

                if(stream->set_direction)
                    stream->set_direction(true);

                packet->sent = true;
                stream->flush_rx_buffer();
                stream->write(((queue_entry_t *)packet)->msg.adu, ((queue_entry_t *)packet)->msg.tx_length);
            }
            break;

        case ModBus_Silent:
            if(ms >= silence_until) {
                silence_until = 0;
                state = ModBus_Idle;
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
                    state = ModBus_Silent;
                else if(stream->read() == 1 && (stream->read() & 0x80)) {
                    exception_code = stream->read();
                    state = ModBus_Exception;
                } else
                    state = ModBus_Timeout;
                packet = NULL;
                spin_lock = false;
                if(state != ModBus_AwaitReply)
                    silence_until = ms + silence_timeout;
                return;
            }

            if(stream->get_rx_buffer_count() >= packet->msg.rx_length) {

                char *buf = ((queue_entry_t *)packet)->msg.adu;

                do {
                    *buf++ = stream->read();
                } while(--packet->msg.rx_length);

                if((state = packet->async ? ModBus_Silent : ModBus_GotReply) == ModBus_Silent)
                    stream->on_rx_packet(&((queue_entry_t *)packet)->msg);

                silence_until = ms + silence_timeout;
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
            grbl.on_execute_realtime(state_get());
            if(ABORTED)
                return false;
        }

        if(stream->set_direction)
            stream->set_direction(true);

        state = ModBus_TX;
        rx_timeout = modbus.rx_timeout;

        memcpy(&sync_msg.msg, msg, sizeof(modbus_message_t));

        sync_msg.async = false;
        stream->flush_rx_buffer();
        stream->write(sync_msg.msg.adu, sync_msg.msg.tx_length);

        packet = &sync_msg;

        while(poll) {

            grbl.on_execute_realtime(state_get());

            if(ABORTED)
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

        state = silence_until > 0 ? ModBus_Silent : ModBus_Idle;

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

    silence_until = 0;
    state = ModBus_Idle;

    stream->flush_tx_buffer();
    stream->flush_rx_buffer();

    driver_reset();
}

static uint32_t get_baudrate (uint32_t rate)
{
    uint32_t idx = sizeof(baud) / sizeof(uint32_t);

    do {
        if(baud[--idx] == rate)
            return idx;
    } while(idx);

    return DEFAULT_BAUDRATE;
}

static const setting_group_detail_t modbus_groups [] = {
    { Group_Root, Group_ModBus, "ModBus"}
};

static const setting_detail_t modbus_settings[] = {
    { Settings_ModBus_BaudRate, Group_ModBus, "ModBus baud rate", NULL, Format_RadioButtons, "2400,4800,9600,19200,38400,115200", NULL, NULL, Setting_NonCoreFn, modbus_set_baud, modbus_get_baud, NULL },
    { Settings_ModBus_RXTimeout, Group_ModBus, "ModBus RX timeout", "milliseconds", Format_Integer, "####0", "50", "250", Setting_NonCore, &modbus.rx_timeout, NULL, NULL }
};

static void modbus_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&modbus, sizeof(modbus_settings_t), true);
}

static setting_details_t details = {
    .groups = modbus_groups,
    .n_groups = sizeof(modbus_groups) / sizeof(setting_group_detail_t),
    .settings = modbus_settings,
    .n_settings = sizeof(modbus_settings) / sizeof(setting_detail_t),
    .save = modbus_settings_save,
    .load = modbus_settings_load,
    .restore = modbus_settings_restore,

};

static setting_details_t *on_get_settings (void)
{
    return &details;
}

static status_code_t modbus_set_baud (setting_id_t id, uint_fast16_t value)
{
    modbus.baud_rate = baud[(uint32_t)value];
    silence_timeout = silence[(uint32_t)value];
    stream->set_baud_rate(modbus.baud_rate);

    return Status_OK;
}

static uint32_t modbus_get_baud (setting_id_t setting)
{
    return get_baudrate(modbus.baud_rate);
}

static void modbus_settings_restore (void)
{
    modbus.rx_timeout = 50;
    modbus.baud_rate = baud[DEFAULT_BAUDRATE];

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&modbus, sizeof(modbus_settings_t), true);
}

static void modbus_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&modbus, nvs_address, sizeof(modbus_settings_t), true) != NVS_TransferResult_OK)
        modbus_settings_restore();

    silence_timeout = silence[get_baudrate(modbus.baud_rate)];

    stream->set_baud_rate(modbus.baud_rate);
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(!newopt)
        hal.stream.write("[PLUGIN:MODBUS v0.04]" ASCII_EOL);
}

bool modbus_init (modbus_stream_t *mstream)
{
    if((nvs_address = nvs_alloc(sizeof(modbus_settings_t)))) {

        stream = mstream;

        if(driver_reset == NULL) {

            driver_reset = hal.driver_reset;
            hal.driver_reset = modbus_reset;

            on_execute_realtime = grbl.on_execute_realtime;
            grbl.on_execute_realtime = modbus_poll;

            on_report_options = grbl.on_report_options;
            grbl.on_report_options = onReportOptions;

            details.on_get_settings = grbl.on_get_settings;
            grbl.on_get_settings = on_get_settings;
        }

        head = tail = &queue[0];

        uint_fast8_t idx;
        for(idx = 0; idx < MODBUS_QUEUE_LENGTH; idx++)
            queue[idx].next = idx == MODBUS_QUEUE_LENGTH - 1 ? &queue[0] : &queue[idx + 1];
    }

    return nvs_address != 0;
}
