/*
  ymodem.c - simple file transfer protocol

  Part of SDCard plugin for grblHAL

  Specification: http://wiki.synchro.net/ref:ymodem

  NOTE: Receiver only, does not send initial 'C' to start transfer.
        Start transfer by sending SOH or STX.

  Copyright (c) 2021 Terje Io

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


#include "sdcard.h"

#if SDCARD_ENABLE == 2

#include <stdlib.h>
#include <string.h>

typedef enum {
    YModem_NOOP = 0,
    YModem_ACK,
    YModem_ACKFile,
    YModem_NoFile,
    YModem_Purge,
    YModem_NAK,
    YModem_EOT,
    YModem_CAN
} ymodem_status_t;

typedef ymodem_status_t (*process_data_ptr)(uint8_t c);

typedef struct {
    FATFS *fs;
    FIL *handle;
    char filename[32];
    uint32_t filelength;
    uint32_t received;
    uint16_t crc;
    uint_fast16_t idx;
    uint_fast16_t errors;
    uint_fast16_t packet_len;
    uint8_t packet_num;
    uint32_t next_timeout;
    process_data_ptr process;
    bool seq_inv;
    bool crc_lsb;
    bool completed;
    bool repeated;
    uint8_t payload[1024];
} ymodem_t;

static ymodem_t ymodem;

static driver_reset_ptr driver_reset;
static on_execute_realtime_ptr on_execute_realtime;
static on_unknown_realtime_cmd_ptr on_unknown_realtime_cmd;
static io_stream_t active_stream;

static ymodem_status_t await_soh (uint8_t c);
static ymodem_status_t await_packetnum (uint8_t c);
static ymodem_status_t get_payload (uint8_t c);
static ymodem_status_t await_crc (uint8_t c);
static ymodem_status_t await_eot (uint8_t c);

// Fast CRC16 implementation
// Original Code: Ashley Roll
// Optimisations: Scott Dattalo
// From http://www.ccsinfo.com/forum/viewtopic.php?t=24977
static uint16_t crc16 (const uint8_t *buf, uint16_t len)
{
    uint16_t x, crc = 0;

    while(len--) {
        x = (crc >> 8) ^ *buf++;
        x ^= x >> 4;
        crc = (crc << 8) ^ (x << 12) ^ (x << 5) ^ x;
    }

    return crc;
}

// End transfer handler.
static void end_transfer (bool send_ack)
{
    if(ymodem.handle) {
        f_close(ymodem.handle);
        ymodem.handle = NULL;
    }

    // Restore stream handlers and detach protocol loop from foreground process.
    memcpy(&hal.stream, &active_stream, sizeof(io_stream_t));
    grbl.on_execute_realtime = on_execute_realtime;

    if(send_ack) {
        hal.stream.write_char(ASCII_ACK);
        hal.stream.write_char('C');
    }
}

// Cancel handler. Waits for second cancel character.
static ymodem_status_t await_cancel (uint8_t c)
{
    if(c == ASCII_CAN)
        end_transfer(false);
    else
        ymodem.process = await_soh;

    return YModem_NOOP;
}

// Purge handler. Sinks incoming characters.
static ymodem_status_t purge (uint8_t c)
{
    return YModem_NOOP;
}

//
// Packet processing
//

// Start of header handler.
static ymodem_status_t await_soh (uint8_t c)
{
    ymodem_status_t status = YModem_NOOP;

    if(c == ASCII_SOH || c == ASCII_STX) {
        ymodem.idx = ymodem.crc = 0;
        ymodem.crc_lsb = ymodem.seq_inv = ymodem.repeated = false;
        ymodem.packet_len = c == ASCII_SOH ? 128 : 1024;
        ymodem.process = await_packetnum; // Set active handler to wait for packet number.
    } else if(c == ASCII_EOT)
        end_transfer(true);
    else if(c == ASCII_CAN)
        ymodem.process = await_cancel;   // Set active handler to wait for second CAN character.
    else
        status = YModem_Purge;

    return status;
}

// Packet number handler. Reads and validates packet number.
static ymodem_status_t await_packetnum (uint8_t c)
{
    ymodem_status_t status = YModem_NOOP;

    if(ymodem.seq_inv) {
        c ^= 0xFF;
        if((ymodem.repeated = ymodem.packet_num == c - 1))
            c++;
        if(c == ymodem.packet_num)
            ymodem.process = get_payload;   // Set active handler to fetch payload
        else
            status = YModem_Purge;
    } else if(!(ymodem.seq_inv = (c == ymodem.packet_num || ymodem.packet_num == c - 1)))
        status = YModem_Purge;

    return status;
}

// Payload handler. Saves incoming characters in payload buffer.
static ymodem_status_t get_payload (uint8_t c)
{
    ymodem.payload[ymodem.idx++] = c;

    if(ymodem.idx == ymodem.packet_len)
        ymodem.process = await_crc;

    return YModem_NOOP;
}

// CRC handler. Reads and validates CRC, open file on packet 0 writes payload to file when valid.
static ymodem_status_t await_crc (uint8_t c)
{
    ymodem_status_t status = YModem_NOOP;

    if(!ymodem.crc_lsb) {
        ymodem.crc_lsb = true;
        ymodem.crc = c;
    } else {

        ymodem.process = await_soh;                                                  // Set active handler to wait for next packet
        ymodem.crc = (ymodem.crc << 8) | c;

        if(crc16((const uint8_t *)&ymodem.payload, ymodem.packet_len) != ymodem.crc) // If CRC invalid
            return YModem_Purge;                                                     // purge input stream and return NAK.

        if(ymodem.packet_num == 0 && *ymodem.filename == 0) { // Open file or end transfer

            const char *data = (const char *)ymodem.payload;

            // If no filename present in payload end transfer by sending ACK else send ACK + C if file could be opened, CAN if not.
            if((status = *data == '\0' ? YModem_NoFile : YModem_ACKFile) == YModem_ACKFile) {

                strcpy(ymodem.filename, (const char *)data); // Save filename

                data += strlen(ymodem.filename) + 1;         // Save file length if present
                if(*data != '\0')
                    ymodem.filelength = atoi(data);

                static FIL handle;
                if(ymodem.fs != NULL && f_open(&handle, ymodem.filename, FA_CREATE_ALWAYS|FA_WRITE) == FR_OK)
                    ymodem.handle = &handle;
                else
                    status = YModem_CAN;
            }

            ymodem.packet_num++;

        } else { // Write payload to file

            UINT wrlen;

            if(!ymodem.repeated) {
                ymodem.packet_num++;
                ymodem.received += ymodem.packet_len;

                // Trim packet length to match file length if last packet
                if((ymodem.completed = ymodem.filelength && ymodem.received > ymodem.filelength))
                    ymodem.packet_len -= ymodem.received - ymodem.filelength;

                // Write payload
                if(f_write(ymodem.handle, ymodem.payload, ymodem.packet_len, &wrlen) == FR_OK) {
                    status = YModem_ACK;
                    if(ymodem.completed) {
                        ymodem.idx = 0;
                        ymodem.process = await_eot; // Set active handler to wait for end of transfer
                    }
                } else
                    status = YModem_CAN;
            } else
                status = YModem_ACK;
        }
    }

    return status;
}

// End of transmission handler.
static ymodem_status_t await_eot (uint8_t c)
{
    if(c == ASCII_EOT)
        end_transfer(true);

    return YModem_NOOP;
}

// Main YModem protocol loop.
// Reads characters off input stream and dispatches them to the appropriate handler.
// Handles timeouts.
static void protocol_loop (sys_state_t state)
{
    int16_t c;

    if(hal.get_elapsed_ticks() >= ymodem.next_timeout) {

        ymodem.next_timeout = hal.get_elapsed_ticks() + 1000;

        ymodem.errors++;
        if(ymodem.errors > 10)
            end_transfer(false);
        else {
            ymodem.process = await_soh;
            hal.stream.write_char(ASCII_NAK);
        }
    }

    while((c = active_stream.read()) != SERIAL_NO_DATA) {

        ymodem.next_timeout = hal.get_elapsed_ticks() + 1000;

        switch(ymodem.process(c)) {

            case YModem_ACK:
                ymodem.errors = 0;
                hal.stream.write_char(ASCII_ACK);
                break;

            case YModem_ACKFile:
                ymodem.errors = 0;
                hal.stream.write_char(ASCII_ACK);
                hal.stream.write_char('C');
                break;

            case YModem_NoFile:
                hal.stream.write_char(ASCII_ACK);
                end_transfer(false);
                break;

            case YModem_NAK:
                ymodem.errors++;
                hal.stream.write_char(ASCII_NAK);
                break;

            case YModem_CAN:
                hal.stream.write_char(ASCII_CAN);
                hal.stream.write_char(ASCII_CAN);
                end_transfer(false);
                break;

            case YModem_Purge:
                ymodem.errors++;
                ymodem.process = purge;
                hal.stream.cancel_read_buffer();
                break;

            default:
                break;
        }

        if(grbl.on_execute_realtime != protocol_loop)
            break;
    }

    on_execute_realtime(state);
}

// Buffer all received characters.
static bool buffer_all (char c)
{
    return false;
}

// Terminate any ongoing transfer on a soft reset.
static void on_soft_reset (void)
{
    if(grbl.on_execute_realtime == protocol_loop) {
        hal.stream.write_char(ASCII_CAN);
        hal.stream.write_char(ASCII_CAN);
        end_transfer(false);
    }

    driver_reset();
}

// Check input stream for file YModem start of header (soh) characters.
// Redirect input stream and start protocol handler when found.
static bool trap_initial_soh (char c)
{
    if(c == ASCII_SOH || c == ASCII_STX) {

        memcpy(&active_stream, &hal.stream, sizeof(io_stream_t));   // Save current stream pointers,
        hal.stream.enqueue_realtime_command = buffer_all;           // stop core real-time command handling and
        hal.stream.read = stream_get_null;                          // block core protocol loop from reading from input

        on_execute_realtime = grbl.on_execute_realtime;             // Add YModem protocol loop
        grbl.on_execute_realtime = protocol_loop;                   // to grblHAL foreground process

        memset(&ymodem, 0, sizeof(ymodem_t));                       // Init YModem variables
        ymodem.process = await_soh;
        ymodem.next_timeout = hal.get_elapsed_ticks() + 1000;
        ymodem.fs = sdcard_getfs();

        return false;                                               // Return false to add character to input buffer
    }

    return on_unknown_realtime_cmd == NULL || on_unknown_realtime_cmd(c);
}

// Add YModem protocol to chain of unknown real-time command handlers
void ymodem_init (void)
{
    driver_reset = hal.driver_reset;
    hal.driver_reset = on_soft_reset;

    on_unknown_realtime_cmd = grbl.on_unknown_realtime_cmd;
    grbl.on_unknown_realtime_cmd = trap_initial_soh;
}

#endif

