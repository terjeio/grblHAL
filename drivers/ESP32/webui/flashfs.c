/*
  flashfs.c - SDCard plugin for FatFs

  Part of GrblHAL

  Copyright (c) 2018-2019 Terje Io

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

#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>

#include "flashfs.h"
#include <esp_log.h>

#if FLASHFS_ENABLE || true

typedef struct
{
    FILE *handle;
    char name[50];
    size_t size;
    size_t pos;
    uint32_t line;
    uint8_t eol;
} file_t;

static file_t file = {
    .handle = NULL,
    .size = 0,
    .pos = 0
};

static bool frewind = false;
static io_stream_t active_stream;
//static report_t active_reports;

static void file_close (void)
{
    if(file.handle) {
        fclose(file.handle);
        file.handle = NULL;
    }
}

static bool file_open (char *filename)
{
	struct stat st;

    if(file.handle)
        file_close();

    if(stat(filename, &st) == 0 && (file.handle = fopen(filename, "r"))) {
        file.size = st.st_size;
        file.pos = 0;
        file.line = 0;
        file.eol = false;
        char *leafname = strrchr(filename, '/');
        strncpy(file.name, leafname ? leafname + 1 : filename, sizeof(file.name));
        file.name[sizeof(file.name) - 1] = '\0';
    }

    return file.handle != NULL;
}

static int16_t file_read (void)
{
    int c;

    if((c = fgetc(file.handle)) != EOF)
        file.pos = ftell(file.handle);

    if(c == '\r' || c == '\n')
        file.eol++;
    else
        file.eol = 0;

    return (int16_t)c;
}

static void flashfs_end_job (void)
{
    file_close();
    memcpy(&hal.stream, &active_stream, sizeof(io_stream_t));   // Restore stream pointers
    hal.stream.reset_read_buffer();                             // and flush input buffer
    hal.driver_rt_report = NULL;
    hal.state_change_requested = NULL;
    hal.report.status_message = report_status_message;
    hal.report.feedback_message = report_feedback_message;
    frewind = false;
}

ISR_CODE static int16_t flashfs_read (void)
{
    int16_t c = -1;

    if(file.eol == 1)
        file.line++;

    if(file.handle) {

        if(sys.state == STATE_IDLE || (sys.state & (STATE_CYCLE|STATE_HOLD)))
            c = file_read();

        if(c == -1) { // EOF or error reading or grbl problem
            file_close();
            if(file.eol == 0) // Return newline if line was incorrectly terminated
                c = '\n';
        }

    } else if(sys.state == STATE_IDLE) // TODO: end on ok count match line count?
        flashfs_end_job();

    return c;
}

static int16_t await_cycle_start (void)
{
    return -1;
}

// Drop input from current stream except realtime commands
static ISR_CODE bool drop_input_stream (char c)
{
    active_stream.enqueue_realtime_command(c);

    return true;
}

static void trap_state_change_request(uint_fast16_t state)
{
    if(state == STATE_CYCLE) {
        if(hal.stream.read == await_cycle_start)
            hal.stream.read = flashfs_read;
        hal.state_change_requested = NULL;
    }
}

static status_code_t trap_status_report (status_code_t status_code)
{
    if(status_code != Status_OK) { // TODO: all errors should terminate job?
        char buf[50]; // TODO: check if extended error reports are permissible
        sprintf(buf, "error:%d in SD file at line %u\r\n", (uint8_t)status_code, file.line);
        hal.stream.write(buf);
        flashfs_end_job();
    }

    return status_code;
}

static message_code_t trap_feedback_message (message_code_t message_code)
{
    report_feedback_message(message_code);

    if(message_code == Message_ProgramEnd) {
        if(frewind) {
            fseek(file.handle, 0, SEEK_SET);
            file.pos = file.line = 0;
            file.eol = false;
            report_feedback_message(Message_CycleStartToRerun);
            hal.stream.read = await_cycle_start;
            hal.state_change_requested = trap_state_change_request;
        } else
            flashfs_end_job();
    }

    return message_code;
}

static void flashfs_report (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    stream_write("|SD:");
    stream_write(ftoa((float)file.pos / (float)file.size * 100.0f, 1));
    stream_write(",");
    stream_write(file.name);
}

#if M6_ENABLE

static bool flashfs_suspend (bool suspend)
{
    if(suspend) {
        hal.stream.reset_read_buffer();
        hal.stream.read = active_stream.read;               // Restore normal stream input for tool change (jog etc)
        hal.stream.enqueue_realtime_command = active_stream.enqueue_realtime_command;
        hal.report.status_message = report_status_message;  // as well as normal status messages reporting
    } else {
        hal.stream.read = flashfs_read;                      // Resume reading from SD card
        hal.stream.enqueue_realtime_command = drop_input_stream;
        hal.report.status_message = trap_status_report;     // and redirect status messages back to us
    }

    return true;
}
#endif

status_code_t flashfs_stream_file (char *filename)
{
    status_code_t retval = Status_Unhandled;

	if (sys.state != STATE_IDLE)
		retval = Status_SystemGClock;
	else {
		if(file_open(filename)) {
			gc_state.last_error = Status_OK;                      		// Start with no errors
			hal.report.status_message(Status_OK);                       // and confirm command to originator
			memcpy(&active_stream, &hal.stream, sizeof(io_stream_t));   // Save current stream pointers
			hal.stream.type = StreamType_FlashFs;                       // then redirect to read from SD card instead
			hal.stream.read = flashfs_read;                             // ...
			hal.stream.enqueue_realtime_command = drop_input_stream;    // Drop input from current stream except realtime commands
#if M6_ENABLE
			hal.stream.suspend_read = flashfs_suspend;                  // ...
#else
			hal.stream.suspend_read = NULL;                             // ...
#endif
			hal.driver_rt_report = flashfs_report;                      // Add percent complete to real time report
			hal.report.status_message = trap_status_report;             // Redirect status message and feedback message
			hal.report.feedback_message = trap_feedback_message;        // reports here
			retval = Status_OK;
		} else
			retval = Status_SDReadError;
	}

    return retval;
}

void flashfs_reset (void)
{
    if(hal.stream.type == StreamType_FlashFs) {
        if(file.line > 0) {
            char buf[70];
            sprintf(buf, "[MSG:Reset during streaming of SPIFFS file at line: %ul]\r\n", file.line);
            hal.stream.write(buf);
        }
        flashfs_end_job();
    }
}

void flashfs_init (void)
{
 //   hal.driver_reset = flashfs_reset;
}

#endif
