/*
  sdcard.c - SDCard plugin for FatFs

  Part of grblHAL

  Copyright (c) 2018-2021 Terje Io

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

#if SDCARD_ENABLE

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#if defined(ESP_PLATFORM) || defined(STM32_PLATFORM) ||  defined(__LPC17XX__) ||  defined(__IMXRT1062__)
#define NEW_FATFS
#endif

#ifdef ARDUINO
  #include "../grbl/report.h"
  #include "../grbl/protocol.h"
  #include "../grbl/state_machine.h"
  #ifdef __IMXRT1062__
    #include "uSDFS.h"
    #define SDCARD_DEV "1:/"
    static FATFS fs;
  #endif
#else
  #include "grbl/report.h"
  #include "grbl/protocol.h"
  #include "grbl/state_machine.h"
#endif

#ifdef __IMXRT1062__
const char *dev = "1:/";
#elif defined(NEW_FATFS)
const char *dev = "";
#endif

#ifndef SDCARD_DEV
#define SDCARD_DEV ""
#endif

#if defined(STM32_PLATFORM) || defined(__LPC17XX__) ||  defined(__IMXRT1062__)
#define UINT32FMT "%lu"
#endif

#ifndef UINT32FMT
#define UINT32FMT "%u"
#endif

// https://e2e.ti.com/support/tools/ccs/f/81/t/428524?Linking-error-unresolved-symbols-rom-h-pinout-c-

/* uses fatfs - http://www.elm-chan.org/fsw/ff/00index_e.html */

#define MAX_PATHLEN 128
#define LCAPS(c) ((c >= 'A' && c <= 'Z') ? c | 0x20 : c)

#if FF_USE_LFN
//#define _USE_LFN FF_USE_LFN
#define _MAX_LFN FF_MAX_LFN
#endif

char const *const filetypes[] = {
    "nc",
    "gcode",
    "txt",
    "text",
    "tap",
    "ngc",
    ""
};

static FIL cncfile;

typedef enum {
    Filename_Filtered = 0,
    Filename_Valid,
    Filename_Invalid
} file_status_t;

typedef struct
{
    FATFS *fs;
    FIL *handle;
    char name[50];
    size_t size;
    size_t pos;
    uint32_t line;
    uint8_t eol;
} file_t;

static file_t file = {
    .fs = NULL,
    .handle = NULL,
    .size = 0,
    .pos = 0
};

static bool frewind = false;
static io_stream_t active_stream;
static driver_reset_ptr driver_reset;
static on_report_command_help_ptr on_report_command_help;
static on_realtime_report_ptr on_realtime_report;
static on_state_change_ptr state_change_requested;
static on_program_completed_ptr on_program_completed;
static on_report_options_ptr on_report_options;

static void sdcard_end_job (void);
static void sdcard_report (stream_write_ptr stream_write, report_tracking_flags_t report);
static void trap_state_change_request(uint_fast16_t state);
static void sdcard_on_program_completed (program_flow_t program_flow);
//static report_t active_reports;

#ifdef __MSP432E401Y__
/*---------------------------------------------------------*/
/* User Provided Timer Function for FatFs module           */
/*---------------------------------------------------------*/
/* This is a real time clock service to be called from     */
/* FatFs module. Any valid time must be returned even if   */
/* the system does not support a real time clock.          */

DWORD fatfs_getFatTime (void)
{

    return    ((2007UL-1980) << 25)    // Year = 2007
            | (6UL << 21)            // Month = June
            | (5UL << 16)            // Day = 5
            | (11U << 11)            // Hour = 11
            | (38U << 5)            // Min = 38
            | (0U >> 1)                // Sec = 0
            ;

}
#endif

static file_status_t allowed (char *filename, bool is_file)
{
    uint_fast8_t idx = 0;
    char filetype[8], *ftptr;
    file_status_t status = is_file ? Filename_Filtered : Filename_Valid;

    if(is_file && (ftptr = strrchr(filename, '.'))) {
        ftptr++;
        if(strlen(ftptr) > sizeof(filetype) - 1)
            return status;
        while(ftptr[idx]) {
            filetype[idx] = LCAPS(ftptr[idx]);
            idx++;
        }
        filetype[idx] = '\0';
        idx = 0;
        while(status == Filename_Filtered && filetypes[idx][0]) {
            if(!strcmp(filetype, filetypes[idx]))
                status = Filename_Valid;
            idx++;;
        }
    }

    if(status == Filename_Valid) {
        if(strchr(filename, ' ') ||
            strchr(filename, CMD_STATUS_REPORT) ||
             strchr(filename, CMD_CYCLE_START) ||
              strchr(filename, CMD_FEED_HOLD))
            status = Filename_Invalid;
    //TODO: check for top bit set characters
    }

    return status;
}

static inline char *get_name (FILINFO *file)
{
#if _USE_LFN
    return *file->lfname == '\0' ? file->fname : file->lfname;
#else
    return file->fname;
#endif
}

static FRESULT scan_dir (char *path, uint_fast8_t depth, char *buf)
{
#if defined(ESP_PLATFORM)
    FF_DIR dir;
#else
    DIR dir;
#endif
    FILINFO fno;
    FRESULT res;
    file_status_t status;
    bool subdirs = false;
#if _USE_LFN
    static TCHAR lfn[_MAX_LFN + 1];   /* Buffer to store the LFN */
    fno.lfname = lfn;
    fno.lfsize = sizeof(lfn);
#endif

    if((res = f_opendir(&dir, path)) != FR_OK)
        return res;

    // Pass 1: Scan files
    while(true) {

        if((res = f_readdir(&dir, &fno)) != FR_OK || fno.fname[0] == '\0')
            break;

        subdirs |= fno.fattrib & AM_DIR;

        if(!(fno.fattrib & AM_DIR) && (status = allowed(get_name(&fno), true)) != Filename_Filtered) {
            sprintf(buf, "[FILE:%s/%s|SIZE:" UINT32FMT "%s]" ASCII_EOL, path, get_name(&fno), (uint32_t)fno.fsize, status == Filename_Invalid ? "|UNUSABLE" : "");
            hal.stream.write(buf);
        }
    }

    if((subdirs = (subdirs && --depth)))
        f_readdir(&dir, NULL); // Rewind

    // Pass 2: Scan directories
    while(subdirs) {

        if((res = f_readdir(&dir, &fno)) != FR_OK || fno.fname[0] == '\0')
            break;

        if(fno.fattrib & AM_DIR) { // It is a directory
            size_t pathlen = strlen(path);
            if(pathlen + strlen(get_name(&fno)) > (MAX_PATHLEN - 1))
                break;
            sprintf(&path[pathlen], "/%s", get_name(&fno));
            if((res = scan_dir(path, depth, buf)) != FR_OK)
                break;
            path[pathlen] = '\0';
        }
    }

#ifdef NEW_FATFS
    f_closedir(&dir);
#endif

    return res;
}

static void file_close (void)
{
    if(file.handle) {
        f_close(file.handle);
        file.handle = NULL;
    }
}

static bool file_open (char *filename)
{
    if(file.handle)
        file_close();

    if(f_open(&cncfile, filename, FA_READ) == FR_OK) {
        file.handle = &cncfile;
        file.size = f_size(file.handle);
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
    signed char c;
    UINT count;

    if(f_read(file.handle, &c, 1, &count) == FR_OK && count == 1)
        file.pos = f_tell(file.handle);
    else
        c = -1;

    if(c == '\r' || c == '\n')
        file.eol++;
    else
        file.eol = 0;

    return (int16_t)c;
}

static bool sdcard_mount (void)
{
#ifdef __MSP432E401Y__
    return SDFatFS_open(Board_SDFatFS0, 0) != NULL;
#else
    if(file.fs == NULL)
  #ifdef __IMXRT1062__
        file.fs = &fs;
  #else
        file.fs = malloc(sizeof(FATFS));
  #endif

  #ifdef NEW_FATFS
    if(file.fs && f_mount(file.fs, dev, 1) != FR_OK) {
  #else
    if(file.fs && f_mount(0, file.fs) != FR_OK) {
  #endif
  #ifndef __IMXRT1062__
        free(file.fs);
  #endif
        file.fs = NULL;
    }
  #if defined(__IMXRT1062__)
    else if(f_chdrive(dev) != FR_OK)
        file.fs = NULL;
  #endif

    return file.fs != NULL;
#endif
}

static status_code_t sdcard_ls (void)
{
    char path[MAX_PATHLEN] = "", name[80]; // NB! also used as work area when recursing directories

    return scan_dir(path, 10, name) == FR_OK ? Status_OK : Status_SDFailedOpenDir;
}

static void sdcard_end_job (void)
{
    file_close();

    if(grbl.on_realtime_report == sdcard_report)
        grbl.on_realtime_report = on_realtime_report;

    if(grbl.on_program_completed == sdcard_on_program_completed)
        grbl.on_program_completed = on_program_completed;

    if(grbl.on_state_change == trap_state_change_request)
        grbl.on_state_change = state_change_requested;

    memcpy(&hal.stream, &active_stream, sizeof(io_stream_t));   // Restore stream pointers
    hal.stream.reset_read_buffer();                             // and flush input buffer
    on_realtime_report = NULL;
    state_change_requested = NULL;

    report_init_fns();

    frewind = false;
}

static int16_t sdcard_read (void)
{
    int16_t c = -1;
    sys_state_t state = state_get();

    if(file.eol == 1)
        file.line++;

    if(file.handle) {

        if(state == STATE_IDLE || (state & (STATE_CYCLE|STATE_HOLD|STATE_CHECK_MODE)))
            c = file_read();

        if(c == -1) { // EOF or error reading or grbl problem
            file_close();
            if(file.eol == 0) // Return newline if line was incorrectly terminated
                c = '\n';
        }

    } else if(state == STATE_IDLE) // TODO: end on ok count match line count?
        sdcard_end_job();

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

static void trap_state_change_request (sys_state_t state)
{
    if(state == STATE_CYCLE) {

        if(hal.stream.read == await_cycle_start)
            hal.stream.read = sdcard_read;

        if(grbl.on_state_change== trap_state_change_request) {
            grbl.on_state_change = state_change_requested;
            state_change_requested = NULL;
        }
    }

    if(state_change_requested)
        state_change_requested(state);
}

static status_code_t trap_status_report (status_code_t status_code)
{
    if(status_code != Status_OK) { // TODO: all errors should terminate job?
        char buf[50]; // TODO: check if extended error reports are permissible
        sprintf(buf, "error:%d in SD file at line " UINT32FMT ASCII_EOL, (uint8_t)status_code, file.line);
        hal.stream.write(buf);
        sdcard_end_job();
    }

    return status_code;
}

static void sdcard_report (stream_write_ptr stream_write, report_tracking_flags_t report)
{
    if(hal.stream.read != await_cycle_start) {
        char *pct_done = ftoa((float)file.pos / (float)file.size * 100.0f, 1);

        if(state_get() != STATE_IDLE && !strncmp(pct_done, "100.0", 5))
            strcpy(pct_done, "99.9");

        stream_write("|SD:");
        stream_write(pct_done);
        stream_write(",");
        stream_write(file.name);
    }

    if(on_realtime_report)
        on_realtime_report(stream_write, report);
}

static void sdcard_restart_msg (sys_state_t state)
{
    report_feedback_message(Message_CycleStartToRerun);
}

static void sdcard_on_program_completed (program_flow_t program_flow)
{
    frewind = frewind || program_flow == ProgramFlow_CompletedM2; // || program_flow == ProgramFlow_CompletedM30;

    if(frewind) {
        f_lseek(file.handle, 0);
        file.pos = file.line = 0;
        file.eol = false;
        hal.stream.read = await_cycle_start;
        if(grbl.on_state_change != trap_state_change_request) {
            state_change_requested = grbl.on_state_change;
            grbl.on_state_change = trap_state_change_request;
        }
        protocol_enqueue_rt_command(sdcard_restart_msg);
    } else
        sdcard_end_job();

    if(on_program_completed)
        on_program_completed(program_flow);
}

#if M6_ENABLE

static bool sdcard_suspend (bool suspend)
{
    if(suspend) {
        hal.stream.reset_read_buffer();
        hal.stream.read = active_stream.read;               // Restore normal stream input for tool change (jog etc)
        hal.stream.enqueue_realtime_command = active_stream.enqueue_realtime_command;
        grbl.report.status_message = report_status_message;  // as well as normal status messages reporting
    } else {
        hal.stream.read = sdcard_read;                      // Resume reading from SD card
        hal.stream.enqueue_realtime_command = drop_input_stream;
        grbl.report.status_message = trap_status_report;     // and redirect status messages back to us
    }

    return true;
}
#endif

static status_code_t sd_cmd_file (sys_state_t state, char *args)
{
    status_code_t retval = Status_Unhandled;

    if(args) {
        if (!(state == STATE_IDLE || state == STATE_CHECK_MODE))
            retval = Status_SystemGClock;
        else {
            if(file_open(args)) {
                gc_state.last_error = Status_OK;                            // Start with no errors
                grbl.report.status_message(Status_OK);                      // and confirm command to originator
                memcpy(&active_stream, &hal.stream, sizeof(io_stream_t));   // Save current stream pointers
                hal.stream.type = StreamType_SDCard;                        // then redirect to read from SD card instead
                hal.stream.read = sdcard_read;                              // ...
                hal.stream.enqueue_realtime_command = drop_input_stream;    // Drop input from current stream except realtime commands
#if M6_ENABLE
                hal.stream.suspend_read = sdcard_suspend;                   // ...
#else
                hal.stream.suspend_read = NULL;                             // ...
#endif
                on_realtime_report = grbl.on_realtime_report;
                grbl.on_realtime_report = sdcard_report;                     // Add percent complete to real time report

                on_program_completed = grbl.on_program_completed;
                grbl.on_program_completed = sdcard_on_program_completed;

                grbl.report.status_message = trap_status_report;             // Redirect status message reports here
                retval = Status_OK;
            } else
                retval = Status_SDReadError;
        }
    } else {
        frewind = false;
        retval = sdcard_ls(); // (re)use line buffer for reporting filenames
    }

    return retval;
}

static status_code_t sd_cmd_mount (sys_state_t state, char *args)
{
    frewind = false;

    return sdcard_mount() ? Status_OK : Status_SDMountError;
}

static status_code_t sd_cmd_rewind (sys_state_t state, char *args)
{
    frewind = true;

    return Status_OK;
}

static status_code_t sd_cmd_to_output (sys_state_t state, char *args)
{
    status_code_t retval = Status_Unhandled;

    if (!(state == STATE_IDLE || state == STATE_CHECK_MODE))
        retval = Status_SystemGClock;
    else if(args) {
        if(file_open(args)) {
            int16_t c;
            char buf[2] = {0};
            while((c = file_read()) != -1) {
                buf[0] = (char)c;
                hal.stream.write(buf);
            }
            file_close();
            retval = Status_OK;
        } else
            retval = Status_SDReadError;
    }

    return retval;
}

static void sdcard_reset (void)
{
    if(hal.stream.type == StreamType_SDCard) {
        if(file.line > 0) {
            char buf[70];
            sprintf(buf, "[MSG:Reset during streaming of SD file at line: " UINT32FMT "]" ASCII_EOL, file.line);
            hal.stream.write(buf);
        }
        sdcard_end_job();
    }

    driver_reset();
}

static void onReportCommandHelp (void)
{
    hal.stream.write("$F - list files on SD card" ASCII_EOL);
    hal.stream.write("$F=<filename> - run SD card file" ASCII_EOL);
    hal.stream.write("$FM - mount SD card" ASCII_EOL);
    hal.stream.write("$FR - enable rewind mode for next SD card file to run" ASCII_EOL);
    hal.stream.write("$F<=<filename> - dump SD card file to output" ASCII_EOL);

    if(on_report_command_help)
        on_report_command_help();
}

static void onReportOptions (bool newopt)
{
    on_report_options(newopt);

    if(newopt)
        hal.stream.write(",SD");
    else
        hal.stream.write("[PLUGIN:SDCARD v1.00]" ASCII_EOL);
}

const sys_command_t sdcard_command_list[] = {
    {"F", false, sd_cmd_file},
    {"FM", true, sd_cmd_mount},
    {"FR", true, sd_cmd_rewind},
    {"F<", false, sd_cmd_to_output},
};

static sys_commands_t sdcard_commands = {
    .n_commands = sizeof(sdcard_command_list) / sizeof(sys_command_t),
    .commands = sdcard_command_list
};

sys_commands_t *sdcard_get_commands()
{
    return &sdcard_commands;
}

void sdcard_init (void)
{
    driver_reset = hal.driver_reset;
    hal.driver_reset = sdcard_reset;

    sdcard_commands.on_get_commands = grbl.on_get_commands;
    grbl.on_get_commands = sdcard_get_commands;

    on_report_command_help = grbl.on_report_command_help;
    grbl.on_report_command_help = onReportCommandHelp;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = onReportOptions;
}

FATFS *sdcard_getfs (void)
{
    if(file.fs == NULL)
        sdcard_mount();

    return file.fs;
}

#endif
