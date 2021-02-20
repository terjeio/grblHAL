/*
  validator.c -  Grbl G-code validation

  Part of Grbl Simulator

  Copyright (c) 2012 Adam Shelly

  2020 - modified for grblHAL by Terje Io

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

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

#include "platform.h"
#include "grbl/hal.h"
#include "grbl/report.h"
#include "grbl/protocol.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"

typedef struct arg_vars {
    // Output file handles
    FILE *input_file;
    FILE *output_file;
    uint8_t echo;
    uint8_t silent;   
} arg_vars_t;

arg_vars_t args;
const char* progname;
uint8_t exit_code = 0;

int usage (const char* badarg)
{
    if (badarg)
        printf("Unrecognized option %s\n", badarg);

    printf("Usage: \n"
     "%s <Options> [input_file]\n"
     "  Options:\n"
     "    -o <output file> : use output file instead of stdout\n"
     "    -e        : echo input to output\n"
     "    -s        : silent, no output only return code \n"
     "\n  Parses gcode from stdin or input line, prints grbl's expected response"
     "\n  Returns 0 on successs, or line number of error",
     progname);

    return -1;
}

status_code_t validator_report_status_message (status_code_t status_code)
{
    report_status_message(status_code);

    if (status_code && !exit_code) {
        printf("EXITING %d\n",status_code);
        exit_code = status_code;
        sys.abort = 1;
    }

    return status_code;
}

// Read fom input
int16_t serial_read()
{
    int16_t data = fgetc(args.input_file);

    if (data == PLATFORM_EXTRA_CR)
        return(0);

    if (args.echo)
        fputc(data, args.output_file); 

    plan_reset();

    if (sys.abort || feof(args.input_file) || data == 0x06 || data == -1) { 
        sys.abort = 1;
        return SERIAL_NO_DATA;
    }

    return data;
}

// Write to output
void serial_write (const char *data)
{
    if (!args.silent) {
        char c, *ptr = (char *)data;
        while((c = *ptr++) != '\0')
            fputc(c, args.output_file);
    }
}

int main(int argc, char *argv[])
{
    int positional_args=0;

    //defaults
    args.input_file = stdin;
    args.output_file = stdout;
    args.echo = 0;
    args.silent = 0;

    progname = argv[0];

    while (argc > 1) {
        argv++; argc--;
        if (argv[0][0] == '-') {
            switch(argv[0][1]) {

                case 'e':  //echo mode
                    args.echo = 1;
                    break;

                case 's': //silent
                    args.silent = 1;
                    break;

                case 'o': //output file
                    argv++; argc--;
                    args.output_file = fopen(*argv,"w");
                    if (!args.output_file) {
                        perror("fopen");
                        printf("Error opening : %s\n",*argv);
                        return(usage(0));
                    }
                    break;

                case 'h':
                    return usage(NULL);
                default:
                    return usage(*argv);
            }
        } else { //handle positional arguments
            positional_args++;
            switch(positional_args) {

                case 1: //input file
                    args.input_file = fopen(*argv,"r");
                    if (!args.input_file) {
                        perror("fopen");
                        printf("Error opening : %s\n",*argv);
                        return(usage(0));
                    }
                    break;

                default:
                    return usage(*argv);
            }
        }
    }

    // Clear all and set some core function pointers
    memset(&grbl, 0, sizeof(grbl_t));
    grbl.on_execute_realtime = protocol_execute_noop;
    grbl.protocol_enqueue_gcode = protocol_enqueue_gcode;

    // Clear all and set some HAL function pointers
    memset(&hal, 0, sizeof(grbl_hal_t));
    hal.version = HAL_VERSION; // Update when signatures and/or contract is changed - driver_init() should fail
    hal.driver_reset = dummy_handler;
    hal.irq_enable = dummy_handler;
    hal.irq_disable = dummy_handler;
    hal.nvs.size = GRBL_NVS_SIZE;

    if(!driver_init())
       return -1;

    // TODO: read settings from EEPROM.dat if exists?

    nvs_buffer_init();
    settings_init();

    report_init_fns();
    grbl.report.status_message = validator_report_status_message;
    grbl.report.feedback_message = report_feedback_message;

    hal.stream.read = serial_read;
    hal.stream.write = serial_write;
    hal.stream.write_all = serial_write;

    protocol_main_loop();

    return exit_code;
}
