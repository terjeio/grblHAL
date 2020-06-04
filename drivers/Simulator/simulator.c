/*
  simulator.c - functions to simulate how the buffer is emptied and the
                stepper interrupt is called

  Part of Grbl Simulator

  Copyright (c) 2012-2014 Jens Geisler
  Copyright (c) 2014-2015 Adam Shelly

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

#include <stdio.h>
#include <stdbool.h>
#include <stdlib.h>
#include <unistd.h>

#include "simulator.h"
#include "eeprom.h"
#include "mcu.h"
#include "driver.h"

#include "grbl/grbl.h"

void sim_nop (void)
{
}

sim_vars_t sim = {
    .on_init = sim_nop,
    .on_tick = sim_nop,
    .on_byte = sim_nop,
    .on_shutdown = sim_nop
};

// Setup 
void init_simulator (float time_multiplier)
{
    sim.speedup = time_multiplier;
    sim.baud_ticks = F_CPU / 115200;

    sim.on_init();
}

// Shutdown simulator - call exit hooks, save eeprom is taken care of in atexit handler
void shutdown_simulator (void)
{
    sim.on_shutdown();
}

void simulate_hardware (bool do_serial)
{
    //do one tick
    sim.masterclock++;
    sim.sim_time = (float)sim.masterclock / (float)F_CPU;

    mcu_master_clock();

    if (do_serial)
        simulate_serial();
/*
    mcu_gpio_in(&gpio[LIMITS_PORT0], sys_position[X_AXIS] >= 250 ? 1 : 0, 1);
    mcu_gpio_in(&gpio[LIMITS_PORT1], sys_position[X_AXIS] >= 280 ? 1 : 0, 1);
*/
  //TODO:
  //  check limit pins,  call pinchange interrupt if enabled
  //  can ignore pinout int vect - hw start/hold not supported
}

// Runs the hardware simulator at the desired rate until sim.exit is set
void sim_loop (void)
{
    uint64_t simulated_ticks=0;
    uint32_t ns_prev = platform_ns();
    uint64_t next_byte_tick = F_CPU;   //wait 1 sec before reading IO.

    while (sim.exit != exit_OK  ) { //don't quit until idle

        if (sim.speedup) {
            //calculate how many ticks to do.
            uint32_t ns_now = platform_ns();
            uint32_t ns_elapsed = (ns_now - ns_prev) * sim.speedup; //todo: try multipling nsnow
            simulated_ticks += F_CPU / 1e9f * ns_elapsed;
            ns_prev = ns_now;
        }
        else
            simulated_ticks++;  //as fast as possible

        while (sim.masterclock < simulated_ticks) {
            // only read serial port as fast as the baud rate allows
            bool read_serial = (sim.masterclock >= next_byte_tick);

            // do low level hardware
            simulate_hardware(read_serial);

            // do app-specific per-tick processing
            sim.on_tick();

            if (read_serial) {
                next_byte_tick += sim.baud_ticks;
                // do app-specific per-byte processing
                sim.on_byte();
            }
        }

        platform_sleep(25); // yield
    }
}

// Print serial output to args.serial_out_file
void sim_serial_out (uint8_t data)
{
    static uint8_t buf[128] = {0};
    static uint8_t len = 0;
    static bool continuation = 0;

    buf[len++] = data;
    // print when we get to newline or run out of buffer
    if(data == '\n' || data == '\r' || len >= 127) {
        if (args.comment_char && !continuation)
            fprintf(args.serial_out_file, "%c ", args.comment_char);
        buf[len] = '\0';
        fprintf(args.serial_out_file, "%s", buf);
        // don't print comment on next line if we are just printing to avoid buffer overflow
        continuation = (len >= 128); 
        len = 0;
    }
}

// Print serial output to sim.socket_fd stream
void sim_socket_out (uint8_t data)
{
    static uint8_t buf[128] = {0};
    static uint8_t len = 0;
    static bool continuation = 0;

    buf[len++] = data;
    // print when we get to newline or run out of buffer
    if(data == '\n' || data == '\r' || len >= 127) {
//        if (args.comment_char && !continuation)
//            fprintf(args.serial_out_file, "%c ", args.comment_char);
        if(sim.socket_fd) {
            if(write(sim.socket_fd, buf, len) < 0)
                exit(-10);
        }
        // don't print comment on next line if we are just printing to avoid buffer overflow
        continuation = (len >= 128); 
        len = 0;
    }
}

