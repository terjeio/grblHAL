/*
  simulator.h - functions to simulate how the buffer is emptied and the
    stepper interrupt is called

  Part of Grbl Simulator

  Copyright (c) 2012 Jens Geisler
  Copyright (c) 2014-2015 Adam Shelly

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

#ifndef simulator_h
#define simulator_h

#include <stdio.h>

#include "platform.h"

typedef void (*sim_hook_fp)(void); // Signature of functions to be inserted in sim loop.

//simulation globals
typedef struct sim_vars {
    uint64_t masterclock;
    double sim_time;  // current time of the simulation.
    uint8_t started;  // don't start timers until first char recieved.
    enum {exit_NO, exit_REQ, exit_OK} exit;
    float speedup;
    int32_t baud_ticks;
    int socket_fd;
    uint8_t (*getchar)(void);
    void (*putchar)(uint8_t);
    sim_hook_fp on_init;
    sim_hook_fp on_tick;
    sim_hook_fp on_byte;
    sim_hook_fp on_shutdown;
} sim_vars_t;

extern sim_vars_t sim;

typedef struct arg_vars {
    // Output file handles
    FILE *block_out_file;
    FILE *step_out_file;
    FILE *serial_out_file;
    char eeprom_file[128];
    double step_time;       // Minimum time step for printing stepper values. Given by user via command line
    uint8_t comment_char;   // Char to prefix comments; default  '#' 
    uint16_t port;          // Port number for telnet communication
} arg_vars_t;

extern arg_vars_t args;

// global system variable structure for position etc.
//extern system_t sys;

// setup avr simulation
void init_simulator (float time_multiplier);

// Shutdown simulator - run shutdown hooks, save eeeprom
void shutdown_simulator (void);

// Simulates the hardware until sim.exit is set.
void sim_loop (void);

// Call the stepper interrupt until one block is finished
// (defined in serial.c)
void simulate_serial (void);

//print serial output to stdout or file
void sim_serial_out (uint8_t data);
void sim_socket_out (uint8_t data);

#endif
