# GRBL SIM 

: by Jens Geisler, Adam Shelly  

Modified by Terje Io for grblHAL. Original implementation for Grbl can be found [here](https://github.com/grbl/grbl-sim).

This repository contains an experimental Grbl simulator that compiles the main Grbl source code into a wrapped executable for use on a computer. No microcontroller required. When the executable is run, the user should be able to interact with the Grbl simulator as if connected to a microcontroller board with Grbl.

*WARNING: Grbl Sim is under heavy development.* So many things may not work, or respond in ways unexpected. At the moment, this code is a proof-of-concept.

## What can you do with Grbl Sim? 

 - Simply check out how Grbl works without needing a microcontroller.
 - Visualize a g-code program by having the simulator parse and execute to a GUI. Fluctuations in feed rates by the acceleration planner can be viewed as well.
 - A powerful debugging tool for development.
 - The microcontroller peripherals are emulated using structs and functions. These could be written to do whatever you need. For example, output simulated step pulses over time and examine its performance.
 - On Linux, hook it to a fake serial port (/dev/ttyFAKE) and use it to test your Grbl interface software:

  -  `> socat PTY,raw,link=/dev/ttyFAKE,echo=0 "EXEC:'./grbl_sim.exe -n -s step.out -b block.out',pty,raw,echo=0" `

 
### Realtime modifications:

  Now simulates microcontroller peripherals in separate thread.  Runs in *aproximate* realtime.  Emphasis on  * **Approximate** *.  Work is underway to speed it up.

## How do you compile Grbl Sim?

- Clone this repository into the directory containing the Grbl source code.  (should be `<repo>/grbl`).  

- Edit the Grbl-Sim Makefile to select the correct `PLATFORM =`  line.  LINUX and WINDOWS are currently supported. 

 - *(You may need to make other modifications to the Makefile and some environment variables for your particular machine. Please share any modifications you find)*

- Run `> make new` to compile Grbl Sim!  


## Validator

Run `gvalidate.exe GCODE_FILE` to validate that grbl will parse your GCODE with no errors.

## Raw telnet connection
**NEW** 

Use the `-p <port>` command line argument to start a raw telnet server for communication instead of using serial simulation via stdin/stdout. This frees up stdin for input to trigger hardware events such as feed hold, cycle start or setting/clearing limit switches. 

