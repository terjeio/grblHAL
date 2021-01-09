## grblHAL driver for STM32F3xx processors

See the Wiki-page for [compiling grblHAL](https://github.com/terjeio/grblHAL/wiki/Compiling-GrblHAL) for instructions for how to import the project, configure the driver and compile.

__NOTE:__ This is an initial version, currently only bench tested with an oscillocope using a [F303 Blackpill board](https://robotdyn.com/stm32f303cct6-256-kb-flash-stm32-arm-cortexr-m4-mini-system-dev-board-3326a9dd-3c19-11e9-910a-901b0ebb3621.html).

The F303 Blackpill board is electrically _and_ pin number compatible with F103 based *pills.  
It is _**not**_ pin function compatible and thus cannot always be used as a drop-in replacement for F103 based *pills.  
Notably routing I2C2 SDA/SCL signals to PB10/PB11 is not possible. The F303 based Blackpill cannot be used as a replacement for breakout boards needing these pins for I2C communication. 

The F303 should be preferred over F103 for grblHAL use as it has a FPU and plenty of RAM \(48K vs 20K\) and flash \(256K vs 128K\).  
It is also a safer alternative for grblHAL as the F103 is at the edge of beeing usable, e.g. adding plugins is not always possible due to memory limitations.

Available driver options can be found [here](Inc/my_machine.h).

---
2021-01-09
