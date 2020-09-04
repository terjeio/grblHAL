## MSP430F5529

A GrblHAL driver for the Texas Instruments MSP430F5529 LaunchPad.

I made this driver just to see if it could be done and for now I do not intend to use it myself... The only 16-bit grbl port available?

Pin assignments matches the [CNC BoosterPack](https://github.com/terjeio/CNC_Boosterpack) and it uses its onboard EEPROM for configuration storage.

See the Wiki-page for [compiling grblHAL](https://github.com/terjeio/grblHAL/wiki/Compiling-GrblHAL) for instructions for how to import the project, configure the driver and compile.

Available driver options can be found [here](./my_machine.h).

**NOTE:** currently only tested on a LaunchPad where the motor step/dir outputs were connected to LEDs.

---
2020-08-23
