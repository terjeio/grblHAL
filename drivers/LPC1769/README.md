## LPC1769GrblDriver

A GrblHAL driver for the NXP LPC176x processors.

Loosely based on the official [grbl-LPC port](https://github.com/gnea/grbl-LPC).

*** Incomplete - needs more work on pin assignments ***

__Update 2020-01-01:__ Added [board map](./Re-ARM%20Shield%20pin%20mappings/ramps_1.6_map.md) file for [Ramps 1.6](https://reprap.org/wiki/RAMPS_1.6) on [Re-ARM board](https://www.panucatt.com/Re_ARM_for_RAMPS_p/ra1768.htm) hacked for programming via Segger J-Link. Improved pin assignment handling and fixed some bugs. USB comms and SD card seems to be working ok for this board, however only limited testing done.

__Update 2019-08-08:__ Changed IDE to [MCUXpresso v11](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE) and linked against [LPCOpen development platform](https://www.nxp.com/design/microcontrollers-developer-resources/lpcopen-libraries-and-examples/lpcopen-software-development-platform-lpc17xx:LPCOPEN-SOFTWARE-FOR-LPC17XX) libraries. I2C EEPROM on [OM13085 LPCXpresso board](https://www.nxp.com/design/microcontrollers-developer-resources/lpc-microcontroller-utilities/lpcxpresso-board-for-lpc1769-with-cmsis-dap-probe:OM13085) and SD card supported. 

Currently tested running on a OM13085 board with oscilloscope only. Communication is configured via CMSIS-DAP VCOM connected to UART0, code for "native" USB VCOM provided but untested.

__NOTE:__ earlier commits of this driver are full of bugs! Notably the bitbanding for GPIO is allocated in the SRAM region for this processor, _not_ PERI region...

---

2020-01-02
