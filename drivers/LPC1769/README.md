## LPC1769GrblDriver

A GrblHAL driver for the NXP LPC1769 processor.

Loosely based on the official [grbl-LPC port](https://github.com/gnea/grbl-LPC).

*** Incomplete - needs work on pin assignments ***

__Update 2019-08-08:__ Changed IDE to [MCUXpresso v11](https://www.nxp.com/design/software/development-software/mcuxpresso-software-and-tools/mcuxpresso-integrated-development-environment-ide:MCUXpresso-IDE) and linked against [LPCOpen development platform](https://www.nxp.com/design/microcontrollers-developer-resources/lpcopen-libraries-and-examples/lpcopen-software-development-platform-lpc17xx:LPCOPEN-SOFTWARE-FOR-LPC17XX) libraries. I2C EEPROM on [OM13085 LPCXpresso board](https://www.nxp.com/design/microcontrollers-developer-resources/lpc-microcontroller-utilities/lpcxpresso-board-for-lpc1769-with-cmsis-dap-probe:OM13085) and SD card supported. 

Currently tested running on a OM13085 board with oscilloscope only. Communication is configured via CMSIS-DAP VCOM connected to UART0, code for "native" USB VCOM provided but untested.

__NOTE:__ earlier commits of this driver are full of bugs! Notably the bitbanding for GPIO is allocated in the SRAM region for this processor, _not_ PERI region...

---

2019-08-08
