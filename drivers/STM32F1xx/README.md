## STM32F1xxGrblDriver

A GrblHAL driver for the STM32F103CB ARM processor.

Loosely based on code from robomechs [6-AXIS-USBCNC-GRBL](https://github.com/robomechs/6-AXIS-USBCNC-GRBL) port, updated for [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.htm) and the latest STM HAL drivers where appropriate.

See the Wiki-page for [compiling grblHAL](https://github.com/terjeio/grblHAL/wiki/Compiling-GrblHAL) for instructions for how to import the project, configure the driver and compile.

Available driver options can be found [here](Inc/my_machine.h).

__NOTE:__ Requires 128KB of flash! The .ioc design file is not included as modifying the project via the designer requires a bit of cleanup after. The STM HAL is bypassed for all time critical code and to avoid bloat.

__Update 2019-08-03:__ Added support for I2C EEPROM and I2C Keypad, SD card and Trinamic TMC2130 plugins (TMC2130 plugin currently via SPI <> I2C bridge).

__NOTE:__ The SD card plugin requires the SPI1 port to be remapped, disabling the JTAG/SWJ programming interfaces. This will be done on the first mount operation (via a `$FM` system command) causing the processor to hang. A power cycle is then required to get it working again.

To reenable programming a special system command, `$PGM`, can be used - issue this followed by a hard reset or power cycle to do so.

---
2020-08-23
