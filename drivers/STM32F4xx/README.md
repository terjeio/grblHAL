## STM32F4xxGrblDriver

A grblHAL driver for the STM32F401CC, STM32F411RE and STM32F446RE ARM processors.

Loosely based on code from robomechs [6-AXIS-USBCNC-GRBL](https://github.com/robomechs/6-AXIS-USBCNC-GRBL) port, updated for [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.htm) and the latest STM HAL drivers where appropriate.

See the Wiki-page for [compiling grblHAL](https://github.com/terjeio/grblHAL/wiki/Compiling-GrblHAL) for instructions for how to import the project, configure the driver and compile.

Available driver options can be found [here](Inc/my_machine.h).

A method for flashing the Nucleo F411RE is to drop the `.bin` file on the NODE_F411RE flash drive. The `.bin` file can be found in the `Debug` or `Release` folder depending on how it was compiled. Note that the file can be dragged from the IDE _Project Explorer_.

If compiling for STM32F411, change/add some symbols in project properties _C\/C++ General > Paths and Symbols > Symbols_.

* remove the symbol `STM32F401xC` and add `STM32F411xE`.

* if for the [Nucleo F411RE development board](https://www.st.com/en/evaluation-tools/nucleo-f411re.html) then add the symbol `NUCLEO_F411`.

* if the oscillator frequency is different from the default 25 MHz then add the symbol `HSE_VALUE` and set the value to the frequency in Hz. E.g. `8000000` for 8 Mhz.

If compiling for STM32F446, change/add some symbols in project properties _C\/C++ General > Paths and Symbols > Symbols_.

* remove the symbol `STM32F401xC` and add `STM32F446xx`.

* if for the [Nucleo F446RE development board](https://www.st.com/en/evaluation-tools/nucleo-f446re.html) then add the symbol `NUCLEO_F446`.

* if the oscillator frequency is different from the default 25 MHz then add the symbol `HSE_VALUE` and set the value to the frequency in Hz. E.g. `8000000` for 8 Mhz.

If compiling for STM32F407 (STM32F4-discovery), change/add some symbols in project properties _C\/C++ General > Paths and Symbols > Symbols_.

* remove the symbol `STM32F401xC` and add `STM32F407xx`.

* if the oscillator frequency is different from the default 25 MHz then add the symbol `HSE_VALUE` and set the value to the frequency in Hz. E.g. `8000000` for 8 Mhz. This is necessary for STM32F4-discovery.

__NOTE:__ Internal flash page for parameters is not at the end of the flash memory due to size restrictions. This means each firmware upgrade will erase any saved parameters.
---

CNC breakout boards:

[CNC breakout for Nucleo-64](https://github.com/terjeio/CNC_Breakout_Nucleo64) by Terje Io.

---
2021-01-02
