## STM32F4xxGrblDriver

A GrblHAL driver for the STM32F401CC and STM32F411RE ARM processors.

Loosely based on code from robomechs [6-AXIS-USBCNC-GRBL](https://github.com/robomechs/6-AXIS-USBCNC-GRBL) port, updated for [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.htm) and the latest STM HAL drivers where appropriate.

See the Wiki-page for [compiling grblHAL](https://github.com/terjeio/grblHAL/wiki/Compiling-GrblHAL) for instructions for how to import the project, configure the driver and compile.

Available driver options can be found [here](Inc/my_machine.h).

A method for flashing the Nucleo F411RE is to drop the `.bin` file on the NODE_F411RE flash drive. The `.bin` file can be found in the `Debug` or `Release` folder depending on how it was compiled. Note that the file can be dragged from the IDE _Project Explorer_.

If compiling for STM32F411:

* change the symbol `STM32F401xC` in project properties _C\/C++ General > Paths and Symbols > Symbols_ to `STM32F411xE`.

* if for the [Nucleo F411RE development board](https://www.st.com/en/evaluation-tools/nucleo-f411re.html) then add the symbol `NUCLEO_F411` to project properties in _C\/C++ General > Paths and Symbols > Symbols_.

__NOTE:__ Internal flash page for parameters, is not at the end of the flash memory due to size restrictions. This means each firmware upgrade will erase any saved parameters. 

---
2020-08-23
