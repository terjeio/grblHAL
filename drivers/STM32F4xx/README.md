## STM32F1xxGrblDriver

A GrblHAL driver for the STM32F401CC ARM processor.

Loosely based on code from robomechs [6-AXIS-USBCNC-GRBL](https://github.com/robomechs/6-AXIS-USBCNC-GRBL) port, updated for [STM32CubeIDE](https://www.st.com/en/development-tools/stm32cubeide.htm) and the latest STM HAL drivers where appropriate.

__NOTE:__ Internal flash page for parameters, is not at the end of the flash memory due to size restrictions. This means each firmware upgrade will erase any saved parameters. 


---
2019-08-03
