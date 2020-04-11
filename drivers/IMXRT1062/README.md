## IMXRT1062 Driver

A GrblHAL driver for the NXP IMXRT1062 processor on a [Teensy 4.0 board](https://www.pjrc.com/store/teensy40.html).

This driver compiles and uploads from the Arduino IDE and is partially dependent on the Arduino framework. [Teensyduino](https://www.pjrc.com/teensy/td_download.html) is required and must be added to the Arduino IDE.

Initial default pin assignments can be found in [driver.h](main/driver.h).

The default serial mode is native USB serial using the PJRC USB driver. This can be changed in [driver.h](main/driver.h) by changing `#define USB_SERIAL_GRBL` to `0` for UART1 or `1` for Arduino USB driver.

---

CNC breakout boards:

[GRBL Header for a Teensy 4](https://github.com/phil-barrett/grbl-teensy-4) by Phil Barrett.

---
2020-03-08
