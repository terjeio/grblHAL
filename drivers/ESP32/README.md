## ESP32GrblDriver

A GrblHAL driver for the ESP32 processor.

*** Preview version ***

Work in progress, I do not currently have access to the hardware I need for complete testing so this is just a code preview.
Parts of the code is commented out as I only have a TTGO WROOM module with less GPIO pins available than the target board\(s\).

I am hopeful that the RMT feature for autonomous step pulse generation will work out ok, I have seen postings on the net that suggest it should. Timing and jitter must be checked out with a scope. Does it work out then step pulse delay and inversion is supported, this with no CPU overhead.

Input signal debouncing and millisecond delay is implemented with the help of FreeRTOS timer tasks, and the previous delay in the stepper ISR for disabling the steppers at the end of a job is done with a callback from the millisecond timer. Also, I have added an option for ramped spindle speed changes.

Since an I2C bus is free I am tinkering with making a "motherboard" for my [CNC BoosterPack](https://github.com/terjeio/CNC_Boosterpack), this has EEPROM on board for storing the settings - to avoid wearing out the flash. I am also considering adding support for a 4-bit I2C IO-expander to that (for coolant and spindle control), thus freeing up enough GPIO pins to allow a SD card and keypad to be added without losing any of the standard Grbl I/O.

Some time ago I started writing code for controlling Trinamic drivers - via a homemade I2C <> SPI bridge (MSP430) - maybe I will add that too...

Please note that this driver is not made using the Arduino IDE/framework, I am using Eclipse and CMake.

---

The standard GRBL/config.h should be modified with these changes at the top:

```
Add: #include "esp_attr.h"
Change: #define ISR_CODE to #define ISR_CODE IRAM_ATTR
```
