## ESP32GrblDriver

A GrblHAL driver for the ESP32 processor.

*** Preview version ***

---

__Update 2018-12-14:__ Prototype [CNC BoosterPack](https://github.com/terjeio/CNC_Boosterpack) "motherboard" up and running.

The board has an 8-bit I2C IO-expander and a micro SD card socket on board. Option to use EEPROM on BoosterPack for settings.

I have reorganized the HAL structure a bit so all drivers and the core grbl code must be published simultaneously.
To aid this work I am contemplating making similar CNC BoosterPack "motherboards" for Arduino MKRZERO and Cypress CY8CKIT-059 (PSoC 5).

Complemented with the Trinamic TMC2130 version of the CNC BoosterPack silent stepping is within reach for all these processors... 

---

__Update 2018-11-08:__ I2C keypad interface done, usable for jogging and overrides.

Driver code published \(still for preview\), but not yet the changed core Grbl code required for compilation. No more work will be done on this driver until I receive dev kits and IO-expanders ordered from China.

--- 

__Update 2018-11-07:__ RMT seems to be usable - pulse delay jitter is in the region of 100-200nS, pulse length jitter none. Pulse inversion ok as is pulse delay and length configurations. Running jobs from SD card ok, with M6 support for manual tool change \(allows jogging\).

One worry remains about ESP32 - what happens when there is a cache miss and code needs to be loaded from \(serial\) flash? CPUs suspended? It may take a long time \(I've seen 300uS mentioned\), but I have not found any documentation regarding this. Espressif documentation is not in the same class as for the other MCUs I am accustomed to...

Next step is to get I2C working, first for a keypad - then for an 8-bit IO-expander I have in order.

Updated code will be published later, changes in the core Grbl code related to reporting/streaming has to be verified first. 

---

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

---
2018-12-14
