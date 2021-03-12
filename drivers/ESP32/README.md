## ESP32GrblDriver

A GrblHAL driver for the ESP32 processor. *** Preview version ***

### How to build using ESP-IDF v4.3:

While this manual briefly describes basic build process on Linux OS, you can find more details
as well as differences for building on other OS at this webpage:

https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html#step-2-get-esp-idf

First you have to prepare esp-idf v4.3:

```bash
#Create directory and clone esp-idf into it:
mkdir -p ~/esp
cd ~/esp
git clone -b release/v4.3 --recursive --shallow-submodules https://github.com/espressif/esp-idf.git

#Prepare build environment and toolchain:
cd ~/esp/esp-idf
./install.sh
. ~/esp/esp-idf/export.sh
```

Once you have ESP-IDF prepared, go back to grblHAL directory and copy `grbl/*` to `drivers/ESP32/grbl/` in grblHAL repo
as well as desired plugins into the respective folders. eg. `plugins/spindle/*` to `drivers/ESP32/spindle/` and so on...

Modify settings in `grbl/config.h` and `CMakeLists.txt` as needed.
Pin assignments and board specific config is in `*_map.h` files for each individual board.

Go into the `drivers/ESP32/` directory and run `idf.py build`.
This will build the firmware image which can be later flashed into ESP32 device.

Note that `idf.py` command is only available in terminal window which was previously configured
using the `. ~/esp/esp-idf/export.sh` command.

After build is completed you will be instructed on how to flash firmware into the device.
Typicaly you can use command similar to this: `idf.py -p /dev/ttyUSB0 flash`

Once flashing is complete, your CNC controller is ready to be configured and used.


### Using Docker

If you're familiar with [Docker](https://docker.io), you can use it to build grblHAL in a self-contained environment without installing the complete toolchain on your system:

- prepare and configure the codebase as described above
- build with `docker run -it --rm -v $(pwd):/grbl -w /grbl/drivers/ESP32 espressif/idf:release-v4.3 idf.py build`
- flash with `docker run -it --rm -v $(pwd):/grbl --privileged -v /dev:/dev -w /grbl/drivers/ESP32 espressif/idf:release-v4.3 idf.py -p /dev/ttyUSB0 flash`

### Changelog/Notes:

---

__NOTE:__ `grbl/config.h` or `CMakeLists.txt` may need modification before compilation. If needed an `#error` (with instructions) will be generated when compiling.

---

__Update 2020-02-06:__ Added option for secondary serial input stream with input pin for switching on/off, indended for external MPGs. **For verification!**

---

---

__Update 2020-02-02:__ Added board mapping files for [Bart's v3.5 and v4 ESP32 boards](http://www.buildlog.net/blog/). v4 is now the default mapping. **For verification!** 

---

__Update 2019-11-25:__ Added basic support for Luc's [ESP3D-WEBUI](https://github.com/luc-github/ESP3D-webui) with a backend written from scratch utilizing GrblHALs function pointer based API.

Currently missing is authentication and notification support, and possibly something else that I have overlooked. All GrblHAL settings are available from the GRBL configuration page, however the extended settings provided by GrblHAL does not show any help text. The ESP3D Settings page does not show settings not supported by this backend implementation.

This addon has not yet been tested connected to a machine - I challenge some brave souls to do that and report back by posting issues!

__NOTE:__ Configuration has been simplified a bit, primarily change options in [`CMakeLists.txt`](https://github.com/terjeio/grblHAL/blob/master/drivers/ESP32/CMakeLists.txt) to enable/disable. Enabled plugin code still has to be copied from the plugins folder though.

---

__Update 2019-11-16:__ Added initial \(and rather crude\) UI for captive portal when WiFi is configured in APSTA mode. Ajax-based, no postbacks for updates. It works with Firefox on my laptop...

---

__Update 2019-11-15:__ Added AP and APSTA WiFi options. Initial webserver support for reading/setting $-settings via JSON-messages. In APSTA WiFi mode webserver supports querying for available access points and connecting to selected, again via JSON messages. No UI for this yet though...

__NOTE:__ IDF version 3.2 or later required for compilation if webserver is to be enabled.

__IMPORTANT:__ [Network setting-id's](https://github.com/terjeio/grblHAL/wiki/Additional-or-extended-settings) are changed and many are added. Any previous network settings will be lost if upgrading.

---

__Update 2019-08-01:__ Now uses code for keypad and SD card options from common plugins. Added support for Trinamic TMC2130 plugin (currently via SPI <> I2C bridge).

__NOTE:__ If plugins are to be used [`driver.h`](https://github.com/terjeio/grblHAL/blob/master/drivers/ESP32/driver.h) and [`CMakeLists.txt`](https://github.com/terjeio/grblHAL/blob/master/drivers/ESP32/CMakeLists.txt) has to be updated. Details can be found in these files.


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

### Credits:

index.htm.gz is Copyright (c) 2019 Luc Lebosse - from his [ESP3D-WEBUI](https://github.com/luc-github/ESP3D-webui), I may have pulled a few lines from his backend code too.

dns_server.c is Copyright (c) 2019 Tony Pottier - from his [ESP32 WiFi Manager](https://github.com/tonyp7/esp32-wifi-manager) 

Snippets of code is extracted from Espressif ESP-IDF examples which are public domain.

---
2020-02-10
