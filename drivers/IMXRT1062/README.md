## IMXRT1062 Driver

A GrblHAL driver for the NXP IMXRT1062 processor on a [Teensy 4.0 board](https://www.pjrc.com/store/teensy40.html).

### *** EXPERIMENTAL *** ###

This driver compiles and uploads from the Arduino IDE and is partially dependent on the Arduino framework. [Teensyduino](https://www.pjrc.com/teensy/td_download.html) is required and must be added to the Arduino IDE.

Initial default pin assignments can be found in [driver.h](main/driver.h)

Note that this driver currently has a couple of known issues:

1. System will hang when a hard limit is triggered. __Fixed__.
2. USB serial stops working when hard limit is triggered. __Fixed__.
3. Keyboard jogging causes intermittent hang, likely on jog cancel (movement stops so the command is acted upon). __Fixed__.
4. There is no hard reset pin available!
5. Delayed step pulse is not yet implemented. __Done__.

The default serial mode is over physical UART connected to pin 0 and 1, an external USB > UART interface connected to these is recommended for now.  
Native USB serial _can_ be enabled in [driver.h](main/driver.h) by changing `#define USB_SERIAL_GRBL` to `1` - note that this has undegone limited testing by me.

---

**NOTE:** Only tested on my bench with an oscilloscope, some switches and LEDs.

---
2020-02-21
