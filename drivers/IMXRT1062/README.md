## IMXRT1062 Driver

A GrblHAL driver for the NXP IMXRT1062 processor on a [Teensy 4.0 board](https://www.pjrc.com/store/teensy40.html).

### *** EXPERIMENTAL *** ###

This driver compiles and uploads from the Arduino IDE and is partially dependent on the Arduino framework. [Teensyduino](https://www.pjrc.com/teensy/td_download.html) is required and must be added to the Arduino IDE.

Initial default pin assignments can be found in [driver.h](main/driver.h)

Note that this driver currently has a couple of known issues:

1. System will hang when a hard limit is triggered.
2. USB serial stops working when hard limit is triggered.
2. Keyboard jogging causes intermittent hang, likely on jog cancel (movement stops so the command is acted upon).
4. There is no hard reset pin available!
5. Delayed step pulse is not yet implemented.

Some of these issues may be due to the processor having a relatively large instruction cache and beeing fast...

1 can be solved by uncommenting line 392 in protocol.c: `hal.delay_ms(20, NULL);`  
The delay should be shortened? I do not remember now why I have left it commented out...  
2 can be likely be solved by adding a call after line 392 in protocol.c: `if(hal.execute_realtime) hal.execute_realtime(sys.state);`  

The default serial mode is over physical UART connected to pin 0 and 1, an external USB > UART interface connected to these is recommended for now.  
Native USB serial _can_ be enabled in [driver.h](main/driver.h) by changing `#define USB_SERIAL` to `1` - note that this has undegone very little testing by me.

---

**NOTE:** Only tested on my bench with an oscilloscope, some switches and LEDs.

---
2020-02-14

