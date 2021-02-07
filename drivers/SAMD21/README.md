## SAMD21 Driver

__IMPORTANT:__ Arduino version 1.8.11 has breaking changes in the library code that causes compilation to fail. Use an earlier version or select another driver for your project.  
See issue #224 for more information. Also note that I am __*not*__ going to make a workaround for this since the compilation error is in the shared grblHAL core code.

A GrblHAL driver for the Atmel SAMD21 processor on a [Arduino MKR ZERO board](https://store.arduino.cc/arduino-mkrzero).

See the Wiki-page for [compiling grblHAL](https://github.com/terjeio/grblHAL/wiki/Compiling-GrblHAL) for instructions for how to import the project, configure the driver and compile.

Available driver options can be found [here](main/my_machine.h).

__Update 2019-08-10:__ Added support for optional I2C IO Expander, I2C EEPROM, I2C keypad and Trinamic TMC2130 drivers \(via I2C bridge\). FatFS updated to R0.13c.

---

This driver compiles and uploads from the Arduino IDE and is partially dependent on the Arduino framework.

For flexibility and to reduce overhead the vector table is relocated to RAM. On my todo list is to improve GPIO speed even more than already done.

Settings are stored in flash and will be overwritten on a fresh upload, this grbl [config app](https://github.com/terjeio/Grbl_CNC_Controls) \(for Windows, a precompiled release is available\) may be used to make a backup of the settings.

Optional streaming of GCode from a SD card is implemented according to [this specification](https://github.com/bdring/Grbl_Esp32/wiki/Using-the-SD-Card) except for filtering that I have retained for now. Implementation is based on a port for [FatFs R0.09b](http://www.elm-chan.org/fsw/ff/00index_e.html), on my todo list is to update this to the latest release \(currently at R0.13c\). Also on my todo list is to automatically mount the SD card when inserted.

Proposed pin assignments:

``` plain
             ARef - PA03 (25) [  ]       [  ] +5V
      Spindle dir - PA02 (15) [  ]       [  ] Vin
      Cycle start - PB02 (16) [02]       [  ] +3V3
        Feed hold - PB03 (17) [03]   M   [  ] GND
            Probe - PA04 (18) [  ]   K   [  ] CPU RESET
           X step - PA05 (19) [  ]   R   [  ] (14) PB22 - UART TX
           Y step - PA06 (20) [  ]       [  ] (13) PB23 - UART RX
           Z step - PA07 (21) [  ]   Z   [  ] (12) PA09 - Flood
            X lim - PA22 (00) [06]   E   [  ] (11) PA08 - Mist
            Y lim - PA23 (01) [07]   R   [  ] (10) PA19 - Steppers enable
            X dir - PA10 (02) [  ]   O   [00] (09) PA17 - Reset
            Y dir - PA11 (03) [  ]       [01] (08) PA16 - Z lim
            Z dir - PB10 (04) [10]       [05] (07) PA21 - Spindle enable
      Safety door - PB11 (05) [11]       [04] (06) PA20 - Spindle PWM
```

---

CNC BoosterPack pin assignments:
``` plain
             ARef - PA03 (25) [  ]       [  ] +5V
            Y dir - PA02 (15) [  ]       [  ] Vin
      Safety door - PB02 (16) [02]       [  ] +3V3
            Reset - PB03 (17) [03]   M   [  ] GND
            Probe - PA04 (18) [  ]   K   [  ] CPU RESET
           X step - PA05 (19) [  ]   R   [  ] (14) PB22 - UART TX
           Y step - PA06 (20) [  ]       [  ] (13) PB23 - UART RX
           Z step - PA07 (21) [  ]   Z   [  ] (12) PA09 - I2C SCL
            Z lim - PA22 (00) [06]   E   [  ] (11) PA08 - I2C SDA
            Y lim - PA23 (01) [07]   R   [  ] (10) PA19 - GPIO1
            Z dir - PA10 (02) [  ]   O   [00] (09) PA17 - Feed hold
            X dir - PA11 (03) [  ]       [01] (08) PA16 - Cycle start
            GPIO2 - PB10 (04) [10]       [05] (07) PA21 - X lim
    Keypad strobe - PB11 (05) [11]       [04] (06) PA20 - Spindle PWM
```

NOTE: I2C I/O-expander is used for coolant, spindle and stepper enable signals, I2C EEPROM for settings. Prototype PCB ready for test.

---

Numbers in round brackets are the Arduino pin assignments, in square brackets the GPIO interrupt (EXT_INT) assignments.

**NOTE:** Only tested on my bench with an oscilloscope, some switches and LEDs.

---
2021-02-07
