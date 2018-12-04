## SAMD21 Driver (RFC - to be published later)

A GrblHAL driver for the Atmel SAMD21 processor on a [Arduion MKR ZERO board](https://store.arduino.cc/arduino-mkrzero).

### *** EXPERIMENTAL *** ###

This driver compiles and uploads from the Arduino IDE and is partially dependent on the Arduino framework.

For flexibility and to reduce overhead the vector table is relocated to RAM. On my todo list is to improve GPIO speed even more than already done.

Settings are stored in flash and will be overwritten on a fresh upload, this grbl [config app](https://github.com/terjeio/Grbl_CNC_Controls) \(for Windows, a precompiled release is available\) may be used to make a backup of the settings.

Optional streaming of GCode from a SD card is implemented according to [this specification](https://github.com/bdring/Grbl_Esp32/wiki/Using-the-SD-Card) except for filtering that I have retained for now. Implementation is based on a port for [FatFs R0.09b](http://www.elm-chan.org/fsw/ff/00index_e.html), on my todo list is to update this to the latest release \(currently at R0.13c\). Also on my todo list is to automatically mount the SD card when inserted.

Proposed pin assignments:

``` plain
  Steppers enable - PA03 (25) [  ]       [  ] +5V
          Coolant - PA02 (15) [  ]       [  ] Vin
   Spindle enable - PB02 (16) [02]       [  ] +3V3
      Spindle dir - PB03 (17) [03]   M   [  ] GND
            Probe - PA04 (18) [  ]   K   [  ] CPU RESET
           X step - PA05 (19) [  ]   R   [  ] (14) PB22 - UART TX
           Y step - PA06 (20) [  ]       [  ] (13) PB23 - UART RX
           Z step - PA07 (21) [  ]   Z   [  ] (12) PA09 - I2C SCL
            X lim - PA22 (00) [06]   E   [  ] (11) PA08 - I2C SDA
            Y lim - PA23 (01) [07]   R   [  ] (10) PA19 - Z lim
            X dir - PA10 (02) [  ]   O   [00] (09) PA17 - Feed hold
            Y dir - PA11 (03) [  ]       [01] (08) PA16 - Cycle start
            Z dir - PB10 (04) [10]       [05] (07) PA21 - Safety door / Mist
            Reset - PB11 (05) [11]       [04] (06) PA20 - Spindle PWM
```

Numbers in round brackets are the Arduino pin assignments, in square brackets the GPIO interrupt (EXT_INT) assignments.

Using pin 25 for stepper enable is perhaps not possible as this pin is decoupled with a total of 4.8uF... Testing required!

__NOTE:__ Only tested on my bench with an oscilloscope, some switches and LEDs.
