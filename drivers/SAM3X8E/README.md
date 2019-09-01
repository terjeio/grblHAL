## SAM3X8E Driver

A GrblHAL driver for the Atmel SAM3X8E processor on a [Arduino Due board](https://store.arduino.cc/arduino-due).

### *** EXPERIMENTAL *** ###

**NOTE:** pin mappings for a selection of shields/boards are in the pipeline. The default mapping below is only used for debugging and will be replaced.

This driver compiles and uploads from the Arduino IDE and is partially dependent on the Arduino framework.

For flexibility and to reduce overhead the IRQ vector table is relocated to RAM.

Settings are stored in flash and will be overwritten on a fresh upload, this grbl [config app](https://github.com/terjeio/Grbl_CNC_Controls) \(for Windows, a precompiled release is available\) may be used to make a backup of the settings.

Initial pin assignments - to be revised:

``` plain
                                                          [SCL1] A.17
                                                          [SDA1] A.18
                                      NOT USED [  ]         [  ] AREF
                                         IOREF [  ]         [  ] GND
                                         RESET [  ]    A    [13] B.27 - 
                                           3V3 [  ]    R    [12] D.08 - 
                                            5V [  ]    D    [11] D.07 - 
                                           GND [  ]    U    [10] C.29/A.28 - 
                                           GND [  ]    I    [09] C.21 - 
                                           VIN [  ]    N    [08] C.22 - 
                                                       O
                                 X step - A.16 [A0]         [07] C.23
                                  X dir - A.24 [A1]    D    [06] C.24 - Z limit
                              Y disable - A.23 [A2]    U    [05] C.25 - Spindle PWM
                                        - A.22 [A3]    E    [04] C.26/A.29 - 
                                        - A.06 [A4]         [03] C.28 - 
                                        - A.04 [A5]    S    [02] B.25 - X limit
                                 Y step - A.03 [A6]    A    [01] A.09
                                  Y dir - A.02 [A7]    M    [00] A.08
                                                       3
                              Z disable - B.17 [A8]    X    [14] D.04
                                        - B.18 [A9]    8    [15] D.05
                                        - B.19 [A10]   E    [16] A.13 
                                        - B.20 [A11]        [17] A.12 - 
                                        - B.15 [DAC0]       [18] A.11 - 
                                        - B.16 [DAC1]       [19] A.10 - 
                                        - A.00 [CANRX]      [20] B.12 - 
                                  Y lim - A.01 [CANTX]      [21] B.13 - 

C.06 - X disable -----------------------------------+     +----------------------------------------------------- C.04
C.08                                                |     |                                                      C.02
A.19                                                |     |                                                      D.10
C.19                                                |     |                                                      D.09
C.17 - Z step ------------------+                   |     |                        +-------------- Spindle dir - D.03
C.15 - Z dir --------------+    |                   |     |                        |                             D.01
C.13 - Probe ---------+    |    |                   |     |                        |         +- Spindle enable - A.15
B.21                  |    |    |                   |     |                        |         |                   B.26
          [GND] [52] [50] [48] [46] [44] [42] [40] [38] [36] [34] [32] [30] [28] [26] [24] [22] [5V]
          [GND] [53] [51] [49] [47] [45] [42] [42] [39] [37] [35] [33] [31] [29] [27] [25] [23] [5V]
B.14                  |    |    |    |                    |    |                                                 A.14
C.12 - Reset ---------+    |    |    |                    |    |                                                 D.00
C.14 - Feed hold ----------+    |    |                    |    |                                                 D.02
C.16 - Cycle start -------------+    |                    |    |                                                 D.06
C.18 - Safety door ------------------+                    |    |                                                 A.07
A.20                                                      |    |                                                 C.01
C.09                                                      |    +--------------------------------- Coolant mist - C.03
C.07                                                      + ------------------------------------ Coolant flood - C.05

```

---

**NOTE:** Only tested on my bench with an oscilloscope, some switches and LEDs.

---
2019-08-31
