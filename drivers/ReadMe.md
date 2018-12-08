## Driver capabilities at a glance:


|                         | MSP432   | MSP432E401Y |TMC123  | TMC129x | MSP430F5529 | PSoC&nbsp;5 | ESP32 | SAMD21 | LPC1769<sup>1</sup> |
|-------------------------|----------|-------------|--------|---------|-------------|--------|-------|-------|---------|
| MCU speed \(MHz\)       | 48       | 120         | 80     | 120     | 25 \(16 bit\) | 80     | 2x240 | 48    | 120     |
| Floating point unit     | yes      | yes         | yes    | yes     | no          | no     | yes   | no    | no    |
| Non-volatile storage    | External | EEPROM      | EEPROM | EEPROM  | External    | EEPROM | Flash | Flash | Flash   |
| Number of axes          | 3        | up to 6     | 3      | up to 6 | 3           | 3<sup>2</sup>      | 3    | 3    | 3       |
| Variable spindle        | yes      | yes         | yes    | yes     | yes         | yes    | yes   | yes   | yes     |
| Ramped spindle          | no       | yes         | yes    | no      | no          | no     | yes   | no    | no      |
| Spindle at speed        | yes      | no          | no     | no      | no          | no     | yes<sup>3</sup>  | no    | no      |
| Spindle sync            | yes<sup>4</sup>      | no          |no     | no      | no          | no     | no    | no    | no      |
| Constant surface speed  | yes      | no          | no     | no      | no          | no     | no    | no    | no      |
| Native USB streaming    | no       | no          | no     | no      | no          | no     | no    | yes   | yes?    |
| Bluetooth streaming     | no       | no          | no     | no      | no          | no     | yes   | no    | no      |
| Ethernet streaming      | no       | yes         | no     | yes     | no          | no     | no    | no    | no      |
| WiFi streaming          | no       | no          | no     | no      | no          | no     | yes   | no    | no      |
| SD Card streaming       | no       | TBC         | no     | yes     | no          | no     | yes   | no    | yes     |
| I2C Keypad              | yes      | no          | yes    | no      | no          | yes    | yes   | no    | no      |
| MPG stream input        | yes      | yes         | no     | yes     | no          | no     | no    | no    | no      |
| Manual tool change<sup>5</sup>      | no          | yes      | no     | yes     | no          | no     | yes   | yes   | no      |
| Automatic tool change<sup>6</sup>    | planned| no          | no     | no      | no          | no     | no    | no    | no      |
| Laser PPI mode<sup>7</sup>           | no     | no          | yes    | no      | no          | no     | no    | no    | no      |
| Trinamic support<sup>8</sup>         | no     | no          | TBC<sup>9</sup>    | no      | no          | no     | no    | no    | no      |
| Runs as FreeRTOS task   | no       | yes         | no     | option  | no          | no     | yes   | no    | no      |
| CNC BoosterPack support | 1        | 2           | 1      | 2       | 1           | no     | no<sup>10</sup>    | no    | no      |

<br><sup>1</sup> Not complete and untested, for someone else to finish? Compiles ok.
<br><sup>2</sup> Should be fairly easy to extend.
<br><sup>3</sup> In combination with ramped spindle, signals end of ramp - not programmed speed obtained.
<br><sup>4</sup> To be completed \(TBC\), work in good progress.
<br><sup>5</sup> Protocol extension, requires compatible GCode sender.
<br><sup>6</sup> Grbl core has functionality and HAL driver entry points are provided, needs verification!
<br><sup>7</sup> Driver specific M codes added for control. PPI = Pulses Per Inch.
<br><sup>8</sup> Code ready for verification for TMC2130 drivers, new [CNC BoosterPack](https://github.com/terjeio/CNC_Boosterpack) design for Texas Instrument LaunchPads complete.
<br><sup>9</sup> Preliminary implementation provided for TMC2130, SPI and [I2C](https://github.com/terjeio/Trinamic_TMC2130_I2C_SPI_Bridge) interfaces. Work in progress.
<br><sup>9</sup> Via "motherboard" that accommodates [CNC BoosterPack](https://github.com/terjeio/CNC_Boosterpack) and processor board. Work in progress.

Please note that some of the capabilities should be fairly easy to port from one driver to another, but be aware some are dependent on MCU peripheral availability and thus not possible, or hard, to port.

The fastest and most deterministic MCUs seems to be MSP432E401Y and TMC129x, ESP32 is not bad but it is a bit unstable - maybe due to outstanding [bugs](https://github.com/espressif/esp-idf/issues) in the [ESP-IDF](https://github.com/espressif/esp-idf) and the system architecture - program code is stored off chip in external serial flash.

---
2018-12-08
