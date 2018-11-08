## Driver capabilities at a glance:


|                        | MSP432   | TMC123 | TMC129x | MSP430F5529 | PSoC&nbsp;5 | ESP32 | LPC1769<sup>1</sup> |
|------------------------|----------|--------|---------|-------------|--------|-------|---------|
| MCU speed \(MHz\)      | 48       | 80     | 120     | 25 \(16 bit\) | 80     | 2x240 | 120     |
| Floating point unit    | yes      | yes    | yes     | no          | no     | yes   | no      |
| Non-volatile storage   | External | EEPROM | EEPROM  | External    | EEPROM | Flash | Flash   |
| Number of axes         | 3        | 3      | up to 6 | 3           | 3<sup>2<sup>      | 3    | 3       |
| Variable spindle       | yes      | yes    | yes     | yes         | yes    | yes   | yes     |
| Ramped spindle         | no       | yes    | no      | no          | no     | yes   | no      |
| Spindle at speed       | yes      | no     | no      | no          | no     | yes<sup>3</sup>  | no      |
| Spindle sync           | yes<sup>4<sup>      | no     | no      | no          | no     | no    | no      |
| Constant surface speed | yes      | no     | no      | no          | no     | no    | no      |
| Bluetooth streaming    | no       | no     | no      | no          | no     | yes   | no      |
| Ethernet streaming     | no       | no     | yes     | no          | no     | no    | no      |
| WiFi streaming         | no       | no     | no      | no          | no     | yes   | no      |
| SD Card streaming      | no       | no     | yes     | no          | no     | yes   | yes     |
| I2C Keypad             | yes      | yes    | no      | no          | yes    | yes   | no      |
| MPG stream input       | yes      | no     | yes     | no          | no     | no    | no      |
| Manual tool change<sup>5</sup>     | yes      | no     | yes     | no          | no     | yes   | no      |
| Automatic tool change<sup>6</sup>   | planned| no     | no      | no          | no     | no    | no      |
| Laser PPI mode<sup>7</sup>          | no     | yes    | no      | no          | no     | no    | no      |
| Trinamic support<sup>8</sup>        | no     | no     | no      | no          | no     | no    | no      |

<br><sup>1</sup> Not complete an untested, for someone else to finish? Compiles ok.
<br><sup>2</sup> Should be fairly easy to extend.
<br><sup>3</sup> In combination with ramped spindle, signals end of ramp - not programmed speed obtained.
<br><sup>4</sup> To be completed, work in good progress.
<br><sup>5</sup> Protocol extension, requires compatible GCode sender.
<br><sup>6</sup> Grbl core has functionality and HAL driver entry points are provided, needs verification!
<br><sup>7</sup> Driver specific M codes added for control. PPI = Pulses Per Inch.
<br><sup>8</sup> Code ready for verification for TMC2130 drivers, new CNC BoosterPack design for Texas Instrument LaunchPads complete.

Please note that some of the capabilities should be fairly easy to port from one driver to another, but be aware some are dependent on MCU peripheral availability and thus not possible, or hard, to port.

The fastest and most deterministic MCU seems to be TMC129x, ESP32 is not bad but it is a bit unstable - maybe due to outstanding [bugs](https://github.com/espressif/esp-idf/issues) in the [ESP-IDF](https://github.com/espressif/esp-idf) and the system architecture - program code is stored off chip in external serial flash.