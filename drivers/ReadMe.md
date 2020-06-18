## Driver capabilities at a glance:

|                         | MSP432   | MSP432E401Y |TMC123  | TMC129x | MSP430F5529 | PSoC&nbsp;5 | ESP32 | SAMD21 | LPC1768/1769<sup>1</sup> | STM32F1xx<sup>11</sup> | STM32F4xx | SAM3X8E |IMXRT1062|
|-------------------------|----------|-------------|--------|---------|-------------|--------|-------|-------|---------|---------|---------|---------|---------|
| Board                   | LaunchPad| LaunchPad   | LaunchPad | LaunchPad| LaunchPad | CY8CKIT-059 |     | MKRZERO| Re-Arm | Bluepill| Blackpill| Due| Teensy 4|
| MCU speed \(MHz\)       | 48       | 120         | 80     | 120     | 25 \(16 bit\)| 80    | 2x240 | 48    | 100/120 | 72      | 84/100  | 84      | 600     |
| Floating point unit     | yes      | yes         | yes    | yes     | no          | no     | yes   | no    | no      | no      | yes     | no      | yes     |
| Non-volatile storage    | I2C EEPROM | EEPROM    | EEPROM | EEPROM  | I2C EEPROM |  EEPROM | Flash/I2C EEPROM | Flash/I2C EEPROM | Flash/I2C EEPROM| Flash/I2C EEPROM| Flash | Flash | Flash/I2C EEPROM|
| Number of axes          | 3        | up to 6     | 3      | up to 6 | 3           | 3<sup>2</sup> | 3 | 3  | up to 5 | up to 6 | up to 6 | up to 6 | up to 5 |
| Variable spindle        | yes      | yes         | yes    | yes     | yes         | yes    | yes   | yes   | yes     | yes     | yes     | yes     | yes     |
| Ramped spindle          | no       | yes         | yes    | yes     | no          | no     | yes   | no    | no      | no      | no      | no      | no      |
| Inverted spindle PWM    | yes      | yes         | yes    | yes     | yes         | no     | yes   | no    | no      | yes     | no      | no      | no      |
| RC Servo/ESC for spindle<sup>13</sup> | yes | yes | yes   | yes     | yes         | no     | yes   | yes   | yes     | yes     | yes     | yes     | ?       |
| Spindle at speed        | yes      | no          | no     | no      | no          | no     | yes<sup>3</sup> | no    | no      | no      | no      | no      |
| Spindle sync            | yes<sup>4</sup> | no   | no     | no      | no          | no     | no    | no    | no      | no      | no      | no      | no      |
| Constant surface speed  | yes      | no          | no     | no      | no          | no     | no    | no    | no      | no      | no      | no      | no      |
| Closed loop spindle RPM | yes<sup>4</sup> | no   | no     | no      | no          | no     | no    | no    | no      | no      | no      | no      | no      |
| Native USB streaming    | no       | no          | no     | no      | no          | no     | no    | yes   | yes?    | yes     | yes     | yes     | yes     |
| Bluetooth streaming     | no       | no          | no     | no      | no          | no     | yes   | no    | no      | no      | no      | no      | no      |
| Telnet streaming \(raw\)| no       | ethernet    | no     | ethernet| no          | no     | wifi  | no    | no      | no      | no      | no      | no      |
| Websocket streaming     | no       | ethernet    | no     | ethernet| no          | no     | wifi  | no    | no      | no      | no      | no      | no      |
| SD Card streaming       | no       | TBC         | no     | yes     | no          | no     | yes   | yes   | yes     | yes     | no      | no      | TBC     |
| I2C Keypad              | yes      | no          | yes    | no      | no          | yes    | yes   | yes   | no      | yes     | no      | no      | TBC     |
| I2C IO Expander         | no       | no          | no     | no      | no          | no     | yes   | yes   | no      | no      | no      | no      | no      |
| MPG stream input        | yes      | yes         | no     | yes     | no          | no     | no    | no    | no      | no      | no      | no      | TBC     |
| Manual tool change<sup>5</sup> |yes| yes         | yes    | yes     | no          | yes    | yes   | yes   | yes     | yes     | no      | yes     | yes     |
| Automatic tool change<sup>6</sup> | planned| no  | no     | no      | no          | no     | no    | no    | no      | no      | no      | no      | no      |
| Laser PPI mode<sup>7</sup>| no     | no          | yes    | no      | no          | no     | no    | no    | no      | no      | no      | no      | no      |
| Trinamic support<sup>8</sup> | TBC<sup>9</sup> | TBC<sup>9</sup> | TBC<sup>9</sup>| TBC<sup>9</sup> | no | no | TBC<sup>9</sup> | TBC<sup>9</sup> | no | TBC<sup>9</sup> | TBC<sup>9</sup> | TBC<sup>9</sup> | TBC<sup>9</sup> |
| Runs as FreeRTOS task   | no       | yes         | no     | option  | no          | no     | yes   | no    | no      | no      | no      | no      | no      |
| CNC BoosterPack support | 1        | 2           | 1      | 2       | 1           | no     | yes<sup>10</sup> | yes<sup>10</sup> | no | yes<sup>10</sup> | no      | no      | yes<sup>10</sup> |
| WebUI support           | no       | no          | no     | no      | no          | no     | yes<sup>12</sup> | no  | no  | no | no      | no      | no      |
| Compiler/IDE            | CCS      | CCS         | CCS    | CCS     | CCS         | PSOC Creator | ESP IDF | Arduino | MCUExpresso | STMCubeIDE | STMCubeIDE | Arduino | Arduino |

<br><sup>1</sup> Not complete and untested, for someone else to finish? Compiles ok.
<br><sup>2</sup> Should be fairly easy to extend.
<br><sup>3</sup> In combination with ramped spindle, signals end of ramp - not programmed speed obtained.
<br><sup>4</sup> To be completed \(TBC\), work in good progress.
<br><sup>5</sup> Protocol extension, requires compatible GCode sender.
<br><sup>6</sup> Grbl core has functionality and HAL driver entry points are provided, needs verification!
<br><sup>7</sup> Driver specific M codes added for control. PPI = Pulses Per Inch.
<br><sup>8</sup> Initial version ready for TMC2130 drivers, new [CNC BoosterPack](https://github.com/terjeio/CNC_Boosterpack) design for Texas Instrument LaunchPads ready and initial testing ok.
<br><sup>9</sup> Implementation provided for TMC2130, with initial focus on [I2C](https://github.com/terjeio/Trinamic_TMC2130_I2C_SPI_Bridge) interfaces. Work in good progress - code published.
<br><sup>10</sup> Via "motherboard" that accommodates [CNC BoosterPack](https://github.com/terjeio/CNC_Boosterpack) and processor board. Prototypes made, includes 8-bit I2C GPIO expander \(not available/needed for STM32F1xx or IMXRT1062\). "motherboards" has option for isolated level-shifted spindle PWM or DC output.  BoosterPack has an onboard EEPROM and an I2C level shifter.
<br><sup>11</sup> Requires 128KB of flash \(STM32F103CB\), many STM32F1038B based Blue Pill boards has that too?
<br><sup>12</sup> Luc's [ESP3D-WEBUI](https://github.com/luc-github/ESP3D-webui), backend partially implemented. Work in progress. 
<br><sup>13</sup> Set `$33=50` (PWM frequency), `$34=5`, `$35=5` and `$36=10` to generate a "standard" PWM signal: 20ms repetition rate, 1 - 2ms pulse length range. 

Please note that some of the capabilities should be fairly easy to port from one driver to another, but be aware some are dependent on MCU peripheral availability and thus not possible, or hard, to port.

The fastest and most deterministic MCUs seems to be MSP432E401Y and TMC129x, ESP32 is not bad but it is a bit unstable - maybe due to outstanding [bugs](https://github.com/espressif/esp-idf/issues) in the [ESP-IDF](https://github.com/espressif/esp-idf) and the system architecture - program code is stored off chip in external serial flash.

---
2020-06-18
