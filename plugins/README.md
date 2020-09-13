## grblHAL plugins

grblHAL's HAL interface is based on function pointers that may be used to add functionality without any need to change the core grbl code. They may also be changed on the fly to redirect calls, eg. the SD-card interface utilizes this to temporarily redirect input from the serial stream to the SD card.

NOTE: A plugin needs to be supported by the processor specific driver - as a minimum a initialization call has to be made. 

* [EEPROM](eeprom/README.md) - for non-volatile storage of settings/data.

* [Encoder](encoder/README.md) - for adjusting overrides. Support for jogging is planned.

* [Keypad](keypad/README.md) - for I2C based keypad. Support for jogging etc.

* [Networking](networking/README.md) - for telnet or websocket communication.

* [Odometer](odometer/README.md) - for logging of distances travelled and machining time. __NOTE:__ For review.

* [Plasma/THC](plasma/README.md) - for plasma machines. __NOTE:__ Under development, testers wanted.

* [SD card](sdcard/README.md) - for executing gcode stored on SD card.

* [Spindle](spindle/README.md) - for spindles controlled via MODBUS. __NOTE:__ Not yet verified, testers wanted.

* [Trinamic](trinamic/README.md) - for Trinamic TMC2130 stepper drivers controlled via SPI or I2C.

---
2020-09-13
