## Trinamic TMC2130 stepper driver plugin

This plugin adds settings, M-Code extensions and reports for TMC1230 stepper drivers.

I supports Marlin-style M-codes such as `M122`, `M911`, `M912`, `M913` and `M914` - some with extensions and some with sligthly different syntax.

Settings \($n=...\) are provided for axis enable, homing, stepper current, microsteps and sensorless homing. More to follow.

The driver and driver configuration has to be extended to support this plugin, currently this has been done for the [MSP432 driver](https://github.com/terjeio/grblHAL/tree/master/drivers/MSP432).

Dependencies:

[Trinamic library](https://github.com/terjeio/Trinamic-library)

[Trinamic TMC2130 I2C<>SPI Bridge](https://github.com/terjeio/Trinamic_TMC2130_I2C_SPI_Bridge) \(optional\)

---
2019-07-25
