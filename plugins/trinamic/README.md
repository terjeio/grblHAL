## Trinamic TMC2130 stepper driver plugin

This plugin adds settings, M-Code extensions and reports for TMC2130 and TMC2209 stepper drivers.

I supports Marlin-style M-codes such as `M122`, `M911`, `M912`, `M913` and `M914` - some with extensions and some with sligthly different syntax.

* M122 XYZHSQ - output debug info
* M906 XYZQ - stepper current
* M911 - ReportPrewarnFlags
* M912 - ClearPrewarnFlags:
* M913 - HybridThreshold
* M914 - HomingSensivity

Settings \($n=...\) are provided for axis enable, homing, stepper current, microsteps and sensorless homing. More to follow.

#### $338 - Driver enable

| Mode | Description |
|------|-------------|
| 0    | Uses an external arc voltage input to calculate both Arc Voltage (for Torch Height Control) and Arc OK.|
| 1    | Uses an external arc voltage input to calculate Arc Voltage (for Torch Height Control).<br>Uses an external Arc OK input for Arc OK.|
| 2    | Uses an external Arc OK input for Arc OK.<br>Use external up/down signals for Torch Height Control.|

#### $339 - Sensorless homing enable

| Mode | Description |
|------|-------------|
| 0    | Uses an external arc voltage input to calculate both Arc Voltage (for Torch Height Control) and Arc OK.|
| 1    | Uses an external arc voltage input to calculate Arc Voltage (for Torch Height Control).<br>Uses an external Arc OK input for Arc OK.|
| 2    | Uses an external Arc OK input for Arc OK.<br>Use external up/down signals for Torch Height Control.|


The driver and driver configuration has to be extended to support this plugin.

Dependencies:

[Trinamic library](https://github.com/terjeio/Trinamic-library)

[Trinamic TMC2130 I2C<>SPI Bridge](https://github.com/terjeio/Trinamic_TMC2130_I2C_SPI_Bridge) \(optional\)

---
2020-12-25
