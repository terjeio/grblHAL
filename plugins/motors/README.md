## Trinamic stepper driver plugins

These plugins adds settings, M-Code extensions and reports for TMC2130, TMC2209 and TMC5160 stepper drivers.

They support Marlin-style M-codes such as `M122`, `M911`, `M912`, `M913` and `M914` - some with extensions and some with sligthly different syntax.

### M-codes

#### M122 - output debug info or reset driver.

`M122 axes <H-> <S-> <Q-> <I>`

* `H` - 0 = SFILT off, 1 = SFILT on \(TMC2130\) only.
* `I` - reinitialize driver.
* `S` - 0 = disable StallGuard and live output of sg-value, 1 = enable StallGuard and live output. 
* `Q` - not yet enabled.

Examples:  
`M122` - output debug info for all enabled drivers. This is a plain text report, do not issue from a sender!  
`M122 Y` - output debug info for Y axis only. This is a plain text report, do not issue from a sender!  
`M122 I` - reset drivers.  
`M122 X S1` - enable live output of StallGuard value for tuning. Do not enable when running g-code jobs!  

#### M906 - set stepper current

`M906 axes`

Note: Stepper current is not permanently stored.

Example:  
`M122 X700 Y950`

#### M911 - Report prewarn flags

`M911`  

#### M912 - Clear prewarn flags

`M912`

#### M913 - Set hybrid threshold

`M913 axes`

Note: Hybrid threshold is not permanently stored.

Example:  
`M913 X31`

 #### M914 - Set homing sensitivity.

`M914 axes`

Note: Homing sensitivity is not permanently stored.

Example:  
`M914 X31`

### Settings

Settings are provided for axis enable, homing, stepper current, microsteps and sensorless homing. More to follow.

#### $14x - Stepper current. x = 0 for X-axis, 1 for Y axis etc. 

#### $15x - Motor microsteps. x = 0 for X-axis, 1 for Y axis etc.

Valid values are 1, 2, 4, , 8, 16, 32, 64, 128 and 256.

#### $338 - Driver enable

Parameter is a axismask where value is sum of X=0, Y=1, Z=2 etc.

__NOTE__: Some boards does not allow mixed drivers, for these this setting is not available.

#### $339 - Sensorless homing enable

Parameter is a axismask where value is sum of X=0, Y=1, Z=2 etc.

---

The driver and driver configuration has to be extended to support these plugins.

Dependencies:

[Trinamic library](https://github.com/terjeio/Trinamic-library)

[Trinamic TMC2130 I2C<>SPI Bridge](https://github.com/terjeio/Trinamic_TMC2130_I2C_SPI_Bridge) \(optional\)

---
2020-01-06
