## Laser PPI

Under development. Adds 3 M-codes for controlling PPI (Pulse Per Inch) mode for lasers.

* `M112 P-` turns PPI mode on or off. The P-word specifies the mode. `0` = off, `1` = on.

* `M113 P-` The P-word specifies the PPI value. Default value on startup is `600`.

* `M114 P-` The P-word specifies the pulse length in microseconds. Default value on startup is `1500`.

__NOTE:__ These M-codes are not standard and may change in a later release. 

A description of what PPI is and how it works can be found [here](http://www.engraversnetwork.com/support/universal-lasers/laser-how-tos/dpi-vs-ppi-laser/).

I have a customized version of [ioSender](https://github.com/terjeio/Grbl-GCode-Sender) that I use for my CO2-laser, this has not yet been published but I may do so if this is of interest.

Dependencies:

Driver must support pulsing spindle on pin. Only for processors having a FPU that can be used in an interrupt context.

## Laser coolant

Under development. Adds one M-code for controlling \(tube\) coolant.

* `M115 P- <Q>- <R>-` turns coolant modes on or off.

The P-word specifies coolant mode. `0` = off, `1` = on.
If a flow detector is connected then the Q-word is a delay in seconds to wait before checking flow is ok.
If a temperature sensor is connected then the R-word specifies the upper temperature limit before an alarm is generated, set to 0 to disable monitoring.

__NOTE:__ This M-code is not standard and may change in a later release. 

_M115 example:_

`M115 P1 Q2 R30 (switch on coolant, wait for two seconds before verifying flow is ok and set monitoring threshold to 30 deg. Celsius)`

Dependencies:

Driver must have at least one [ioports port](../../templates/ioports.c) available for turning coolant on/off.
Additional ports are required for coolant ok and temperature input in order to enable use of the `Q` and `R` words.

---
2020-10-18
