## Plasma/THC

Under development. Based on [LinuxCNC specification](http://linuxcnc.org/docs/devel/html/plasma/plasmac-user-guide.html#config-panel), with limitations.

#### $350 - Mode of operation

| Mode | Description |
|------|-------------|
| 0    | Uses an external arc voltage input to calculate both Arc Voltage (for Torch Height Control) and Arc OK.|
| 1    | Uses an external arc voltage input to calculate Arc Voltage (for Torch Height Control).<br>Uses an external Arc OK input for Arc OK.|
| 2    | Uses an external Arc OK input for Arc OK.<br>Use external up/down signals for Torch Height Control.|

#### THC

| Setting                    | Modes | Description |
|----------------------------|-------|-------------|
| $351 - Delay               | 0,1,2 | This sets the delay (in seconds) measured from the time the Arc OK signal is received until Torch Height Controller (THC) activates.|
| $352 - Threshold \(V\)     | 0,1,2 | This sets the voltage variation allowed from the target voltage before for THC makes movements to correct the torch height.|
| $353 - P Gain              | - | This sets the Proportional gain for the THC PID loop.<br>This roughly equates to how quickly the THC attempts to correct changes in height. |
| $354 - I Gain              | - | This sets the Integral gain for the THC PID loop.<br>Integral gain is associated with the sum of errors in the system over time and is not always needed.|
| $355 - D Gain              | - | This sets the Derivative gain for the THC PID loop.<br>Derivative gain works to dampen the system and reduce over correction oscillations and is not always needed.|
| $356 - VAD Threshold \(%\) | - | \(Velocity Anti Dive\) This sets the percentage of the current cut feed rate the machine can slow to before locking the THC to prevent torch dive.|
| $357 - Void Override \(%\) | - | This sets the size of the change in cut voltage necessary to lock the THC to prevent torch dive \(higher values need greater voltage change to lock THC\)|

#### ARC

| Setting                | Modes | Description |
|------------------------|-------|-------------|
| $358 - Fail Timeout    | 0,1,2 | This sets the amount of time (in seconds) PlasmaC will wait between commanding a "Torch On"<br>and receiving an Arc OK signal before timing out and displaying an error message.|
| $359 - Retry Delay     | 0,1,2 | This sets the time (in seconds) between an arc failure and another arc start attempt.
| $360 - Max Retries     | 0,1,2 | This sets the number of times PlasmaC will attempt to start the arc.|
| $361 - Voltage Scale   | - | This sets the arc voltage input scale and is used to display the correct arc voltage.|
| $362 - Voltage Offset  | - | This sets the arc voltage offset and is used to display zero volts when there is zero arc voltage input.|
| $363 - Height Per Volt | - | This sets the distance the torch would need to move to change the arc voltage by one volt.<br>Used for manual height manipulation only.|
| $364 - Ok High Voltage | - | This sets the voltage threshold below which Arc OK signal is valid.|
| $365 - Ok Low Voltage  | - | This sets the voltage threshold above which the Arc OK signal is valid.|

#### Supported M-Codes

* `M62 P2` will disable THC \(Synchronized with Motion\)

* `M63 P2` will enable THC \(Synchronized with Motion\)

* `M64 P2` will disable THC \(Immediately\)

* `M65 P2` will enable THC \(Immediately\)

* `M67 E3 Q-` Velocity Reduction \(Immediately\)

* `M68 E3 Q-` Velocity Reduction \(Synchronized with Motion)

The Q-word for M67 and M68 is the percentage of the programmed feed rate the actual feed rate will be changed to.

The minimum percentage allowed is 10%, values below this will be set to 10%.  
The maximum percentage allowed is 100%, values above this will be set to 100%.

#### Dependencies:

Driver must support a number of [ioports ports](../../templates/ioports.c).

#### Credits:

LinuxCNC documentation linked to above.

---
2020-10-21
