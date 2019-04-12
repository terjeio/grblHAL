## Spindle Simulator ##

#### A spindle simulator for Texas Instuments [MSP430 Value Line LaunchPads](http://www.ti.com/tool/MSP-EXP430G2#) ####

The simulator takes spindle on/off signal and filtered PWM input and converts it to encoder outputs. Its main use is for excercising closed loop feedback loops in order to see how they responds and for verifying encoder interfaces.

Spindle parameters are set via a simple command interface via a serial connection over USB \(the MSP 430 ApplicationUART\).

#### Command set: ####

RPM:\<n\>

Calculates and set spindle encoder output to correspond \<n\> revolutions per minute when AUTO mode is off, default is 400.

PPR:\<n\>

Sets spindle encoder output to \<n\> pulses per revolution, default is 120.

AUTO:\<0|1\>

Specifies operating mode: 0, manual mode - encoder outputs are controlled by RPM setting. 1 \(default\), automatic mode - encoder outputs are controlled by spindle on/off and PWM input.

SPINDLE:\<0|1\>

Specifies spindle encoder outputs in manual mode: 0 - off, 1 - on.

STEP:\<n\>

Adds \<n\> RPM offset to spindle encoder output in automatic mode.

LOCK:\<0|1\>

Specifies lock mode, 0 \(default\) - AUTO mode on: add STEP RPM offset to encoder output. AUTO mode off: adds a fraction of PWM input to encoder output, 1 - ignore STEP inputs When AUTO mode is on, scaled PWM input otherwise.

---

The PWM signal has to be RC-filtered in order to provide a DC signal to the ADC input. The time constant of this filter can be regarded as an approximation of how the spindle/spindle motor mass will affect the control loop. Currently I am using 1K and 10uF when testing.

#### Pin assignments: ####

```
P1.3 - filtered PWM input to ADC
P1.6 - encoder index pulse output
P2.0 - trigger output, provides a 100 uS pulse on STEP changes
P2.1 - encoder pulse output
P2.2 - spindle on/off (input)
```

---

**NOTES:**

The LaunchPad runs at 3.6V, ADC conversion is scaled to 3.3V for max output. RPM range is currently hardcoded to 0 - 1023.

Do not use this in setups using more than 3.6V for signalling without appropriate level shifting!

---
