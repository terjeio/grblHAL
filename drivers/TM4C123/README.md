## TM4C123GrblDriver

A GrblHAL driver for the Texas Instruments Tiva C \(TM4C123G\) Launchpad

This driver needs to be complemented with an UART or USB driver for host communication.

Optional support for I2C keypad added, can be used for jogging and mist/coolant overrides. Jogging can be toggled between fast, slow and step mode. Laser PPI mode (PPI - Pulses Per Inch) may also be enabled, this to ensure constant power delivered to the workpiece regardless of speed. PPI mode parameters are set by some user-defined M-codes:

#### M123P\<n\>

\<n\> = pulses per inch.

#### M124P\<n\>

\<n\> = pulse length in microseconds

Default pin assignments matches the [CNC BoosterPack](https://github.com/terjeio/CNC_Boosterpack)

__NOTE:__ R9 and R10 should be removed from the LaunchPad and the appropriate shorts \(0R resistors\) added to the CNC BoosterPack when this is used with the default pin assignments. Also, Q1-Q3 should possibly be removed as they act as pull-downs for limit switches inputs, not the best if these inputs are to be configured with weak pull-ups.

__NOTE:__ Only tested on my bench with an oscilloscope. A slightly different driver has been used for my CO2 laser cutter/engraver for a while, this on a custom PCB.