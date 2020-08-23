## TM4C123GrblDriver

A GrblHAL driver for the Texas Instruments [Tiva C \(EK-TM4C123GXL\) Launchpad](http://www.ti.com/tool/EK-TM4C123GXL)

See the Wiki-page for [compiling grblHAL](https://github.com/terjeio/grblHAL/wiki/Compiling-GrblHAL) for instructions for how to import the project, configure the driver and compile.

Available driver options can be found [here](./my_machine.h).

Optional support for [I2C keypad](https://github.com/terjeio/I2C-interface-for-4x4-keyboard) added, can be used for jogging and mist/coolant overrides. Jogging can be toggled between fast, slow and step mode. Laser PPI mode (PPI - Pulses Per Inch) may also be enabled, this to ensure constant power delivered to the workpiece regardless of speed. PPI mode parameters are set by some user-defined M-codes:

#### M123P\<n\>

\<n\> = pulses per inch.

#### M124P\<n\>

\<n\> = pulse length in microseconds

Default pin assignments matches the [CNC BoosterPack](https://github.com/terjeio/CNC_Boosterpack)

**NOTE:** R9 and R10 should be removed from the LaunchPad and the appropriate shorts \(0R resistors\) added to the CNC BoosterPack when this is used with the default pin assignments. Also, Q1-Q3 should possibly be removed as they act as pull-downs for limit switches inputs, not the best if these inputs are to be configured with weak pull-ups.

**NOTE:** Only tested on my bench with an oscilloscope. A slightly different driver has been used for my CO2 laser cutter/engraver for a while, this on a custom PCB.


---
2020-08-23
