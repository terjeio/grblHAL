## MSP432P401R

A GrblHAL driver for the MSP432P401R ARM processors.

See the Wiki-page for [compiling grblHAL](https://github.com/terjeio/grblHAL/wiki/Compiling-GrblHAL) for instructions for how to import the project, configure the driver and compile.

Available driver options can be found [here](./my_machine.h).

__Update 2019-04-12:__ Some progress...

I am currently working on my EMCO Compact 5 lathe conversion for which this driver is used. I have made a new cross slide and replaced the original spindle motor with a BLDC motor and driver. As the motor/driver-combo turned out to be quite simple I had to add closed loop spindle speed control to the HAL, this triggered quite a few changed in the core grbl code. The core now only handles RPM, RPM -> PWM conversion is completely moved to the driver domain (although the core has some supporting functions that the driver can use). This means that the driver is free to implement closed loop spindle control as part of the driver (likely PWM based) or via external spindle drivers, eg. via ModBus.

The lathe support is still to be regarded as in an alpha phase, my focus will be on closed loop spindle support \(PID\) and constant surface speed \(CSS\) - after that is completed I will revisit threading support.

---

Complemented with the Trinamic TMC2130 version of the CNC BoosterPack silent stepping is within reach for all these processors... 

A GrblHAL driver for the Texas Instruments [MSP432P401R LaunchPad](http://www.ti.com/tool/MSP-EXP432P401R) \(**NOTE:** for the retired Black Edition, will soon be updated for the current Red Edition\).

Optional support for I2C keypad added, can be used for jogging and mist/coolant overrides. Jogging can be toggled between fast, slow and step mode.

Default pin assignments matches the [CNC BoosterPack](https://github.com/terjeio/CNC_Boosterpack) and it uses its onboard EEPROM for configuration storage.

Currently I am using this driver to control a mini mill/router that is mainly intended for drilling/milling PCBs, and I have started work on an automatic tool changer \(ATC\) for this. I do not intend to do isolation milling of my PCBs, I use the [PCB Laser Exposer](https://github.com/terjeio/PCBLaserDesktopApp) to create the layout and add a soldermask. For drilling I use G81 \(a [canned cycle](http://linuxcnc.org/docs/2.4/html/gcode_mill_canned.html#r1_3)\).

I am also working on a mini lathe conversion which uses this driver, code is in place to support my [Grbl DRO \& MPG project](https://github.com/terjeio/GRBL_MPG_DRO_BoosterPack). I have successfully run tests for threading with [G33](http://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G33-Spindle-Sync) so it seems that should be within reach - however, I have to complete my conversion before that part of the driver is going to be finished. If I can sucessfully implement G33 I intend to add support for the [G76](http://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G76-Threading-Canned) canned cycle.

---
2020-08-23
