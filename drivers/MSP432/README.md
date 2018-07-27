## MSP432

A GrblHAL driver for Texas Instruments MSP432 Launchpad \(Black Edition\)

Optional support for I2C keypad added, can be used for jogging and mist/coolant overrides. Jogging can be toggled between fast, slow and step mode.

Default pin assignments matches the [CNC BoosterPack](https://github.com/terjeio/CNC_Boosterpack) and it uses its onboard EEPROM for configuration storage.

Currently I am using this driver to control a mini mill/router that is mainly intended for drilling/milling PCBs, and I have started work on an automatic tool changer \(ATC\) for this. I do not intend to do isolation milling of my PCBs, I use the [PCB Laser Exposer](https://github.com/terjeio/PCBLaserDesktopApp) to create the layout and add a soldermask. For drilling I use G81 \(a [canned cycle](http://linuxcnc.org/docs/2.4/html/gcode_mill_canned.html#r1_3)\).

I am also working on a mini lathe conversion which uses this driver, code is in place to support my [Grbl DRO \& MPG project](https://github.com/terjeio/GRBL_MPG_DRO_BoosterPack). I have successfully run tests for threading with [G33](http://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G33-Spindle-Sync) so it seems that should be within reach - however, I have to complete my conversion before that part of the driver is going to be finished. If I can sucessfully implement G33 I intend to add support for the [G76](http://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G76-Threading-Canned) canned cycle.
