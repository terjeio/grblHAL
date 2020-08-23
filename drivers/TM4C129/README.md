## TM4C1294GrblDriver

A GrblHAL 6-axis driver for the Texas Instruments [EK-TM4C1294XL Launchpad](http://www.ti.com/tool/EK-TM4C1294XL#)

See the Wiki-page for [compiling grblHAL](https://github.com/terjeio/grblHAL/wiki/Compiling-GrblHAL) for instructions for how to import the project, configure the driver and compile.

Available driver options can be found [here](base/my_machine.h).

Default pin assignments matches the [CNC BoosterPack](https://github.com/terjeio/CNC_Boosterpack)
Two CNC BoosterPacks are needed to drive more than 3 axes.

Now with experimental SD card support and manual tool change via M6 (requires compatible sender). 
Manual tool change also works when streaming from SD card!

**NOTE:** Only tested on my bench with an oscilloscope.

**NOTE:** How to build instructions to be added later - some TivaWare files has to be copied to the project.

**NOTE:** Some of the files in this project has a license text in the header that has been superseded by the text in [TI-BSD-EULA.txt](TI-BSD-EULA.txt). TODO: create a manifest detailing which parts are third-party.

---
2020-08-23