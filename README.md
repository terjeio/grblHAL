## GrblHAL ##

GrblHAL has [many extensions](https://github.com/terjeio/grblHAL/wiki) that may cause issues with some senders. As a workaround for these a [compile time option](https://github.com/terjeio/grblHAL/wiki/Changes-from-grbl-1.1#workaround) has been added that disables extensions selectively. 

Windows users may try [my sender](https://github.com/terjeio/Grbl-GCode-Sender), binary releases can be found [here](https://github.com/terjeio/Grbl-GCode-Sender/releases). It has been written to complement grblHAL and has features such as proper keyboard jogging, automatic reconfiguration of DRO display for up to 6 axes, lathe mode including conversational G-Code generation, 3D rendering, macro support etc. etc.

---

Build 20200811:

* Part one of large overhaul of configuration system.
This has been done to allow a global configuration file and/or setting compile time options as compiler symbols/macros with the `-D` compiler command line option.
* Networking \(cabled ethernet\), SD Card and I2C keypad plugin support added to [Teensy 4.x driver.](drivers/IMXRT1062) A Teensy 4.1 is required for networking.
* Tool number range changed from 8-bit \(0-255\) to 32-bit \(0-4294967295\). Note that if the optional tool table is enabled the max tool number is limited by number of entries in the tool table.
* M60 and corresponding pallet shuttle HAL entry point added. No driver support for this yet.

_Configuration system changes:_

Symbols \(#define macros\) that are unlikely to be changed or is used for conditional compilation has been moved from [grbl/config.h](grbl/config.h) to [grbl/grbl.h](grbl/grbl.h).
Symbols used for conditional compilation can be overridden by `-D` compiler command line options or by editing [grbl/config.h](grbl/config.h).

[grbl/defaults.h](grbl/defaults.h) is now used for default values for settings that can be changed at run-time with the `$$` command. __Important:__ grbl/defaults.h should only be changed when adding new settings.

[grbl/config.h](grbl/config.h) is now used for overriding default values in grbl/grbl.h or grbl/defaults.h. Out of the box all overridable symbols are commented out, uncomment and change as needed to override default definitions in grbl/grbl.h or grbl/defaults.h.

These changes are part of a long term plan to create a user friendly front end for configuring and building grblHAL.

---

Build 20200805:
* **Important:** settings has been changed again and settings will be restored to defaults after updating. Backup & restore!
* Added tool change handler for [manual tool change](https://github.com/terjeio/grblHAL/wiki/Manual,-semi-automatic-and-automatic-tool-change) on M6. Four different modes available, selectable with [`$341` setting](https://github.com/terjeio/grblHAL/wiki/Additional-or-extended-settings#manual-tool-change-settings).
* STMF4xx driver updated for optional I2C EEPROM plugin support.

---

Build 20200722:
* **Important:** settings version has been changed again and settings will be restored to defaults after updating. Backup & restore! 
* Changed step pulse width and delay settings from int to float and reduced minimum allowed value to 2 microseconds<sup>1</sup>. Useful for very high step rates.
* New plugin for [quadrature encoder input](https://github.com/terjeio/grblHAL/issues/73#issuecomment-659222664) for up to 5 encoders \(driver dependent\). Can be used to adjust overrides and has rudimentary support for MPG functionality. Work in progress and the iMXRT1062 \(Teensy 4\) driver is currently the only driver with low-level support for this (one encoder).
* New plugin for [ModBus VFD](https://github.com/terjeio/grblHAL/issues/68) spindle controllers. Untested and with limited driver support in this build.
* Added setting, `$340` for spindle at speed tolerance \(percent\). If spindle fails to reach speed within limits in 4 seconds alarm 14 will be raised. Set to 0 to disable. Availability driver dependent.
* All [new settings](https://github.com/terjeio/grblHAL/wiki/Additional-or-extended-settings) are now possibly to set independent of the [compatibility level](https://github.com/terjeio/grblHAL/wiki/Changes-from-grbl-1.1#workaround) except some settings that has flags added to them. The added flags will not be available at all compatibilty levels. A new command, `$+`, can be used to list all settings independent of compatibilty level.
* Internal changes to settings data in order to simplify automatic migration on changes. Automatic migration is on the roadmap.

<sup>1</sup> Note that several factors may affect the accuracy of these settings such as step output mode, number of axes defined and compiler optimization settings.
A new #define, `STEP_PULSE_LATENCY`, has been added to driver.h for those drivers that requires it so that fine tuning can be done. I may move this to a setting later.
In setups where very high step rates are used the actual step pulse width should be confirmed with an oscilloscope.

The step pulse delay has not been fine tuned yet as setting this to a value > 0 is not normally needed as there seems to be a implicit delay on direction changes when AMASS is enabled.

__Note:__ high step rates \(e.g. above 80 kHz\) cannot be achieved with the step pulse setting set to the default 10 microseconds. This has to be reduced so that both the on and off time is within specifications of the stepper driver. At 100 kHz the time available for a pulse \(on + off\) is 10 microseconds.

__Note:__ The SAMD21 \(MKRZERO\) driver needs updating in for it to allow short step pulses. My mistake was to use some Arduino code so that the Arduino pin numbers could be used for pin mappings. This adds around 500 ns per axis of overhead... To be fixed later.

For the curious: I have managed to achieve a 400 kHz step rate with the iMXRT1062 before everything breaks down, this with a command entered from MDI. I have not tested this with a running program, but I am pretty sure that a step rate at or above 200 kHz is sustainable.

---

2020/06/18: Added driver for STM32F4xx [Black Pill](https://www.cnx-software.com/2019/12/24/stm32-black-pill-board-features-stm32f4-cortex-m4-mcu-optional-spi-flash/), code modified by @shaise from the STM32F1xx driver. This is the first driver provided by someone else than me, thanks for that.

This driver is a candidate along with the IMXRT1062 \(Teensy 4.x\) driver to get spindle sync support. I have a [NucleoF411RE development board](https://www.st.com/en/evaluation-tools/nucleo-f411re.html) on order and will look into adding a pin mapping for that when it arrives. 

---


Build 20200603:
* **Important:** settings version has been changed and settings will be restored to defaults after updating. Backup & restore! 
* Optimizations for ring buffer handling in planner and step generator.
* New optional input signal for probe connected status, driver support will be added later to selected drivers.
* Automatic reporting of tool length offset \(`[TLO:...]`\) when changed.
* Support for [G5](http://www.linuxcnc.org/docs/2.5/html/gcode/gcode.html#sec:G5-Cubic-Spline) \(cubic spline\) added.
* `G43.x`, `G49` and `G92` added to parser state report.
* `G76` [threading cycle](https://hackaday.io/project/165248-mini-lathe-emco-compact-5-cnc-conversion) refactored.
* \(Re\)added `REPORT_PROBE_COORDINATES` and `TOOL_LENGTH_OFFSET_AXIS` [configuration](grbl/config.h) options, the latter available when `COMPATIBILITY_LEVEL` > 2.
* Improved backwards compatibility with vanilla grbl, e.g. G92 and tool offset\(s\) will be lost on a soft reset. Dependent on `COMPATIBILITY_LEVEL` setting.
* Board name added to `$I` report if provided by driver.
* [Grbl-Sim](https://github.com/grbl/grbl-sim) ported to grblHAL as a [driver](drivers/Simulator). Added telnet support++. Can be used to test senders. Note: currently only compiled/tested for Linux.
* Some minor bug fixes.

---

Build 20200503: Added configuration flag for manual homing. \(Re\)added compile time option `ENABLE_SAFETY_DOOR_INPUT_PIN` for [safety door switch](https://github.com/terjeio/grblHAL/blob/master/grbl/config.h), default is now disabled. Some bug fixes and "hardening" of code.

---

Added some [template code](./templates/README.md) to aid customizations such as driver support for M62 - M68 M-codes mentioned below.

---

Build 20191222: Added digital and analog output support to the core \(and HAL\) as per [linuxcnc specifications for M62 - M68](http://linuxcnc.org/docs/html/gcode/m-code.html#mcode:m62-m65), number of outputs available \(if any\) is driver dependent. Adding support for these M-commands makes it fairly easy to add driver code \(for up to 256 outputs\) as parsing and synchronization is taken care of by the core.

---

Build 20191215: Moved spindle RPM linearization to $-settings, option needs to be enabled in config.h - driver support required. Optimized EEPROM allocation handling. WebUI support for ESP32 driver improved.  

MSP432 driver enhanced for spindle linearization app in the pipeline \(for Windows only - needs input from spindle encoder\), more work done on closed loop spindle RPM control and spindle synchronized motion - still at experimental stage.

__NOTE:__ settings version number has been increased so settings will be reset to default after update, make a backup first!

---

Added [#define COMPATIBILITY_LEVEL to config.h](grbl/config.h) for backwards compatibility with Grbl v1.1 protocol definition, this for enabling the use of more GCode senders. Please raise an issue if your sender still does not behave well after setting this as the current implementation does not yet disable all extensions, notably [new $xx settings](doc/markdown/grblHAL%20extensions.md#settings).

G76 threading support added to grblHAL in combination with the [MSP432 driver](drivers/MSP432/README.md). Extensive testing is required before it can be regarded as safe.

**WARNING!** This is a potentially dangerous addition. Do NOT use if you do not understand the risks. A proper E-Stop is a must, it should cut power to the steppers and if possible engage any spindle brake. The implementation is based on the [linuxcnc specification](http://linuxcnc.org/docs/2.6/html/gcode/gcode.html#sec:G76-Threading-Canned). Please note that I am not a machinist so my interpretation and implementation may be wrong!

G76 availablity requires a spindle encoder with index pulse, grblHAL configured to [lathe mode](doc/markdown/settings.md#opmode) and tuning of the spindle sync PID loop.  
__NOTE:__ Feed hold is delayed until spindle synced cut is complete, spindle RPM overrides and CSS mode disabled through the whole cycle. 

---

GrblHAL is a no-compromise, high performance, low cost alternative to parallel-port-based motion control for CNC milling based on the Arduino version of grbl. It is mainly aimed at ARM processors \(or other 32-bit MCUs\) with ample amounts of RAM and flash \(compared to AVR 328p\) and requires a [hardware driver](drivers/ReadMe.md) to be functional.

The driver interface \(HAL\) has entry points for extending the supported M-codes (adding user defined M-codes) as well as an entry point for the driver to execute G-code when Grbl is in idle or jog state.

HAL = Hardware Abstraction Layer

The controller is written in highly optimized C utilizing features of the ARM-chips to achieve precise timing and asynchronous operation. It is able to maintain over 30kHz of stable, jitter free control pulses.

**WARNING:** I have started to introduce bitfield unions instead of using \#defines and macros, even for axis bits. This is potentially a big problem if bit-order and alignment does not match the implementation of the C-compiler I am using \(CCS v6 IDE - TI v5.2.6 compiler\). I am doing this as a coding excercise and I am well aware of the risks involved...

It accepts standards-compliant g-code and has been tested with the output of several CAM tools with no problems. Arcs, circles and helical motion are fully supported, as well as, all other primary g-code commands. Macro functions, variables, and most canned cycles are not supported, but we think GUIs can do a much better job at translating them into straight g-code anyhow.

Grbl includes full acceleration management with look ahead. That means the controller will look up to 16 motions into the future and plan its velocities ahead to deliver smooth acceleration and jerk-free cornering.

* This version is a port/rewrite of [grbl 1.1f](https://github.com/gnea/grbl) and should be compatible with GCode senders compliant with the specifications for that version. It should be possible to change default compile-time configurations if problems arise, eg. the default serial buffer sizes has been increased in some of the [drivers](drivers/ReadMe.md) provided.

**NOTE:** As there are many changes to the codebase this version should **not** be regarded as stable. Added features has only undergone light testing if any at all. Also, there is a completely new state machine that is not yet fully verified - it seems to have issues regarding safety door handling when parking is enabled.

I am currently running this version in three CNC machines, a CO2 laser \(TM4C123\), a router/mill \(MSP432\) and a lathe \(MSP432\) The last two are builds in progress.

***

```
List of Supported G-Codes in GrblHAL v1.1:
  - Non-Modal Commands: G4, G10L2, G10L20, G28, G30, G28.1, G30.1, G53, G92, G92.1
  - Additional Non-Modal Commands: G10L1*, G10L10*, G10L11*
  - Motion Modes: G0, G1, G2, G3, G5, G38.2, G38.3, G38.4, G38.5, G80, G33*
  - Canned cycles: G73, G81, G82, G83, G85, G86, G89, G98, G99
  - Repetitive cycles: G76*
  - Feed Rate Modes: G93, G94, G95*, G96*, G97*
  - Unit Modes: G20, G21
  - Scaling: G50, G51
  - Lathe modes: G7*, G8*
  - Distance Modes: G90, G91
  - Arc IJK Distance Modes: G91.1
  - Plane Select Modes: G17, G18, G19
  - Tool Length Offset Modes: G43*, G43.1, G43.2*, G49
  - Cutter Compensation Modes: G40
  - Coordinate System Modes: G54, G55, G56, G57, G58, G59, G59.1, G59.2, G59.3
  - Control Modes: G61
  - Program Flow: M0, M1, M2, M30, M60
  - Coolant Control: M7, M8, M9
  - Spindle Control: M3, M4, M5
  - Tool Change: M6* (Two modes possible: manual** - supports jogging, ATC), M61
  - Switches: M49, M50, M51, M53
  - Output control***: M62, M63, M64, M65, M66, M67, M68
  - Valid Non-Command Words: A*, B*, C*, F, H*, I, J, K, L, N, P, Q*, R, S, T, X, Y, Z

  *  driver/configuration dependent.
  ** requires compatible GCode sender due to protocol extensions, new state and RT command.
  *** number of outputs supported dependent on driver implementation.
```

---
2020-08-11
