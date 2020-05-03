## GrblHAL ##
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
  - Motion Modes: G0, G1, G2, G3, G38.2, G38.3, G38.4, G38.5, G80, G33*
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
  - Program Flow: M0, M1, M2, M30
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
