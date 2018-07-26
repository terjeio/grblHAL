## GrblHAL ##

GrblHAL is a no-compromise, high performance, low cost alternative to parallel-port-based motion control for CNC milling based on the Arduino version of grbl. It is mainly aimed at ARM processors \(or other 32-bit MCUs\) with ample amounts of RAM and flash \(compared to AVR 328p\) and requires a hardware driver to be functional.

The driver interface \(HAL\) has entry points for extending the supported M-codes (adding user defined M-codes) as well as an entry point for the driver to execute G-code when Grbl is in idle or jog state.

HAL = Hardware Abstraction Layer

The controller is written in highly optimized C utilizing features of the ARM-chips to achieve precise timing and asynchronous operation. It is able to maintain over 30kHz of stable, jitter free control pulses.

**WARNING:** I have started to introduce bitfield unions instead of using \#defines and macros, even for axis bits. This is potentially a big problem if bit-order and alignment does not match the implementation of the C-compiler I am using \(CCS v6 IDE - TI v5.2.6 compiler\). I am doing this as a coding excercise and I am well aware of the risks involved...

It accepts standards-compliant g-code and has been tested with the output of several CAM tools with no problems. Arcs, circles and helical motion are fully supported, as well as, all other primary g-code commands. Macro functions, variables, and most canned cycles are not supported, but we think GUIs can do a much better job at translating them into straight g-code anyhow.

Grbl includes full acceleration management with look ahead. That means the controller will look up to 16 motions into the future and plan its velocities ahead to deliver smooth acceleration and jerk-free cornering.

* This version is a port/rewrite of [grbl 1.1f](https://github.com/gnea/grbl) and should be compatible with GCode senders compliant with the specifications for that version. It should be possible to change default compile-time configurations if problems arise, eg. the default serial buffer sizes has been increased in some of the drivers provided.

**NOTE:** As there are many changes to the codebase this version should **not** be regarded as stable. Added features has only undergone light testing if any at all. Also, there is a completely new state machine that is not yet fully verified - it seems to have issues regarding safety door handling when parking is enabled.

I am currently running this version in three CNC machines, a CO2 laser (TM4C123), a router/mill (MSP432) and a lathe (MSP432) The last two are builds in progress.

***

```
List of Supported G-Codes in GrblHAL v1.1:
  - Non-Modal Commands: G4, G10L2, G10L20, G28, G30, G28.1, G30.1, G53, G92, G92.1
  - Additional Non-Modal Commands: G10L1*, G10L10*, G10L11*
  - Motion Modes: G0, G1, G2, G3, G38.2, G38.3, G38.4, G38.5, G80, G33*
  - Canned cycles: G73, G81, G82, G83, G85, G86, G89, G98, G99
  - Feed Rate Modes: G93, G94, G95*, G96*, G97*
  - Unit Modes: G20, G21
  - Scaling: G50, G51
  - Distance Modes: G90, G91
  - Arc IJK Distance Modes: G91.1
  - Plane Select Modes: G17, G18, G19
  - Tool Length Offset Modes: G43.1, G49
  - Cutter Compensation Modes: G40
  - Coordinate System Modes: G54, G55, G56, G57, G58, G59, G59.1, G59.2, G59.3
  - Control Modes: G61
  - Program Flow: M0, M1, M2, M30
  - Coolant Control: M7, M8, M9
  - Spindle Control: M3, M4, M5
  - Switches: M49, M50, M51, M53
  - Valid Non-Command Words: A*, B*, C*, F, H*, I, J, K, L, N, P, Q*, R, S, T, X, Y, Z

  * driver/configuration dependent
```
