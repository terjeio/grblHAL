## grblHAL templates, arm-driver

This is a "bare bones" driver template.

Function calls that are commented out and in upper case are pseudo code that needs to be replaced, possibly by reading/writing directly from/to registers.

To make a functional driver MCU documentation has to be consulted and quite a bit of code added, mostly for peripheral configuration. It may also be helpful to peek at the existing drivers to see how they are implemented.

Tip: I like to use the bit-band region for flexibility, and sometimes speed of execution, for GPIO pin access.

---
2019-12-29
