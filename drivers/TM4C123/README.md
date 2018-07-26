## TM4C123GrblDriver

A GrblHAL driver for Texas Instruments Tiva C Launchpad

This driver needs to be complemented with an UART or USB driver for host communication.

Optional support for I2C keypad added, can be used for jogging and mist/coolant overrides. Jogging can be toggled between fast, slow and step mode.

Pin allocations matches the [CNC BoosterPack](https://github.com/terjeio/CNC_Boosterpack)

Features:

* All hardware dependent code, except host communication, in a separate project \(needs access to grbl library includes\).

**NOTE:** not yet tested with the CNC BoosterPack.
