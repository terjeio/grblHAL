## Trinamic Stepper Drivers

Free standing drivers, may be used outside of grblHAL. SPI hardware driver needs to be added to make them work.

As it stands now it is mainly the complete set of register definitions with a few functions on top.

I  intend to add this to one or more of the grblHAL drivers, examples of hardware drivers and usage will be linked to here when published. As a part of that a [I2C <> SPI bridge based](https://github.com/terjeio/Trinamic_TMC2130_I2C_SPI_Bridge) on a MSP430G2553 has been made, this for hardware with limited pins available.

2021-02-07: Moved to [new repository](https://github.com/terjeio/Trinamic-library).
