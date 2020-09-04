## I2C EEPROM plugin

This plugin can be used for storing settings on an EEPROM (or FRAM) for processors thas does not have EEPROM on-chip.

Currently Microchip 24LC16 2K EEPROM (single byte address mode) and 24AA32 4K - 24AA256 32K EEPROMS (two byte address mode) or equivalent is supported.

Dependencies:

Driver must support I2C communication via custom API.

---
2020-08-05
