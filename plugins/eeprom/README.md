## I2C EEPROM plugin

This plugin can be used for storing settings on an EEPROM \(or FRAM\) for processors thas does not have EEPROM on-chip.

Currently Microchip 24LC16 2K EEPROM \(single byte address mode\) and 24AA32 4K - 24AA256 32K EEPROMS \(two byte address mode\) or equivalent is supported.

`#define EEPROM_ENABLE` in _my_machine.h_ is used to select type:

`1` - 16 Kbit (2K)  
`2` - typically 128 Kbit or larger (16K+), 64 byte page size  
`3` - typically 32 Kbit or 64 Kbit (4 or 8K), 32 byte page size

Dependencies:

Driver must support I2C communication via custom API.

---
2020-11-18
