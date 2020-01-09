/*
  eeprom.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Template driver code for ARM processors

  Part of GrblHAL

  By Terje Io, public domain

*/

#include "grbl/grbl.h"

// NOTE: See driver for TM4C123 for additional code if EEPROM peripheral only support word (32-bit) access.
//       Also, many drivers has code for external EEPROM access, typically via I2C interface.

// Read single byte from EEPROM.
// addr is 0-based offset from start.
uint8_t eepromGetByte (uint32_t addr)
{
    uint32_t data;

//    EEPROM_READ(addr);

    return data;

}

// Write single byte to EEPROM.
// addr is 0-based offset from start.
void eepromPutByte (uint32_t addr, uint8_t new_value)
{
//    EEPROM_WRITE(new_value);
}

// Read block of data from EEPROM, return true if checksum matches.
// Checksum is stored in the byte following the last byte read from the block.
bool eepromReadBlockWithChecksum (uint8_t *destination, uint32_t source, uint32_t size)
{
    uint8_t *data = destination;

    for(; size > 0; size--)
      *data++ = eepromGetByte(source++);

    return calc_checksum(destination, size) == eepromGetByte(source);
}

// Write block of data to EEPROM followed by a checksum byte.
void eepromWriteBlockWithChecksum (uint32_t destination, uint8_t *source, uint32_t size) {

    uint8_t *data = source;
    uint32_t remaining = size;

    for(; remaining > 0; remaining--)
        eepromPutByte(destination++, *data++);

    eepromPutByte(destination, calc_checksum(source, size));
}
