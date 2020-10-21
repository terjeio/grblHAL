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
static uint8_t getByte (uint32_t addr)
{
    uint32_t data;

//    EEPROM_READ(addr);

    return data;

}

// Write single byte to EEPROM.
// addr is 0-based offset from start.
static void putByte (uint32_t addr, uint8_t new_value)
{
//    EEPROM_WRITE(new_value);
}

// Read block of data from EEPROM, return true if checksum matches or no checksum used.
// Checksum is stored in the byte following the last byte read from the block.
static nvs_transfer_result_t readBlock (uint8_t *destination, uint32_t source, uint32_t size, bool with_checksum)
{
    uint8_t *data = destination;

    for(; size > 0; size--)
      *data++ = getByte(source++);

    return with_checksum ? (calc_checksum(destination, size) == getByte(source + size) ? NVS_TransferResult_OK : NVS_TransferResult_Failed) : NVS_TransferResult_OK;
}

// Write block of data to EEPROM followed by an optional checksum byte.
static nvs_transfer_result_t writeBlock (uint32_t destination, uint8_t *source, uint32_t size, bool with_checksum)
{

    uint8_t *data = source;
    uint32_t remaining = size;

    for(; remaining > 0; remaining--)
        putByte(destination++, *data++);

	if(with_checksum)
		putByte(destination + size, calc_checksum(source, size));
}

void eeprom_init (void)
{
    hal.nvs.type = NVS_EEPROM;
    hal.nvs.get_byte = getByte;
    hal.nvs.put_byte = putByte;
    hal.nvs.memcpy_to_nvs = writeBlock;
    hal.nvs.memcpy_from_nvs = readBlock;
}
