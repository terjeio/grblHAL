/*

  eeprom.h - driver code for Espressif ESP32 processor

  for 2K EEPROM on CNC Boosterpack (Microchip 24LC16B)

  Part of GrblHAL

  Copyright (c) 2017-2018 Terje Io

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.

*/

#include <stdint.h>
#include <stdbool.h>

#include "driver.h"

#if EEPROM_ENABLE

#define EEPROM_I2C_ADDRESS (0xA0 >> 1)
#define EEPROM_ADDR_BITS_LO 8
#define EEPROM_BLOCK_SIZE (2 ^ EEPROM_LO_ADDR_BITS)
#define EEPROM_PAGE_SIZE 16

typedef struct {
    uint8_t addr;
    volatile int16_t count;
    uint8_t *data;
    uint8_t word_addr;
} i2c_trans_t;

static i2c_trans_t i2c;

static void StartI2C (bool read)
{
    if(i2cBusy != NULL && xSemaphoreTake(i2cBusy, 5 / portTICK_PERIOD_MS) == pdTRUE) {

        i2c.addr <<= 1;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, i2c.addr|I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, i2c.word_addr, true);

        if(read) {
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, i2c.addr|I2C_MASTER_READ, true);
            if (i2c.count > 1)
                i2c_master_read(cmd, i2c.data, i2c.count - 1, I2C_MASTER_ACK);
            i2c_master_read_byte(cmd, i2c.data + i2c.count - 1, I2C_MASTER_NACK);
            i2c_master_stop(cmd);
        } else {
            i2c_master_write(cmd, i2c.data, i2c.count, true);
            i2c_master_stop(cmd);
        }

        i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
//      printf("EE %d %d %d\n", read, i2c.count, ret);
        i2c_cmd_link_delete(cmd);

        xSemaphoreGive(i2cBusy);

        if(!read) // Delay 5ms for write to complete
            vTaskDelay(10 / portTICK_PERIOD_MS);
    }
}

uint8_t eepromGetByte (uint32_t addr)
{
    uint8_t value = 0;

    i2c.addr = EEPROM_I2C_ADDRESS | (addr >> 8);
    i2c.word_addr = addr & 0xFF;
    i2c.data = &value;
    i2c.count = 1;
    StartI2C(true);

    return value;
}

void eepromPutByte (uint32_t addr, uint8_t new_value)
{
    i2c.addr = EEPROM_I2C_ADDRESS | (addr >> 8);
    i2c.word_addr = addr & 0xFF;
    i2c.data = &new_value;
    i2c.count = 1;

    StartI2C(false);
}

nvs_transfer_result_t eepromWriteBlockWithChecksum (uint32_t destination, uint8_t *source, uint32_t size, bool with_checksum)
{
    uint32_t bytes = size;

    i2c.word_addr = destination & 0xFF;
    i2c.data = source;

    while(bytes > 0) {
        i2c.count = EEPROM_PAGE_SIZE - (destination & (EEPROM_PAGE_SIZE - 1));
        i2c.count = bytes < i2c.count ? bytes : i2c.count;
        i2c.addr = EEPROM_I2C_ADDRESS | (destination >> EEPROM_ADDR_BITS_LO);
        bytes -= i2c.count;
        destination += i2c.count;

        StartI2C(false);

        i2c.data += i2c.count;
        i2c.word_addr = destination & 0xFF;
    }

    if(size > 0 && with_checksum)
        eepromPutByte(destination, calc_checksum(source, size));

    return NVS_TransferResult_OK;
}

nvs_transfer_result_t eepromReadBlockWithChecksum (uint8_t *destination, uint32_t source, uint32_t size, bool with_checksum)
{
    i2c.addr = EEPROM_I2C_ADDRESS | (source >> 8);
    i2c.word_addr = source & 0xFF;
    i2c.count = size;
    i2c.data = destination;

    StartI2C(true);

    return with_checksum ? (calc_checksum(destination, size) == eepromGetByte(source + size) ? NVS_TransferResult_OK : NVS_TransferResult_Failed) : NVS_TransferResult_OK;

}

#endif
