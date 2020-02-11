/*

  ioexpand.c - driver code for Espressif ESP32 processor

  I2C I/O expander, PCA9654E - with address pins to GND. PCA9654EA has a different address!

  Part of Grbl

  Copyright (c) 2018-2020 Terje Io

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

#include "driver.h"

#if IOEXPAND_ENABLE

#include "ioexpand.h"

void ioexpand_init (void)
{
    if(i2cBusy != NULL && xSemaphoreTake(i2cBusy, 5 / portTICK_PERIOD_MS) == pdTRUE) {

        // 0 = output, 1 = input
        // TODO: move to driver.h?
        const ioexpand_t cfg = {
            .spindle_on = 0,
            .spindle_dir = 0,
            .mist_on = 0,
            .flood_on = 0,
            .stepper_enable_z = 0,
            .stepper_enable_x = 0,
            .stepper_enable_y = 0,
            .reserved = 1
        };

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, IOEX_ADDRESS|I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, RW_CONFIG, true);
        i2c_master_write_byte(cmd, cfg.mask, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, IOEX_ADDRESS|I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, RW_INVERSION, true);
        i2c_master_write_byte(cmd, 0, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);

        i2c_cmd_link_delete(cmd);

        xSemaphoreGive(i2cBusy);
    }
}

IRAM_ATTR void ioexpand_out (ioexpand_t pins)
{
    static i2c_task_t i2c_task = {
        .action = 2,
        .params = NULL
    };

    if(xPortInIsrContext()) {
        i2c_task.params = (void *)((uint32_t)pins.mask);
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        xQueueSendFromISR(i2cQueue, (void *)&i2c_task, &xHigherPriorityTaskWoken);
    } else if(i2cBusy != NULL && xSemaphoreTake(i2cBusy, 5 / portTICK_PERIOD_MS) == pdTRUE) {

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, IOEX_ADDRESS|I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, RW_OUTPUT, true);
        i2c_master_write_byte(cmd, pins.mask, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        xSemaphoreGive(i2cBusy);
    }
}

ioexpand_t ioexpand_in (void)
{
    ioexpand_t pins = {0};

    if(i2cBusy != NULL && xSemaphoreTake(i2cBusy, 5 / portTICK_PERIOD_MS) == pdTRUE) {

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, IOEX_ADDRESS|I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, READ_INPUT, true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, IOEX_ADDRESS|I2C_MASTER_READ, true);
        i2c_master_read_byte(cmd, &pins.mask, I2C_MASTER_NACK);
        i2c_master_stop(cmd);

        i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        xSemaphoreGive(i2cBusy);
    }

    return pins;
}

#endif
