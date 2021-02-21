/*
  i2c.c - I2C support for keypad and Trinamic plugins

  Part of grblHAL driver for ESP32

  Copyright (c) 2018-2021 Terje Io

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

#include "i2c.h"

#ifdef I2C_PORT

#if IOEXPAND_ENABLE
#include "ioexpand.h"
#endif

#if KEYPAD_ENABLE
#include "keypad/keypad.h"
#endif

#if TRINAMIC_ENABLE && TRINAMIC_I2C
#define I2C_ADR_I2CBRIDGE 0x47
#endif

QueueHandle_t i2cQueue = NULL;
SemaphoreHandle_t i2cBusy = NULL;

void I2CTask (void *queue)
{
    i2c_task_t task;

    while(xQueueReceive((QueueHandle_t)queue, &task, portMAX_DELAY) == pdPASS) {

#if KEYPAD_ENABLE
        if(task.action == 1) { // Read keypad character and add to input buffer
            if(i2cBusy != NULL && xSemaphoreTake(i2cBusy, 20 / portTICK_PERIOD_MS) == pdTRUE) {
                char keycode;
                i2c_cmd_handle_t cmd = i2c_cmd_link_create();
                i2c_master_start(cmd);
                i2c_master_write_byte(cmd, (KEYPAD_I2CADDR << 1) | I2C_MASTER_READ, true);
                i2c_master_read_byte(cmd, (uint8_t*)&keycode, I2C_MASTER_NACK);
                i2c_master_stop(cmd);
                if(i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_RATE_MS) == ESP_OK)
                    ((keycode_callback_ptr)task.params)(keycode);
                i2c_cmd_link_delete(cmd);
                xSemaphoreGive(i2cBusy);
            }
        }
#endif
#if IOEXPAND_ENABLE
        if(task.action == 2) { // Write to I/O expander (from ISR)
            if(i2cBusy != NULL && xSemaphoreTake(i2cBusy, 5 / portTICK_PERIOD_MS) == pdTRUE) {

                i2c_cmd_handle_t cmd = i2c_cmd_link_create();
                i2c_master_start(cmd);
                i2c_master_write_byte(cmd, IOEX_ADDRESS|I2C_MASTER_WRITE, true);
                i2c_master_write_byte(cmd, RW_OUTPUT, true);
                i2c_master_write_byte(cmd, (uint8_t)((uint32_t)task.params & 0xFF), true);
                i2c_master_stop(cmd);
                i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
                i2c_cmd_link_delete(cmd);

                xSemaphoreGive(i2cBusy);
            }
        }
#endif
    }
}

#endif

#if EEPROM_ENABLE

nvs_transfer_result_t i2c_nvs_transfer (nvs_transfer_t *i2c, bool read)
{
    if(i2cBusy != NULL && xSemaphoreTake(i2cBusy, 5 / portTICK_PERIOD_MS) == pdTRUE) {

        i2c->address <<= 1;
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, i2c->address|I2C_MASTER_WRITE, true);
        if(i2c->word_addr_bytes == 2) {
            i2c_master_write_byte(cmd, i2c->word_addr >> 8, true);
            i2c_master_write_byte(cmd, i2c->word_addr & 0xFF, true);
        } else
            i2c_master_write_byte(cmd, i2c->word_addr, true);

        if(read) {
            i2c_master_start(cmd);
            i2c_master_write_byte(cmd, i2c->address|I2C_MASTER_READ, true);
            if (i2c->count > 1)
                i2c_master_read(cmd, i2c->data, i2c->count - 1, I2C_MASTER_ACK);
            i2c_master_read_byte(cmd, i2c->data + i2c->count - 1, I2C_MASTER_NACK);
            i2c_master_stop(cmd);
        } else {
            i2c_master_write(cmd, i2c->data, i2c->count, true);
            i2c_master_stop(cmd);
        }

        i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
//      printf("EE %d %d %d\n", read, i2c.count, ret);
        i2c_cmd_link_delete(cmd);

        xSemaphoreGive(i2cBusy);

#if !EEPROM_IS_FRAM
        if(!read) // Delay 5ms for write to complete
            vTaskDelay(10 / portTICK_PERIOD_MS);
#endif
    }

    return NVS_TransferResult_OK;
}

#endif


#if KEYPAD_ENABLE

void I2C_GetKeycode (uint32_t i2cAddr, keycode_callback_ptr callback)
{
    static i2c_task_t i2c_task = {
        .action = 1,
        .params = NULL
    };

    i2c_task.params = callback;

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xQueueSendFromISR(i2cQueue, (void *)&i2c_task, &xHigherPriorityTaskWoken);
}

#endif

#if TRINAMIC_ENABLE && TRINAMIC_I2C

static uint8_t axis = 0xFF;
static const uint8_t tmc_addr = I2C_ADR_I2CBRIDGE << 1;

inline static bool tmc_set_axis (uint8_t axis) {

    esp_err_t ret = ESP_FAIL;

    if(i2cBusy != NULL && xSemaphoreTake(i2cBusy, 5 / portTICK_PERIOD_MS) == pdTRUE) {

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, tmc_addr|I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, axis | 0x80, true);
        i2c_master_stop(cmd);
        ret = i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        xSemaphoreGive(i2cBusy);
    }

    return ret == ESP_OK;
}

TMC_spi_status_t tmc_spi_read (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    uint8_t buffer[8];
    TMC_spi_status_t status = 0;

    if(driver.axis != axis) {
        tmc_set_axis(driver.axis);
        axis = driver.axis;
    }

    buffer[0] = reg->addr.idx;
    buffer[1] = 0;
    buffer[2] = 0;
    buffer[3] = 0;
    buffer[4] = 0;

    if(i2cBusy != NULL && xSemaphoreTake(i2cBusy, 5 / portTICK_PERIOD_MS) == pdTRUE) {

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, tmc_addr|I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, buffer[0], true);
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, tmc_addr|I2C_MASTER_READ, true);
        i2c_master_read(cmd, buffer, 4, I2C_MASTER_ACK);
        i2c_master_read_byte(cmd, buffer + 4, I2C_MASTER_NACK);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);

        xSemaphoreGive(i2cBusy);
    }

    status = buffer[0];
    reg->payload.value = buffer[4];
    reg->payload.value |= buffer[3] << 8;
    reg->payload.value |= buffer[2] << 16;
    reg->payload.value |= buffer[1] << 24;

    return status;
}

TMC_spi_status_t tmc_spi_write (trinamic_motor_t driver, TMC_spi_datagram_t *reg)
{
    uint8_t buffer[8];
    TMC_spi_status_t status = 0;

    if(driver.axis != axis) {
        tmc_set_axis(driver.axis);
        axis = driver.axis;
    }

    reg->addr.write = 1;
    buffer[0] = reg->addr.value;
    reg->addr.write = 0;
    buffer[1] = (reg->payload.value >> 24) & 0xFF;
    buffer[2] = (reg->payload.value >> 16) & 0xFF;
    buffer[3] = (reg->payload.value >> 8) & 0xFF;
    buffer[4] = reg->payload.value & 0xFF;

    if(i2cBusy != NULL && xSemaphoreTake(i2cBusy, 5 / portTICK_PERIOD_MS) == pdTRUE) {

        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, tmc_addr|I2C_MASTER_WRITE, true);
        i2c_master_write(cmd, buffer, 5, true);
        i2c_master_stop(cmd);

        i2c_master_cmd_begin(I2C_PORT, cmd, 1000 / portTICK_PERIOD_MS);
//      printf("EE %d %d %d\n", read, i2c.count, ret);
        i2c_cmd_link_delete(cmd);

        xSemaphoreGive(i2cBusy);
    }

    return status;
}

#endif

#ifdef I2C_PORT

void I2CInit (void)
{
    static bool init_ok = false;

    if(!init_ok) {

        init_ok = true;

        i2c_config_t i2c_config = {
            .mode = I2C_MODE_MASTER,
            .sda_io_num = I2C_SDA,
            .scl_io_num = I2C_SCL,
            .sda_pullup_en = GPIO_PULLUP_DISABLE,
            .scl_pullup_en = GPIO_PULLUP_DISABLE,
            .master.clk_speed = I2C_CLOCK
        };

        i2c_param_config(I2C_PORT, &i2c_config);
        i2c_driver_install(I2C_PORT, i2c_config.mode, 0, 0, 0);

        i2cQueue = xQueueCreate(5, sizeof(i2c_task_t));
        i2cBusy = xSemaphoreCreateBinary();

        TaskHandle_t I2CTaskHandle;

        xTaskCreatePinnedToCore(I2CTask, "I2C", 2048, (void *)i2cQueue, configMAX_PRIORITIES, &I2CTaskHandle, 1);

        xSemaphoreGive(i2cBusy);
    }
}

#endif
