/*
  bluetooth.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Bluetooth comms

  Part of grblHAL

  Copyright (c) 2018-2021 Terje Io

  Some parts of the code is based on example code by Espressif, in the public domain

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
#include <string.h>
#include <stdbool.h>
#include <stdio.h>
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "driver.h"
#include "grbl/grbl.h"
#include "grbl/report.h"
#include "grbl/nvs_buffer.h"

#define SPP_RUNNING      (1 << 0)
#define SPP_CONNECTED    (1 << 1)
#define SPP_CONGESTED    (1 << 2)
#define SPP_DISCONNECTED (1 << 3)
#define BT_TX_QUEUE_ENTRIES 32
#define BT_TX_BUFFER_SIZE 250

#define USE_BT_MUTEX 0

#if USE_BT_MUTEX
#define BT_MUTEX_LOCK()   do {} while (xSemaphoreTake(lock, portMAX_DELAY) != pdPASS)
#define BT_MUTEX_UNLOCK() xSemaphoreGive(lock)
static SemaphoreHandle_t lock = NULL;
#else
#define BT_MUTEX_LOCK()
#define BT_MUTEX_UNLOCK()
#endif

#define SPP_TAG "BLUETOOTH"

typedef struct {
    volatile uint16_t head;
    char data[BT_TX_BUFFER_SIZE];
} bt_tx_buffer_t;

typedef struct {
    uint16_t length;
    uint8_t data[1];
} tx_chunk_t;

static uint32_t connection = 0;
static bool is_second_attempt = false;
static bluetooth_settings_t bluetooth;
static SemaphoreHandle_t tx_busy = NULL;
static EventGroupHandle_t event_group = NULL;
static TaskHandle_t polltask = NULL;
static xQueueHandle tx_queue = NULL;
static portMUX_TYPE tx_flush_mux = portMUX_INITIALIZER_UNLOCKED;
static char client_mac[18];

static bt_tx_buffer_t txbuffer;
static stream_rx_buffer_t rxbuffer = {0};
static stream_rx_buffer_t rxbackup;
static nvs_address_t nvs_address;
static on_report_options_ptr on_report_options;

uint32_t BTStreamAvailable (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

uint16_t BTStreamRXFree (void)
{
    uint16_t head = rxbuffer.head, tail = rxbuffer.tail;

    return (RX_BUFFER_SIZE - 1) - BUFCOUNT(head, tail, RX_BUFFER_SIZE);
}

// "dummy" version of serialGetC
static int16_t BTStreamGetNull (void)
{
    return -1;
}

int16_t BTStreamGetC (void)
{
    BT_MUTEX_LOCK();

    int16_t data;
    uint16_t bptr = rxbuffer.tail;

    if(bptr == rxbuffer.head) {
        BT_MUTEX_UNLOCK();
        return -1; // no data available else EOF
    }

    data = rxbuffer.data[bptr++];                 // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);  // and update pointer

    BT_MUTEX_UNLOCK();

    return data;
}

static inline bool enqueue_tx_chunk (uint16_t length, uint8_t *data)
{
    tx_chunk_t *chunk;

    if((chunk = malloc(sizeof(tx_chunk_t) + length))) {
        chunk->length = length;
        memcpy(&chunk->data, data, length);
        if (xQueueSendToBack(tx_queue, &chunk, portMAX_DELAY) != pdPASS) {
            free(chunk);
            chunk = NULL;
        }
    }

    return chunk != NULL;
}

// Since grbl always sends cr/lf terminated strings we can send complete strings to improve throughput
bool BTStreamPutC (const char c)
{
    if(txbuffer.head < BT_TX_BUFFER_SIZE)
        txbuffer.data[txbuffer.head++] = c;

    if(c == '\n') {
        enqueue_tx_chunk(txbuffer.head, (uint8_t *)txbuffer.data);
        txbuffer.head = 0;
    }

    return true;
}

void BTStreamWriteS (const char *data)
{
    char c, *ptr = (char *)data;

    while((c = *ptr++) != '\0')
        BTStreamPutC(c);
}

void BTStreamFlush (void)
{
    BT_MUTEX_LOCK();

    rxbuffer.tail = rxbuffer.head;

    BT_MUTEX_UNLOCK();
}

IRAM_ATTR void BTStreamCancel (void)
{
    BT_MUTEX_LOCK();

    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);

    BT_MUTEX_UNLOCK();
}

char *bluetooth_get_device_mac (void)
{
    static char device_mac[18];
    const uint8_t *mac = esp_bt_dev_get_address();

    sprintf(device_mac, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    return device_mac;
}

char *bluetooth_get_client_mac (void)
{
    return client_mac[0] == '\0' ? NULL : client_mac;
}

static void report_bt_MAC (bool newopt)
{
    char *client_mac;

    on_report_options(newopt);

    if(newopt)
        hal.stream.write(",BT");
    else {
        hal.stream.write("[BT DEVICE MAC:");
        hal.stream.write(bluetooth_get_device_mac());
        hal.stream.write("]" ASCII_EOL);

        if((client_mac = bluetooth_get_client_mac())) {
            hal.stream.write("[BT CLIENT MAC:");
            hal.stream.write(client_mac);
            hal.stream.write("]" ASCII_EOL);
        }
    }
}

bool BTStreamSuspendInput (bool suspend)
{
    BT_MUTEX_LOCK();

    if(suspend)
        hal.stream.read = BTStreamGetNull;
    else if(rxbuffer.backup)
        memcpy(&rxbuffer, &rxbackup, sizeof(stream_rx_buffer_t));

    BT_MUTEX_UNLOCK();

    return rxbuffer.tail != rxbuffer.head;
}

static void flush_tx_queue (void)
{
    if(uxQueueMessagesWaiting(tx_queue)) {

        tx_chunk_t *chunk;

        portENTER_CRITICAL(&tx_flush_mux);

        while(uxQueueMessagesWaiting(tx_queue)) {
            if(xQueueReceive(tx_queue, &chunk, (TickType_t)0) == pdTRUE)
                free(chunk);
        }

        portEXIT_CRITICAL(&tx_flush_mux);
    }
}

static void esp_spp_cb (esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {

        case ESP_SPP_INIT_EVT:
            esp_bt_dev_set_device_name(bluetooth.device_name);
            esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
            esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, bluetooth.service_name);
            break;

        case ESP_SPP_SRV_OPEN_EVT:
            if(connection == 0) {
                connection = param->open.handle;
                txbuffer.head = 0;
                uint8_t *mac = param->srv_open.rem_bda;
                sprintf(client_mac, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
                selectStream(StreamType_Bluetooth);

                if(eTaskGetState(polltask) == eSuspended)
                    vTaskResume(polltask);

                hal.stream.write_all("[MSG:BT OK]\r\n");
            } else {
                is_second_attempt = true;
                esp_spp_disconnect(param->open.handle);
            }

            break;

        case ESP_SPP_CLOSE_EVT:
            if(is_second_attempt)
                is_second_attempt = false;

            else { // flush TX queue and reenable serial stream
                connection = 0;
                client_mac[0] = '\0';
                flush_tx_queue();
                selectStream(StreamType_Serial);
            }
            break;

        case ESP_SPP_DATA_IND_EVT:;
            char c;
            uint16_t len = param->data_ind.len;
            uint8_t *data = param->data_ind.data;

            while(len--) {
                c = (char)*data++;
                // discard input if MPG has taken over...
                if(hal.stream.type != StreamType_MPG) {

                    if(c == CMD_TOOL_ACK && !rxbuffer.backup) {

                        memcpy(&rxbackup, &rxbuffer, sizeof(stream_rx_buffer_t));
                        rxbuffer.backup = true;
                        rxbuffer.tail = rxbuffer.head;
                        hal.stream.read = BTStreamGetC; // restore normal input

                    } else if(!hal.stream.enqueue_realtime_command(c)) {

                        uint32_t bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer

                        if(bptr == rxbuffer.tail)               // If buffer full
                            rxbuffer.overflow = 1;              // flag overflow,
                        else {
                            rxbuffer.data[rxbuffer.head] = c;   // else add data to buffer
                            rxbuffer.head = bptr;               // and update pointer
                        }
                    }
                }
            }
            break;

        case ESP_SPP_CONG_EVT:
            if(param->cong.cong)
                xEventGroupClearBits(event_group, SPP_CONGESTED);
            else
                xEventGroupSetBits(event_group, SPP_CONGESTED);
            ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
            break;

        case ESP_SPP_WRITE_EVT:
            if(param->write.cong)
                xEventGroupClearBits(event_group, SPP_CONGESTED);
            xSemaphoreGive(tx_busy);
            break;

        default:
            break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {

        case ESP_BT_GAP_AUTH_CMPL_EVT:
            if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
                esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
            else
                ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
            break;

        case ESP_BT_GAP_PIN_REQ_EVT:
            if (param->pin_req.min_16_digit) {
                ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
                esp_bt_pin_code_t pin_code = {0};
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
            } else {
                ESP_LOGI(SPP_TAG, "Input pin code: 1234");
                esp_bt_pin_code_t pin_code = {'1', '2', '3', '4'};
                esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
            }
            break;

        case ESP_BT_GAP_CFM_REQ_EVT:
            ESP_LOGI(SPP_TAG, "ESP_BT_GAP_CFM_REQ_EVT Please compare the numeric value: %d", param->cfm_req.num_val);
            esp_bt_gap_ssp_confirm_reply(param->cfm_req.bda, true);
            break;

        case ESP_BT_GAP_KEY_NOTIF_EVT:
            ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_NOTIF_EVT passkey:%d", param->key_notif.passkey);
            break;

        case ESP_BT_GAP_KEY_REQ_EVT:
            ESP_LOGI(SPP_TAG, "ESP_BT_GAP_KEY_REQ_EVT Please enter passkey!");
            break;

        default:
            break;
    }
}

static void pollTX (void * arg)
{
    tx_chunk_t *chunk = NULL;
    bool data_sent = false;
    uint8_t buffer[256], *ptr;
    size_t remaining;

    while(true) {

        if(connection &&
            xEventGroupWaitBits(event_group, SPP_CONGESTED, pdFALSE, pdTRUE, 0) &&
             uxQueueMessagesWaiting(tx_queue) &&
              xSemaphoreTake(tx_busy, (TickType_t)0)) {

            data_sent = false;

            if(chunk || xQueueReceive(tx_queue, &chunk, (TickType_t)0) == pdTRUE) {

                remaining = sizeof(buffer);
                ptr = buffer;

                if(chunk->length >= remaining) {
                    data_sent = esp_spp_write(connection, chunk->length, chunk->data) == ESP_OK;
                    free(chunk);
                    chunk = NULL;
                } else while(chunk && chunk->length < remaining) {

                    memcpy(ptr, chunk->data, chunk->length);
                    ptr += chunk->length;
                    remaining -= chunk->length;
                    free(chunk);

                    if(xQueueReceive(tx_queue, &chunk, (TickType_t)0) != pdTRUE)
                        chunk = NULL;
                }

                if(remaining != sizeof(buffer))
                    data_sent = esp_spp_write(connection, sizeof(buffer) - remaining, buffer) == ESP_OK;
            }

            if(!data_sent)
               xSemaphoreGive(tx_busy);
        }

        if(connection)
            vTaskDelay(pdMS_TO_TICKS(10));
        else
            vTaskSuspend(NULL);
    }
}

bool bluetooth_start (void)
{
    client_mac[0] = '\0';

    if(bluetooth.device_name[0] == '\0' || !(event_group || (event_group = xEventGroupCreate())))
        return false;

    xEventGroupClearBits(event_group, 0xFFFFFF);
    xEventGroupSetBits(event_group, SPP_CONGESTED);

#if USE_BT_MUTEX
    if(!(lock || (lock = xSemaphoreCreateMutex())))
        return false;
#endif

    if(!(tx_queue || (tx_queue = xQueueCreate(BT_TX_QUEUE_ENTRIES, sizeof(tx_chunk_t *)))))
        return false;

    if(!(tx_busy || (tx_busy = xSemaphoreCreateBinary())))
        return false;

    xSemaphoreGive(tx_busy);

    if(polltask == NULL) {
        if(xTaskCreatePinnedToCore(pollTX, "btTX", 4096, NULL, 2, &polltask, 1) == pdPASS)
            vTaskSuspend(polltask);
        else
            return false;
    }

    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED)
        return true;

    if(esp_bt_controller_mem_release(ESP_BT_MODE_BLE) == ESP_OK) {

        esp_err_t ret;

        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();

        if ((ret = esp_bt_controller_init(&bt_cfg)) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
            return false;
        }

        if ((ret = esp_bt_controller_enable(ESP_BT_MODE_CLASSIC_BT)) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
            return false;
        }

        if ((ret = esp_bluedroid_init()) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s initialize bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
            return false;
        }

        if ((ret = esp_bluedroid_enable()) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s enable bluedroid failed: %s\n", __func__, esp_err_to_name(ret));
            return false;
        }

        if ((ret = esp_bt_gap_register_callback(esp_bt_gap_cb)) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s gap register failed: %s\n", __func__, esp_err_to_name(ret));
            return false;
        }

        if ((ret = esp_spp_register_callback(esp_spp_cb)) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s spp register failed: %s\n", __func__, esp_err_to_name(ret));
            return false;
        }

        if ((ret = esp_spp_init(ESP_SPP_MODE_CB)) != ESP_OK) {
            ESP_LOGE(SPP_TAG, "%s spp init failed: %s\n", __func__, esp_err_to_name(ret));
            return false;
        }

        // Set default parameters for Secure Simple Pairing;
        esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
        esp_bt_gap_set_security_param(ESP_BT_SP_IOCAP_MODE, &iocap, sizeof(uint8_t));

        // Set default parameters for Legacy Pairing. Use variable pin, input pin code when pairing
        esp_bt_pin_code_t pin_code;
        esp_bt_gap_set_pin(ESP_BT_PIN_TYPE_VARIABLE, 0, pin_code);

        // The default BTA_DM_COD_LOUDSPEAKER does not work with the macOS BT stack
        esp_bt_cod_t cod = {
            .major = 0b00001,
            .minor = 0b000100,
            .service = 0b00000010110
        };
        if (esp_bt_gap_set_cod(cod, ESP_BT_INIT_COD) != ESP_OK)
            return false;

        return true;
    }

    return false;
}

bool bluetooth_disable (void)
{
    if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {

        if(connection)
            esp_spp_disconnect(connection);

        esp_spp_deinit();
        esp_bluedroid_disable();
        esp_bluedroid_deinit();

        if (esp_bt_controller_disable() != ESP_OK)
            return false;

        while(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED);

        if(esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {

            if (esp_bt_controller_deinit() != ESP_OK)
                return false;

            vTaskDelay(1);

            if (esp_bt_controller_get_status() != ESP_BT_CONTROLLER_STATUS_IDLE)
                return false;
        }

        if(polltask) {
            flush_tx_queue();
            vTaskDelete(polltask);
            vEventGroupDelete(event_group);
            vSemaphoreDelete(tx_busy);
            vQueueDelete(tx_queue);
            polltask = event_group = tx_busy = tx_queue = NULL;
#if USE_BT_MUTEX
            vSemaphoreDelete(lock);
            lock = NULL;
#endif
        }
    }
    return true;
}

#if BLUETOOTH_ENABLE

static const setting_group_detail_t bluetooth_groups [] = {
    { Group_Root, Group_Bluetooth, "Bluetooth"},
};

static const setting_detail_t bluetooth_settings[] = {
    { Setting_BlueToothDeviceName, Group_Bluetooth, "Bluetooth device name", NULL, Format_String, "x(32)", NULL, "32", Setting_NonCore, bluetooth.device_name, NULL, NULL },
    { Setting_BlueToothServiceName, Group_Bluetooth, "Bluetooth service name", NULL, Format_String, "x(32)", NULL, "32", Setting_NonCore, bluetooth.service_name, NULL, NULL }
};

static void bluetooth_settings_restore (void)
{
    strcpy(bluetooth.device_name, BLUETOOTH_DEVICE);
    strcpy(bluetooth.service_name, BLUETOOTH_SERVICE);

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&bluetooth, sizeof(bluetooth_settings_t), true);
}

static void bluetooth_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&bluetooth, nvs_address, sizeof(bluetooth_settings_t), true) != NVS_TransferResult_OK)
        bluetooth_settings_restore();
}

static void bluetooth_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&bluetooth, sizeof(bluetooth_settings_t), true);
}

static setting_details_t details = {
    .groups = bluetooth_groups,
    .n_groups = sizeof(bluetooth_groups) / sizeof(setting_group_detail_t),
    .settings = bluetooth_settings,
    .n_settings = sizeof(bluetooth_settings) / sizeof(setting_detail_t),
    .save = bluetooth_settings_save,
    .load = bluetooth_settings_load,
    .restore = bluetooth_settings_restore
};

static setting_details_t *on_get_settings (void)
{
    return &details;
}

bool bluetooth_init (void)
{
    if((nvs_address = nvs_alloc(sizeof(bluetooth_settings_t)))) {

        hal.driver_cap.bluetooth = On;

        on_report_options = grbl.on_report_options;
        grbl.on_report_options = report_bt_MAC;

        details.on_get_settings = grbl.on_get_settings;
        grbl.on_get_settings = on_get_settings;
    }

    return nvs_address != 0;
}

#endif
