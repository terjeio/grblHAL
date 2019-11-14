/*
  bluetooth.h - An embedded CNC Controller with rs274/ngc (g-code) support

  Bluetooth comms

  Part of Grbl

  Copyright (c) 2018-2019 Terje Io

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
#include "esp_log.h"
#include "esp_bt.h"
#include "esp_bt_main.h"
#include "esp_gap_bt_api.h"
#include "esp_bt_device.h"
#include "esp_spp_api.h"

#include "driver.h"
#include "serial.h"
#include "GRBL/grbl.h"

typedef struct {
    volatile uint16_t head;
//    bool congested;
//    xSemaphoreHandle lock;
    char data[TX_BUFFER_SIZE];
} bt_tx_buffer_t;

#define BT_MUTEX_LOCK()    do {} while (xSemaphoreTake(lock, portMAX_DELAY) != pdPASS)
#define UART_MUTEX_UNLOCK()  xSemaphoreGive(lock)

#define BT_MUTEX_TXLOCK()    do {} while (xSemaphoreTake(txbuffer_send->lock, portMAX_DELAY) != pdPASS)
#define BT_MUTEX_TXUNLOCK()  xSemaphoreGive(txbuffer_send->lock)

#define SPP_TAG "SPP_ACCEPTOR_DEMO"

static uint32_t connection = 0;
static bluetooth_settings_t *bt_settings;
static xSemaphoreHandle lock = NULL;

static serial_rx_buffer_t rxbuffer = {
	.head = 0,
	.tail = 0,
	.backup = false,
	.overflow = false,
	.rts_state = false
};

static serial_rx_buffer_t rxbackup;

static bt_tx_buffer_t tx_buffers[2], *txbuffer, *txbuffer_send;

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
        UART_MUTEX_UNLOCK();
        return -1; // no data available else EOF
    }
    data = rxbuffer.data[bptr++];                 // Get next character, increment tmp pointer
    rxbuffer.tail = bptr & (RX_BUFFER_SIZE - 1);  // and update pointer
    UART_MUTEX_UNLOCK();
    return data;
}

// Since grbl always sends cr/lf terminated strings we can use double
// buffering and only send complete strings to improve throughput
// Note to self: seems like esp_spp_write does a deep copy of payload so db likely not needed...
bool BTStreamPutC (const char c)
{
//	while(xSemaphoreTake(txbuffer->lock, portMAX_DELAY) != pdPASS);
    txbuffer->data[txbuffer->head++] = c;
//	xSemaphoreGive(txbuffer->lock);

    if(c == '\n') {
//    	while(xSemaphoreTake(txbuffer->lock, portMAX_DELAY) != pdPASS);
    	esp_spp_write(connection, txbuffer->head, (uint8_t *)txbuffer->data);
    	txbuffer->head = 0;
    	txbuffer_send = txbuffer;
    	txbuffer = txbuffer == &tx_buffers[0] ? &tx_buffers[1] : &tx_buffers[0];
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
    UART_MUTEX_UNLOCK();
}

IRAM_ATTR void BTStreamCancel (void)
{
//    BT_MUTEX_LOCK();
    rxbuffer.data[rxbuffer.head] = ASCII_CAN;
    rxbuffer.tail = rxbuffer.head;
    rxbuffer.head = (rxbuffer.tail + 1) & (RX_BUFFER_SIZE - 1);
//    UART_MUTEX_UNLOCK();
}

bool BTStreamSuspendInput (bool suspend)
{
    BT_MUTEX_LOCK();
    if(suspend)
        hal.stream.read = BTStreamGetNull;
    else if(rxbuffer.backup)
        memcpy(&rxbuffer, &rxbackup, sizeof(serial_rx_buffer_t));
    UART_MUTEX_UNLOCK();
    return rxbuffer.tail != rxbuffer.head;
}

static void esp_spp_cb (esp_spp_cb_event_t event, esp_spp_cb_param_t *param)
{
    switch (event) {

    	case ESP_SPP_INIT_EVT:

			esp_bt_dev_set_device_name(bt_settings->device_name);
			esp_bt_gap_set_scan_mode(ESP_BT_SCAN_MODE_CONNECTABLE_DISCOVERABLE);
			esp_spp_start_srv(ESP_SPP_SEC_AUTHENTICATE, ESP_SPP_ROLE_SLAVE, 0, bt_settings->service_name);
			break;

		case ESP_SPP_SRV_OPEN_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_OPEN_EVT");
			connection = param->open.handle;
		    tx_buffers[0].head = tx_buffers[1].head = 0;
			txbuffer = txbuffer_send = &tx_buffers[0];
			selectStream(StreamType_Bluetooth);
			hal.stream.write_all("[MSG:BT OK]\r\n");
			break;

		case ESP_SPP_CLOSE_EVT:
//	        BT_MUTEX_TXUNLOCK();
			selectStream(StreamType_Serial);
			connection = 0;
			ESP_LOGI(SPP_TAG, "ESP_SPP_CLOSE_EVT");
			break;

		case ESP_SPP_DATA_IND_EVT:;
			char c;
			while(param->data_ind.len) {
				c = *param->data_ind.data;
				if(c == CMD_TOOL_ACK && !rxbuffer.backup) {

					memcpy(&rxbackup, &rxbuffer, sizeof(serial_rx_buffer_t));
					rxbuffer.backup = true;
					rxbuffer.tail = rxbuffer.head;
					hal.stream.read = BTStreamGetC; // restore normal input

				} else if(!hal.stream.enqueue_realtime_command(c)) {

					uint32_t bptr = (rxbuffer.head + 1) & (RX_BUFFER_SIZE - 1);  // Get next head pointer

					if(bptr == rxbuffer.tail)                   // If buffer full
						rxbuffer.overflow = 1;                  // flag overflow,
					else {
						rxbuffer.data[rxbuffer.head] = (char)c; // else add data to buffer
						rxbuffer.head = bptr;                   // and update pointer
					}
				}
				param->data_ind.len--;
				param->data_ind.data++;
			}
			break;

		case ESP_SPP_CONG_EVT:
			ESP_LOGI(SPP_TAG, "ESP_SPP_CONG_EVT");
			break;

		case ESP_SPP_WRITE_EVT:
		//	ESP_LOGI(SPP_TAG, "ESP_SPP_WRITE_EVT");
		//	xSemaphoreGive(txbuffer_send->lock);
			break;

		default:
			break;
    }
}

void esp_bt_gap_cb(esp_bt_gap_cb_event_t event, esp_bt_gap_cb_param_t *param)
{
    switch (event) {

		case ESP_BT_GAP_AUTH_CMPL_EVT:{
			if (param->auth_cmpl.stat == ESP_BT_STATUS_SUCCESS)
				esp_log_buffer_hex(SPP_TAG, param->auth_cmpl.bda, ESP_BD_ADDR_LEN);
			else {
				ESP_LOGE(SPP_TAG, "authentication failed, status:%d", param->auth_cmpl.stat);
			}
			break;
		}

		case ESP_BT_GAP_PIN_REQ_EVT:{
			if (param->pin_req.min_16_digit) {
				ESP_LOGI(SPP_TAG, "Input pin code: 0000 0000 0000 0000");
				esp_bt_pin_code_t pin_code = {0};
				esp_bt_gap_pin_reply(param->pin_req.bda, true, 16, pin_code);
			} else {
				esp_bt_pin_code_t pin_code;
				pin_code[0] = '1';
				pin_code[1] = '2';
				pin_code[2] = '3';
				pin_code[3] = '4';
				esp_bt_gap_pin_reply(param->pin_req.bda, true, 4, pin_code);
			}
			break;
		}

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

bool bluetooth_init (bluetooth_settings_t *settings)
{
	bt_settings = settings;

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if((ret = nvs_flash_erase()) == ESP_OK)
        	ret = nvs_flash_init();
    }

    if(ret == ESP_OK && (ret = esp_bt_controller_mem_release(ESP_BT_MODE_BLE)) == ESP_OK) {

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

		/* Set default parameters for Secure Simple Pairing */
		esp_bt_sp_param_t param_type = ESP_BT_SP_IOCAP_MODE;
		esp_bt_io_cap_t iocap = ESP_BT_IO_CAP_IO;
		esp_bt_gap_set_security_param(param_type, &iocap, sizeof(uint8_t));

		/*
		 * Set default parameters for Legacy Pairing
		 * Use variable pin, input pin code when pairing
		 */
		esp_bt_pin_type_t pin_type = ESP_BT_PIN_TYPE_VARIABLE;
		esp_bt_pin_code_t pin_code;
		esp_bt_gap_set_pin(pin_type, 0, pin_code);

	//    tx_buffers[0].lock = xSemaphoreCreateMutex();
	//    tx_buffers[1].lock = xSemaphoreCreateMutex();

		lock = xSemaphoreCreateMutex();

		return lock != NULL;
    }

    return false;
}

#if BLUETOOTH_ENABLE

status_code_t bluetooth_setting (uint_fast16_t param, float value, char *svalue)
{
	status_code_t status = Status_Unhandled;

	if(svalue) switch(param) {

		case Settings_BlueToothDeviceName:
			if(strlcpy(driver_settings.bluetooth.device_name, svalue, sizeof(driver_settings.bluetooth.device_name)) <= sizeof(driver_settings.bluetooth.device_name))
				status = Status_OK;
			else
				status = Status_InvalidStatement; // too long...				;
			break;

		case Settings_BlueToothServiceName:
			if(strlcpy(driver_settings.bluetooth.service_name, svalue, sizeof(driver_settings.bluetooth.service_name)) <= sizeof(driver_settings.bluetooth.service_name))
				status = Status_OK;
			else
				status = Status_InvalidStatement; // too long...				;
			break;
	}

	return status;
}

void bluetooth_settings_report (void)
{
	report_string_setting(Settings_BlueToothDeviceName, driver_settings.bluetooth.device_name);
	report_string_setting(Settings_BlueToothServiceName, driver_settings.bluetooth.service_name);
}

void bluetooth_settings_restore (void)
{
	strcpy(driver_settings.bluetooth.device_name, "GRBL");
	strcpy(driver_settings.bluetooth.service_name, "GRBL Serial Port");
}

#endif
