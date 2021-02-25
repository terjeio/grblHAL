/*
  mainc.c - An embedded CNC Controller with rs274/ngc (g-code) support

  Startup entry point for ESP32

  Part of grblHAL

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

/*
 * IMPORTANT:
 *
 * grbl/config.h changes needed for this driver
 *
 * Add: #include "esp_attr.h"
 * Change: #define ISR_CODE to #define ISR_CODE IRAM_ATTR
 */

// idf.py app-flash -p COM23

#include <stdint.h>
#include <stdbool.h>

#include "grbl/grbllib.h"

#include "nvs.h"
#include "nvs_flash.h"

/* Scheduler includes. */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "sdkconfig.h"

static void vGrblTask (void *pvParameters)
{
    grbl_enter();
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        if((ret = nvs_flash_erase()) == ESP_OK)
            ret = nvs_flash_init();
    }

    xTaskCreatePinnedToCore(vGrblTask, "Grbl", 4096, NULL, 0, NULL, 1);
}
