#ifndef OTA_SETUP_H
#define OTA_SETUP_H

/* Advanced HTTPS OTA example
   This example code is in the Public Domain (or CC0 licensed, at your option.)
   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include "global_data.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_https_ota.h"
#include "nvs.h"
#include "nvs_flash.h"

#define OTA_URL_SIZE 256

esp_err_t validate_image_header(esp_app_desc_t *new_app_info);
esp_err_t _http_client_init_cb(esp_http_client_handle_t http_client);
void ota_task(void *pvParameters);

#endif /* OTA_SETUP_H */
