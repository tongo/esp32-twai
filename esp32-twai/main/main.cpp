/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "can_manager.hpp"

static const char *TAG = "main";

extern "C" void app_main(void)
{
    printf("=================== TWAI Example Starting... ===================\n");
    ESP_LOGI(TAG, "START");
    CanManager* canManager = new CanManager(true);
    canManager->begin();
    canManager->enableNode();
    
    vTaskDelay(pdMS_TO_TICKS(20000));
    printf("=================== DISABLE NODE ===================\n");
    canManager->disableNode();

    vTaskDelay(pdMS_TO_TICKS(5000));
    printf("=================== ENABLE NODE ===================\n");
    canManager->enableNode();
    vTaskDelay(pdMS_TO_TICKS(20000));
    printf("=================== DISABLE NODE ===================\n");
    canManager->disableNode();

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        ESP_LOGI(TAG, "Tutto fermo");
    }
}
