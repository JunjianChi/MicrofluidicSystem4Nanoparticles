/********************************************************
 * @file        main.c
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V1.0.0
 * @date        06/02/2026
 * @brief       Main application for microfluidic control system
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "i2c_interface.h"
#include "SLF3S_flow_sensor.h"
#include "sensor_config.h"

static const char *TAG = "MAIN";

/* Flow sensor handle */
static slf3s_handle_t flow_sensor;

void app_main(void)
{
    /* Enable debug logging for flow sensor */
    esp_log_level_set("SLF3S", ESP_LOG_DEBUG);

    ESP_LOGI(TAG, "=== Microfluidic Control System ===");
    ESP_LOGI(TAG, "Firmware Version: 1.0.0");

    /* Initialize I2C interface */
    ESP_LOGI(TAG, "Initializing I2C interface...");
    esp_err_t ret = i2c_interface_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed!");
        return;
    }

    /* Scan I2C bus to detect devices */
    ESP_LOGI(TAG, "Scanning I2C bus...");
    uint8_t devices[128];
    uint8_t num_found = 0;
    i2c_interface_scan(devices, 128, &num_found);

    if (num_found == 0) {
        ESP_LOGW(TAG, "No I2C devices found! Check your connections.");
    }

    /* Initialize flow sensor */
    ESP_LOGI(TAG, "Initializing flow sensor...");
    ret = slf3s_init(&flow_sensor);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Flow sensor initialization failed!");
        return;
    }

    /* Start flow measurement */
    ESP_LOGI(TAG, "Starting flow measurement...");
    ret = slf3s_start_measurement(&flow_sensor);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Flow measurement start failed, trying simple mode...");
        ret = slf3s_start_measurement_simple(&flow_sensor);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Could not start flow measurement!");
            return;
        }
    }

    ESP_LOGI(TAG, "System ready! Waiting for sensor to stabilize...");

    /* Wait for sensor to stabilize after starting measurement */
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "Starting to read flow sensor...");

    /* Main loop - read flow sensor */
    while (1) {
        float flow_rate = 0.0f;

        ret = slf3s_read_flow(&flow_sensor, &flow_rate);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Flow rate: %.2f µl/min", flow_rate);
        } else {
            ESP_LOGW(TAG, "Failed to read flow sensor: %s", esp_err_to_name(ret));
        }

        /* Wait before next reading (sensor updates ~1kHz, but we read slower) */
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
