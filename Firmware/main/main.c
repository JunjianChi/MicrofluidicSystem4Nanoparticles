/********************************************************
 * @file        main.c
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V2.1.0
 * @date        12/02/2026
 * @brief       Main application - DAC test
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#include "i2c_interface.h"
#include "mcp4726_dac.h"
#include "sensor_config.h"

static const char *TAG = "MAIN";

/********************************************************
 * DAC TEST
 *
 * Set a voltage, measure with multimeter on DAC output pin.
 * Valid range: 0 ~ 5.0V (VDD)
 * MP-Driver amplitude range: 0.35 ~ 1.3V
 ********************************************************/

#define TEST_VOLTAGE  0.35f   /* Change this to test (volts) */

void app_main(void)
{
    ESP_LOGI(TAG, "=== MCP4726 DAC Test ===");

    /* Initialize I2C */
    esp_err_t ret = i2c_interface_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed!");
        return;
    }

    /* Scan I2C bus - verify MCP4726 at 0x61 */
    uint8_t devices[16];
    uint8_t num_found = 0;
    i2c_interface_scan(devices, 16, &num_found);

    /* Initialize DAC */
    ret = mcp4726_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DAC init failed!");
        return;
    }

    vTaskDelay(pdMS_TO_TICKS(500));

    /* Set DAC output voltage */
    ret = mcp4726_set_voltage(TEST_VOLTAGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DAC set voltage failed!");
        return;
    }

    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, "  Target voltage: %.3f V", TEST_VOLTAGE);
    ESP_LOGI(TAG, "  Expected bit:   %d", (int)(TEST_VOLTAGE / 5.0f * 4096));
    ESP_LOGI(TAG, "  Measure DAC output pin with multimeter.");
    ESP_LOGI(TAG, "========================================");

    // /* Hold forever */
    // while (1) {
    //     vTaskDelay(pdMS_TO_TICKS(5000));
    //     ESP_LOGI(TAG, "Holding %.3f V ...", TEST_VOLTAGE);
    // }
}
