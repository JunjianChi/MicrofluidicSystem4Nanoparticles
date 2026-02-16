/********************************************************
 * @file        sensor_config.c
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V1.0.0
 * @date        06/02/2026
 * @brief       Sensor and system configuration implementation
 *
 * @details
 *  - System configuration initialization
 *  - Configuration utility functions
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#include "sensor_config.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "SYS_CONFIG";

void system_config_init(system_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Config pointer is NULL");
        return;
    }

    memset(config, 0, sizeof(system_config_t));

    /* Initialize sensor enable flags */
    config->pressure_enabled = PRESSURE_SENSOR_ENABLED;
    config->temperature_enabled = TEMPERATURE_SENSOR_ENABLED;
    config->flow_enabled = FLOW_SENSOR_ENABLED;

    /* Initialize pump driver type */
    config->pump_driver = PUMP_DRIVER_TYPE;

    /* Initialize communication type */
    config->comm_type = COMMUNICATION_TYPE;

    /* Initialize pin configuration */
    config->pins.i2c_sda = I2C_MASTER_SDA_IO;
    config->pins.i2c_scl = I2C_MASTER_SCL_IO;
    config->pins.mp_enable = MP_DRIVER_ENABLE_IO;
    config->pins.mp_clock = MP_DRIVER_CLOCK_IO;

#ifdef CONFIG_COMM_WIFI
    /* Initialize WiFi configuration if enabled */
    config->wifi.ssid = WIFI_SSID;
    config->wifi.password = WIFI_PASSWORD;
    config->wifi.server_addr = SERVER_ADDRESS;
    config->wifi.server_port = SERVER_PORT;
#endif

    // ESP_LOGI(TAG, "System configuration initialized");
}

void system_config_print(const system_config_t *config)
{
    if (config == NULL) {
        ESP_LOGE(TAG, "Config pointer is NULL");
        return;
    }

    // ESP_LOGI(TAG, "========== System Configuration ==========");

    /* Sensor configuration */
    // ESP_LOGI(TAG, "Sensors:");
    // ESP_LOGI(TAG, "  Pressure:    %s", config->pressure_enabled ? "ENABLED" : "DISABLED");
    // ESP_LOGI(TAG, "  Temperature: %s", config->temperature_enabled ? "ENABLED" : "DISABLED");
    // ESP_LOGI(TAG, "  Flow:        %s", config->flow_enabled ? "ENABLED" : "DISABLED");

    /* Pump driver configuration */
    // ESP_LOGI(TAG, "Pump Driver:");
    switch (config->pump_driver) {
        case PUMP_DRIVER_LOW:
            // ESP_LOGI(TAG, "  Type: LOW MP-Driver");
            break;
        case PUMP_DRIVER_HIGH:
            // ESP_LOGI(TAG, "  Type: HIGH MP-Driver");
            break;
        case PUMP_DRIVER_STANDARD:
            // ESP_LOGI(TAG, "  Type: Standard MP-Driver");
            break;
        default:
            // ESP_LOGI(TAG, "  Type: Unknown");
            break;
    }

    /* Communication configuration */
    // ESP_LOGI(TAG, "Communication:");
    if (config->comm_type == COMM_TYPE_WIFI) {
        // ESP_LOGI(TAG, "  Type: WiFi");
#ifdef CONFIG_COMM_WIFI
        // ESP_LOGI(TAG, "  SSID: %s", config->wifi.ssid);
        // ESP_LOGI(TAG, "  Server: %s:%d", config->wifi.server_addr, config->wifi.server_port);
#endif
    } else {
        // ESP_LOGI(TAG, "  Type: Serial");
    }

    /* Pin configuration */
    // ESP_LOGI(TAG, "Pin Configuration:");
    // ESP_LOGI(TAG, "  I2C SDA:       GPIO %d", config->pins.i2c_sda);
    // ESP_LOGI(TAG, "  I2C SCL:       GPIO %d", config->pins.i2c_scl);
    // ESP_LOGI(TAG, "  MP Enable:     GPIO %d", config->pins.mp_enable);
    // ESP_LOGI(TAG, "  MP Clock:      GPIO %d", config->pins.mp_clock);

    // ESP_LOGI(TAG, "==========================================");
}
