/********************************************************
 * @file        i2c_interface.c
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V1.0.0
 * @date        06/02/2026
 * @brief       I2C interface implementation
 *
 * @details
 *  - I2C master initialization and management
 *  - Helper functions for sensor communication
 *  - Board-specific hardware abstraction
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#include "i2c_interface.h"
#include "sensor_config.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "I2C_IF";

/* Flag to track I2C initialization status */
static bool i2c_initialized = false;

esp_err_t i2c_interface_init(void)
{
    if (i2c_initialized) {
        ESP_LOGW(TAG, "I2C already initialized");
        return ESP_OK;
    }

    /* Configure I2C parameters */
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };

    /* Configure I2C parameters */
    esp_err_t err = i2c_param_config(I2C_MASTER_PORT, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C param config failed: %s", esp_err_to_name(err));
        return err;
    }

    /* Install I2C driver */
    err = i2c_driver_install(I2C_MASTER_PORT, conf.mode,
                             I2C_MASTER_RX_BUF_DISABLE,
                             I2C_MASTER_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_initialized = true;

    ESP_LOGI(TAG, "I2C master initialized successfully");
    ESP_LOGI(TAG, "  Port: %d", I2C_MASTER_PORT);
    ESP_LOGI(TAG, "  SDA: GPIO%d", I2C_MASTER_SDA_IO);
    ESP_LOGI(TAG, "  SCL: GPIO%d", I2C_MASTER_SCL_IO);
    ESP_LOGI(TAG, "  Frequency: %d Hz", I2C_MASTER_FREQ_HZ);

    return ESP_OK;
}

esp_err_t i2c_interface_deinit(void)
{
    if (!i2c_initialized) {
        ESP_LOGW(TAG, "I2C not initialized");
        return ESP_OK;
    }

    esp_err_t err = i2c_driver_delete(I2C_MASTER_PORT);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver delete failed: %s", esp_err_to_name(err));
        return err;
    }

    i2c_initialized = false;
    ESP_LOGI(TAG, "I2C master deinitialized");

    return ESP_OK;
}

esp_err_t i2c_interface_write(uint8_t device_addr, const uint8_t *data, size_t len)
{
    if (!i2c_initialized) {
        ESP_LOGE(TAG, "I2C not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_PORT, device_addr,
                                                data, len,
                                                pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write to device 0x%02X failed: %s (len=%d)",
                 device_addr, esp_err_to_name(ret), len);
        ESP_LOGE(TAG, "Possible causes:");
        ESP_LOGE(TAG, "  1. Device not connected or powered");
        ESP_LOGE(TAG, "  2. Wrong I2C address (current: 0x%02X)", device_addr);
        ESP_LOGE(TAG, "  3. SDA/SCL pins incorrect (SDA=%d, SCL=%d)",
                 I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
        ESP_LOGE(TAG, "  4. Missing pull-up resistors");
    }

    return ret;
}

esp_err_t i2c_interface_read(uint8_t device_addr, uint8_t *data, size_t len)
{
    if (!i2c_initialized) {
        ESP_LOGE(TAG, "I2C not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_master_read_from_device(I2C_MASTER_PORT, device_addr,
                                                 data, len,
                                                 pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Read from device 0x%02X failed: %s", device_addr, esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t i2c_interface_write_read(uint8_t device_addr, uint8_t reg_addr,
                                    uint8_t *data, size_t len)
{
    if (!i2c_initialized) {
        ESP_LOGE(TAG, "I2C not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = i2c_master_write_read_device(I2C_MASTER_PORT, device_addr,
                                                  &reg_addr, 1,
                                                  data, len,
                                                  pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Write-read from device 0x%02X failed: %s",
                 device_addr, esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t i2c_interface_write_reg(uint8_t device_addr, uint8_t reg_addr,
                                   const uint8_t *data, size_t len)
{
    if (!i2c_initialized) {
        ESP_LOGE(TAG, "I2C not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (data == NULL || len == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Prepare write buffer: register address + data */
    uint8_t write_buf[len + 1];
    write_buf[0] = reg_addr;
    memcpy(&write_buf[1], data, len);

    esp_err_t ret = i2c_master_write_to_device(I2C_MASTER_PORT, device_addr,
                                                write_buf, len + 1,
                                                pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    if (ret != ESP_OK) {
        ESP_LOGD(TAG, "Write to register 0x%02X of device 0x%02X failed: %s",
                 reg_addr, device_addr, esp_err_to_name(ret));
    }

    return ret;
}

esp_err_t i2c_interface_read_reg(uint8_t device_addr, uint8_t reg_addr,
                                  uint8_t *data, size_t len)
{
    return i2c_interface_write_read(device_addr, reg_addr, data, len);
}

esp_err_t i2c_interface_probe(uint8_t device_addr)
{
    if (!i2c_initialized) {
        ESP_LOGE(TAG, "I2C not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (device_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);

    esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_PORT, cmd,
                                          pdMS_TO_TICKS(I2C_TIMEOUT_MS));
    i2c_cmd_link_delete(cmd);

    return (ret == ESP_OK) ? ESP_OK : ESP_ERR_NOT_FOUND;
}

esp_err_t i2c_interface_scan(uint8_t *devices, uint8_t max_devices, uint8_t *num_found)
{
    if (!i2c_initialized) {
        ESP_LOGE(TAG, "I2C not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (devices == NULL || num_found == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    *num_found = 0;

    ESP_LOGI(TAG, "Scanning I2C bus...");

    for (uint8_t addr = 0x03; addr < 0x78; addr++) {
        if (i2c_interface_probe(addr) == ESP_OK) {
            ESP_LOGI(TAG, "  Found device at address: 0x%02X", addr);

            if (*num_found < max_devices) {
                devices[*num_found] = addr;
                (*num_found)++;
            }
        }
    }

    ESP_LOGI(TAG, "I2C scan complete. Found %d device(s)", *num_found);

    return ESP_OK;
}

i2c_port_t i2c_interface_get_port(void)
{
    return I2C_MASTER_PORT;
}
