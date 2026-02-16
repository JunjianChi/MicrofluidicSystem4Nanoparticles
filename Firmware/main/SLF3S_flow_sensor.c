/********************************************************
 * @file        SLF3S_flow_sensor.c
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V1.0.0
 * @date        06/02/2026
 * @brief       Driver implementation for Sensirion SLF3S flow sensor
 *
 * @details
 *  - I2C communication with SLF3S liquid flow sensor
 *  - Auto-detection of sensor model (1300F or 0600F)
 *  - Water and IPA calibration support
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#include "SLF3S_flow_sensor.h"
#include "i2c_interface.h"
#include "sensor_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "SLF3S";

/* Command definitions */
#define CMD_SOFT_RESET      0x06
#define CMD_START_MEAS_H    0x36
#define CMD_STOP_MEAS_H     0x3F
#define CMD_STOP_MEAS_L     0xF9
#define CMD_READ_PROD_H     0x36
#define CMD_READ_PROD_L     0x7C
#define CMD_READ_PROD_H2    0xE1
#define CMD_READ_PROD_L2    0x02

/**
 * @brief Write command to SLF3S sensor
 */
static esp_err_t slf3s_write_cmd(const uint8_t *cmd, size_t cmd_len)
{
    esp_err_t ret = i2c_interface_write(SLF3S_I2C_ADDR, cmd, cmd_len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

/**
 * @brief Read data from SLF3S sensor
 */
static esp_err_t slf3s_read_data(uint8_t *data, size_t len)
{
    esp_err_t ret = i2c_interface_read(SLF3S_I2C_ADDR, data, len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t slf3s_init(slf3s_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    memset(handle, 0, sizeof(slf3s_handle_t));
    handle->calibration_cmd = SLF3S_CALIBRATION_WATER;

    /* Use configured scale factor and product number from Kconfig */
    handle->scale_factor = FLOW_SENSOR_SCALE_FACTOR;
    handle->product_number = FLOW_SENSOR_PRODUCT_NUMBER;
    handle->is_initialized = false;

    // ESP_LOGI(TAG, "SLF3S flow sensor initialized");
    // ESP_LOGI(TAG, "Configured model: %s", FLOW_SENSOR_MODEL_NAME);
    // ESP_LOGI(TAG, "Scale factor: %.1f", handle->scale_factor);

    return ESP_OK;
}

esp_err_t slf3s_start_measurement(slf3s_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint8_t cmd[2];
    uint8_t data[6];
    uint32_t product_number;

    /* Stop any ongoing measurement */
    cmd[0] = CMD_STOP_MEAS_H;
    cmd[1] = CMD_STOP_MEAS_L;
    slf3s_write_cmd(cmd, 2);
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Read Product Identifier */
    cmd[0] = CMD_READ_PROD_H;
    cmd[1] = CMD_READ_PROD_L;
    ret = slf3s_write_cmd(cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    cmd[0] = CMD_READ_PROD_H2;
    cmd[1] = CMD_READ_PROD_L2;
    ret = slf3s_write_cmd(cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(50));

    /* Read product number (6 bytes: MSB MSB CRC LSB LSB CRC) */
    ret = slf3s_read_data(data, 6);
    if (ret != ESP_OK) {
        return ret;
    }

    product_number = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                     ((uint32_t)data[3] << 8) | (uint32_t)data[4];

    // ESP_LOGI(TAG, "Detected product number: 0x%08lX", product_number);

    /* Verify detected product number matches configured model */
    if ((product_number & 0xFFFFFF00) == SLF3S_PN_1300) {
        // ESP_LOGI(TAG, "Auto-detected: SLF3S-1300F");
        if (FLOW_SENSOR_PRODUCT_NUMBER != SLF3S_PN_1300) {
            ESP_LOGW(TAG, "WARNING: Detected 1300F but configured as %s", FLOW_SENSOR_MODEL_NAME);
            ESP_LOGW(TAG, "Using configured scale factor: %.1f", FLOW_SENSOR_SCALE_FACTOR);
        }
    } else if ((product_number & 0xFFFFFF00) == SLF3S_PN_0600) {
        // ESP_LOGI(TAG, "Auto-detected: SLF3S-0600F");
        if (FLOW_SENSOR_PRODUCT_NUMBER != SLF3S_PN_0600) {
            ESP_LOGW(TAG, "WARNING: Detected 0600F but configured as %s", FLOW_SENSOR_MODEL_NAME);
            ESP_LOGW(TAG, "Using configured scale factor: %.1f", FLOW_SENSOR_SCALE_FACTOR);
        }
    } else {
        ESP_LOGW(TAG, "Unknown product number: 0x%08lX", product_number);
        // ESP_LOGI(TAG, "Using configured model: %s", FLOW_SENSOR_MODEL_NAME);
    }

    /* Always use configured scale factor from Kconfig */
    handle->scale_factor = FLOW_SENSOR_SCALE_FACTOR;
    // ESP_LOGI(TAG, "Using scale factor: %.1f", handle->scale_factor);

    /* Soft reset */
    cmd[0] = CMD_SOFT_RESET;
    ret = slf3s_write_cmd(cmd, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(100));

    /* Start continuous measurement */
    cmd[0] = CMD_START_MEAS_H;
    cmd[1] = handle->calibration_cmd;
    ret = slf3s_write_cmd(cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    handle->is_initialized = true;
    // ESP_LOGI(TAG, "Flow measurement started (calibration: 0x%02X)", handle->calibration_cmd);

    return ESP_OK;
}

esp_err_t slf3s_start_measurement_simple(slf3s_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint8_t cmd[2];

    /* Soft reset */
    cmd[0] = CMD_SOFT_RESET;
    ret = slf3s_write_cmd(cmd, 1);
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(50));

    /* Start continuous measurement */
    cmd[0] = CMD_START_MEAS_H;
    cmd[1] = handle->calibration_cmd;
    ret = slf3s_write_cmd(cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    handle->is_initialized = true;
    // ESP_LOGI(TAG, "Flow measurement started (simple mode)");
    // ESP_LOGI(TAG, "Using configured model: %s (scale: %.1f)",
             // FLOW_SENSOR_MODEL_NAME, handle->scale_factor);

    return ESP_OK;
}

esp_err_t slf3s_stop_measurement(slf3s_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint8_t cmd[2];

    /* Stop continuous measurement */
    cmd[0] = CMD_STOP_MEAS_H;
    cmd[1] = CMD_STOP_MEAS_L;
    ret = slf3s_write_cmd(cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    vTaskDelay(pdMS_TO_TICKS(100));
    handle->is_initialized = false;

    // ESP_LOGI(TAG, "Flow measurement stopped");
    return ESP_OK;
}

esp_err_t slf3s_read_flow(slf3s_handle_t *handle, float *flow_value)
{
    if (handle == NULL || flow_value == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->is_initialized) {
        ESP_LOGW(TAG, "Sensor not initialized, call start_measurement first");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;
    uint8_t data[3];
    uint16_t sensor_flow_value;
    int16_t signed_flow_value;

    /* Read 3 bytes: MSB LSB CRC */
    ret = slf3s_read_data(data, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read flow data from sensor");
        return ret;
    }

    /* Debug: Print raw data */
    ESP_LOGD(TAG, "Raw data: 0x%02X 0x%02X 0x%02X", data[0], data[1], data[2]);

    /* Combine MSB and LSB */
    sensor_flow_value = ((uint16_t)data[0] << 8) | (uint16_t)data[1];
    /* CRC is in data[2], but not checked in this version */

    /* Convert to signed value */
    signed_flow_value = (int16_t)sensor_flow_value;

    /* Scale the value */
    *flow_value = ((float)signed_flow_value) / handle->scale_factor;

    ESP_LOGD(TAG, "Sensor value: %d, Scale factor: %.1f, Flow: %.2f µl/min",
             signed_flow_value, handle->scale_factor, *flow_value);

    return ESP_OK;
}

esp_err_t slf3s_set_calibration(slf3s_handle_t *handle, uint8_t calibration_cmd)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    handle->calibration_cmd = calibration_cmd;
    // ESP_LOGI(TAG, "Calibration set to: 0x%02X", calibration_cmd);

    return ESP_OK;
}
