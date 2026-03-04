/********************************************************
 * @file        abp_pressure_sensor.c
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V1.0.0
 * @date        04/03/2026
 * @brief       Driver for Honeywell ABP pressure sensor (ABPMANV015PG7A5)
 *
 * @details
 *  - Simple I2C read interface (no register addressing)
 *  - Status check via 1-byte read
 *  - Pressure read via 2-byte read with 14-bit extraction
 *  - Conversion: raw -> psi -> bar
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#include "abp_pressure_sensor.h"
#include "i2c_interface.h"
#include "esp_log.h"

static const char *TAG = "ABP";

esp_err_t abp_init(abp_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    handle->is_initialized = false;
    handle->last_pressure = 0.0f;

    /* Verify sensor is present on bus */
    esp_err_t ret = i2c_interface_probe(ABP_I2C_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "ABP sensor not found at 0x%02X", ABP_I2C_ADDR);
        return ret;
    }

    handle->is_initialized = true;
    ESP_LOGI(TAG, "ABP pressure sensor initialized at 0x%02X", ABP_I2C_ADDR);
    return ESP_OK;
}

esp_err_t abp_check_ready(abp_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t status_byte = 0;
    esp_err_t ret = i2c_interface_read(ABP_I2C_ADDR, &status_byte, 1);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    uint8_t status = status_byte >> 6;
    if (status != ABP_STATUS_NORMAL) {
        return ESP_ERR_INVALID_STATE;
    }

    return ESP_OK;
}

esp_err_t abp_read_pressure(abp_handle_t *handle, float *pressure_bar)
{
    if (handle == NULL || pressure_bar == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t raw[2] = {0};
    esp_err_t ret = i2c_interface_read(ABP_I2C_ADDR, raw, 2);
    if (ret != ESP_OK) {
        return ESP_FAIL;
    }

    /* Check status bits (top 2 bits of first byte) */
    uint8_t status = raw[0] >> 6;
    if (status != ABP_STATUS_NORMAL) {
        return ESP_ERR_INVALID_STATE;
    }

    /* Extract 14-bit pressure data */
    uint16_t pdata = ((uint16_t)raw[0] << 8) | raw[1];
    pdata &= 0x3FFF;

    /* Convert to psi, then to bar */
    float psi = (float)pdata * 1.1443e-3f - 1.8744f;
    float bar = psi * 0.0689476f;

    *pressure_bar = bar;
    handle->last_pressure = bar;

    return ESP_OK;
}
