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
#define CMD_SOFT_RESET      0x06    /*!< Sent to general call addr 0x00 */
#define CMD_START_MEAS_H    0x36
#define CMD_STOP_MEAS_H     0x3F
#define CMD_STOP_MEAS_L     0xF9
#define CMD_READ_PROD_H     0x36
#define CMD_READ_PROD_L     0x7C
#define CMD_READ_PROD_H2    0xE1
#define CMD_READ_PROD_L2    0x02

/* Timing constants (ms) */
#define SLF3S_RESET_TIME_MS     25
#define SLF3S_STOP_TIME_MS      1
#define SLF3S_STARTUP_TIME_MS   12

/**
 * @brief CRC-8 checksum (polynomial 0x31, init 0xFF)
 */
static uint8_t slf3s_crc8(const uint8_t *data, size_t len)
{
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

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
 * @brief Send soft reset via general call address (0x00)
 */
static esp_err_t slf3s_soft_reset(void)
{
    uint8_t cmd = CMD_SOFT_RESET;
    esp_err_t ret = i2c_interface_write(SLF3S_GENERAL_CALL_ADDR, &cmd, 1);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Soft reset failed: %s", esp_err_to_name(ret));
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

/**
 * @brief Verify CRC for a 2-byte data word
 */
static esp_err_t slf3s_check_crc(const uint8_t *data)
{
    uint8_t expected = slf3s_crc8(data, 2);
    if (expected != data[2]) {
        ESP_LOGE(TAG, "CRC mismatch: expected 0x%02X, got 0x%02X", expected, data[2]);
        return ESP_ERR_INVALID_CRC;
    }
    return ESP_OK;
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
    uint8_t data[18];
    uint32_t product_number;

    /* Stop any ongoing measurement */
    cmd[0] = CMD_STOP_MEAS_H;
    cmd[1] = CMD_STOP_MEAS_L;
    slf3s_write_cmd(cmd, 2);
    vTaskDelay(pdMS_TO_TICKS(SLF3S_STOP_TIME_MS + 1));

    /* Read Product Identifier: send two 16-bit commands */
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

    /* Read 18 bytes: Product Number (4B + 2 CRC) + Serial Number (8B + 4 CRC) */
    ret = slf3s_read_data(data, 18);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Verify CRC for all 6 words */
    for (int i = 0; i < 18; i += 3) {
        ret = slf3s_check_crc(&data[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Product ID CRC error at byte %d", i);
            return ret;
        }
    }

    /* Product number: bytes 0,1 (MSW) + bytes 3,4 (LSW), skip CRC bytes */
    product_number = ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) |
                     ((uint32_t)data[3] << 8) | (uint32_t)data[4];
    handle->product_number = product_number;

    /* Serial number: bytes 6,7 + 9,10 + 12,13 + 15,16 (skip CRC at 8,11,14,17) */
    handle->serial_number = ((uint64_t)data[6]  << 56) | ((uint64_t)data[7]  << 48) |
                            ((uint64_t)data[9]  << 40) | ((uint64_t)data[10] << 32) |
                            ((uint64_t)data[12] << 24) | ((uint64_t)data[13] << 16) |
                            ((uint64_t)data[15] << 8)  | (uint64_t)data[16];

    ESP_LOGI(TAG, "Product: 0x%08lX, Serial: 0x%016llX",
             (unsigned long)product_number, (unsigned long long)handle->serial_number);

    /* Verify detected product number matches configured model */
    if ((product_number & 0xFFFFFF00) == SLF3S_PN_1300) {
        if (FLOW_SENSOR_PRODUCT_NUMBER != SLF3S_PN_1300) {
            ESP_LOGW(TAG, "WARNING: Detected 1300F but configured as %s", FLOW_SENSOR_MODEL_NAME);
        }
    } else if ((product_number & 0xFFFFFF00) == SLF3S_PN_0600) {
        if (FLOW_SENSOR_PRODUCT_NUMBER != SLF3S_PN_0600) {
            ESP_LOGW(TAG, "WARNING: Detected 0600F but configured as %s", FLOW_SENSOR_MODEL_NAME);
        }
    } else {
        ESP_LOGW(TAG, "Unknown product number: 0x%08lX", (unsigned long)product_number);
    }

    /* Always use configured scale factor from Kconfig */
    handle->scale_factor = FLOW_SENSOR_SCALE_FACTOR;

    /* Soft reset via general call address 0x00 */
    ret = slf3s_soft_reset();
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(SLF3S_RESET_TIME_MS + 5));

    /* Start continuous measurement */
    cmd[0] = CMD_START_MEAS_H;
    cmd[1] = handle->calibration_cmd;
    ret = slf3s_write_cmd(cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Wait for first measurement to be available */
    vTaskDelay(pdMS_TO_TICKS(SLF3S_STARTUP_TIME_MS + 5));

    handle->is_initialized = true;

    return ESP_OK;
}

esp_err_t slf3s_start_measurement_simple(slf3s_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    uint8_t cmd[2];

    /* Soft reset via general call address 0x00 */
    ret = slf3s_soft_reset();
    if (ret != ESP_OK) {
        return ret;
    }
    vTaskDelay(pdMS_TO_TICKS(SLF3S_RESET_TIME_MS + 5));

    /* Start continuous measurement */
    cmd[0] = CMD_START_MEAS_H;
    cmd[1] = handle->calibration_cmd;
    ret = slf3s_write_cmd(cmd, 2);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Wait for first measurement to be available */
    vTaskDelay(pdMS_TO_TICKS(SLF3S_STARTUP_TIME_MS + 5));

    handle->is_initialized = true;

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

    vTaskDelay(pdMS_TO_TICKS(SLF3S_STOP_TIME_MS + 1));
    handle->is_initialized = false;

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

    /* Read 3 bytes: Flow MSB, Flow LSB, CRC */
    ret = slf3s_read_data(data, 3);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read flow data from sensor");
        return ret;
    }

    /* Verify CRC */
    ret = slf3s_check_crc(data);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Convert raw to flow (signed 16-bit / scale factor) */
    int16_t raw_flow = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    *flow_value = (float)raw_flow / handle->scale_factor;

    ESP_LOGD(TAG, "Raw: %d, Flow: %.2f µl/min", raw_flow, *flow_value);

    return ESP_OK;
}

esp_err_t slf3s_read_measurement(slf3s_handle_t *handle, slf3s_measurement_t *measurement)
{
    if (handle == NULL || measurement == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    if (!handle->is_initialized) {
        ESP_LOGW(TAG, "Sensor not initialized, call start_measurement first");
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret;
    uint8_t data[9];

    /* Read 9 bytes: Flow(2)+CRC, Temp(2)+CRC, Flags(2)+CRC */
    ret = slf3s_read_data(data, 9);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read measurement data");
        return ret;
    }

    /* Verify CRC for each word */
    for (int i = 0; i < 9; i += 3) {
        ret = slf3s_check_crc(&data[i]);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "CRC error at byte %d", i);
            return ret;
        }
    }

    /* Flow rate: signed 16-bit / scale factor → µl/min */
    int16_t raw_flow = (int16_t)(((uint16_t)data[0] << 8) | data[1]);
    measurement->flow = (float)raw_flow / handle->scale_factor;

    /* Temperature: signed 16-bit / 200 → °C */
    int16_t raw_temp = (int16_t)(((uint16_t)data[3] << 8) | data[4]);
    measurement->temperature = (float)raw_temp / SLF3S_TEMP_SCALE_FACTOR;

    /* Signaling flags */
    measurement->flags = ((uint16_t)data[6] << 8) | data[7];

    /* Cache in handle */
    handle->last_temperature = measurement->temperature;
    handle->last_flags = measurement->flags;

    ESP_LOGD(TAG, "Flow: %.2f µl/min, Temp: %.2f °C, Flags: 0x%04X",
             measurement->flow, measurement->temperature, measurement->flags);

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
