/********************************************************
 * @file        abp_pressure_sensor.h
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V3.1.0
 * @date        04/03/2026
 * @brief       Driver for Honeywell ABP pressure sensor (ABPMANV015PG7A5)
 *
 * @details
 *  - I2C command-mode interface (no register addressing)
 *  - 14-bit pressure data with 2-bit status
 *  - Output in bar (converted from psi)
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#ifndef ABP_PRESSURE_SENSOR_H
#define ABP_PRESSURE_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* I2C Address */
#define ABP_I2C_ADDR                0x78

/* Status codes (top 2 bits of first byte) */
#define ABP_STATUS_NORMAL           0   /*!< Normal operation */
#define ABP_STATUS_COMMAND_MODE     1   /*!< Device in command mode */
#define ABP_STATUS_STALE_DATA       2   /*!< Stale data (already fetched) */
#define ABP_STATUS_DIAGNOSTIC       3   /*!< Diagnostic condition */

/* Pressure sensor handle */
typedef struct {
    bool is_initialized;
    float last_pressure;            /*!< Last pressure reading in bar */
} abp_handle_t;

/**
 * @brief Initialize the ABP pressure sensor handle
 *
 * @param handle Pointer to pressure sensor handle
 * @return ESP_OK on success
 */
esp_err_t abp_init(abp_handle_t *handle);

/**
 * @brief Check if the sensor is ready (status == 0)
 *
 * Sends a 1-byte read request and checks the status bits.
 *
 * @param handle Pointer to pressure sensor handle
 * @return
 *     - ESP_OK: Sensor is ready
 *     - ESP_ERR_INVALID_STATE: Sensor not ready (command mode, stale, or diagnostic)
 *     - ESP_FAIL: I2C communication failed
 */
esp_err_t abp_check_ready(abp_handle_t *handle);

/**
 * @brief Read pressure from the sensor
 *
 * Reads 2 bytes, extracts 14-bit pressure data, and converts to bar.
 * Conversion: psi = raw * 1.1443e-3 - 1.8744, then bar = psi * 0.0689476
 *
 * @param handle Pointer to pressure sensor handle
 * @param pressure_bar Pointer to store pressure value in bar
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_STATE: Sensor status indicates error
 *     - ESP_FAIL: I2C communication failed
 */
esp_err_t abp_read_pressure(abp_handle_t *handle, float *pressure_bar);

#ifdef __cplusplus
}
#endif

#endif /* ABP_PRESSURE_SENSOR_H */
