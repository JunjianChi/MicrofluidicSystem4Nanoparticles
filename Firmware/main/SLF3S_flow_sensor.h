/********************************************************
 * @file        SLF3S_flow_sensor.h
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V1.0.0
 * @date        06/02/2026
 * @brief       Driver for the Sensirion SLF3S liquid flow sensor
 *
 * @details
 *  - Declaration of initialization and control functions for SLF3S
 *  - Liquid flow sensor with I2C interface
 *  - Supports SLF3S-1300F and SLF3S-0600F models
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#ifndef SLF3S_FLOW_SENSOR_H
#define SLF3S_FLOW_SENSOR_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* I2C Address */
#define SLF3S_I2C_ADDR          0x08

/* Product Numbers */
#define SLF3S_PN_1300           0x07030200
#define SLF3S_PN_0600           0x07030300

/* Scale Factors */
#define SLF3S_SCALE_FACTOR_1300 500.0f
#define SLF3S_SCALE_FACTOR_0600 10000.0f

/* Calibration Command Bytes */
#define SLF3S_CALIBRATION_WATER 0x08
#define SLF3S_CALIBRATION_IPA   0x15

/* Flow sensor data structure */
typedef struct {
    float scale_factor;
    uint32_t product_number;
    uint8_t calibration_cmd;
    bool is_initialized;
} slf3s_handle_t;

/**
 * @brief Initialize the SLF3S flow sensor
 *
 * @note I2C interface must be initialized before calling this function
 *
 * @param handle Pointer to flow sensor handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t slf3s_init(slf3s_handle_t *handle);

/**
 * @brief Start continuous flow measurement
 *
 * @param handle Pointer to flow sensor handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t slf3s_start_measurement(slf3s_handle_t *handle);

/**
 * @brief Start continuous flow measurement (simplified version)
 *
 * @param handle Pointer to flow sensor handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t slf3s_start_measurement_simple(slf3s_handle_t *handle);

/**
 * @brief Stop continuous flow measurement
 *
 * @param handle Pointer to flow sensor handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t slf3s_stop_measurement(slf3s_handle_t *handle);

/**
 * @brief Read flow value from sensor
 *
 * @param handle Pointer to flow sensor handle
 * @param flow_value Pointer to store the flow value (in µl/min)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t slf3s_read_flow(slf3s_handle_t *handle, float *flow_value);

/**
 * @brief Set calibration mode
 *
 * @param handle Pointer to flow sensor handle
 * @param calibration_cmd Calibration command byte (WATER or IPA)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t slf3s_set_calibration(slf3s_handle_t *handle, uint8_t calibration_cmd);

#ifdef __cplusplus
}
#endif

#endif /* SLF3S_FLOW_SENSOR_H */
