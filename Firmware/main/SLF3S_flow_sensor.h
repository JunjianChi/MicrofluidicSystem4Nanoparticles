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
#define SLF3S_I2C_ADDR              0x08
#define SLF3S_GENERAL_CALL_ADDR     0x00    /*!< General call address for soft reset */

/* Product Numbers */
#define SLF3S_PN_1300               0x07030200
#define SLF3S_PN_0600               0x07030300

/* Scale Factors */
#define SLF3S_SCALE_FACTOR_1300     500.0f
#define SLF3S_SCALE_FACTOR_0600     10.0f
#define SLF3S_TEMP_SCALE_FACTOR     200.0f  /*!< Temperature scale factor (°C) */

/* Calibration Command Bytes (sent as part of the start-measurement I2C command) */
#define SLF3S_CALIBRATION_WATER     0x08    /*!< Water calibration (default) */
#define SLF3S_CALIBRATION_IPA       0x15    /*!< Isopropanol (IPA) calibration */

/* Signaling flags bit masks */
#define SLF3S_FLAG_AIR_IN_LINE      (1 << 0)
#define SLF3S_FLAG_HIGH_FLOW        (1 << 1)
#define SLF3S_FLAG_EXP_SMOOTHING    (1 << 5)

/* Measurement result structure */
typedef struct {
    float flow;             /*!< Flow rate in µl/min */
    float temperature;      /*!< Temperature in °C */
    uint16_t flags;         /*!< Signaling flags (air-in-line, high flow, etc.) */
} slf3s_measurement_t;

/* Flow sensor data structure */
typedef struct {
    float scale_factor;
    uint32_t product_number;
    uint64_t serial_number;         /*!< 64-bit unique serial number */
    uint8_t calibration_cmd;
    bool is_initialized;
    float last_temperature;         /*!< Last read temperature in °C */
    uint16_t last_flags;            /*!< Last read signaling flags */
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
 * @brief Start continuous flow measurement (full initialization)
 *
 * Full sequence: stop measurement -> read product number -> verify model ->
 * soft reset -> start measurement.  Automatically detects sensor model and
 * sets the correct scale factor.  Used at boot.
 *
 * @param handle Pointer to flow sensor handle
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t slf3s_start_measurement(slf3s_handle_t *handle);

/**
 * @brief Start continuous flow measurement (simplified, skip product ID)
 *
 * Sequence: soft reset -> start measurement.
 * Skips product identification; reuses the existing scale factor.
 * Used after calibration switching (CAL WATER/IPA) and for hot-plug recovery.
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
 * @brief Read flow value from sensor (flow only, 3 bytes)
 *
 * @param handle Pointer to flow sensor handle
 * @param flow_value Pointer to store the flow value (in µl/min)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t slf3s_read_flow(slf3s_handle_t *handle, float *flow_value);

/**
 * @brief Read full measurement from sensor (flow + temperature + flags, 9 bytes)
 *
 * @param handle Pointer to flow sensor handle
 * @param measurement Pointer to store the measurement result
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t slf3s_read_measurement(slf3s_handle_t *handle, slf3s_measurement_t *measurement);

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
