/********************************************************
 * @file        mp6_driver.h
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V2.1.0
 * @date        12/02/2026
 * @brief       MP6 micropump driver via MP-Driver board
 *
 * @details
 *  Control interface:
 *    1. Amplitude: ESP32 --I2C--> MCP4726 DAC --Vout(0.35~1.3V)--> MP-Driver
 *    2. Frequency: ESP32 --GPIO PWM(95% duty)--> MP-Driver clock
 *    3. Enable:    ESP32 --GPIO--> MP-Driver shutdown (HIGH=run)
 *
 *  Amplitude mapping:
 *    amplitude 80  -> 0.35V -> DAC 287
 *    amplitude 250 -> 1.30V -> DAC 1065
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#ifndef MP6_DRIVER_H
#define MP6_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************
 * MP6 DRIVER PARAMETERS
 ********************************************************/

/* Amplitude: user-facing range 80-250, mapped to voltage 0.35-1.3V */
#define MP6_AMPLITUDE_MIN       80
#define MP6_AMPLITUDE_MAX       250
#define MP6_VOLTAGE_MIN         0.35f   /*!< DAC output at amplitude 80 (V) */
#define MP6_VOLTAGE_MAX         1.3f    /*!< DAC output at amplitude 250 (V) */

/* Clock signal */
#define MP6_CLOCK_DUTY_PERCENT  95      /*!< Clock duty cycle (%) */
#define MP6_FREQ_MIN            25      /*!< Min driver frequency (Hz) */
#define MP6_FREQ_MAX            300     /*!< Max driver frequency (Hz) */

/********************************************************
 * DATA TYPES
 ********************************************************/

typedef struct {
    uint16_t amplitude;         /*!< Current amplitude setting (80-250) */
    uint16_t freq_hz;           /*!< Current driver frequency (25-226 Hz) */
    bool is_running;            /*!< Pump running state */
    bool is_initialized;        /*!< Driver initialized state */
} mp6_handle_t;

/********************************************************
 * FUNCTION DECLARATIONS
 ********************************************************/

/**
 * @brief Initialize MP6 driver (DAC, clock PWM, shutdown GPIO)
 *
 * @param handle Pointer to MP6 driver handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mp6_init(mp6_handle_t *handle);

/**
 * @brief Start the micropump
 *
 * @param handle Pointer to MP6 driver handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mp6_start(mp6_handle_t *handle);

/**
 * @brief Stop the micropump
 *
 * @param handle Pointer to MP6 driver handle
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mp6_stop(mp6_handle_t *handle);

/**
 * @brief Set pump amplitude
 *
 * Maps amplitude 80-250 -> voltage 0.35-1.3V, then writes to DAC.
 *
 * @param handle Pointer to MP6 driver handle
 * @param amplitude Amplitude value (80-250, clamped to range). 0 = off.
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mp6_set_amplitude(mp6_handle_t *handle, uint16_t amplitude);

/**
 * @brief Set driver clock frequency
 *
 * @param handle Pointer to MP6 driver handle
 * @param freq_hz Driver frequency in Hz (25-226)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mp6_set_frequency(mp6_handle_t *handle, uint16_t freq_hz);

#ifdef __cplusplus
}
#endif

#endif /* MP6_DRIVER_H */
