/********************************************************
 * @file        i2c_interface.h
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V1.0.0
 * @date        06/02/2026
 * @brief       I2C interface abstraction layer
 *
 * @details
 *  - Hardware abstraction for I2C communication
 *  - Board-specific I2C configuration
 *  - Helper functions for sensor drivers
 *  - Easy to modify for different boards
 *
 *  Design: Two API styles are provided:
 *    - write()/read():          Raw byte I/O without register addressing.
 *                               Used by SLF3S which uses command mode (send command
 *                               bytes, then read result bytes, no register concept).
 *    - write_reg()/read_reg():  Register-addressed I/O (write reg addr, then data).
 *                               Reserved for future sensors (pressure, temperature)
 *                               that follow the standard register-based I2C model.
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#ifndef I2C_INTERFACE_H
#define I2C_INTERFACE_H

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include "driver/i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************
 * I2C CONFIGURATION (Board-specific)
 ********************************************************/

/* I2C Port Configuration */
#define I2C_MASTER_PORT         I2C_NUM_0

/* I2C Parameters - frequency and buffer config defined in sensor_config.h */
#define I2C_TIMEOUT_MS          1000        /*!< I2C timeout in milliseconds */

/********************************************************
 * FUNCTION DECLARATIONS
 ********************************************************/

/**
 * @brief Initialize I2C master interface
 *
 * @note This function reads pin configuration from Kconfig
 *       and initializes the I2C master with appropriate settings.
 *       Call this function once during system startup.
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid I2C parameters
 *     - ESP_FAIL: I2C driver installation failed
 */
esp_err_t i2c_interface_init(void);

/**
 * @brief Deinitialize I2C master interface
 *
 * @return
 *     - ESP_OK: Success
 *     - ESP_FAIL: I2C driver deletion failed
 */
esp_err_t i2c_interface_deinit(void);

/**
 * @brief Write data to I2C device
 *
 * @param device_addr I2C device address (7-bit)
 * @param data Pointer to data buffer to write
 * @param len Length of data to write
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid arguments
 *     - ESP_FAIL: I2C communication failed
 */
esp_err_t i2c_interface_write(uint8_t device_addr, const uint8_t *data, size_t len);

/**
 * @brief Read data from I2C device
 *
 * @param device_addr I2C device address (7-bit)
 * @param data Pointer to buffer to store read data
 * @param len Length of data to read
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid arguments
 *     - ESP_FAIL: I2C communication failed
 */
esp_err_t i2c_interface_read(uint8_t device_addr, uint8_t *data, size_t len);

/**
 * @brief Write to register and then read from I2C device
 *
 * @param device_addr I2C device address (7-bit)
 * @param reg_addr Register address to write
 * @param data Pointer to buffer to store read data
 * @param len Length of data to read
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid arguments
 *     - ESP_FAIL: I2C communication failed
 */
esp_err_t i2c_interface_write_read(uint8_t device_addr, uint8_t reg_addr,
                                    uint8_t *data, size_t len);

/**
 * @brief Write data to specific register of I2C device
 *
 * @param device_addr I2C device address (7-bit)
 * @param reg_addr Register address
 * @param data Pointer to data buffer to write
 * @param len Length of data to write
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid arguments
 *     - ESP_FAIL: I2C communication failed
 */
esp_err_t i2c_interface_write_reg(uint8_t device_addr, uint8_t reg_addr,
                                   const uint8_t *data, size_t len);

/**
 * @brief Read data from specific register of I2C device
 *
 * @param device_addr I2C device address (7-bit)
 * @param reg_addr Register address
 * @param data Pointer to buffer to store read data
 * @param len Length of data to read
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid arguments
 *     - ESP_FAIL: I2C communication failed
 */
esp_err_t i2c_interface_read_reg(uint8_t device_addr, uint8_t reg_addr,
                                  uint8_t *data, size_t len);

/**
 * @brief Check if I2C device is present on the bus
 *
 * @param device_addr I2C device address (7-bit)
 * @return
 *     - ESP_OK: Device is present
 *     - ESP_ERR_NOT_FOUND: Device not found
 */
esp_err_t i2c_interface_probe(uint8_t device_addr);

/**
 * @brief Scan I2C bus for all connected devices
 *
 * @param devices Array to store found device addresses
 * @param max_devices Maximum number of devices to find
 * @param num_found Pointer to store number of devices found
 * @return
 *     - ESP_OK: Success
 *     - ESP_ERR_INVALID_ARG: Invalid arguments
 */
esp_err_t i2c_interface_scan(uint8_t *devices, uint8_t max_devices, uint8_t *num_found);

/**
 * @brief Get I2C master port number
 *
 * @return I2C port number
 */
i2c_port_t i2c_interface_get_port(void);

#ifdef __cplusplus
}
#endif

#endif /* I2C_INTERFACE_H */
