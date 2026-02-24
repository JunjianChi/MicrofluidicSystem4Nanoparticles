/********************************************************
 * @file        mcp4726_dac.h
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V1.1.0
 * @date        12/02/2026
 * @brief       MCP4726 DAC driver
 *
 * @details
 *  12-bit DAC, Vref = VDD.
 *  bit / 4096 = Vout / VDD  =>  bit = Vout / VDD * 4096
 *
 *  VDD calibration: The nominal VDD is 5.0V, but measured supply is 4.734V.
 *  Using the measured value (MCP4726_VDD = 4.734) ensures accurate voltage
 *  output.  If your power supply differs, update MCP4726_VDD accordingly.
 *
 *  I2C implementation note: This driver uses the low-level i2c_cmd_link API
 *  (driver/i2c.h) rather than i2c_interface_write(), because the MCP4726
 *  requires ACK checking to be disabled on data bytes to match the Arduino
 *  Wire library behavior used in the reference implementation.
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#ifndef MCP4726_DAC_H
#define MCP4726_DAC_H

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************
 * MCP4726 CONFIGURATION
 ********************************************************/

#define MCP4726_I2C_ADDR        0x61
#define MCP4726_REG_VOLTAGE     0x00    /*!< DAC voltage output register */
#define MCP4726_REG_VREF        0x08    /*!< DAC voltage reference register */
#define MCP4726_VDD             4.734f    /*!< DAC supply / reference voltage (V) */
#define MCP4726_RESOLUTION      4096    /*!< 12-bit DAC */

/********************************************************
 * FUNCTION DECLARATIONS
 ********************************************************/

/**
 * @brief Initialize MCP4726 DAC
 *
 * Configures VREF to VDD and sets output to 0V.
 *
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mcp4726_init(void);

/**
 * @brief Write raw bit value to DAC
 *
 * @param value 12-bit DAC value (0-4095)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mcp4726_set_raw(uint16_t value);

/**
 * @brief Set DAC output voltage
 *
 * Converts voltage to bit value: bit = voltage / VDD * 4096
 *
 * @param voltage Output voltage in volts (0 ~ VDD)
 * @return esp_err_t ESP_OK on success
 */
esp_err_t mcp4726_set_voltage(float voltage);

#ifdef __cplusplus
}
#endif

#endif /* MCP4726_DAC_H */
