/********************************************************
 * @file        mcp4726_dac.c
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V1.3.0
 * @date        12/02/2026
 * @brief       MCP4726 DAC driver implementation
 *
 * @details
 *  Ported 1:1 from Arduino reference code (MP-Driver datasheet).
 *  Each Wire.write() maps to one i2c_master_write_byte().
 *
 *  Voltage conversion:  bit = voltage / VDD * 4096
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#include "mcp4726_dac.h"
#include "i2c_interface.h"
#include "esp_log.h"
#include "driver/i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "MCP4726";

esp_err_t mcp4726_init(void)
{
    i2c_port_t port = i2c_interface_get_port();
    i2c_cmd_handle_t cmd;
    esp_err_t ret;

    /*
     * Arduino reference:
     *   Wire.beginTransmission(0x61);
     *   Wire.write(0x08);   // I2C_DACVREF
     *   Wire.write(0x02);
     *   Wire.endTransmission();
     */
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                          // beginTransmission
    i2c_master_write_byte(cmd, (MCP4726_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);  // address
    i2c_master_write_byte(cmd, MCP4726_REG_VREF, false);                            // Wire.write(0x08)
    i2c_master_write_byte(cmd, 0x02, false);                                        // Wire.write(0x02)
    i2c_master_stop(cmd);                                                           // endTransmission
    ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "VREF config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    // ESP_LOGI(TAG, "VREF config OK");

    vTaskDelay(pdMS_TO_TICKS(10));

    /*
     * Arduino reference:
     *   Wire.beginTransmission(0x61);
     *   Wire.write(0x00);   // I2C_DACVOLTAGE
     *   Wire.write(0);
     *   Wire.write(0);
     *   Wire.endTransmission();
     */
    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);                                                          // beginTransmission
    i2c_master_write_byte(cmd, (MCP4726_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);  // address
    i2c_master_write_byte(cmd, MCP4726_REG_VOLTAGE, false);                         // Wire.write(0x00)
    i2c_master_write_byte(cmd, 0x00, false);                                        // Wire.write(0)
    i2c_master_write_byte(cmd, 0x00, false);                                        // Wire.write(0)
    i2c_master_stop(cmd);                                                           // endTransmission
    ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DAC zero failed: %s", esp_err_to_name(ret));
        return ret;
    }
    // ESP_LOGI(TAG, "DAC zero OK");

    // ESP_LOGI(TAG, "MCP4726 initialized (addr=0x%02X)", MCP4726_I2C_ADDR);
    return ESP_OK;
}

esp_err_t mcp4726_set_raw(uint16_t value)
{
    if (value >= MCP4726_RESOLUTION) {
        value = MCP4726_RESOLUTION - 1;
    }

    i2c_port_t port = i2c_interface_get_port();

    /*
     * Arduino reference:
     *   Wire.beginTransmission(0x61);
     *   Wire.write(0x00);              // I2C_DACVOLTAGE
     *   Wire.write(_dacValue >> 8);
     *   Wire.write(_dacValue & 0xFF);
     *   Wire.endTransmission();
     */
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (MCP4726_I2C_ADDR << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, MCP4726_REG_VOLTAGE, false);             // 0x00
    i2c_master_write_byte(cmd, (uint8_t)(value >> 8), false);           // MSB
    i2c_master_write_byte(cmd, (uint8_t)(value & 0xFF), false);         // LSB
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(port, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "DAC write failed: %s", esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t mcp4726_set_voltage(float voltage)
{
    if (voltage < 0.0f) voltage = 0.0f;
    if (voltage > MCP4726_VDD) voltage = MCP4726_VDD;

    /* bit / 4096 = Vout / VDD  =>  bit = Vout / VDD * 4096 */
    uint16_t bit_value = (uint16_t)(voltage / MCP4726_VDD * MCP4726_RESOLUTION);

    // ESP_LOGI(TAG, "Voltage: %.3f V -> bit: %d", voltage, bit_value);

    return mcp4726_set_raw(bit_value);
}
