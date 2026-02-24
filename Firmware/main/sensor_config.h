/********************************************************
 * @file        sensor_config.h
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V1.0.0
 * @date        06/02/2026
 * @brief       Sensor and system configuration header
 *
 * @details
 *  - Configuration for pressure, temperature, and flow sensors
 *  - Pump driver settings
 *  - Communication interface settings
 *  - Pin assignments from Kconfig
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include "sdkconfig.h"
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************
 * SENSOR ENABLE/DISABLE CONFIGURATION
 ********************************************************/

/* Sensor enable flags from Kconfig */
#ifdef CONFIG_PRESSURE_SENSOR_ENABLE
    #define PRESSURE_SENSOR_ENABLED     1
#else
    #define PRESSURE_SENSOR_ENABLED     0
#endif

#ifdef CONFIG_TEMPERATURE_SENSOR_ENABLE
    #define TEMPERATURE_SENSOR_ENABLED  1
#else
    #define TEMPERATURE_SENSOR_ENABLED  0
#endif

#ifdef CONFIG_FLOW_SENSOR_ENABLE
    #define FLOW_SENSOR_ENABLED         1
#else
    #define FLOW_SENSOR_ENABLED         0
#endif

/*
 * Flow sensor product model selection.
 * Scale factor converts raw I2C 16-bit signed value to ul/min:
 *   flow_ul_min = raw_value / SCALE_FACTOR
 *
 *   SLF3S-0600F: scale=10.0  -> range +/-3250 ul/min (low-flow microfluidic)
 *   SLF3S-1300F: scale=500.0 -> range 0-40 ml/min    (higher-flow applications)
 */
#if defined(CONFIG_FLOW_SENSOR_0600F)
    #define FLOW_SENSOR_PRODUCT_NUMBER  0x07030300
    #define FLOW_SENSOR_SCALE_FACTOR    10.0f
    #define FLOW_SENSOR_MODEL_NAME      "SLF3S-0600F"
#elif defined(CONFIG_FLOW_SENSOR_1300F)
    #define FLOW_SENSOR_PRODUCT_NUMBER  0x07030200
    #define FLOW_SENSOR_SCALE_FACTOR    500.0f
    #define FLOW_SENSOR_MODEL_NAME      "SLF3S-1300F"
#else
    /* Default to 0600F if not specified */
    #define FLOW_SENSOR_PRODUCT_NUMBER  0x07030300
    #define FLOW_SENSOR_SCALE_FACTOR    10.0f
    #define FLOW_SENSOR_MODEL_NAME      "SLF3S-0600F"
#endif

/********************************************************
 * PUMP DRIVER CONFIGURATION
 ********************************************************/

/* Pump driver type selection */
typedef enum {
    PUMP_DRIVER_LOW = 0,
    PUMP_DRIVER_HIGH,
    PUMP_DRIVER_STANDARD
} pump_driver_type_t;

/* Current pump driver type from Kconfig */
#if defined(CONFIG_LOW_MP_DRIVER)
    #define PUMP_DRIVER_TYPE    PUMP_DRIVER_LOW
#elif defined(CONFIG_HIGH_MP_DRIVER)
    #define PUMP_DRIVER_TYPE    PUMP_DRIVER_HIGH
#elif defined(CONFIG_MP_DRIVER)
    #define PUMP_DRIVER_TYPE    PUMP_DRIVER_STANDARD
#else
    #define PUMP_DRIVER_TYPE    PUMP_DRIVER_STANDARD  // Default
#endif

/********************************************************
 * COMMUNICATION CONFIGURATION
 ********************************************************/

/* Communication type selection */
typedef enum {
    COMM_TYPE_SERIAL = 0,
    COMM_TYPE_WIFI
} comm_type_t;

#ifdef CONFIG_COMM_WIFI
    #define COMMUNICATION_TYPE  COMM_TYPE_WIFI
#else
    #define COMMUNICATION_TYPE  COMM_TYPE_SERIAL
#endif

/********************************************************
 * PIN CONFIGURATION
 *
 * Kconfig -> compile macro mapping:
 *   CONFIG_I2C_SDA_PIN (default 21)         -> I2C_MASTER_SDA_IO    (used by i2c_interface.c)
 *   CONFIG_I2C_SCL_PIN (default 22)         -> I2C_MASTER_SCL_IO    (used by i2c_interface.c)
 *   CONFIG_MP_DRIVER_ENABLE_PIN (default 14) -> MP_DRIVER_ENABLE_IO  (used by mp6_driver.c)
 *   CONFIG_MP_DRIVER_CLOCK_PIN (default 27)  -> MP_DRIVER_CLOCK_IO   (used by mp6_driver.c)
 ********************************************************/

/* I2C Pin Configuration */
#define I2C_MASTER_SCL_IO       CONFIG_I2C_SCL_PIN      /*!< GPIO number for I2C master clock */
#define I2C_MASTER_SDA_IO       CONFIG_I2C_SDA_PIN      /*!< GPIO number for I2C master data  */
#define I2C_MASTER_NUM          I2C_NUM_0               /*!< I2C port number */
#define I2C_MASTER_FREQ_HZ      100000                  /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                     /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                     /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS   1000

/* MP6 Pump Driver Pin Configuration */
#define MP_DRIVER_ENABLE_IO     CONFIG_MP_DRIVER_ENABLE_PIN  /*!< GPIO for MP driver enable */
#define MP_DRIVER_CLOCK_IO      CONFIG_MP_DRIVER_CLOCK_PIN   /*!< GPIO for MP driver clock */

/********************************************************
 * SENSOR I2C ADDRESSES
 *
 * I2C bus device map (all on I2C_NUM_0 at 100 kHz):
 *   0x08  SLF3S flow sensor     (flow, temperature, signaling flags)
 *   0x48  Temperature sensor    (TMP117 or similar, reserved)
 *   0x61  MCP4726 DAC           (amplitude output for MP-Driver)
 *   0x76  Pressure sensor       (MS5837 or similar, reserved)
 ********************************************************/

/* Pressure Sensor (example: MS5837 or similar) */
#define PRESSURE_SENSOR_I2C_ADDR    0x76

/* Temperature Sensor (example: TMP117 or similar) */
#define TEMPERATURE_SENSOR_I2C_ADDR 0x48

/* Flow Sensor (SLF3S) */
#define FLOW_SENSOR_I2C_ADDR        0x08

/* DAC for MP-Driver (MCP4726) */
#define MP_DRIVER_DAC_I2C_ADDR      0x61

/********************************************************
 * SENSOR SAMPLING CONFIGURATION
 ********************************************************/

/* Sensor sampling rates (in milliseconds) */
#define PRESSURE_SAMPLE_PERIOD_MS       100     /*!< Pressure sensor sampling period */
#define TEMPERATURE_SAMPLE_PERIOD_MS    1000    /*!< Temperature sensor sampling period */
#define FLOW_SAMPLE_PERIOD_MS           100     /*!< Flow sensor sampling period */

/* Data buffer sizes */
#define SENSOR_DATA_BUFFER_SIZE         128     /*!< Size of sensor data buffer */

/********************************************************
 * WIFI CONFIGURATION (if enabled)
 ********************************************************/

#ifdef CONFIG_COMM_WIFI
    #define WIFI_SSID           CONFIG_WIFI_SSID
    #define WIFI_PASSWORD       CONFIG_WIFI_PASS
    #define SERVER_ADDRESS      CONFIG_SERVICE_ADDRESS
    #define SERVER_PORT         CONFIG_SERVICE_PORT
#endif

/********************************************************
 * SYSTEM CONFIGURATION STRUCTURE
 ********************************************************/

typedef struct {
    /* Sensor enable flags */
    bool pressure_enabled;
    bool temperature_enabled;
    bool flow_enabled;

    /* Pump driver configuration */
    pump_driver_type_t pump_driver;

    /* Communication type */
    comm_type_t comm_type;

    /* Pin configuration */
    struct {
        uint8_t i2c_sda;
        uint8_t i2c_scl;
        uint8_t mp_enable;
        uint8_t mp_clock;
    } pins;

    /* WiFi configuration (if applicable) */
#ifdef CONFIG_COMM_WIFI
    struct {
        const char *ssid;
        const char *password;
        const char *server_addr;
        uint16_t server_port;
    } wifi;
#endif
} system_config_t;

/********************************************************
 * HELPER MACROS
 ********************************************************/

/* Check if sensor is enabled */
#define IS_PRESSURE_SENSOR_ENABLED()    (PRESSURE_SENSOR_ENABLED)
#define IS_TEMPERATURE_SENSOR_ENABLED() (TEMPERATURE_SENSOR_ENABLED)
#define IS_FLOW_SENSOR_ENABLED()        (FLOW_SENSOR_ENABLED)

/* Check communication type */
#define IS_WIFI_ENABLED()               (COMMUNICATION_TYPE == COMM_TYPE_WIFI)
#define IS_SERIAL_ENABLED()             (COMMUNICATION_TYPE == COMM_TYPE_SERIAL)

/********************************************************
 * FUNCTION DECLARATIONS
 ********************************************************/

/**
 * @brief Initialize system configuration with Kconfig values
 *
 * @param config Pointer to system configuration structure
 */
void system_config_init(system_config_t *config);

/**
 * @brief Print current system configuration
 *
 * @param config Pointer to system configuration structure
 */
void system_config_print(const system_config_t *config);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_CONFIG_H */
