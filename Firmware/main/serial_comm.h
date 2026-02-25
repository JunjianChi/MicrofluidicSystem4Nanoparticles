/********************************************************
 * @file        serial_comm.h
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V1.0.0
 * @date        13/02/2026
 * @brief       Serial communication module for PC-ESP32 protocol
 *
 * @details
 *  - UART0 command parsing (ASCII text, newline-terminated)
 *  - Data stream reporting (10Hz sensor data)
 *  - Event notifications (PID_DONE, FLOW_ERR)
 *  - See API.md for full protocol specification
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#ifndef SERIAL_COMM_H
#define SERIAL_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************
 * CONFIGURATION
 ********************************************************/

#define SERIAL_UART_NUM         UART_NUM_0
#define SERIAL_BAUD_RATE        115200
#define SERIAL_RX_BUF_SIZE      256
#define SERIAL_TX_BUF_SIZE      0       /* No TX buffer (blocking write) */
#define SERIAL_CMD_MAX_LEN      128

/********************************************************
 * DATA TYPES
 ********************************************************/

/* Operating mode */
typedef enum {
    MODE_MANUAL = 0,
    MODE_PID
} system_mode_t;

/* Forward declaration - full definition in main.h */
typedef struct system_state system_state_t;

/* Command handler callback type */
typedef void (*cmd_handler_t)(system_state_t *state, const char *args);

/********************************************************
 * FUNCTION DECLARATIONS
 ********************************************************/

/**
 * @brief Initialize UART for serial communication
 * @return ESP_OK on success
 */
esp_err_t serial_comm_init(void);

/**
 * @brief Read one line from UART (blocking until \n received)
 *
 * @param buf       Buffer to store the line (null-terminated, \n stripped)
 * @param max_len   Maximum buffer length
 * @return Number of characters read, or -1 on error
 */
int serial_comm_read_line(char *buf, int max_len);

/**
 * @brief Send a formatted string over UART
 * @param fmt   printf-style format string
 */
void serial_comm_send(const char *fmt, ...);

/**
 * @brief Send OK response
 */
void serial_comm_send_ok(void);

/**
 * @brief Send error response
 * @param reason    Error reason string
 */
void serial_comm_send_err(const char *reason);

/**
 * @brief Send data stream line: D <flow> <temperature>
 * @param flow  Flow rate in ul/min
 * @param temperature  Sensor temperature in °C
 */
void serial_comm_send_data(float flow, float temperature);

/**
 * @brief Send status response: S <mode> <pump> <amp> <freq> <flow> <target> <elapsed> <duration> <pump_hw> <sensor_hw>
 */
void serial_comm_send_status(const system_state_t *state);

/**
 * @brief Send I2C scan result: SCAN <addr1> <addr2> ...
 * @param devices   Array of found device addresses
 * @param count     Number of devices found
 */
void serial_comm_send_scan(const uint8_t *devices, uint8_t count);

/**
 * @brief Send EVENT PID_DONE notification
 */
void serial_comm_send_event_pid_done(void);

/**
 * @brief Send EVENT FLOW_ERR notification
 * @param target    Target flow rate
 * @param actual    Actual flow rate
 */
void serial_comm_send_event_flow_err(float target, float actual);

/**
 * @brief Send EVENT AIR_IN_LINE notification
 */
void serial_comm_send_event_air_in_line(void);

/**
 * @brief Send EVENT HIGH_FLOW notification
 */
void serial_comm_send_event_high_flow(void);

/**
 * @brief Send EVENT AIR_CLEAR notification (air-in-line condition resolved)
 */
void serial_comm_send_event_air_clear(void);

/**
 * @brief Send EVENT HIGH_FLOW_CLEAR notification (high-flow condition resolved)
 */
void serial_comm_send_event_high_flow_clear(void);

/**
 * @brief Parse and execute a command line
 *
 * @param state     Pointer to system state
 * @param cmd_line  Null-terminated command string
 */
void serial_comm_process_cmd(system_state_t *state, const char *cmd_line);

#ifdef __cplusplus
}
#endif

#endif /* SERIAL_COMM_H */
