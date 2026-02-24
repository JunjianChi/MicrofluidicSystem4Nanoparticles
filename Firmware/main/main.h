/********************************************************
 * @file        main.h
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V3.0.0
 * @date        13/02/2026
 * @brief       Main application header - shared state and command handlers
 *
 * @details
 *  - System state structure definition
 *  - Command handler declarations (called by serial_comm)
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#ifndef MAIN_H
#define MAIN_H

#include <stdint.h>
#include <stdbool.h>
#include "serial_comm.h"
#include "pid_controller.h"
#include "mp6_driver.h"
#include "SLF3S_flow_sensor.h"

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************
 * SYSTEM STATE
 ********************************************************/

struct system_state {
    /* Operating mode: MODE_MANUAL (open-loop) or MODE_PID (closed-loop) */
    system_mode_t mode;

    /* Pump state */
    bool pump_on;                   /*!< true when pump is actively running */
    uint16_t amplitude;             /*!< Current amplitude setting (80-250, 0 when off) */
    uint16_t frequency;             /*!< Current frequency in Hz (25-300, default 100) */

    /* Sensor data (updated every 100ms by sensor_read_task) */
    float current_flow;             /*!< Latest flow reading (ul/min) */
    float current_temperature;      /*!< Flow sensor temperature (°C) */
    uint16_t sensor_flags;          /*!< SLF3S signaling flags (air-in-line, high flow) */

    /* PID state (valid only when mode == MODE_PID) */
    float pid_target;               /*!< Target flow rate (ul/min) */
    uint32_t pid_elapsed_s;         /*!< Seconds since PID started */
    uint32_t pid_duration_s;        /*!< Total PID duration (0 = infinite) */

    /* Data stream: when true, sensor_read_task sends "D <flow> <temp>" at 10Hz */
    bool stream_enabled;

    /* Edge-triggered flags: EVENT sent only on 0->1 transition */
    bool air_in_line;               /*!< Air bubble detected in flow path */
    bool high_flow;                 /*!< Flow rate exceeds sensor range */

    /* Hardware availability (set at boot, updated by 5s hot-plug re-probe) */
    bool pump_available;            /*!< MCP4726 DAC at 0x61 detected */
    bool sensor_available;          /*!< SLF3S flow sensor at 0x08 detected */
    bool pressure_available;        /*!< Pressure sensor at 0x76 detected */

    /* Hardware handles */
    mp6_handle_t pump;
    slf3s_handle_t flow_sensor;
    pid_controller_t pid;
};

/********************************************************
 * COMMAND HANDLERS (called by serial_comm)
 ********************************************************/

void app_cmd_pump_on(system_state_t *state);
void app_cmd_pump_off(system_state_t *state);
void app_cmd_set_amplitude(system_state_t *state, uint16_t amplitude);
void app_cmd_set_frequency(system_state_t *state, uint16_t freq_hz);
void app_cmd_pid_start(system_state_t *state, float target, uint32_t duration);
void app_cmd_pid_stop(system_state_t *state);
void app_cmd_pid_target(system_state_t *state, float target);
void app_cmd_pid_tune(system_state_t *state, float kp, float ki, float kd);
void app_cmd_status(system_state_t *state);
void app_cmd_scan(system_state_t *state);
void app_cmd_set_calibration(system_state_t *state, const char *liquid);

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H */
