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
    /* Operating mode */
    system_mode_t mode;

    /* Pump state */
    bool pump_on;
    uint16_t amplitude;
    uint16_t frequency;

    /* Sensor data */
    float current_flow;
    float current_temperature;  /*!< Flow sensor temperature (°C) */
    uint16_t sensor_flags;      /*!< Flow sensor signaling flags */

    /* PID state */
    float pid_target;
    uint32_t pid_elapsed_s;
    uint32_t pid_duration_s;

    /* Data stream */
    bool stream_enabled;

    /* Air-in-line detection (edge-triggered) */
    bool air_in_line;

    /* High-flow detection (edge-triggered) */
    bool high_flow;

    /* Hardware availability (updated by hot-plug detection) */
    bool pump_available;
    bool sensor_available;
    bool pressure_available;

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
