/********************************************************
 * @file        pid_controller.h
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V1.0.0
 * @date        13/02/2026
 * @brief       PID closed-loop controller for flow rate control
 *
 * @details
 *  - PID algorithm with anti-windup (integral clamping)
 *  - Output mapped to mp6_set_amplitude() range
 *  - Flow deviation detection (>20% for 10s)
 *  - Duration timer for auto-stop
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/********************************************************
 * CONFIGURATION
 ********************************************************/

#define PID_OUTPUT_MIN          80.0f   /* Maps to MP6_AMPLITUDE_MIN */
#define PID_OUTPUT_MAX          250.0f  /* Maps to MP6_AMPLITUDE_MAX */
#define PID_INTEGRAL_LIMIT      500.0f  /* Anti-windup integral clamp */

/* Flow deviation alert thresholds */
#define PID_FLOW_ERR_PERCENT    20.0f   /* Deviation threshold (%) */
#define PID_FLOW_ERR_TIME_S     10      /* Sustained deviation time (s) */

/********************************************************
 * DATA TYPES
 ********************************************************/

typedef struct {
    /* PID gains */
    float kp;
    float ki;
    float kd;

    /* Setpoint */
    float target;

    /* Internal state */
    float integral;
    float prev_error;
    float output;

    /* Duration timer */
    uint32_t duration_s;        /* Total duration (0 = infinite) */
    uint32_t elapsed_s;         /* Elapsed seconds */

    /* Flow deviation detection */
    float flow_err_accum_s;     /* Accumulated deviation time */
    bool flow_err_triggered;    /* Whether FLOW_ERR event was sent */

    bool is_running;
} pid_controller_t;

/********************************************************
 * FUNCTION DECLARATIONS
 ********************************************************/

/**
 * @brief Initialize PID controller with default gains
 * @param pid   Pointer to PID controller
 */
void pid_init(pid_controller_t *pid);

/**
 * @brief Start PID controller with target and duration
 *
 * @param pid       Pointer to PID controller
 * @param target    Target flow rate (ul/min)
 * @param duration  Duration in seconds (0 = infinite)
 */
void pid_start(pid_controller_t *pid, float target, uint32_t duration);

/**
 * @brief Stop PID controller and reset internal state
 * @param pid   Pointer to PID controller
 */
void pid_stop(pid_controller_t *pid);

/**
 * @brief Set PID gains
 *
 * @param pid   Pointer to PID controller
 * @param kp    Proportional gain
 * @param ki    Integral gain
 * @param kd    Derivative gain
 */
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);

/**
 * @brief Update target flow rate while running
 * @param pid       Pointer to PID controller
 * @param target    New target flow rate (ul/min)
 */
void pid_set_target(pid_controller_t *pid, float target);

/**
 * @brief Compute one PID iteration
 *
 * Call at a fixed interval (e.g. 100ms = FLOW_SAMPLE_PERIOD_MS).
 *
 * @param pid       Pointer to PID controller
 * @param measured  Current measured flow rate (ul/min)
 * @param dt        Time step in seconds
 * @return Computed output (amplitude value, PID_OUTPUT_MIN to PID_OUTPUT_MAX)
 */
float pid_compute(pid_controller_t *pid, float measured, float dt);

/**
 * @brief Update elapsed time and check duration expiry
 *
 * Call once per second.
 *
 * @param pid   Pointer to PID controller
 * @return true if duration expired (caller should stop PID)
 */
bool pid_tick_second(pid_controller_t *pid);

/**
 * @brief Check flow deviation and update accumulator
 *
 * Call at each PID compute interval.
 *
 * @param pid       Pointer to PID controller
 * @param measured  Current measured flow rate
 * @param dt        Time step in seconds
 * @return true if sustained deviation detected (caller should send FLOW_ERR event)
 */
bool pid_check_flow_deviation(pid_controller_t *pid, float measured, float dt);

#ifdef __cplusplus
}
#endif

#endif /* PID_CONTROLLER_H */
