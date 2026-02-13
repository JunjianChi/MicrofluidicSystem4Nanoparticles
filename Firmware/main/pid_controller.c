/********************************************************
 * @file        pid_controller.c
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V1.0.0
 * @date        13/02/2026
 * @brief       PID closed-loop controller for flow rate control
 *
 * @details
 *  - Standard PID with anti-windup (integral clamping)
 *  - Output clamped to amplitude range [80, 250]
 *  - Flow deviation detection for safety alerts
 *  - Duration countdown for auto-stop
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#include <string.h>
#include <math.h>
#include "pid_controller.h"

/********************************************************
 * DEFAULT PID GAINS
 ********************************************************/

#define PID_DEFAULT_KP  2.0f
#define PID_DEFAULT_KI  0.5f
#define PID_DEFAULT_KD  0.1f

/********************************************************
 * IMPLEMENTATION
 ********************************************************/

void pid_init(pid_controller_t *pid)
{
    memset(pid, 0, sizeof(pid_controller_t));
    pid->kp = PID_DEFAULT_KP;
    pid->ki = PID_DEFAULT_KI;
    pid->kd = PID_DEFAULT_KD;
    pid->is_running = false;
}

void pid_start(pid_controller_t *pid, float target, uint32_t duration)
{
    pid->target = target;
    pid->duration_s = duration;
    pid->elapsed_s = 0;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = PID_OUTPUT_MIN;   /* Start at minimum amplitude */
    pid->flow_err_accum_s = 0.0f;
    pid->flow_err_triggered = false;
    pid->is_running = true;
}

void pid_stop(pid_controller_t *pid)
{
    pid->is_running = false;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
    pid->elapsed_s = 0;
    pid->flow_err_accum_s = 0.0f;
    pid->flow_err_triggered = false;
}

void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_target(pid_controller_t *pid, float target)
{
    pid->target = target;
    /* Reset flow deviation detector on target change */
    pid->flow_err_accum_s = 0.0f;
    pid->flow_err_triggered = false;
}

float pid_compute(pid_controller_t *pid, float measured, float dt)
{
    if (!pid->is_running || dt <= 0.0f) {
        return pid->output;
    }

    float error = pid->target - measured;

    /* Proportional */
    float p_term = pid->kp * error;

    /* Integral with anti-windup */
    pid->integral += error * dt;
    if (pid->integral > PID_INTEGRAL_LIMIT) {
        pid->integral = PID_INTEGRAL_LIMIT;
    } else if (pid->integral < -PID_INTEGRAL_LIMIT) {
        pid->integral = -PID_INTEGRAL_LIMIT;
    }
    float i_term = pid->ki * pid->integral;

    /* Derivative */
    float d_term = pid->kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    /* Sum and clamp output to amplitude range */
    float output = p_term + i_term + d_term;

    if (output < PID_OUTPUT_MIN) {
        output = PID_OUTPUT_MIN;
    } else if (output > PID_OUTPUT_MAX) {
        output = PID_OUTPUT_MAX;
    }

    pid->output = output;
    return output;
}

bool pid_tick_second(pid_controller_t *pid)
{
    if (!pid->is_running) {
        return false;
    }

    pid->elapsed_s++;

    /* Check duration expiry (0 = infinite) */
    if (pid->duration_s > 0 && pid->elapsed_s >= pid->duration_s) {
        return true;    /* Duration expired */
    }

    return false;
}

bool pid_check_flow_deviation(pid_controller_t *pid, float measured, float dt)
{
    if (!pid->is_running || pid->target <= 0.0f) {
        return false;
    }

    /* Already triggered - don't re-trigger */
    if (pid->flow_err_triggered) {
        return false;
    }

    float deviation_percent = fabsf(measured - pid->target) / pid->target * 100.0f;

    if (deviation_percent > PID_FLOW_ERR_PERCENT) {
        pid->flow_err_accum_s += dt;
    } else {
        /* Reset accumulator when deviation is within threshold */
        pid->flow_err_accum_s = 0.0f;
    }

    if (pid->flow_err_accum_s >= (float)PID_FLOW_ERR_TIME_S) {
        pid->flow_err_triggered = true;
        return true;    /* Sustained deviation detected */
    }

    return false;
}
