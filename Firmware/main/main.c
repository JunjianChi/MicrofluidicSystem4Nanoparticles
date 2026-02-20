/********************************************************
 * @file        main.c
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V3.0.0
 * @date        13/02/2026
 * @brief       Main application - FreeRTOS multi-task microfluidic control
 *
 * @details
 *  - Hardware initialization (I2C, pump, flow sensor, UART)
 *  - FreeRTOS tasks:
 *    - serial_cmd_task:    Parse commands from PC
 *    - sensor_read_task:   10Hz sensor reading + data stream
 *    - pid_control_task:   PID computation (when in PID mode)
 *  - Global system state management
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

#include "main.h"
#include "i2c_interface.h"
#include "mcp4726_dac.h"
#include "sensor_config.h"
#include "mp6_driver.h"
#include "SLF3S_flow_sensor.h"
#include "serial_comm.h"
#include "pid_controller.h"

static const char *TAG = "MAIN";

/********************************************************
 * GLOBAL STATE
 ********************************************************/

static system_state_t g_state;
static SemaphoreHandle_t g_state_mutex;

/* Mutex helpers */
#define STATE_LOCK()    xSemaphoreTake(g_state_mutex, portMAX_DELAY)
#define STATE_UNLOCK()  xSemaphoreGive(g_state_mutex)

/********************************************************
 * TASK CONFIGURATION
 ********************************************************/

#define SERIAL_CMD_TASK_STACK   4096
#define SERIAL_CMD_TASK_PRIO    5

#define SENSOR_READ_TASK_STACK  4096
#define SENSOR_READ_TASK_PRIO   6   /* Higher priority for consistent timing */

#define PID_TICK_TASK_STACK     2048
#define PID_TICK_TASK_PRIO      4

/* Sensor read interval = FLOW_SAMPLE_PERIOD_MS (100ms → 10Hz) */
#define SENSOR_INTERVAL_MS      FLOW_SAMPLE_PERIOD_MS

/********************************************************
 * COMMAND HANDLERS
 *
 * Called by serial_comm_process_cmd() within serial_cmd_task.
 * g_state_mutex is NOT held when these are called - they
 * must lock as needed.
 ********************************************************/

void app_cmd_pump_on(system_state_t *state)
{
    STATE_LOCK();
    mp6_start(&state->pump);
    state->pump_on = true;
    STATE_UNLOCK();
    // ESP_LOGI(TAG, "Pump ON");
}

void app_cmd_pump_off(system_state_t *state)
{
    STATE_LOCK();
    mp6_stop(&state->pump);
    state->pump_on = false;
    state->amplitude = 0;
    STATE_UNLOCK();
    // ESP_LOGI(TAG, "Pump OFF");
}

void app_cmd_set_amplitude(system_state_t *state, uint16_t amplitude)
{
    STATE_LOCK();
    mp6_set_amplitude(&state->pump, amplitude);
    state->amplitude = amplitude;
    STATE_UNLOCK();
    // ESP_LOGI(TAG, "Amplitude set to %d", amplitude);
}

void app_cmd_set_frequency(system_state_t *state, uint16_t freq_hz)
{
    STATE_LOCK();
    mp6_set_frequency(&state->pump, freq_hz);
    state->frequency = freq_hz;
    STATE_UNLOCK();
    // ESP_LOGI(TAG, "Frequency set to %d Hz", freq_hz);
}

void app_cmd_pid_start(system_state_t *state, float target, uint32_t duration)
{
    STATE_LOCK();

    /* Start pump if not already running */
    if (!state->pump_on) {
        mp6_start(&state->pump);
        state->pump_on = true;
    }

    /* Set initial frequency if not set */
    if (state->frequency == 0) {
        state->frequency = 100;
        mp6_set_frequency(&state->pump, 100);
    }

    /* Start PID */
    pid_start(&state->pid, target, duration);
    state->mode = MODE_PID;
    state->pid_target = target;
    state->pid_duration_s = duration;
    state->pid_elapsed_s = 0;

    STATE_UNLOCK();
    // ESP_LOGI(TAG, "PID START: target=%.2f ul/min, duration=%lus", target, (unsigned long)duration);
}

void app_cmd_pid_stop(system_state_t *state)
{
    STATE_LOCK();

    pid_stop(&state->pid);
    mp6_stop(&state->pump);
    state->mode = MODE_MANUAL;
    state->pump_on = false;
    state->amplitude = 0;
    state->pid_target = 0;
    state->pid_elapsed_s = 0;
    state->pid_duration_s = 0;

    STATE_UNLOCK();
    // ESP_LOGI(TAG, "PID STOP → MANUAL mode");
}

void app_cmd_pid_target(system_state_t *state, float target)
{
    STATE_LOCK();
    pid_set_target(&state->pid, target);
    state->pid_target = target;
    STATE_UNLOCK();
    // ESP_LOGI(TAG, "PID target updated to %.2f ul/min", target);
}

void app_cmd_pid_tune(system_state_t *state, float kp, float ki, float kd)
{
    STATE_LOCK();
    pid_set_gains(&state->pid, kp, ki, kd);
    STATE_UNLOCK();
    // ESP_LOGI(TAG, "PID gains: Kp=%.2f Ki=%.2f Kd=%.2f", kp, ki, kd);
}

void app_cmd_status(system_state_t *state)
{
    STATE_LOCK();
    serial_comm_send_status(state);
    STATE_UNLOCK();
}

void app_cmd_scan(system_state_t *state)
{
    uint8_t devices[16];
    uint8_t num_found = 0;

    i2c_interface_scan(devices, 16, &num_found);
    serial_comm_send_scan(devices, num_found);
}

void app_cmd_set_calibration(system_state_t *state, const char *liquid)
{
    uint8_t cal_cmd;
    if (strcmp(liquid, "WATER") == 0) {
        cal_cmd = SLF3S_CALIBRATION_WATER;
    } else if (strcmp(liquid, "IPA") == 0) {
        cal_cmd = SLF3S_CALIBRATION_IPA;
    } else {
        serial_comm_send_err("INVALID_ARG");
        return;
    }

    /* Perform I2C operations outside the mutex to avoid stalling sensor_read_task.
     * The CAL command is already rejected during PID mode, so no concurrent
     * sensor reads will occur (sensor_read_task only reads when sensor_available). */
    STATE_LOCK();
    state->sensor_available = false;  /* prevent sensor_read_task from reading */
    STATE_UNLOCK();

    /* Stop current measurement */
    slf3s_stop_measurement(&state->flow_sensor);

    /* Update calibration setting */
    slf3s_set_calibration(&state->flow_sensor, cal_cmd);

    /* Restart measurement with new calibration */
    esp_err_t ret = slf3s_start_measurement_simple(&state->flow_sensor);

    STATE_LOCK();
    if (ret == ESP_OK) {
        state->sensor_available = true;
    }
    STATE_UNLOCK();

    if (ret == ESP_OK) {
        serial_comm_send_ok();
    } else {
        serial_comm_send_err("SENSOR_UNAVAIL");
    }
}

/********************************************************
 * FREERTOS TASKS
 ********************************************************/

/**
 * @brief Serial command parsing task
 *
 * Reads lines from UART and dispatches commands.
 */
static void serial_cmd_task(void *param)
{
    char cmd_buf[SERIAL_CMD_MAX_LEN];

    while (1) {
        int len = serial_comm_read_line(cmd_buf, sizeof(cmd_buf));
        if (len > 0) {
            serial_comm_process_cmd(&g_state, cmd_buf);
        }
    }
}

/**
 * @brief Sensor reading + data stream + PID compute task
 *
 * Runs at 10Hz (100ms interval).
 * - Reads flow sensor
 * - Runs PID compute if in PID mode
 * - Sends data stream if enabled
 */
static void sensor_read_task(void *param)
{
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t interval = pdMS_TO_TICKS(SENSOR_INTERVAL_MS);
    const float dt = (float)SENSOR_INTERVAL_MS / 1000.0f;

    /* Tick counter for 1-second PID elapsed time */
    uint32_t tick_count = 0;
    const uint32_t ticks_per_second = 1000 / SENSOR_INTERVAL_MS;

    /* Hot-plug re-probe counter (every 5 seconds) */
    uint32_t reprobe_count = 0;
    const uint32_t REPROBE_TICKS = 5 * ticks_per_second;

    while (1) {
        vTaskDelayUntil(&last_wake, interval);

        /* --- Hot-plug: re-probe unavailable devices every 5s --- */
        reprobe_count++;
        if (reprobe_count >= REPROBE_TICKS) {
            reprobe_count = 0;

            if (!g_state.pump_available) {
                if (i2c_interface_probe(MCP4726_I2C_ADDR) == ESP_OK) {
                    if (mp6_init(&g_state.pump) == ESP_OK) {
                        STATE_LOCK();
                        g_state.pump_available = true;
                        STATE_UNLOCK();
                    }
                }
            }

            if (!g_state.sensor_available) {
                if (i2c_interface_probe(SLF3S_I2C_ADDR) == ESP_OK) {
                    if (slf3s_init(&g_state.flow_sensor) == ESP_OK &&
                        slf3s_start_measurement(&g_state.flow_sensor) == ESP_OK) {
                        STATE_LOCK();
                        g_state.sensor_available = true;
                        STATE_UNLOCK();
                    }
                }
            }

            if (!g_state.pressure_available) {
                if (i2c_interface_probe(PRESSURE_SENSOR_I2C_ADDR) == ESP_OK) {
                    STATE_LOCK();
                    g_state.pressure_available = true;
                    STATE_UNLOCK();
                }
            }
        }

        /* Read flow sensor (skip if unavailable) */
        float flow = 0.0f;
        if (g_state.sensor_available) {
            slf3s_measurement_t meas = {0};
            esp_err_t ret = slf3s_read_measurement(&g_state.flow_sensor, &meas);
            if (ret == ESP_OK) {
                flow = meas.flow;
                STATE_LOCK();
                g_state.current_temperature = meas.temperature;
                g_state.sensor_flags = meas.flags;

                /* Air-in-line detection (edge-triggered: notify once on rising edge) */
                bool air_now = (meas.flags & SLF3S_FLAG_AIR_IN_LINE) != 0;
                if (air_now && !g_state.air_in_line) {
                    serial_comm_send_event_air_in_line();
                }
                g_state.air_in_line = air_now;

                /* High-flow detection (edge-triggered: notify once on rising edge) */
                bool high_now = (meas.flags & SLF3S_FLAG_HIGH_FLOW) != 0;
                if (high_now && !g_state.high_flow) {
                    serial_comm_send_event_high_flow();
                }
                g_state.high_flow = high_now;
                STATE_UNLOCK();
            }
        }

        STATE_LOCK();
        g_state.current_flow = flow;

        /* PID compute if in PID mode */
        if (g_state.mode == MODE_PID && g_state.pid.is_running) {
            float output = pid_compute(&g_state.pid, flow, dt);
            uint16_t amp = (uint16_t)(output + 0.5f);  /* Round to nearest int */
            mp6_set_amplitude(&g_state.pump, amp);
            g_state.amplitude = amp;

            /* Check flow deviation */
            if (pid_check_flow_deviation(&g_state.pid, flow, dt)) {
                serial_comm_send_event_flow_err(g_state.pid_target, flow);
            }

            /* 1-second tick for elapsed time and duration check */
            tick_count++;
            if (tick_count >= ticks_per_second) {
                tick_count = 0;
                g_state.pid_elapsed_s = g_state.pid.elapsed_s + 1;

                if (pid_tick_second(&g_state.pid)) {
                    /* Duration expired - auto stop */
                    pid_stop(&g_state.pid);
                    mp6_stop(&g_state.pump);
                    g_state.mode = MODE_MANUAL;
                    g_state.pump_on = false;
                    g_state.amplitude = 0;
                    g_state.pid_target = 0;
                    g_state.pid_elapsed_s = 0;
                    g_state.pid_duration_s = 0;

                    STATE_UNLOCK();
                    serial_comm_send_event_pid_done();
                    // ESP_LOGI(TAG, "PID duration expired → MANUAL mode");
                    tick_count = 0;
                    continue;
                }
            }
        } else {
            tick_count = 0;
        }

        /* Send data stream if enabled */
        bool stream = g_state.stream_enabled;
        float temp = g_state.current_temperature;
        STATE_UNLOCK();

        if (stream) {
            serial_comm_send_data(flow, temp);
        }
    }
}

/********************************************************
 * HARDWARE INITIALIZATION
 ********************************************************/

static esp_err_t hw_init(void)
{
    esp_err_t ret;

    /* Initialize UART first - must always work for PC communication */
    ret = serial_comm_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Serial comm init failed!");
        return ret;
    }

    /* Initialize I2C bus */
    ret = i2c_interface_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed!");
        return ret;
    }

    /* Scan I2C bus and detect hardware by address */
    uint8_t devices[16];
    uint8_t num_found = 0;
    i2c_interface_scan(devices, 16, &num_found);

    /* Detect MP6 pump (MCP4726 DAC at 0x61) */
    if (i2c_interface_probe(MCP4726_I2C_ADDR) == ESP_OK) {
        ret = mp6_init(&g_state.pump);
        if (ret == ESP_OK) {
            g_state.pump_available = true;
            ESP_LOGI(TAG, "MP6 pump driver detected (DAC at 0x%02X)", MCP4726_I2C_ADDR);
        } else {
            ESP_LOGW(TAG, "MP6 DAC found but driver init failed");
        }
    } else {
        ESP_LOGW(TAG, "MP6 pump not detected (no DAC at 0x%02X)", MCP4726_I2C_ADDR);
    }

    /* Detect flow sensor (SLF3S at 0x08) */
    if (i2c_interface_probe(SLF3S_I2C_ADDR) == ESP_OK) {
        ret = slf3s_init(&g_state.flow_sensor);
        if (ret == ESP_OK) {
            ret = slf3s_start_measurement(&g_state.flow_sensor);
        }
        if (ret == ESP_OK) {
            g_state.sensor_available = true;
            ESP_LOGI(TAG, "Flow sensor detected (SLF3S at 0x%02X)", SLF3S_I2C_ADDR);
        } else {
            ESP_LOGW(TAG, "Flow sensor found but init failed");
        }
    } else {
        ESP_LOGW(TAG, "Flow sensor not detected (no device at 0x%02X)", SLF3S_I2C_ADDR);
    }

    /* Detect pressure sensor (0x76) - detection only, no driver yet */
    if (i2c_interface_probe(PRESSURE_SENSOR_I2C_ADDR) == ESP_OK) {
        g_state.pressure_available = true;
        ESP_LOGI(TAG, "Pressure sensor detected (at 0x%02X)", PRESSURE_SENSOR_I2C_ADDR);
    } else {
        ESP_LOGW(TAG, "Pressure sensor not detected (no device at 0x%02X)", PRESSURE_SENSOR_I2C_ADDR);
    }

    return ESP_OK;
}

/********************************************************
 * MAIN ENTRY POINT
 ********************************************************/

void app_main(void)
{
    // ESP_LOGI(TAG, "=== Microfluidic Control System ===");
    // ESP_LOGI(TAG, "Firmware V3.0.0");

    /* Initialize system state */
    memset(&g_state, 0, sizeof(g_state));
    g_state.mode = MODE_MANUAL;
    g_state.pump_on = false;
    g_state.stream_enabled = false;
    g_state.frequency = 100;    /* Default frequency */
    g_state.pump_available = false;
    g_state.sensor_available = false;
    g_state.pressure_available = false;

    /* Initialize PID controller (default gains) */
    pid_init(&g_state.pid);

    /* Create state mutex */
    g_state_mutex = xSemaphoreCreateMutex();
    if (g_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create state mutex!");
        return;
    }

    /* Initialize hardware (non-fatal: tasks always start) */
    hw_init();

    // ESP_LOGI(TAG, "Hardware initialized successfully");
    // ESP_LOGI(TAG, "Mode: MANUAL (default)");

    /* Create FreeRTOS tasks */
    xTaskCreate(serial_cmd_task, "serial_cmd", SERIAL_CMD_TASK_STACK,
                NULL, SERIAL_CMD_TASK_PRIO, NULL);

    xTaskCreate(sensor_read_task, "sensor_read", SENSOR_READ_TASK_STACK,
                NULL, SENSOR_READ_TASK_PRIO, NULL);

    // ESP_LOGI(TAG, "Tasks started. Ready for commands.");
}
