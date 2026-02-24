/********************************************************
 * @file        serial_comm.c
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V1.0.0
 * @date        13/02/2026
 * @brief       Serial communication module for PC-ESP32 protocol
 *
 * @details
 *  - UART0 initialization (115200, 8N1)
 *  - Command parsing (line-based text protocol)
 *  - Data stream and event reporting
 *  - See API.md for full protocol specification
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <stdlib.h>
#include "serial_comm.h"
#include "driver/uart.h"
#include "esp_log.h"

/* main.c exposes the full system_state definition */
#include "main.h"

static const char *TAG = "SERIAL";

/********************************************************
 * UART INITIALIZATION
 ********************************************************/

esp_err_t serial_comm_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate  = SERIAL_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    esp_err_t ret = uart_driver_install(SERIAL_UART_NUM,
                                        SERIAL_RX_BUF_SIZE * 2,
                                        SERIAL_TX_BUF_SIZE,
                                        0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART driver install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = uart_param_config(SERIAL_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "UART param config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // ESP_LOGI(TAG, "UART%d initialized: %d baud", SERIAL_UART_NUM, SERIAL_BAUD_RATE);
    return ESP_OK;
}

/********************************************************
 * LOW-LEVEL I/O
 ********************************************************/

int serial_comm_read_line(char *buf, int max_len)
{
    int pos = 0;
    while (pos < max_len - 1) {
        uint8_t ch;
        int len = uart_read_bytes(SERIAL_UART_NUM, &ch, 1, portMAX_DELAY);
        if (len <= 0) {
            continue;
        }
        if (ch == '\n' || ch == '\r') {
            if (pos > 0) {
                break;  /* end of line */
            }
            continue;   /* skip leading CR/LF */
        }
        buf[pos++] = (char)ch;
    }
    buf[pos] = '\0';
    return pos;
}

void serial_comm_send(const char *fmt, ...)
{
    char buf[256];
    va_list args;
    va_start(args, fmt);
    int len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len > 0) {
        uart_write_bytes(SERIAL_UART_NUM, buf, len);
    }
}

/********************************************************
 * RESPONSE HELPERS
 ********************************************************/

void serial_comm_send_ok(void)
{
    serial_comm_send("OK\n");
}

void serial_comm_send_err(const char *reason)
{
    serial_comm_send("ERR %s\n", reason);
}

void serial_comm_send_data(float flow, float temperature)
{
    serial_comm_send("D %.2f %.2f\n", flow, temperature);
}

void serial_comm_send_status(const system_state_t *state)
{
    serial_comm_send("S %s %d %d %d %.2f %.2f %d %d %d %d %d %.2f\n",
                     (state->mode == MODE_PID) ? "PID" : "MANUAL",
                     state->pump_on ? 1 : 0,
                     (int)state->amplitude,
                     (int)state->frequency,
                     state->current_flow,
                     state->pid_target,
                     (int)state->pid_elapsed_s,
                     (int)state->pid_duration_s,
                     state->pump_available ? 1 : 0,
                     state->sensor_available ? 1 : 0,
                     state->pressure_available ? 1 : 0,
                     state->current_temperature);
}

void serial_comm_send_scan(const uint8_t *devices, uint8_t count)
{
    char buf[128];
    int pos = snprintf(buf, sizeof(buf), "SCAN");
    for (uint8_t i = 0; i < count && pos < (int)sizeof(buf) - 5; i++) {
        pos += snprintf(buf + pos, sizeof(buf) - pos, " %02X", devices[i]);
    }
    snprintf(buf + pos, sizeof(buf) - pos, "\n");
    serial_comm_send("%s", buf);
}

void serial_comm_send_event_pid_done(void)
{
    serial_comm_send("EVENT PID_DONE\n");
}

void serial_comm_send_event_flow_err(float target, float actual)
{
    serial_comm_send("EVENT FLOW_ERR %.2f %.2f\n", target, actual);
}

void serial_comm_send_event_air_in_line(void)
{
    serial_comm_send("EVENT AIR_IN_LINE\n");
}

void serial_comm_send_event_high_flow(void)
{
    serial_comm_send("EVENT HIGH_FLOW\n");
}

/********************************************************
 * COMMAND PARSING
 *
 * Command validation quick-reference (12 commands):
 *   PUMP ON         - requires pump_available, rejects if PID mode
 *   PUMP OFF        - requires pump_available (auto PID STOP if PID mode)
 *   AMP <80-250>    - requires pump_available, rejects if PID mode, range check
 *   FREQ <25-300>   - requires pump_available, rejects if PID mode, range check
 *   PID START <t><d>- requires pump + sensor available, target > 0
 *   PID STOP        - unconditional
 *   PID TARGET <v>  - requires MODE_PID, value > 0
 *   PID TUNE <K>    - requires 3 float args
 *   STATUS          - unconditional
 *   SCAN            - unconditional
 *   STREAM ON/OFF   - unconditional
 *   CAL <WATER|IPA> - requires sensor_available, rejects if PID mode
 ********************************************************/

/* Helper: skip leading spaces and return pointer to next token */
static const char *skip_spaces(const char *s)
{
    while (*s == ' ') s++;
    return s;
}

/* Helper: check if string starts with prefix (case-sensitive) */
static bool starts_with(const char *str, const char *prefix)
{
    return strncmp(str, prefix, strlen(prefix)) == 0;
}

void serial_comm_process_cmd(system_state_t *state, const char *cmd_line)
{
    const char *cmd = skip_spaces(cmd_line);

    if (strlen(cmd) == 0) {
        return;
    }

    // ESP_LOGI(TAG, "CMD: %s", cmd);

    /* ---- PUMP ON ---- */
    if (strcmp(cmd, "PUMP ON") == 0) {
        if (!state->pump_available) {
            serial_comm_send_err("PUMP_UNAVAIL");
            return;
        }
        if (state->mode == MODE_PID) {
            serial_comm_send_err("PID_ACTIVE");
            return;
        }
        app_cmd_pump_on(state);
        serial_comm_send_ok();
        return;
    }

    /* ---- PUMP OFF ---- */
    if (strcmp(cmd, "PUMP OFF") == 0) {
        if (!state->pump_available) {
            serial_comm_send_err("PUMP_UNAVAIL");
            return;
        }
        if (state->mode == MODE_PID) {
            /* PUMP OFF during PID → auto PID STOP */
            app_cmd_pid_stop(state);
        } else {
            app_cmd_pump_off(state);
        }
        serial_comm_send_ok();
        return;
    }

    /* ---- AMP <value> ---- */
    if (starts_with(cmd, "AMP ")) {
        if (!state->pump_available) {
            serial_comm_send_err("PUMP_UNAVAIL");
            return;
        }
        if (state->mode == MODE_PID) {
            serial_comm_send_err("PID_ACTIVE");
            return;
        }
        int val = atoi(cmd + 4);
        if (val < MP6_AMPLITUDE_MIN || val > MP6_AMPLITUDE_MAX) {
            serial_comm_send_err("INVALID_ARG");
            return;
        }
        app_cmd_set_amplitude(state, (uint16_t)val);
        serial_comm_send_ok();
        return;
    }

    /* ---- FREQ <value> ---- */
    if (starts_with(cmd, "FREQ ")) {
        if (!state->pump_available) {
            serial_comm_send_err("PUMP_UNAVAIL");
            return;
        }
        if (state->mode == MODE_PID) {
            serial_comm_send_err("PID_ACTIVE");
            return;
        }
        int val = atoi(cmd + 5);
        if (val < MP6_FREQ_MIN || val > MP6_FREQ_MAX) {
            serial_comm_send_err("INVALID_ARG");
            return;
        }
        app_cmd_set_frequency(state, (uint16_t)val);
        serial_comm_send_ok();
        return;
    }

    /* ---- PID START <target> <duration> ---- */
    if (starts_with(cmd, "PID START ")) {
        if (!state->pump_available) {
            serial_comm_send_err("PUMP_UNAVAIL");
            return;
        }
        if (!state->sensor_available) {
            serial_comm_send_err("SENSOR_UNAVAIL");
            return;
        }
        const char *args = skip_spaces(cmd + 10);
        float target = 0;
        int duration = 0;
        if (sscanf(args, "%f %d", &target, &duration) < 2) {
            serial_comm_send_err("INVALID_ARG");
            return;
        }
        if (target <= 0) {
            serial_comm_send_err("INVALID_ARG");
            return;
        }
        app_cmd_pid_start(state, target, (uint32_t)duration);
        serial_comm_send_ok();
        return;
    }

    /* ---- PID STOP ---- */
    if (strcmp(cmd, "PID STOP") == 0) {
        app_cmd_pid_stop(state);
        serial_comm_send_ok();
        return;
    }

    /* ---- PID TARGET <value> ---- */
    if (starts_with(cmd, "PID TARGET ")) {
        if (state->mode != MODE_PID) {
            serial_comm_send_err("NOT_PID_MODE");
            return;
        }
        float val = atof(cmd + 11);
        if (val <= 0) {
            serial_comm_send_err("INVALID_ARG");
            return;
        }
        app_cmd_pid_target(state, val);
        serial_comm_send_ok();
        return;
    }

    /* ---- PID TUNE <Kp> <Ki> <Kd> ---- */
    if (starts_with(cmd, "PID TUNE ")) {
        float kp, ki, kd;
        if (sscanf(cmd + 9, "%f %f %f", &kp, &ki, &kd) != 3) {
            serial_comm_send_err("INVALID_ARG");
            return;
        }
        app_cmd_pid_tune(state, kp, ki, kd);
        serial_comm_send_ok();
        return;
    }

    /* ---- STATUS ---- */
    if (strcmp(cmd, "STATUS") == 0) {
        app_cmd_status(state);
        return;
    }

    /* ---- SCAN ---- */
    if (strcmp(cmd, "SCAN") == 0) {
        app_cmd_scan(state);
        return;
    }

    /* ---- STREAM ON ---- */
    if (strcmp(cmd, "STREAM ON") == 0) {
        state->stream_enabled = true;
        serial_comm_send_ok();
        return;
    }

    /* ---- STREAM OFF ---- */
    if (strcmp(cmd, "STREAM OFF") == 0) {
        state->stream_enabled = false;
        serial_comm_send_ok();
        return;
    }

    /* ---- CAL <WATER|IPA> ---- */
    if (starts_with(cmd, "CAL ")) {
        if (!state->sensor_available) {
            serial_comm_send_err("SENSOR_UNAVAIL");
            return;
        }
        if (state->mode == MODE_PID) {
            serial_comm_send_err("PID_ACTIVE");
            return;
        }
        const char *liquid = skip_spaces(cmd + 4);
        app_cmd_set_calibration(state, liquid);
        return;
    }

    /* ---- Unknown command ---- */
    serial_comm_send_err("UNKNOWN_CMD");
}
