/********************************************************
 * @file        mp6_driver.c
 * @author      Junjian Chi (jc2592@cam.ac.uk)
 * @version     V2.0.0
 * @date        12/02/2026
 * @brief       MP6 micropump driver implementation
 *
 * @details
 *  Uses mcp4726_dac module for amplitude control.
 *  Clock signal via LEDC PWM (95% duty cycle, 25-300 Hz).
 *  Enable/shutdown via GPIO.
 *
 * SPDX-License-Identifier: MIT
 ********************************************************/

#include "mp6_driver.h"
#include "mcp4726_dac.h"
#include "sensor_config.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

static const char *TAG = "MP6";

/********************************************************
 * LEDC PWM CONFIGURATION
 ********************************************************/

#define MP6_LEDC_TIMER          LEDC_TIMER_0
#define MP6_LEDC_MODE           LEDC_LOW_SPEED_MODE
#define MP6_LEDC_CHANNEL        LEDC_CHANNEL_0
#define MP6_LEDC_RESOLUTION     LEDC_TIMER_10_BIT   /* 1024 steps */
#define MP6_LEDC_MAX_DUTY       ((1 << 10) - 1)     /* 1023 */

/********************************************************
 * INTERNAL: CLOCK PWM FUNCTIONS
 ********************************************************/

static esp_err_t clock_pwm_init(uint32_t freq_hz)
{
    ledc_timer_config_t timer_conf = {
        .speed_mode       = MP6_LEDC_MODE,
        .timer_num        = MP6_LEDC_TIMER,
        .duty_resolution  = MP6_LEDC_RESOLUTION,
        .freq_hz          = freq_hz,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    esp_err_t ret = ledc_timer_config(&timer_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC timer config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    uint32_t duty = (uint32_t)(MP6_LEDC_MAX_DUTY * MP6_CLOCK_DUTY_PERCENT / 100);

    ledc_channel_config_t channel_conf = {
        .speed_mode     = MP6_LEDC_MODE,
        .channel        = MP6_LEDC_CHANNEL,
        .timer_sel      = MP6_LEDC_TIMER,
        .intr_type      = LEDC_INTR_DISABLE,
        .gpio_num       = MP_DRIVER_CLOCK_IO,
        .duty           = duty,
        .hpoint         = 0,
    };
    ret = ledc_channel_config(&channel_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LEDC channel config failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // ESP_LOGI(TAG, "Clock PWM: %lu Hz, %d%% duty", freq_hz, MP6_CLOCK_DUTY_PERCENT);
    return ESP_OK;
}

static esp_err_t clock_pwm_set_freq(uint32_t freq_hz)
{
    return ledc_set_freq(MP6_LEDC_MODE, MP6_LEDC_TIMER, freq_hz);
}

static esp_err_t clock_pwm_stop(void)
{
    esp_err_t ret = ledc_set_duty(MP6_LEDC_MODE, MP6_LEDC_CHANNEL, 0);
    if (ret != ESP_OK) return ret;
    return ledc_update_duty(MP6_LEDC_MODE, MP6_LEDC_CHANNEL);
}

static esp_err_t clock_pwm_start(void)
{
    uint32_t duty = (uint32_t)(MP6_LEDC_MAX_DUTY * MP6_CLOCK_DUTY_PERCENT / 100);
    esp_err_t ret = ledc_set_duty(MP6_LEDC_MODE, MP6_LEDC_CHANNEL, duty);
    if (ret != ESP_OK) return ret;
    return ledc_update_duty(MP6_LEDC_MODE, MP6_LEDC_CHANNEL);
}

/********************************************************
 * PUBLIC API
 ********************************************************/

esp_err_t mp6_init(mp6_handle_t *handle)
{
    if (handle == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;

    handle->amplitude = 0;
    handle->freq_hz = MP6_FREQ_MIN;
    handle->is_running = false;
    handle->is_initialized = false;

    /* Configure shutdown pin: start in shutdown (LOW) */
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MP_DRIVER_ENABLE_IO),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ret = gpio_config(&io_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Shutdown GPIO config failed: %s", esp_err_to_name(ret));
        return ret;
    }
    gpio_set_level(MP_DRIVER_ENABLE_IO, 0);

    /* Initialize DAC */
    ret = mcp4726_init();
    if (ret != ESP_OK) return ret;

    /* Initialize clock PWM at minimum frequency */
    ret = clock_pwm_init(handle->freq_hz);
    if (ret != ESP_OK) return ret;

    clock_pwm_stop();

    handle->is_initialized = true;
    // ESP_LOGI(TAG, "MP6 driver initialized");
    // ESP_LOGI(TAG, "  Enable: GPIO%d, Clock: GPIO%d, DAC: 0x%02X",
             // MP_DRIVER_ENABLE_IO, MP_DRIVER_CLOCK_IO, MCP4726_I2C_ADDR);

    return ESP_OK;
}

esp_err_t mp6_start(mp6_handle_t *handle)
{
    if (handle == NULL || !handle->is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    gpio_set_level(MP_DRIVER_ENABLE_IO, 1);

    esp_err_t ret = clock_pwm_start();
    if (ret != ESP_OK) return ret;

    handle->is_running = true;
    // ESP_LOGI(TAG, "Pump started (%d Hz, amplitude %d)",
             // handle->freq_hz, handle->amplitude);
    return ESP_OK;
}

esp_err_t mp6_stop(mp6_handle_t *handle)
{
    if (handle == NULL || !handle->is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    clock_pwm_stop();
    mcp4726_set_voltage(0.0f);
    gpio_set_level(MP_DRIVER_ENABLE_IO, 0);

    handle->amplitude = 0;
    handle->is_running = false;
    // ESP_LOGI(TAG, "Pump stopped");
    return ESP_OK;
}

esp_err_t mp6_set_amplitude(mp6_handle_t *handle, uint16_t amplitude)
{
    if (handle == NULL || !handle->is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (amplitude == 0) {
        handle->amplitude = 0;
        return mcp4726_set_voltage(0.0f);
    }

    /* Clamp to valid range */
    if (amplitude < MP6_AMPLITUDE_MIN) amplitude = MP6_AMPLITUDE_MIN;
    if (amplitude > MP6_AMPLITUDE_MAX) amplitude = MP6_AMPLITUDE_MAX;

    /* Map amplitude 80-250 -> voltage 0.35-1.3V */
    float voltage = MP6_VOLTAGE_MIN +
        (float)(amplitude - MP6_AMPLITUDE_MIN) *
        (MP6_VOLTAGE_MAX - MP6_VOLTAGE_MIN) /
        (float)(MP6_AMPLITUDE_MAX - MP6_AMPLITUDE_MIN);

    esp_err_t ret = mcp4726_set_voltage(voltage);
    if (ret != ESP_OK) return ret;

    handle->amplitude = amplitude;
    // ESP_LOGI(TAG, "Amplitude: %d -> %.3f V", amplitude, voltage);

    return ESP_OK;
}

esp_err_t mp6_set_frequency(mp6_handle_t *handle, uint16_t freq_hz)
{
    if (handle == NULL || !handle->is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    if (freq_hz < MP6_FREQ_MIN || freq_hz > MP6_FREQ_MAX) {
        ESP_LOGE(TAG, "Freq out of range: %d Hz (valid: %d-%d Hz)",
                 freq_hz, MP6_FREQ_MIN, MP6_FREQ_MAX);
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = clock_pwm_set_freq(freq_hz);
    if (ret != ESP_OK) return ret;

    handle->freq_hz = freq_hz;
    // ESP_LOGI(TAG, "Driver freq: %d Hz", freq_hz);
    return ESP_OK;
}
