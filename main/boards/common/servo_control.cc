#include "servo_control.h"
#include <esp_log.h>

static const char* TAG = "ServoControl";

ServoControl::ServoControl(gpio_num_t gpio_pin, ledc_timer_t timer_num, ledc_channel_t channel_num)
    : gpio_pin_(gpio_pin), timer_num_(timer_num), channel_num_(channel_num) {

    ledc_timer_config_t timer_conf = {
        .speed_mode = LEDC_LOW_SPEED_MODE, // Or LEDC_HIGH_SPEED_MODE
        .duty_resolution = LEDC_TIMER_RESOLUTION,
        .timer_num = timer_num_,
        .freq_hz = SERVO_TIMER_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ESP_ERROR_CHECK(ledc_timer_config(&timer_conf));

    ledc_channel_config_t channel_conf = {
        .gpio_num = gpio_pin_,
        .speed_mode = LEDC_LOW_SPEED_MODE, // Or LEDC_HIGH_SPEED_MODE
        .channel = channel_num_,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = timer_num_,
        .duty = 0,
        .hpoint = 0,
        .flags = { .output_invert = 0 }
    };
    ESP_ERROR_CHECK(ledc_channel_config(&channel_conf));
    ESP_LOGI(TAG, "ServoControl initialized on GPIO %d, timer %d, channel %d", gpio_pin_, timer_num_, channel_num_);
}

void ServoControl::set_angle(int angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    uint32_t duty = angle_to_duty(angle);
    ESP_LOGD(TAG, "Setting angle to %d, duty: %" PRIu32, angle, duty);

    ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, channel_num_, duty));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, channel_num_));
}

uint32_t ServoControl::angle_to_duty(int angle) {
    // Using standard pulse widths: 1ms to 2ms for 0-180 degrees.
    // For 50Hz (20ms period) and 10-bit resolution (1024 steps):
    // Min duty (0 deg, 1ms pulse): (1ms / 20ms) * 1023 = 51.15 (approx 51)
    // Max duty (180 deg, 2ms pulse): (2ms / 20ms) * 1023 = 102.3 (approx 102)
    // Using previously defined constants from .h for consistency if they were meant for this calculation.
    // Calculate duty based on defined pulse widths and timer resolution
    // Max duty value for the configured resolution (e.g., 1023 for 10-bit)
    uint32_t max_duty_val = (1 << LEDC_TIMER_RESOLUTION) - 1;

    // Calculate pulse width in microseconds for the given angle
    float pulse_width_us = SERVO_MIN_PULSEWIDTH_US + ((float)angle / 180.0f) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US);

    // Calculate duty cycle: (pulse_width_us / period_us) * max_duty_val
    // Period in microseconds: (1 / SERVO_TIMER_FREQ_HZ) * 1,000,000
    uint32_t period_us = 1000000 / SERVO_TIMER_FREQ_HZ;
    uint32_t duty = (uint32_t)((pulse_width_us / (float)period_us) * max_duty_val);
    
    return duty;
}
