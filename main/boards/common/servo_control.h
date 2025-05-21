#ifndef _SERVO_CONTROL_H_
#define _SERVO_CONTROL_H_

#include <driver/ledc.h>
#include <driver/gpio.h>

class ServoControl {
public:
    ServoControl(gpio_num_t gpio_pin, ledc_timer_t timer_num, ledc_channel_t channel_num);
    void set_angle(int angle);

private:
    uint32_t angle_to_duty(int angle);

    gpio_num_t gpio_pin_;
    ledc_timer_t timer_num_;
    ledc_channel_t channel_num_;
    
    // Define pulse width limits for servo (in microseconds)
    static constexpr int SERVO_MIN_PULSEWIDTH_US = 1000; // Minimum pulse width for 0 degrees
    static constexpr int SERVO_MAX_PULSEWIDTH_US = 2000; // Maximum pulse width for 180 degrees
    static constexpr int SERVO_TIMER_FREQ_HZ = 50;       // Servo timer frequency (50Hz -> 20ms period)
    // LEDC timer resolution (e.g., 10-bit for 1024 steps)
    static constexpr ledc_timer_bit_t LEDC_TIMER_RESOLUTION = LEDC_TIMER_10_BIT;
};

#endif // _SERVO_CONTROL_H_
