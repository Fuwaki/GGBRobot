#include "servo_controller.hpp"

namespace ServoController {

Servo::Servo(gpio_num_t pin, ledc_timer_t timer, ledc_channel_t channel) {
    ledc_channel = channel;
    ledc_timer = timer;
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,
        .max_width_us = 2500,
        .freq = 50,
        .timer_number = timer,
        .channels = {
            .servo_pin = {
                pin,
            },
            .ch = {
                channel,
            },
        },
        .channel_number = 1,
    };
    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);
}

Servo::~Servo() {
    // 用户指定的 espressif/servo 版本没有反初始化函数。
    // 我们可以停止LEDC通道。
    ledc_stop(LEDC_LOW_SPEED_MODE, ledc_channel, 0);
}

void Servo::change_angle(float angle) {
    if (angle < 0) angle = 0;
    if (angle > 180) angle = 180;

    float target_angle = 180.0f - angle;

    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, ledc_channel, target_angle);
}

} // namespace ServoController