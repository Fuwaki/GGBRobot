#include "servo_controller.hpp"

namespace ServoController
{

Servo::Servo(gpio_num_t pin, ledc_timer_t timer, ledc_channel_t channel)
{
    ledc_channel = channel;
    ledc_timer = timer;

    // 配置舵机参数
    servo_config_t servo_cfg = {
        .max_angle = 180,
        .min_width_us = 500,  // 500us对应0度
        .max_width_us = 2500, // 2500us对应180度
        .freq = 50,           // 50Hz频率
        .timer_number = timer,
        .channels =
            {
                .servo_pin =
                    {
                        pin,
                    },
                .ch =
                    {
                        channel,
                    },
            },
        .channel_number = 1,
    };

    // 初始化舵机
    iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);

    // 初始化时, 将舵机设置到一个中间或安全的角度
    this->change_angle(90.0);
}

Servo::~Servo()
{
    // espressif/servo 库没有提供反初始化函数,
    // 因此我们手动停止LEDC通道以释放资源。
    ledc_stop(LEDC_LOW_SPEED_MODE, ledc_channel, 0);
}

void Servo::change_angle(float angle)
{
    // 限制角度在0到180度之间
    if (angle < 0)
        angle = 0;
    if (angle > 180)
        angle = 180;

    // 注意: 这里的 `180.0f - angle` 是一个转换
    // 这可能是因为舵机的物理安装方向与期望的逻辑方向相反
    // 如果舵机转向错误, 请调整此处的计算
    float target_angle = 180.0f - angle;

    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, ledc_channel, target_angle);
}

} // namespace ServoController
