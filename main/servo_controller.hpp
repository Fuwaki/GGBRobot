#pragma once

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "iot_servo.h"

/**
 * @brief 舵机控制模块
 */
namespace ServoController
{

/**
 * @brief 封装了iot_servo库的舵机控制类
 */
class Servo
{
  public:
    /**
     * @brief 构造一个新的舵机对象并初始化
     * @param pin 舵机信号线连接的GPIO引脚
     * @param timer 使用的LEDC定时器
     * @param channel 使用的LEDC通道
     */
    Servo(gpio_num_t pin, ledc_timer_t timer, ledc_channel_t channel);

    /**
     * @brief 析构函数, 停止LEDC通道
     */
    ~Servo();

    /**
     * @brief 改变舵机的角度
     * @param angle 目标角度 (0.0 to 180.0)
     */
    void change_angle(float angle);

  private:
    ledc_channel_t ledc_channel;
    ledc_timer_t ledc_timer;
};

} // namespace ServoController
