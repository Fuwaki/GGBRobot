#pragma once // 使用 #pragma once 防止头文件重复包含

#include <algorithm>
#include <cstdio>

/**
 * @brief 位置式PID控制器
 */
class pid_positional_controller
{
private:
    double kp, ki, kd;      // PID增益: 比例, 积分, 微分
    double integral;        // 积分项累加值
    double previous_error;  // 上一次的误差
    double err;             // 当前误差
    double dt;              // 采样时间 (秒)

    bool enable_output_limit; // 是否启用输出限幅
    double output_limit_down; // 输出下限
    double output_limit_up;   // 输出上限

public:
    /**
     * @brief 构造一个PID控制器
     * @param kp 比例增益
     * @param ki 积分增益
     * @param kd 微分增益
     * @param dt 采样时间(秒)
     */
    pid_positional_controller(double kp, double ki, double kd, double dt = 1.0)
        : kp(kp), ki(ki), kd(kd), integral(0), previous_error(0), err(0), dt(dt),
          enable_output_limit(false), output_limit_down(-1.0), output_limit_up(1.0)
    {
    }

    /**
     * @brief 计算PID调整量
     * @return double 控制器的输出值
     */
    double get_adjustment()
    {
        double derivative = 0;
        if (dt != 0.0) {
            derivative = (err - previous_error) / dt;
        }

        double result = kp * err + ki * integral + kd * derivative;
        
        if (enable_output_limit) {
            return std::clamp(result, output_limit_down, output_limit_up);
        }
        return result;
    }

    /**
     * @brief 设置新的误差值
     * @param error 新的误差输入
     */
    void set_error(double error)
    {
        previous_error = err;  // 保存上一次的误差
        err = error;
        integral += error * dt; // 使用采样时间累加积分
    }

    /**
     * @brief 设置采样时间(秒)
     * @param new_dt 新的采样时间
     */
    void set_dt(double new_dt)
    {
        if (new_dt > 0) {
            dt = new_dt;
        }
    }

    /**
     * @brief 启用输出限幅
     * @param down 输出下限
     * @param up 输出上限
     */
    void enable_limit(double down, double up)
    {
        enable_output_limit = true;
        output_limit_down = down;
        output_limit_up = up;
    }

    /**
     * @brief 清除控制器内部状态(积分、误差)
     */
    void clear()
    {
        integral = 0;
        previous_error = 0;
        err = 0;
    }
};
