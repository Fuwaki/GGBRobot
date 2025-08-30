#include <algorithm>
#include <cstdio>
class pid_positional_controller
{
  private:
    double kp, ki, kd;
    double integral;
    double previous_error, err;
    double dt; // 采样时间（单位：秒）
    bool enable_output_limit = false;
    double output_limit_down = -1;
    double output_limit_up = 1;


  public:
    pid_positional_controller(double kp, double ki, double kd, double dt = 1.0)
        : kp(kp), ki(ki), kd(kd),  integral(0), previous_error(0),err(0), dt(dt)
    {
    }

    double get_adjustment() 
    {
        double derivative = 0;
        if (dt != 0.0)
            derivative = (err - previous_error) / dt;

        auto res = kp * err + ki * integral + kd * derivative;
        // printf("p:%f i:%f d:%f\n", kp * err, ki * integral, kd * derivative);
        return enable_output_limit?std::clamp(res, output_limit_down, output_limit_up):res;
    }

    void set_error(double error) 
    {
        previous_error = err;  // 保存上一次的误差
        err = error;
        // 使用采样时间累加积分
        integral += error * dt;
    }
    // 设置采样时间（秒）
    void set_dt(double new_dt)
    {
        if (new_dt > 0)
            dt = new_dt;
    }
    void enable_limit(float down, float up)
    {
        enable_output_limit = true;
        output_limit_down = down;
        output_limit_up = up;
    }
    void clear()
    {
        integral = 0;
        previous_error = 0;
        err = 0;
    }
};