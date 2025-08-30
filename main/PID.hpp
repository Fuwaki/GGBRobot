#include <algorithm>
class pid_positional_controller
{
  private:
    double kp, ki, kd;
    double integral;
    double previous_error, err;
    bool enable_output_limit = false;
    double output_limit_down = -1;
    double output_limit_up = 1;


  public:
    pid_positional_controller(double kp, double ki, double kd)
        : kp(kp), ki(ki), kd(kd),  integral(0), previous_error(0)
    {
    }

    double get_adjustment() 
    {
        auto res = kp * err + ki * integral + kd * (err-previous_error);
        return enable_output_limit?std::clamp(res, output_limit_down, output_limit_up):res;
    }

    void set_error(double error) 
    {
        integral += error;
        err = error;
        previous_error = error;
    }
    void enable_limit(float down, float up)
    {
        enable_output_limit = true;
        output_limit_down = down;
        output_limit_up = up;
    }
    void clear()
    {
        integral=0;
    }
};