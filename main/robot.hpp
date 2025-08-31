#pragma once

#include "PID.hpp"
#include "OneEuroFilter.h"
#include "config.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "image_detector.hpp"
#include "servo_controller.hpp"
#include "timing_profiler.hpp"
#include <array>
#include <memory>

/**
 * @brief 代表整个机器人系统的核心类
 *
 * 该类封装了所有硬件模块(摄像头、舵机)和控制逻辑(PID、运动学),
 * 负责初始化系统并运行主控制循环。
 */
class Robot
{
  public:
    Robot();
    ~Robot();

    /**
     * @brief 初始化所有硬件和软件模块
     * @return esp_err_t ESP_OK表示成功,否则表示失败
     */
    esp_err_t init();

    /**
     * @brief 实时闭环控制任务的主循环
     */
    void run_realtime_control();

    /**
     * @brief 视频流任务的主循环
     */
    void run_streaming();

  private:
    /**
     * @brief 初始化舵机和PID控制器
     */
    void init_controllers();

    /**
     * @brief 将计算出的角度应用到三个舵机上
     * @param angles 包含三个舵机角度的数组 {a, b, c}
     */
    void apply_servo_angles(const std::array<float, 3> &angles);

    /**
     * @brief 执行一次完整的实时控制循环迭代
     */
    void control_loop_iteration();

    /**
     * @brief 处理图像、运行检测并返回检测结果
     * @param img 输入的灰度图像
     * @return ImageDetector::Circle 检测到的圆形
     */
    ImageDetector::Circle detect_ball(cv::Mat &img);

    /**
     * @brief 根据检测到的球的位置计算PID输出
     * @param ball The detected circle object.
     * @param frame_size The size of the image frame.
     */
    void update_pid_controllers(const ImageDetector::Circle &ball, const cv::Size &frame_size);

    /**
     * @brief 记录帧率和性能分析数据
     */
    void log_performance();

    // 状态变量
    bool is_initialized_ = false;

    // 硬件模块
    std::unique_ptr<ServoController::Servo> servo_a_;
    std::unique_ptr<ServoController::Servo> servo_b_;
    std::unique_ptr<ServoController::Servo> servo_c_;

    // 控制模块
    std::unique_ptr<pid_positional_controller> x_controller_;
    std::unique_ptr<pid_positional_controller> y_controller_;

    // 滤波器
    std::unique_ptr<OneEuroFilter> x_filter_;
    std::unique_ptr<OneEuroFilter> y_filter_;

#ifdef ENABLE_TIMING_PROFILE
    TimingProfiler profiler_;
#endif

    // 任务状态变量
    uint32_t loop_counter_ = 0;
    int frame_count_ = 0;
    int64_t last_fps_time_us_ = 0;
    int64_t last_pid_time_us_ = 0;
};