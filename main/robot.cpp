#include "robot.hpp"
#include "camera_module.hpp"
#include "config.hpp"
#include "esp_log.h"
#include "esp_timer.h"
#include "image_detector.hpp"
#include "image_processor.hpp"
#include "kinematics_solver.hpp"
#include "stream_server.hpp"
#include <opencv2/opencv.hpp>

static const char *TAG = "机器人";

Robot::Robot() = default;
Robot::~Robot() = default;

void Robot::init_controllers()
{
    // 使用 std::make_unique 创建智能指针,实现自动内存管理
    servo_a_ = std::make_unique<ServoController::Servo>(Pins::SERVO_A, LEDC_TIMER_0, LEDC_CHANNEL_0);
    servo_b_ = std::make_unique<ServoController::Servo>(Pins::SERVO_B, LEDC_TIMER_0, LEDC_CHANNEL_1);
    servo_c_ = std::make_unique<ServoController::Servo>(Pins::SERVO_C, LEDC_TIMER_0, LEDC_CHANNEL_2);

    x_controller_ = std::make_unique<pid_positional_controller>(Params::PID_P, Params::PID_I, Params::PID_D);
    y_controller_ = std::make_unique<pid_positional_controller>(Params::PID_P, Params::PID_I, Params::PID_D);

    x_controller_->enable_limit(Params::PID_OUTPUT_LIMIT_MIN, Params::PID_OUTPUT_LIMIT_MAX);
    y_controller_->enable_limit(Params::PID_OUTPUT_LIMIT_MIN, Params::PID_OUTPUT_LIMIT_MAX);
}

esp_err_t Robot::init()
{
    esp_err_t cam_err = CameraModule::init();
    if (cam_err != ESP_OK)
    {
        ESP_LOGE(TAG, "摄像头初始化失败，错误码: 0x%x", cam_err);
        return cam_err;
    }
    ESP_LOGI(TAG, "摄像头初始化成功");

    init_controllers();
    ESP_LOGI(TAG, "控制器(舵机和PID)初始化成功");

    is_initialized_ = true;
    return ESP_OK;
}

void Robot::apply_servo_angles(const std::array<float, 3> &angles)
{
    // 舵机安装方向可能不同,这里根据实际情况调整
    // angles[0] -> servo_c, angles[1] -> servo_b, angles[2] -> servo_a
    servo_c_->change_angle(angles[0] + Params::SERVO_C_COMPENSATION);
    servo_b_->change_angle(angles[1] + Params::SERVO_B_COMPENSATION);
    servo_a_->change_angle(angles[2] + Params::SERVO_A_COMPENSATION);
}

ImageDetector::Circle Robot::detect_ball(cv::Mat &img)
{
    ImageProcessor::binarize(img, Params::BINARIZE_THRESHOLD);

    ImageDetector::ContourFindParams params;
    params.selection_method = ImageDetector::ContourFindParams::Selection::CLOSEST_TO_POINT;
    params.expected_pos = cv::Point2f(img.cols / 2.0f, img.rows / 2.0f);
    params.min_area = Params::DETECT_MIN_AREA;
    params.max_area = Params::DETECT_MAX_AREA;
    params.min_circularity = Params::DETECT_MIN_CIRCULARITY;
    params.min_radius = Params::DETECT_MIN_RADIUS;
    params.max_radius = Params::DETECT_MAX_RADIUS;

    return ImageDetector::find_ball_by_contour(img, params);
}

void Robot::update_pid_controllers(const ImageDetector::Circle &ball, const cv::Size &frame_size)
{
    if (ball.found)
    {
        gpio_set_level(Pins::LED_B, 1);

        float x_error = ball.center.x - frame_size.width / 2.0f;
        float y_error = ball.center.y - frame_size.height / 2.0f;

        // 使用低通滤波器平滑误差,减少抖动
        filtered_x_error_ =
            Params::LOW_PASS_FILTER_ALPHA * x_error + (1 - Params::LOW_PASS_FILTER_ALPHA) * filtered_x_error_;
        filtered_y_error_ =
            Params::LOW_PASS_FILTER_ALPHA * y_error + (1 - Params::LOW_PASS_FILTER_ALPHA) * filtered_y_error_;

        // 计算两次调用之间的时间差 (dt)
        int64_t now_us = esp_timer_get_time();
        double dt_sec = (last_pid_time_us_ == 0) ? 0.02 : (now_us - last_pid_time_us_) / 1000000.0;
        dt_sec = (dt_sec <= 0) ? 0.000001 : dt_sec; // 防止dt为0
        last_pid_time_us_ = now_us;

        x_controller_->set_dt(dt_sec);
        y_controller_->set_dt(dt_sec);
        x_controller_->set_error(-filtered_x_error_); // x方向控制roll
        y_controller_->set_error(-filtered_y_error_); // y方向控制pitch
    }
    else
    {
        gpio_set_level(Pins::LED_B, 0);
        x_controller_->clear();
        y_controller_->clear();
        filtered_x_error_ = 0;
        filtered_y_error_ = 0;
    }
}

void Robot::log_performance()
{
    frame_count_++;
    if (frame_count_ % 100 == 0)
    {
        int64_t current_time = esp_timer_get_time();
        float fps = 100.0f / ((current_time - last_fps_time_us_) / 1000000.0f);
        ESP_LOGI(TAG, "FPS: %.2f", fps);
#ifdef ENABLE_TIMING_PROFILE
        profiler_.log_results(TAG, 100);
        profiler_.reset();
#endif
        last_fps_time_us_ = current_time;
    }
}

void Robot::control_loop_iteration()
{
    loop_counter_++;
#ifdef ENABLE_TIMING_PROFILE
    profiler_.start();
#endif
    gpio_set_level(Pins::LED_A, loop_counter_ % 2);
#ifdef ENABLE_TIMING_PROFILE
    profiler_.end("LED");
    profiler_.start();
#endif

    cv::Mat grayscale_img = CameraModule::get_grayscale_frame(10);

#ifdef ENABLE_TIMING_PROFILE
    profiler_.end("解码");
#endif

    if (grayscale_img.empty())
    {
        return;
    }

    this->log_performance();

#ifdef ENABLE_TIMING_PROFILE
    profiler_.start();
#endif
    ImageDetector::Circle ball = this->detect_ball(grayscale_img);
#ifdef ENABLE_TIMING_PROFILE
    profiler_.end("检测");
    profiler_.start();
#endif

    this->update_pid_controllers(ball, grayscale_img.size());

#ifdef ENABLE_TIMING_PROFILE
    profiler_.end("PID计算");
    profiler_.start();
#endif

    KinematicsSolver::ServoAngles angles = KinematicsSolver::move_platform(
        y_controller_->get_adjustment(), x_controller_->get_adjustment(), Params::KINEMATICS_DEFAULT_HEIGHT);

#ifdef ENABLE_TIMING_PROFILE
    profiler_.end("运动学求解");
    profiler_.start();
#endif

    this->apply_servo_angles({angles.angle_a, angles.angle_b, angles.angle_c});

#ifdef ENABLE_TIMING_PROFILE
    profiler_.end("舵机控制");
#endif
}

void Robot::run_realtime_control()
{
    if (!is_initialized_)
    {
        ESP_LOGE(TAG, "机器人未初始化，无法运行");
        return;
    }
    last_fps_time_us_ = esp_timer_get_time();
    while (true)
    {
        this->control_loop_iteration();
    }
}

void Robot::run_streaming()
{
    if (!is_initialized_)
    {
        ESP_LOGE(TAG, "机器人未初始化，无法运行");
        return;
    }

    StreamServer::init_wifi_and_start_server(WIFI_SSID, WIFI_PASSWORD, SERVER_PORT);

    while (true)
    {
        if (!StreamServer::is_client_connected())
        {
            ESP_LOGI(TAG, "等待客户端连接...");
            vTaskDelay(pdMS_TO_TICKS(500));
            continue;
        }

        cv::Mat frame = CameraModule::get_grayscale_frame(100);
        if (frame.empty())
        {
            continue;
        }

        cv::Mat display_frame;
        cv::cvtColor(frame, display_frame, cv::COLOR_GRAY2BGR);

        ImageDetector::Circle ball = this->detect_ball(frame);
        if (ball.found)
        {
            cv::circle(display_frame, ball.center, ball.radius, cv::Scalar(0, 255, 0), 2);
            cv::circle(display_frame, ball.center, 2, cv::Scalar(0, 0, 255), -1);
        }

        if (!StreamServer::send_image(display_frame))
        {
            ESP_LOGW(TAG, "发送图像失败,可能连接已断开");
            vTaskDelay(pdMS_TO_TICKS(1000)); // 等待套接字清理
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(50)); // 限制帧率
    }
}