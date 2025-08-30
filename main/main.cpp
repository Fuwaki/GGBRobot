/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "driver/gpio.h"
#include "esp_log.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

// FreeRTOS
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 引入项目模块
#include "camera_module.hpp"
#include "image_detector.hpp"
#include "image_processor.hpp"
#include "kinematics_solver.hpp"
#include "servo_controller.hpp"
#include "stream_server.hpp"

#include "PID.hpp"

// --- Wi-Fi凭证 ---
const char *ssid = "Fuwaki's";
const char *password = "114514qwq";
// -------------------

#define SERVER_PORT 8080
static const char *TAG = "主程序";

/*
// 新的测试任务，用于生成并发送图像
void stream_test_images_task(void* pvParameters) {
    const int IMG_WIDTH = 160;
    const int IMG_HEIGHT = 120;

    // 配置轮廓检测参数
    ImageDetector::ContourFindParams params;
    params.selection_method =
ImageDetector::ContourFindParams::Selection::LARGEST_AREA;

    while (1) {
        // 等待客户端连接
        if (!StreamServer::is_client_connected()) {
            ESP_LOGI(TAG, "等待客户端连接...");
            while (!StreamServer::is_client_connected()) {
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            ESP_LOGI(TAG, "客户端已连接，开始发送。");
        }

        // 1. 创建一个带有一个圆形的测试图像
        ImageDetector::GroundTruth gt; //
GroundTruth仍会生成，但我们不再直接使用它 cv::Mat img =
ImageDetector::create_test_image(IMG_WIDTH, IMG_HEIGHT, gt);

        // 2. 对图像运行真实的轮廓检测
        ImageDetector::Circle result = ImageDetector::find_ball_by_contour(img,
params);

        // 3. 发送图像
        if (!StreamServer::send_image(img)) {
            ESP_LOGE(TAG, "发送图像失败，连接可能已断开。");
            // 等待后台任务清理套接字，避免刷屏错误
            vTaskDelay(pdMS_TO_TICKS(1100));
            continue;
        }

        // 4. 发送真实的检测结果
        if (!StreamServer::send_detection_result(result)) {
            ESP_LOGE(TAG, "发送检测结果失败。");
        }

        // 5. 延时
        vTaskDelay(pdMS_TO_TICKS(100)); // 每秒发送10帧
    }
}
*/

// // 实时检测与推流任务
// void realtime_detection_task(void *pvParameters)
// {
//     /*
//     // 1. 为模板匹配创建模板 (一个理想的圆形)
//     const int TEMPLATE_RADIUS = 20; // 模板半径
//     cv::Mat templ(TEMPLATE_RADIUS * 2 + 1, TEMPLATE_RADIUS * 2 + 1, CV_8UC1);
//     templ.setTo(cv::Scalar(255)); // 白色背景
//     cv::circle(templ, cv::Point(TEMPLATE_RADIUS, TEMPLATE_RADIUS),
//     TEMPLATE_RADIUS, cv::Scalar(0), -1); // 黑色实心圆

//     const double CONFIDENCE_THRESHOLD = 0.7; // 置信度阈值
//     */

//     const uint8_t threshold = 70; // 二值化阈值

//     while (1)
//     {
//         // 等待客户端连接
//         if (!StreamServer::is_client_connected())
//         {
//             ESP_LOGI(TAG, "等待客户端连接...");
//             while (!StreamServer::is_client_connected())
//             {
//                 vTaskDelay(pdMS_TO_TICKS(200));
//             }
//             ESP_LOGI(TAG, "客户端已连接，开始发送。");
//         }

//         // 获取摄像头帧
//         camera_fb_t *fb = CameraModule::get_frame();
//         if (!fb)
//         {
//             ESP_LOGE(TAG, "获取摄像头帧失败");
//             vTaskDelay(pdMS_TO_TICKS(1000));
//             continue;
//         }

//         // 直接从帧缓冲区创建灰度图像的Mat
//         cv::Mat grayscale_img(fb->height, fb->width, CV_8UC1, fb->buf);

//         /*
//         // 5. 运行模板匹配
//         cv::Rect search_window(0, 0, grayscale_img.cols, grayscale_img.rows);
//         ImageDetector::MatchResult match =
//         ImageDetector::find_ball_by_template(grayscale_img, templ,
//         search_window, CONFIDENCE_THRESHOLD);

//         // 6. 将匹配结果转换为Circle结构以进行流式传输
//         ImageDetector::Circle result;
//         result.found = match.found;
//         if (match.found) {
//             result.center.x = match.box.x + match.box.width / 2.0f;
//             result.center.y = match.box.y + match.box.height / 2.0f;
//             result.radius = (match.box.width + match.box.height) / 4.0f; //
//             平均半径
//         } else {
//             result.center.x = 0;
//             result.center.y = 0;
//             result.radius = 0;
//         }

//         // 7. 发送灰度图像
//         if (!StreamServer::send_image(grayscale_img)) {
//             ESP_LOGE(TAG, "发送图像失败，连接可能已断开。");
//             CameraModule::return_frame(fb);
//             vTaskDelay(pdMS_TO_TICKS(1100));
//             continue;
//         }

//         // 8. 发送检测结果
//         if (!StreamServer::send_detection_result(result)) {
//             ESP_LOGE(TAG, "发送检测结果失败。");
//         }
//         */

//         // 对图像进行二值化
//         ImageProcessor::binarize(grayscale_img, threshold);

//         // 设置轮廓检测参数
//         ImageDetector::ContourFindParams params;
//         params.selection_method = ImageDetector::ContourFindParams::Selection::CLOSEST_TO_POINT;
//         params.expected_pos = cv::Point2f(grayscale_img.cols / 2.0f, grayscale_img.rows / 2.0f);
//         // 假设期望位置为中心 params.min_area =500;
//         // params.max_area = 5000; params.min_circularity = 0.5;
//         params.min_radius = 20;
//         params.max_radius = 50;

//         // 调用轮廓检测
//         ImageDetector::Circle result = ImageDetector::find_ball_by_contour(grayscale_img, params);

//         // 发送二值化后的图像
//         if (!StreamServer::send_image(grayscale_img))
//         {
//             ESP_LOGE(TAG, "发送图像失败，连接可能已断开。");
//             CameraModule::return_frame(fb);
//             vTaskDelay(pdMS_TO_TICKS(1100));
//             continue;
//         }

//         // 发送真实的检测结果
//         if (!StreamServer::send_detection_result(result))
//         {
//             ESP_LOGE(TAG, "发送检测结果失败。");
//         }

//         // 归还帧缓冲区
//         CameraModule::return_frame(fb);
//     }
// }

// 根据Python代码定义LED引脚
const gpio_num_t led_a_pin = GPIO_NUM_7;
const gpio_num_t led_b_pin = GPIO_NUM_15;
void apply_servo(std::array<float, 3> angles)
{
    static ServoController::Servo servo_a(GPIO_NUM_1, LEDC_TIMER_0, LEDC_CHANNEL_0);
    static ServoController::Servo servo_b(GPIO_NUM_2, LEDC_TIMER_0, LEDC_CHANNEL_1);
    static ServoController::Servo servo_c(GPIO_NUM_3, LEDC_TIMER_0, LEDC_CHANNEL_2);

    float compensations[3] = {66, 85, 40};
    servo_a.change_angle(angles[2] + compensations[0]);
    servo_b.change_angle(angles[1] + compensations[1]);
    servo_c.change_angle(angles[0] + compensations[2]);
}
void control()
{

    pid_positional_controller x_controller(0.20, 0.001, 0.02);
    pid_positional_controller y_controller(0.20, 0.001, 0.02);
    x_controller.enable_limit(-30, 30);
    y_controller.enable_limit(-30, 30);

    const uint8_t threshold = 80; // 二值化阈值
    uint32_t b = 0;
#define TIMING_PROFILE

#ifdef TIMING_PROFILE
    // 耗时统计变量
    static int64_t total_led_time = 0;
    static int64_t total_get_frame_time = 0;
    static int64_t total_create_mat_time = 0;
    static int64_t total_binarize_time = 0;
    static int64_t total_detect_time = 0;
    static int64_t total_return_frame_time = 0;
    static int64_t total_error_calc_time = 0;
    static int64_t total_kinematics_time = 0;
    static int64_t total_servo_time = 0;
#endif
    static int frame_count = 0;

    while (true)
    {
        b++;
        // 闪烁led
#ifdef TIMING_PROFILE
        int64_t start_led = esp_timer_get_time();
#endif
        gpio_set_level(led_a_pin, b % 2);
#ifdef TIMING_PROFILE
        int64_t led_time = esp_timer_get_time() - start_led;
        total_led_time += led_time;
#endif

        // 获取摄像头帧
#ifdef TIMING_PROFILE
        int64_t start_get_frame = esp_timer_get_time();
#endif
        camera_fb_t *fb = CameraModule::get_frame();
#ifdef TIMING_PROFILE
        int64_t get_frame_time = esp_timer_get_time() - start_get_frame;
        total_get_frame_time += get_frame_time;
#endif
        if (!fb)
        {
            // No new frame available yet, just continue to the next loop iteration.
            // This allows the PID controllers and servos to keep running smoothly.
            // Add a small delay to prevent busy-waiting.
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        frame_count++;
        static int64_t last_time = esp_timer_get_time();
        if (frame_count % 100 == 0)
        {
            int64_t current_time = esp_timer_get_time();
            float fps = 100.0f / ((current_time - last_time) / 1000000.0f);
            ESP_LOGI(TAG, "FPS: %.2f", fps);
#ifdef TIMING_PROFILE
            // 打印平均耗时
            ESP_LOGI(TAG,
                     "平均耗时 (us): LED: %lld, GetFrame: %lld, CreateMat: %lld, Binarize: %lld, Detect: %lld, "
                     "ReturnFrame: %lld, ErrorCalc: %lld, Kinematics: %lld, Servo: %lld",
                     (long long)(total_led_time / 100), (long long)(total_get_frame_time / 100),
                     (long long)(total_create_mat_time / 100), (long long)(total_binarize_time / 100),
                     (long long)(total_detect_time / 100), (long long)(total_return_frame_time / 100),
                     (long long)(total_error_calc_time / 100), (long long)(total_kinematics_time / 100),
                     (long long)(total_servo_time / 100));
            // 重置累加器
            total_led_time = 0;
            total_get_frame_time = 0;
            total_create_mat_time = 0;
            total_binarize_time = 0;
            total_detect_time = 0;
            total_return_frame_time = 0;
            total_error_calc_time = 0;
            total_kinematics_time = 0;
            total_servo_time = 0;
#endif
            last_time = current_time;
        }

        // 直接从帧缓冲区创建灰度图像的Mat
#ifdef TIMING_PROFILE
        int64_t start_create_mat = esp_timer_get_time();
#endif
        cv::Mat grayscale_img(fb->height, fb->width, CV_8UC1, fb->buf);
#ifdef TIMING_PROFILE
        int64_t create_mat_time = esp_timer_get_time() - start_create_mat;
        total_create_mat_time += create_mat_time;
#endif

        // 二值化
#ifdef TIMING_PROFILE
        int64_t start_binarize = esp_timer_get_time();
#endif
        ImageProcessor::binarize(grayscale_img, threshold);
#ifdef TIMING_PROFILE
        int64_t binarize_time = esp_timer_get_time() - start_binarize;
        total_binarize_time += binarize_time;
#endif

        // 设置轮廓检测参数
        ImageDetector::ContourFindParams params;
        params.selection_method = ImageDetector::ContourFindParams::Selection::CLOSEST_TO_POINT;
        params.expected_pos = cv::Point2f(grayscale_img.cols / 2.0f, grayscale_img.rows / 2.0f);
        // 假设期望位置为中心
        params.min_area = 1000;
        params.max_area = 6000;
        params.min_circularity = 0.3;
        params.min_radius = 20;
        params.max_radius = 50;

        // 调用轮廓检测
#ifdef TIMING_PROFILE
        int64_t start_detect = esp_timer_get_time();
#endif
        ImageDetector::Circle result = ImageDetector::find_ball_by_contour(grayscale_img, params);
#ifdef TIMING_PROFILE
        int64_t detect_time = esp_timer_get_time() - start_detect;
        total_detect_time += detect_time;
#endif

        // 归还帧缓冲区
#ifdef TIMING_PROFILE
        int64_t start_return_frame = esp_timer_get_time();
#endif
        CameraModule::return_frame(fb);
#ifdef TIMING_PROFILE
        int64_t return_frame_time = esp_timer_get_time() - start_return_frame;
        total_return_frame_time += return_frame_time;
#endif

        if (result.found)
        {
            gpio_set_level(led_b_pin, 1);
#ifdef TIMING_PROFILE
            int64_t start_error_calc = esp_timer_get_time();
#endif
            float x_error = result.center.x - grayscale_img.cols / 2;
            float y_error = result.center.y - grayscale_img.rows / 2;
            static float filtered_x_error = 0;
            static float filtered_y_error = 0;
            const float alpha = 0.2; // 滤波系数，0到1之间，越大滤波越平滑

            filtered_x_error = alpha * x_error + (1 - alpha) * filtered_x_error;
            filtered_y_error = alpha * y_error + (1 - alpha) * filtered_y_error;

            // 测量两次 set_error 调用之间的时间间隔，并传入 PID
            static int64_t last_set_error_time_us = 0;
            int64_t now_us = esp_timer_get_time();
            double dt_sec = 0.02; // 默认值，单位秒
            if (last_set_error_time_us != 0)
            {
                dt_sec = (now_us - last_set_error_time_us) / 1000000.0;
                if (dt_sec <= 0) // 防止异常值
                    dt_sec = 0.000001;
            }
            last_set_error_time_us = now_us;

            // 将测得的采样时间传给 PID 控制器
            x_controller.set_dt(dt_sec);
            y_controller.set_dt(dt_sec);

            // 只用滤波后的误差更新控制器，避免一次循环内重复调用导致微分项计算出错
            x_controller.set_error(-filtered_x_error);
            y_controller.set_error(-filtered_y_error);

            printf("误差 %f %f\n", x_error, y_error);
#ifdef TIMING_PROFILE
            int64_t error_calc_time = esp_timer_get_time() - start_error_calc;
            total_error_calc_time += error_calc_time;
#endif
        }
        else
        {
            gpio_set_level(led_b_pin, 0);
            x_controller.set_error(0);
            y_controller.set_error(0);
            x_controller.clear();
            y_controller.clear();
#ifdef TIMING_PROFILE
            total_error_calc_time += 0; // 或者不加，但为了平均，假设为0
#endif
        }

        // 2. 调用运动学解算器获取舵机角度
#ifdef TIMING_PROFILE
        int64_t start_kinematics = esp_timer_get_time();
#endif
        KinematicsSolver::ServoAngles angles =
            KinematicsSolver::move_platform(y_controller.get_adjustment(), x_controller.get_adjustment(), 70.0f);
#ifdef TIMING_PROFILE
        int64_t kinematics_time = esp_timer_get_time() - start_kinematics;
        total_kinematics_time += kinematics_time;
#endif

        // // 3. 驱动舵机到目标角度
#ifdef TIMING_PROFILE
        int64_t start_servo = esp_timer_get_time();
#endif
        apply_servo({angles.angle_a, angles.angle_b, angles.angle_c});
#ifdef TIMING_PROFILE
        int64_t servo_time = esp_timer_get_time() - start_servo;
        total_servo_time += servo_time;
#endif
        // printf("%f %f %f\n", angles.angle_a, angles.angle_b, angles.angle_c);

        // // 延时20毫秒，与 time.sleep(0.02) 对应
        // vTaskDelay(1 / portTICK_PERIOD_MS);
    }
}

extern "C" void app_main()
{
    gpio_set_direction(led_a_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(led_b_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(led_a_pin, 1);
    gpio_set_level(led_b_pin, 1);

    // 添加 sizeof 调试信息
    // ESP_LOGI(TAG, "SIZEOF CHECK: sizeof(ImageDetector::Circle) = %d bytes",
    // sizeof(ImageDetector::Circle));

    ESP_LOGI(TAG, "程序启动，初始化中...");

    // 初始化摄像头
    esp_err_t cam_err = CameraModule::init();
    if (cam_err != ESP_OK)
    {
        ESP_LOGE(TAG, "摄像头初始化失败，错误代码: 0x%x", cam_err);
        return; // 摄像头失败，无法继续
    }
    ESP_LOGI(TAG, "摄像头初始化成功。");

    // apply_servo({0.0, 0.0, 0.0});

    // 初始化WiFi并启动流服务器
    // StreamServer::init_wifi_and_start_server(ssid, password, SERVER_PORT);

    ESP_LOGI(TAG, "初始化完成，创建实时检测任务。");
    // KinematicsSolver::ServoAngles angles =
    //     KinematicsSolver::move_platform(0, 30, 60.0f);
    //     apply_servo({angles.angle_a, angles.angle_b, angles.angle_c});
    // 创建发送测试图像的任务
    // xTaskCreate(stream_test_images_task, "stream_test_task", 16000, NULL, 5,
    // NULL);

    // 创建实时检测任务
    // xTaskCreate(realtime_detection_task, "realtime_detection_task", 16000,
    // NULL, 5, NULL);
    control();
}
