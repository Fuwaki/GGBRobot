/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 引入项目模块
#include "camera_module.hpp"
#include "image_detector.hpp"
#include "image_processor.hpp"
#include "kinematics_solver.hpp"
#include "servo_controller.hpp"
#include "stream_server.hpp"
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
// void realtime_detection_task(void* pvParameters) {
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

//     while (1) {
//         // 等待客户端连接
//         if (!StreamServer::is_client_connected()) {
//             ESP_LOGI(TAG, "等待客户端连接...");
//             while (!StreamServer::is_client_connected()) {
//                 vTaskDelay(pdMS_TO_TICKS(200));
//             }
//             ESP_LOGI(TAG, "客户端已连接，开始发送。");
//         }

//         // 获取摄像头帧
//         camera_fb_t *fb = CameraModule::get_frame();
//         if (!fb) {
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
//         params.selection_method =
//         ImageDetector::ContourFindParams::Selection::CLOSEST_TO_POINT;
//         params.expected_pos = cv::Point2f(grayscale_img.cols / 2.0f,
//         grayscale_img.rows / 2.0f); // 假设期望位置为中心 params.min_area =
//         500; params.max_area = 5000; params.min_circularity = 0.5;
//         params.min_radius = 20;
//         params.max_radius = 50;

//         // 调用轮廓检测
//         ImageDetector::Circle result =
//         ImageDetector::find_ball_by_contour(grayscale_img, params);

//         // 发送二值化后的图像
//         if (!StreamServer::send_image(grayscale_img)) {
//             ESP_LOGE(TAG, "发送图像失败，连接可能已断开。");
//             CameraModule::return_frame(fb);
//             vTaskDelay(pdMS_TO_TICKS(1100));
//             continue;
//         }

//         // 发送真实的检测结果
//         if (!StreamServer::send_detection_result(result)) {
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

    float compensations[3] = {70, 60, 40};
    servo_a.change_angle(angles[0] + compensations[0]);
    servo_b.change_angle(angles[1] + compensations[1]);
    servo_c.change_angle(angles[2] + compensations[2]);
}
void control()
{
    float a = 0.0f;
    int b = 0;
    while (true)
    {
        // 记录循环开始时间
        TickType_t start_tick = xTaskGetTickCount();

        a += 0.002f;
        b++;
        gpio_set_level(led_a_pin, b % 2);

        // 1. 计算期望的平台姿态
        float current_pitch = 10.0f * cos(a);
        float current_roll = 10.0f * sin(a);

        // 2. 调用运动学解算器获取舵机角度
        KinematicsSolver::ServoAngles angles = KinematicsSolver::move_platform(current_pitch, current_roll, 60.0f);

        // // 3. 驱动舵机到目标角度
        apply_servo({angles.angle_a, angles.angle_b, angles.angle_c});
        printf("%f %f %f\n", angles.angle_a, angles.angle_b, angles.angle_c);

        // 记录循环结束时间
        TickType_t end_tick = xTaskGetTickCount();
        float elapsed_ms = (end_tick - start_tick) * portTICK_PERIOD_MS;
        printf("Loop time: %.4f ms\n", elapsed_ms);

        // 延时20毫秒，与 time.sleep(0.02) 对应
        vTaskDelay(1 / portTICK_PERIOD_MS);
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

    // 创建发送测试图像的任务
    // xTaskCreate(stream_test_images_task, "stream_test_task", 16000, NULL, 5,
    // NULL);

    // 创建实时检测任务
    // xTaskCreate(realtime_detection_task, "realtime_detection_task", 16000,
    // NULL, 5, NULL);
    control();
}
