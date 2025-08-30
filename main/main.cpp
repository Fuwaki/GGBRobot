/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include <cmath>
#include <cstdio>
#include <cstring>

// FreeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 引入项目模块
#include "stream_server.hpp"
#include "image_detector.hpp"
#include "camera_module.hpp"

// --- Wi-Fi凭证 ---
const char* ssid = "Fuwaki's";
const char* password = "114514qwq";
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
    params.selection_method = ImageDetector::ContourFindParams::Selection::LARGEST_AREA;

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
        ImageDetector::GroundTruth gt; // GroundTruth仍会生成，但我们不再直接使用它
        cv::Mat img = ImageDetector::create_test_image(IMG_WIDTH, IMG_HEIGHT, gt);

        // 2. 对图像运行真实的轮廓检测
        ImageDetector::Circle result = ImageDetector::find_ball_by_contour(img, params);

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

// 实时检测与推流任务
void realtime_detection_task(void* pvParameters) {
    // 1. 为模板匹配创建模板 (一个理想的圆形)
    const int TEMPLATE_RADIUS = 20; // 模板半径
    cv::Mat templ(TEMPLATE_RADIUS * 2 + 1, TEMPLATE_RADIUS * 2 + 1, CV_8UC1);
    templ.setTo(cv::Scalar(255)); // 白色背景
    cv::circle(templ, cv::Point(TEMPLATE_RADIUS, TEMPLATE_RADIUS), TEMPLATE_RADIUS, cv::Scalar(0), -1); // 黑色实心圆

    const double CONFIDENCE_THRESHOLD = 0.7; // 置信度阈值

    while (1) {
        // 2. 等待客户端连接
        if (!StreamServer::is_client_connected()) {
            ESP_LOGI(TAG, "等待客户端连接...");
            while (!StreamServer::is_client_connected()) {
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            ESP_LOGI(TAG, "客户端已连接，开始发送。");
        }

        // 3. 获取摄像头帧
        camera_fb_t *fb = CameraModule::get_frame();
        if (!fb) {
            ESP_LOGE(TAG, "获取摄像头帧失败");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }

        // 4. 直接从帧缓冲区创建灰度图像的Mat
        cv::Mat grayscale_img(fb->height, fb->width, CV_8UC1, fb->buf);

        // 5. 运行模板匹配
        cv::Rect search_window(0, 0, grayscale_img.cols, grayscale_img.rows);
        ImageDetector::MatchResult match = ImageDetector::find_ball_by_template(grayscale_img, templ, search_window, CONFIDENCE_THRESHOLD);

        // 6. 将匹配结果转换为Circle结构以进行流式传输
        ImageDetector::Circle result;
        result.found = match.found;
        if (match.found) {
            result.center.x = match.box.x + match.box.width / 2.0f;
            result.center.y = match.box.y + match.box.height / 2.0f;
            result.radius = (match.box.width + match.box.height) / 4.0f; // 平均半径
        } else {
            result.center.x = 0;
            result.center.y = 0;
            result.radius = 0;
        }

        // 7. 发送灰度图像
        if (!StreamServer::send_image(grayscale_img)) {
            ESP_LOGE(TAG, "发送图像失败，连接可能已断开。");
            CameraModule::return_frame(fb);
            vTaskDelay(pdMS_TO_TICKS(1100));
            continue;
        }

        // 8. 发送检测结果
        if (!StreamServer::send_detection_result(result)) {
            ESP_LOGE(TAG, "发送检测结果失败。");
        }

        // 9. 归还帧缓冲区
        CameraModule::return_frame(fb);

        // 10. 短暂延时
        vTaskDelay(pdMS_TO_TICKS(100)); // 约10FPS
    }
}


extern "C" void app_main()
{
    // 添加 sizeof 调试信息
    // ESP_LOGI(TAG, "SIZEOF CHECK: sizeof(ImageDetector::Circle) = %d bytes", sizeof(ImageDetector::Circle));

    ESP_LOGI(TAG, "程序启动，初始化中...");

    // 初始化摄像头
    esp_err_t cam_err = CameraModule::init();
    if (cam_err != ESP_OK) {
        ESP_LOGE(TAG, "摄像头初始化失败，错误代码: 0x%x", cam_err);
        return; // 摄像头失败，无法继续
    }
    ESP_LOGI(TAG, "摄像头初始化成功。");

    // 初始化WiFi并启动流服务器
    StreamServer::init_wifi_and_start_server(ssid, password, SERVER_PORT);

    ESP_LOGI(TAG, "初始化完成，创建实时检测任务。");

    // 创建发送测试图像的任务
    // xTaskCreate(stream_test_images_task, "stream_test_task", 16000, NULL, 5, NULL);
    
    // 创建实时检测任务
    xTaskCreate(realtime_detection_task, "realtime_detection_task", 16000, NULL, 5, NULL);
}
