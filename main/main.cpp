#include "esp_log.h"
#include <cmath> // 用于 sin/cos
#include <cstdio>

// FreeRTOS
#include "esp_random.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 引入项目模块
#include "camera_module.hpp"
#include "stream_server.hpp"
#include "servo_controller.hpp"
#include "kinematics_solver.hpp"
#include "image_detector.hpp"


// --- Wi-Fi凭证 ---
const char* ssid = "Fuwaki's";
const char* password = "114514qwq";
// -------------------

#define SERVER_PORT 8080
static const char *TAG = "主程序";


// --- 舵机和平台控制 ---
// 根据Python代码定义舵机对象，GPIO引脚为 1, 2, 3
// ServoController::Servo servo_a(GPIO_NUM_1, LEDC_TIMER_0, LEDC_CHANNEL_0);
// ServoController::Servo servo_b(GPIO_NUM_2, LEDC_TIMER_0, LEDC_CHANNEL_1);
// ServoController::Servo servo_c(GPIO_NUM_3, LEDC_TIMER_0, LEDC_CHANNEL_2);

// 根据Python代码定义LED引脚
const gpio_num_t led_a_pin = GPIO_NUM_7;
const gpio_num_t led_b_pin = GPIO_NUM_15;

void test(){
  ImageDetector detector;
    detector.test_all();
    ESP_LOGI(TAG, "开始图像检测测试...");
    detector.detect_from_input();
    ESP_LOGI(TAG, "图像检测测试完成。");
}

extern "C" void app_main()
{
    // --- 初始化硬件 ---
    // gpio_set_direction(led_a_pin, GPIO_MODE_OUTPUT);
    // gpio_set_direction(led_b_pin, GPIO_MODE_OUTPUT);
    // gpio_set_level(led_a_pin, 1);
    // gpio_set_level(led_b_pin, 1);

    // // 初始化摄像头
    // if (CameraModule::init() != ESP_OK) {
    //     ESP_LOGE(TAG, "摄像头初始化失败，程序停止。");
    //     return;
    // }

    // 初始化WiFi并启动流服务器
    //StreamServer::init_wifi_and_start_server(ssid, password, SERVER_PORT);

    // 舵机对象在构造时已自动 attach，此处延时等待舵机初始化完成
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    ESP_LOGI(TAG, "初始化完成，开始主循环。");

    // --- 主循环 ---
    float a = 0.0f;
    int b = 0;
    test();
    // while(true) {
    //     // 记录循环开始时间
    //     TickType_t start_tick = xTaskGetTickCount();

    //     a += 0.1f;
    //     b++;
    //     gpio_set_level(led_a_pin, b % 2);

    //     // 1. 计算期望的平台姿态
    //     float current_pitch = 20.0f * cos(a);
    //     float current_roll = 20.0f * sin(a);

    //     // 2. 调用运动学解算器获取舵机角度
    //     KinematicsSolver::ServoAngles angles = KinematicsSolver::move_platform(20.0, -10.0, 60.0f);

    //     // // 3. 驱动舵机到目标角度
    //     // servo_a.change_angle(angles.angle_a);
    //     // servo_b.change_angle(angles.angle_b);
    //     // servo_c.change_angle(angles.angle_c);
    //     printf("%f %f %f\n", angles.angle_a, angles.angle_b, angles.angle_c);

    //     // 记录循环结束时间
    //     TickType_t end_tick = xTaskGetTickCount();
    //     float elapsed_ms = (end_tick - start_tick) * portTICK_PERIOD_MS;
    //     printf("Loop time: %.4f ms\n", elapsed_ms);

    //     // 延时20毫秒，与 time.sleep(0.02) 对应
    //     vTaskDelay(20 / portTICK_PERIOD_MS);
    // }
}