#include "esp_log.h"
#include <cmath> // 用于 sin/cos
#include <cstdio>
#include <cstring> // for memcpy

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

// --- SIMD 测试相关 ---
// 声明汇编函数
extern "C" {
  int s3_add16x8(uint16_t *pA, uint16_t *pB, uint16_t *pC, uint8_t sar);
}

const int ARRAY_SIZE = 1024;

// 标准C版本函数
void c_add16x8(uint16_t *pA, uint16_t *pB, uint16_t *pC, uint8_t sar, int size) {
    for(int i = 0; i < size; i++) {
        pC[i] = (pA[i] * pB[i % 8]) >> sar;
    }
}

// 全局变量
uint16_t __attribute__((aligned (16))) u16_A[ARRAY_SIZE];
uint16_t __attribute__((aligned (16))) u16_B[8] = {64,64,64,64,128,128,128,128};
uint16_t __attribute__((aligned (16))) u16_C[ARRAY_SIZE];
uint8_t sar = 0;

void test(){
  ImageDetector::test_all();
    ESP_LOGI(TAG, "开始图像检测测试...");
    ImageDetector::detect_from_input();
    ESP_LOGI(TAG, "图像检测测试完成。");
}

// SIMD 测试函数
void test_simd() {
    ESP_LOGI(TAG, "开始SIMD测试...");

    // 初始化u16_A数组（重复原来的值）
    uint16_t original[] = {26, 225, 38, 21, 42, 217, 140, 102, 213, 15, 196, 94, 47, 176, 165, 233, 84, 211, 115, 241, 129, 207, 117, 52, 255, 48, 244, 149, 44, 10, 134, 96, 80, 23, 206, 119, 4, 164, 242, 154, 203, 158, 142, 79, 249, 118, 19, 82, 130, 128, 192, 5, 201, 222, 200, 83, 24, 191, 41, 66, 111, 193, 220, 147, 194, 236, 37, 181, 125, 182, 124, 252, 122, 180, 169, 74, 59, 174, 20, 146, 30, 95, 110, 13, 178, 32, 195, 152, 11, 27, 145, 224, 9, 229, 109, 148, 251, 72, 153, 136, 86, 199, 92, 55, 239, 25, 1, 114, 104, 230, 198, 131, 246, 168, 76, 159, 101, 113, 250, 187, 135, 71, 34, 179, 6, 57, 18, 184};
    int original_size = sizeof(original) / sizeof(original[0]);
    for(int i = 0; i < ARRAY_SIZE; i++) {
        u16_A[i] = original[i % original_size];
    }

    // 复制u16_A到u16_C用于C版本
    memcpy(u16_C, u16_A, sizeof(u16_A));

    // 测试SIMD版本
    int64_t start_time = esp_timer_get_time();
    s3_add16x8(u16_A, u16_B, u16_A, sar);
    int64_t end_time = esp_timer_get_time();
    int64_t simd_time = end_time - start_time;

    // 测试C版本
    start_time = esp_timer_get_time();
    c_add16x8(u16_C, u16_B, u16_C, sar, ARRAY_SIZE);
    end_time = esp_timer_get_time();
    int64_t c_time = end_time - start_time;

    ESP_LOGI(TAG, "SIMD时间: %ld 微秒", (long)simd_time);
    ESP_LOGI(TAG, "C语言时间: %ld 微秒", (long)c_time);
    ESP_LOGI(TAG, "速度提升: %.2f 倍", (float)c_time / (float)simd_time);

    // 打印一些结果
    for (int i = 0; i < 10; i++) {
        printf("SIMD: %3d * %3d >> %d = %5d | C: %5d\n", u16_A[i], u16_B[i%8], sar, u16_A[i], u16_C[i]);
    }

    sar++;
    if (sar > 8) sar = 0;
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
    test_simd();
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