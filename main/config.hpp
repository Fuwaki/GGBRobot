#pragma once
#include "driver/gpio.h"
#include "driver/ledc.h"

// =================================================================
// 应用程序核心配置
// =================================================================

// --- 操作模式 ---
// 选择一个输出目标:
// - OUTPUT_REALTIME_CONTROL: 实时闭环控制机械臂
// - OUTPUT_STREAM:           通过Wi-Fi将视频和检测结果流式传输到客户端
#define OUTPUT_REALTIME_CONTROL 1
#define OUTPUT_STREAM 2
#define OUTPUT_TARGET OUTPUT_STREAM

// --- 性能分析 ---
// 启用此项以在日志中打印每个处理步骤的执行时间
#define ENABLE_TIMING_PROFILE

// =================================================================
// 网络配置 (仅在 OUTPUT_TARGET == OUTPUT_STREAM 时使用)
// =================================================================
#define WIFI_SSID "stdio.h"
#define WIFI_PASSWORD "1145141919810"
#define SERVER_PORT 8080

// =================================================================
// 硬件引脚定义
// =================================================================
namespace Pins {
    // --- LED指示灯 ---
    const gpio_num_t LED_A = GPIO_NUM_7;
    const gpio_num_t LED_B = GPIO_NUM_15;

    // --- 舵机 ---
    const gpio_num_t SERVO_A = GPIO_NUM_1;
    const gpio_num_t SERVO_B = GPIO_NUM_2;
    const gpio_num_t SERVO_C = GPIO_NUM_3;
}

// =================================================================
// 控制与算法参数
// =================================================================
namespace Params {
    // --- PID控制器参数 ---
    const double PID_P = 0.20;
    const double PID_I = 0.001;
    const double PID_D = 0.02;
    const float PID_OUTPUT_LIMIT_MIN = -30.0f;
    const float PID_OUTPUT_LIMIT_MAX = 30.0f;

    // --- 舵机控制参数 ---
    // 舵机零点偏移补偿（单位：度），根据实际机械结构进行微调
    const float SERVO_A_COMPENSATION = 66.0f;
    const float SERVO_B_COMPENSATION = 85.0f;
    const float SERVO_C_COMPENSATION = 40.0f;

    // --- 运动学参数 ---
    const float KINEMATICS_DEFAULT_HEIGHT = 70.0f; // 平台默认高度
    // 平台几何尺寸 (单位: mm)
    const float KINEMATICS_BASE_RADIUS = 32.9f / 1.73205f; // 基础平台半径 (32.9 / sqrt(3))
    const float KINEMATICS_SERVO_ARM_LENGTH = 50.0f;      // 舵机摇臂长度
    const float KINEMATICS_LINK_LENGTH = 39.2f;           // 连杆长度

    // --- 图像处理参数 ---
    const uint8_t BINARIZE_THRESHOLD = 80;

    // --- 目标检测参数 ---
    const float DETECT_MIN_AREA = 1000.0f;
    const float DETECT_MAX_AREA = 6000.0f;
    const float DETECT_MIN_CIRCULARITY = 0.3f;
    const float DETECT_MIN_RADIUS = 20.0f;
    const float DETECT_MAX_RADIUS = 50.0f;
    
    // --- 滤波器参数 ---
    const float LOW_PASS_FILTER_ALPHA = 0.2f; // 低通滤波器系数，用于平滑误差
}
