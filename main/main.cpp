#include "benchmark.hpp"
#include "config.hpp"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "image_detector.hpp"
#include "robot.hpp"
#include <memory>

static const char *TAG = "主程序";

// 使用智能指针管理全局机器人实例
static std::unique_ptr<Robot> robot_instance;

/**
 * @brief FreeRTOS 任务函数, 运行机器人的主循环
 * @param pvParameters 任务参数(未使用)
 */
void robot_task(void *pvParameters)
{
    ESP_LOGI(TAG, "机器人任务已启动。");

    // 根据配置选择运行模式
#if OUTPUT_TARGET == OUTPUT_REALTIME_CONTROL
    robot_instance->run_realtime_control();
#elif OUTPUT_TARGET == OUTPUT_STREAM
    robot_instance->run_streaming();
#else
#error "无效的 OUTPUT_TARGET, 请在 config.hpp 中选择一个有效的模式。"
#endif

    // 理论上不应该执行到这里
    ESP_LOGE(TAG, "机器人任务意外退出。");
    vTaskDelete(NULL);
}

/**
 * @brief 应用程序主入口
 */
extern "C" void app_main()
{
    // 初始化板载LED作为状态指示灯
    gpio_set_direction(Pins::LED_A, GPIO_MODE_OUTPUT);
    gpio_set_direction(Pins::LED_B, GPIO_MODE_OUTPUT);
    gpio_set_level(Pins::LED_A, 1); // LED_A 亮, 表示正在初始化
    gpio_set_level(Pins::LED_B, 0);

    ESP_LOGI(TAG, "应用程序启动, 开始初始化...");

    // 运行性能测试
    //     Benchmark::run_matrix_benchmark();
    // 运行内置图像检测测试
    ImageDetector::test_all();
    ImageDetector::detect_from_input();

    //     // 创建并初始化机器人实例
    //     robot_instance = std::make_unique<Robot>();
    //     esp_err_t init_status = robot_instance->init();

    //     if (init_status != ESP_OK) {
    //         ESP_LOGE(TAG, "机器人初始化失败, 程序将中止。");
    //         // 使用LED指示错误状态 (例如, 闪烁)
    //         while(true) {
    //             gpio_set_level(Pins::LED_A, 1);
    //             vTaskDelay(pdMS_TO_TICKS(200));
    //             gpio_set_level(Pins::LED_A, 0);
    //             vTaskDelay(pdMS_TO_TICKS(200));
    //         }
    //         return;
    //     }

    //     ESP_LOGI(TAG, "机器人初始化成功, 正在创建主任务...");
    //     gpio_set_level(Pins::LED_A, 0); // LED_A 灭, 表示初始化完成

    //     // 创建主任务, 核心堆栈大小增加到16KB以适应OpenCV和网络需求
    //     xTaskCreate(robot_task, "robot_task", 16 * 1024, NULL, 5, NULL);
}
