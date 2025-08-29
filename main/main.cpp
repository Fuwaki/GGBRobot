#include <opencv2/opencv.hpp>

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




// --- Wi-Fi凭证 ---
const char* ssid = "Fuwaki's";
const char* password = "114514qwq";
// -------------------

#define SERVER_PORT 8080
static const char *TAG = "主程序";
using namespace cv;



// --- 舵机和平台控制 ---
// 根据Python代码定义舵机对象，GPIO引脚为 1, 2, 3
// ServoController::Servo servo_a(GPIO_NUM_1, LEDC_TIMER_0, LEDC_CHANNEL_0);
// ServoController::Servo servo_b(GPIO_NUM_2, LEDC_TIMER_0, LEDC_CHANNEL_1);
// ServoController::Servo servo_c(GPIO_NUM_3, LEDC_TIMER_0, LEDC_CHANNEL_2);

// 根据Python代码定义LED引脚
const gpio_num_t led_a_pin = GPIO_NUM_7;
const gpio_num_t led_b_pin = GPIO_NUM_15;
struct GroundTruth {
    int cx, cy, r;
};

/* 生成一张带已知真值圆的单通道图 */
static Mat create_test_image(int w, int h, GroundTruth& gt)
{
    Mat img(h, w, CV_8UC1, Scalar(0));

    /* 真值坐标固定用当前随机种子，保证后续能复现 */
    gt.cx = w * (300 + esp_random() % 400) / 1000;
    gt.cy = h * (300 + esp_random() % 400) / 1000;
    gt.r  = 15 + esp_random() % 25;

    circle(img, Point(gt.cx, gt.cy), gt.r, Scalar(255), -1);
    return img;
}

/* 1. findContours + minEnclosingCircle */
void test_findContours()
{
    const int W = 160, H = 120, N = 100;
    int ok = 0;
    int64_t t_total = 0;

    for (int i = 0; i < N; ++i) {
        GroundTruth gt;
        Mat img = create_test_image(W, H, gt);

        Mat bin;
        threshold(img, bin, 0, 255, THRESH_BINARY | THRESH_OTSU);

        int64_t t0 = esp_timer_get_time();
        std::vector<std::vector<Point>> cnts;
        findContours(bin, cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        bool hit = false;
        for (auto& c : cnts) {
            Point2f center; float r;
            minEnclosingCircle(c, center, r);
            if (norm(center - Point2f(gt.cx, gt.cy)) < 3 && fabs(r - gt.r) < 3) {
                hit = true; break;
            }
        }
        t_total += esp_timer_get_time() - t0;
        ok += hit;
    }

    float avg_ms = t_total / 1000.0f / N;
    ESP_LOGI(TAG, "findContours: %.1f fps, 准确率 %.1f%%",
             1000.0f / avg_ms, 100.0f * ok / N);
}



void test_houghCircles()
{
    const int W = 160, H = 120, N = 100;
    int ok = 0;
    int64_t t_total = 0;

    for (int i = 0; i < N; ++i) {
        GroundTruth gt;
        Mat img = create_test_image(W, H, gt);

        int64_t t0 = esp_timer_get_time();
        std::vector<Vec3f> circles;
        HoughCircles(img, circles, HOUGH_GRADIENT, 1, 80, 100, 12, 10, 50);

        bool hit = false;
        for (auto& c : circles) {
            Point2f center(c[0], c[1]);
            float r = c[2];
            if (norm(center - Point2f(gt.cx, gt.cy)) < 3 && fabs(r - gt.r) < 3) {
                hit = true; break;
            }
        }
        t_total += esp_timer_get_time() - t0;
        ok += hit;
    }

    float avg_ms = t_total / 1000.0f / N;
    ESP_LOGI(TAG, "HoughCircles: %.1f fps, 准确率 %.1f%%",
             1000.0f / avg_ms, 100.0f * ok / N);
}
void test(){
  test_findContours();
  test_houghCircles();
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
