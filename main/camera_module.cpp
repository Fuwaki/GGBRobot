#include "camera_module.hpp"
#include "driver/gpio.h"
#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "esp_dsp.h"
#include "esp_timer.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "sensor.h"
#include <esp_jpeg_dec.h>
#include <opencv2/opencv.hpp>

namespace CameraModule
{

static const char *TAG = "摄像头模块";

// WROVER-KIT 引脚定义 (根据您的硬件连接修改)
#define CAM_PIN_PWDN -1
#define CAM_PIN_RESET 4
#define CAM_PIN_XCLK -1
#define CAM_PIN_SIOD 17
#define CAM_PIN_SIOC 18

#define CAM_PIN_D7 39
#define CAM_PIN_D6 41
#define CAM_PIN_D5 42
#define CAM_PIN_D4 5
#define CAM_PIN_D3 40
#define CAM_PIN_D2 14
#define CAM_PIN_D1 47
#define CAM_PIN_D0 45
#define CAM_PIN_VSYNC 21
#define CAM_PIN_HREF 38
#define CAM_PIN_PCLK 48

// 模块内的全局变量
static camera_config_t camera_config;               // 摄像头配置结构体
static camera_fb_t *latest_fb = nullptr;            // 指向最新帧缓冲区的指针
static SemaphoreHandle_t latest_fb_mutex = nullptr; // 用于保护 latest_fb 访问的互斥锁
static SemaphoreHandle_t latest_fb_sem = nullptr;   // 用于通知新帧到达的信号量

/**
 * @brief 后台任务, 持续从摄像头获取帧并更新 latest_fb
 */
static void camera_producer_task(void *pvParameters)
{
    (void)pvParameters;

    // 用于FPS计算
    int frame_count = 0;
    long last_fps_time = esp_timer_get_time();

    while (true)
    {
        camera_fb_t *fb = esp_camera_fb_get();
        if (!fb)
        {
            vTaskDelay(pdMS_TO_TICKS(10)); // 获取失败, 稍后重试
            continue;
        }

        // --- FPS 计算开始 ---
        frame_count++;
        long current_time = esp_timer_get_time();
        long time_diff = current_time - last_fps_time;
        if (time_diff >= 1000000) // 1,000,000 微秒 = 1 秒
        {
            float fps = (float)frame_count / (time_diff / 1000000.0f);
            ESP_LOGI(TAG, "Camera FPS: %.2f", fps);
            frame_count = 0;
            last_fps_time = current_time;
        }
        // --- FPS 计算结束 ---

        // 使用互斥锁保护对全局指针的访问
        if (xSemaphoreTake(latest_fb_mutex, portMAX_DELAY) == pdTRUE)
        {
            // 如果上一帧还未被处理, 则归还它
            if (latest_fb)
            {
                esp_camera_fb_return(latest_fb);
            }
            // 更新为最新帧
            latest_fb = fb;
            // 发出信号, 通知消费者有新帧可用
            xSemaphoreGive(latest_fb_sem);
            xSemaphoreGive(latest_fb_mutex);
        }
        else
        {
            // 无法获取互斥锁, 直接归还当前帧, 避免内存泄漏
            esp_camera_fb_return(fb);
        }
    }
}

esp_err_t init()
{
    // 根据引脚定义配置摄像头
    camera_config.pin_pwdn = CAM_PIN_PWDN;
    camera_config.pin_reset = CAM_PIN_RESET;
    camera_config.pin_xclk = CAM_PIN_XCLK;
    camera_config.pin_sccb_sda = CAM_PIN_SIOD;
    camera_config.pin_sccb_scl = CAM_PIN_SIOC;
    camera_config.pin_d7 = CAM_PIN_D7;
    camera_config.pin_d6 = CAM_PIN_D6;
    camera_config.pin_d5 = CAM_PIN_D5;
    camera_config.pin_d4 = CAM_PIN_D4;
    camera_config.pin_d3 = CAM_PIN_D3;
    camera_config.pin_d2 = CAM_PIN_D2;
    camera_config.pin_d1 = CAM_PIN_D1;
    camera_config.pin_d0 = CAM_PIN_D0;
    camera_config.pin_vsync = CAM_PIN_VSYNC;
    camera_config.pin_href = CAM_PIN_HREF;
    camera_config.pin_pclk = CAM_PIN_PCLK;

    // 图像格式和质量配置
    camera_config.xclk_freq_hz = 24000000;
    camera_config.ledc_timer = LEDC_TIMER_0;
    camera_config.ledc_channel = LEDC_CHANNEL_0;
    camera_config.pixel_format = PIXFORMAT_JPEG;    // 使用JPEG格式以获得更高的帧率
    camera_config.frame_size = FRAMESIZE_QQVGA;     // 160x120, 适合低性能MCU
    camera_config.jpeg_quality = 12;                // 0-63, 数字越小质量越高
    camera_config.fb_count = 2;                     // 使用2个帧缓冲区进行双缓冲
    camera_config.grab_mode = CAMERA_GRAB_LATEST;   // 缓冲区满时, 丢弃旧帧
    camera_config.fb_location = CAMERA_FB_IN_PSRAM; // 帧缓冲区位于PSRAM中
    camera_config.sccb_i2c_port = 0;

    // 初始化摄像头
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "摄像头初始化失败, 错误码: 0x%x", err);
        return err;
    }

    // 创建同步所需的信号量和互斥锁
    latest_fb_mutex = xSemaphoreCreateMutex();
    if (!latest_fb_mutex)
    {
        ESP_LOGE(TAG, "创建 latest_fb_mutex 失败");
        return ESP_ERR_NO_MEM;
    }
    latest_fb_sem = xSemaphoreCreateBinary();
    if (!latest_fb_sem)
    {
        ESP_LOGE(TAG, "创建 latest_fb_sem 失败");
        return ESP_ERR_NO_MEM;
    }

    // 创建并启动后台捕获任务
    BaseType_t rc = xTaskCreate(camera_producer_task, "camera_producer", 4 * 1024, NULL, tskIDLE_PRIORITY + 3, NULL);
    if (rc != pdPASS)
    {
        ESP_LOGE(TAG, "启动 camera_producer 任务失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "摄像头初始化成功 (后台捕获任务已启动)");
    return ESP_OK;
}

camera_fb_t *get_frame()
{
    if (!latest_fb_mutex)
        return nullptr;
    camera_fb_t *out = nullptr;
    if (xSemaphoreTake(latest_fb_mutex, (TickType_t)0) == pdTRUE)
    {
        out = latest_fb;
        latest_fb = nullptr; // 取走后将全局指针置空
        xSemaphoreGive(latest_fb_mutex);
    }
    return out;
}

void return_frame(camera_fb_t *fb)
{
    if (fb)
    {
        esp_camera_fb_return(fb);
    }
}

camera_fb_t *blocking_get_frame(uint32_t timeout_ms)
{
    if (timeout_ms == 0)
        return get_frame();
    if (!latest_fb_sem || !latest_fb_mutex)
        return nullptr;

    // 等待新帧到达的信号, 带超时
    if (xSemaphoreTake(latest_fb_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE)
    {
        return nullptr; // 超时
    }

    // 获取帧
    camera_fb_t *out = nullptr;
    if (xSemaphoreTake(latest_fb_mutex, portMAX_DELAY) == pdTRUE)
    {
        out = latest_fb;
        latest_fb = nullptr;
        xSemaphoreGive(latest_fb_mutex);
    }
    return out;
}

cv::Mat get_grayscale_frame(uint32_t timeout_ms)
{
    // 1. 获取原始JPEG帧
    camera_fb_t *fb = blocking_get_frame(timeout_ms);
    if (!fb)
    {
        return cv::Mat(); // 返回空Mat表示失败
    }

    // 2. 解码JPEG图像
    jpeg_dec_handle_t jpeg_handle = NULL;
    uint8_t *outbuf = nullptr;
    cv::Mat grayscale_img;

    jpeg_dec_config_t jpeg_cfg = DEFAULT_JPEG_DEC_CONFIG();
    if (jpeg_dec_open(&jpeg_cfg, &jpeg_handle) != JPEG_ERR_OK || !jpeg_handle)
    {
        ESP_LOGE(TAG, "JPEG解码器打开失败");
        esp_camera_fb_return(fb);
        return cv::Mat();
    }

    jpeg_dec_io_t jpeg_io = {.inbuf = (uint8_t *)fb->buf,
                             .inbuf_len = (int)fb->len,
                             .inbuf_remain = (int)fb->len,
                             .outbuf = nullptr,
                             .out_size = 0};
    jpeg_dec_header_info_t jpeg_info;
    if (jpeg_dec_parse_header(jpeg_handle, &jpeg_io, &jpeg_info) != JPEG_ERR_OK)
    {
        ESP_LOGE(TAG, "JPEG头解析失败");
        jpeg_dec_close(jpeg_handle);
        esp_camera_fb_return(fb);
        return cv::Mat();
    }

    int outbuf_len = 0;
    if (jpeg_dec_get_outbuf_len(jpeg_handle, &outbuf_len) != JPEG_ERR_OK || outbuf_len <= 0)
    {
        ESP_LOGE(TAG, "获取JPEG输出缓冲区长度失败");
        jpeg_dec_close(jpeg_handle);
        esp_camera_fb_return(fb);
        return cv::Mat();
    }

    outbuf = (uint8_t *)heap_caps_malloc(outbuf_len, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!outbuf)
    {
        ESP_LOGE(TAG, "为JPEG输出分配内存失败");
        jpeg_dec_close(jpeg_handle);
        esp_camera_fb_return(fb);
        return cv::Mat();
    }

    jpeg_io.outbuf = outbuf;
    if (jpeg_dec_process(jpeg_handle, &jpeg_io) != JPEG_ERR_OK)
    {
        ESP_LOGE(TAG, "JPEG解码处理失败");
        free(outbuf);
        jpeg_dec_close(jpeg_handle);
        esp_camera_fb_return(fb);
        return cv::Mat();
    }

    // 3. 转换为灰度图
    cv::Mat rgb_img(jpeg_info.height, jpeg_info.width, CV_8UC3, outbuf);
    cv::cvtColor(rgb_img, grayscale_img, cv::COLOR_RGB2GRAY);

    // 4. 释放所有资源
    free(outbuf);
    jpeg_dec_close(jpeg_handle);
    esp_camera_fb_return(fb);

    return grayscale_img;
}

} // namespace CameraModule