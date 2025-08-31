#include "camera_module.hpp"
#include "esp_camera.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "sensor.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <opencv2/opencv.hpp>
#include <esp_jpeg_dec.h>

namespace CameraModule {

static const char* TAG = "摄像头模块";

// WROVER-KIT 引脚定义
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

static camera_config_t camera_config;

static camera_fb_t* latest_fb = nullptr;
static SemaphoreHandle_t latest_fb_mutex = nullptr;
static SemaphoreHandle_t latest_fb_sem = nullptr;

static void camera_producer_task(void* pvParameters) {
    (void)pvParameters;
    while (true) {
        camera_fb_t* fb = esp_camera_fb_get();
        if (!fb) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (latest_fb_mutex) {
            if (xSemaphoreTake(latest_fb_mutex, portMAX_DELAY) == pdTRUE) {
                if (latest_fb) {
                    esp_camera_fb_return(latest_fb);
                }
                latest_fb = fb;
                if (latest_fb_sem) xSemaphoreGive(latest_fb_sem);
                xSemaphoreGive(latest_fb_mutex);
            } else {
                esp_camera_fb_return(fb);
            }
        } else {
            esp_camera_fb_return(fb);
        }
    }
}

esp_err_t init() {
    if (CAM_PIN_PWDN != -1) {
        gpio_set_direction((gpio_num_t)CAM_PIN_PWDN, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)CAM_PIN_PWDN, 0);
    }

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
    camera_config.xclk_freq_hz = 20000000;
    camera_config.ledc_timer = LEDC_TIMER_0;
    camera_config.ledc_channel = LEDC_CHANNEL_0;
    camera_config.pixel_format = PIXFORMAT_JPEG;
    camera_config.frame_size = FRAMESIZE_QQVGA;
    camera_config.jpeg_quality = 12;
    camera_config.fb_count = 2;
    camera_config.grab_mode = CAMERA_GRAB_LATEST;
    camera_config.fb_location = CAMERA_FB_IN_PSRAM;
    camera_config.sccb_i2c_port = 0;

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "摄像头初始化失败，错误码: 0x%x", err);
        return err;
    }

    latest_fb_mutex = xSemaphoreCreateMutex();
    if (!latest_fb_mutex) {
        ESP_LOGE(TAG, "创建 latest_fb_mutex 失败");
        return ESP_ERR_NO_MEM;
    }
    latest_fb_sem = xSemaphoreCreateBinary();
    if (!latest_fb_sem) {
        ESP_LOGE(TAG, "创建 latest_fb_sem 失败");
        return ESP_ERR_NO_MEM;
    }

    BaseType_t rc = xTaskCreate(camera_producer_task, "camera_producer", 4 * 1024, NULL, tskIDLE_PRIORITY + 3, NULL);
    if (rc != pdPASS) {
        ESP_LOGE(TAG, "启动 camera_producer 任务失败");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "摄像头初始化成功 (producer started)");
    return ESP_OK;
}

camera_fb_t* get_frame() {
    if (!latest_fb_mutex) return nullptr;
    camera_fb_t* out = nullptr;
    if (xSemaphoreTake(latest_fb_mutex, (TickType_t)0) == pdTRUE) {
        out = latest_fb;
        latest_fb = nullptr;
        xSemaphoreGive(latest_fb_mutex);
    }
    return out;
}

void return_frame(camera_fb_t *fb) {
    if (fb) {
        esp_camera_fb_return(fb);
    }
}

camera_fb_t* blocking_get_frame(uint32_t timeout_ms) {
    if (timeout_ms == 0) return get_frame();
    if (!latest_fb_sem || !latest_fb_mutex) return nullptr;

    if (xSemaphoreTake(latest_fb_sem, pdMS_TO_TICKS(timeout_ms)) != pdTRUE) {
        return nullptr;
    }

    camera_fb_t* out = nullptr;
    if (xSemaphoreTake(latest_fb_mutex, portMAX_DELAY) == pdTRUE) {
        out = latest_fb;
        latest_fb = nullptr;
        xSemaphoreGive(latest_fb_mutex);
    }
    return out;
}

cv::Mat get_grayscale_frame(uint32_t timeout_ms) {
    camera_fb_t* fb = blocking_get_frame(timeout_ms);
    if (!fb) return cv::Mat();

    jpeg_dec_config_t jpeg_cfg = DEFAULT_JPEG_DEC_CONFIG();
    jpeg_dec_handle_t jpeg_handle = NULL;
    if (jpeg_dec_open(&jpeg_cfg, &jpeg_handle) != JPEG_ERR_OK || !jpeg_handle) {
        esp_camera_fb_return(fb);
        return cv::Mat();
    }

    jpeg_dec_io_t jpeg_io = { .inbuf = (uint8_t*)fb->buf, .inbuf_len = (int)fb->len };
    jpeg_dec_header_info_t jpeg_info;
    if (jpeg_dec_parse_header(jpeg_handle, &jpeg_io, &jpeg_info) != JPEG_ERR_OK) {
        jpeg_dec_close(jpeg_handle);
        esp_camera_fb_return(fb);
        return cv::Mat();
    }

    int outbuf_len = 0;
    if (jpeg_dec_get_outbuf_len(jpeg_handle, &outbuf_len) != JPEG_ERR_OK || outbuf_len <= 0) {
        jpeg_dec_close(jpeg_handle);
        esp_camera_fb_return(fb);
        return cv::Mat();
    }

    uint8_t* outbuf = (uint8_t*)heap_caps_malloc(outbuf_len, MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (!outbuf) {
        jpeg_dec_close(jpeg_handle);
        esp_camera_fb_return(fb);
        return cv::Mat();
    }

    jpeg_io.outbuf = outbuf;
    if (jpeg_dec_process(jpeg_handle, &jpeg_io) != JPEG_ERR_OK) {
        free(outbuf);
        jpeg_dec_close(jpeg_handle);
        esp_camera_fb_return(fb);
        return cv::Mat();
    }

    cv::Mat rgb_img(jpeg_info.height, jpeg_info.width, CV_8UC3, outbuf);
    cv::Mat grayscale_img;
    cv::cvtColor(rgb_img, grayscale_img, cv::COLOR_RGB2GRAY);
    
    free(outbuf);
    jpeg_dec_close(jpeg_handle);
    esp_camera_fb_return(fb);

    return grayscale_img;
}

} // namespace CameraModule
