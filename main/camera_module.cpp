#include "camera_module.hpp"
#include "driver/gpio.h"
#include "esp_log.h"

namespace CameraModule {

static const char* TAG = "摄像头模块";

// WROVER-KIT 引脚定义
#define CAM_PIN_PWDN -1 // PWDN引脚未使用
#define CAM_PIN_RESET 4 // 将执行软件复位
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

static camera_config_t camera_config = {
    .pin_pwdn = CAM_PIN_PWDN,
    .pin_reset = CAM_PIN_RESET,
    .pin_xclk = CAM_PIN_XCLK,
    .pin_sccb_sda = CAM_PIN_SIOD,
    .pin_sccb_scl = CAM_PIN_SIOC,

    .pin_d7 = CAM_PIN_D7,
    .pin_d6 = CAM_PIN_D6,
    .pin_d5 = CAM_PIN_D5,
    .pin_d4 = CAM_PIN_D4,
    .pin_d3 = CAM_PIN_D3,
    .pin_d2 = CAM_PIN_D2,
    .pin_d1 = CAM_PIN_D1,
    .pin_d0 = CAM_PIN_D0,
    .pin_vsync = CAM_PIN_VSYNC,
    .pin_href = CAM_PIN_HREF,
    .pin_pclk = CAM_PIN_PCLK,

    .xclk_freq_hz = 20000000,
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    
    .pixel_format = PIXFORMAT_GRAYSCALE,
    .frame_size = FRAMESIZE_QQVGA, // 160x120
    .fb_count = 2, // 对于原始数据流，2个缓冲区就足够了，可以节省内存
    .grab_mode = CAMERA_GRAB_LATEST
};

esp_err_t init() {
    if (CAM_PIN_PWDN != -1) {
        gpio_set_direction((gpio_num_t)CAM_PIN_PWDN, GPIO_MODE_OUTPUT);
        gpio_set_level((gpio_num_t)CAM_PIN_PWDN, 0);
    }

    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "摄像头初始化失败，错误码: 0x%x", err);
        return err;
    }
    ESP_LOGI(TAG, "摄像头初始化成功");
    return ESP_OK;
}

camera_fb_t* get_frame() {
    return esp_camera_fb_get();
}

void return_frame(camera_fb_t *fb) {
    if (fb) {
        esp_camera_fb_return(fb);
    }
}

} // namespace CameraModule