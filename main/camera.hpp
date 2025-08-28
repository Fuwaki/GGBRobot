#include "esp_camera.h"
#include "Arduino.h"
#include "sensor.h"

// WROVER-KIT PIN Map
#define CAM_PIN_PWDN -1 // power down is not used
#define CAM_PIN_RESET 4 // software reset will be performed
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
#define TAG "CAMERA"

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

    .xclk_freq_hz = 20000000, // EXPERIMENTAL: Set to 16MHz on ESP32-S2 or
                              // ESP32-S3 to enable EDMA mode
    .ledc_timer = LEDC_TIMER_0,
    .ledc_channel = LEDC_CHANNEL_0,
    // 更改为GRAYSCALE格式以进行逐像素访问。JPEG是压缩格式，不适合此目的。
    .pixel_format = PIXFORMAT_GRAYSCALE,
    // 使用较小的分辨率以加快处理速度。HVGA(480x320)对于像素级处理可能太慢。
    // QQVGA (160x120) 是一个很好的起点。
    .frame_size = FRAMESIZE_QQVGA,
    .fb_count = 4, // 对于原始数据流，2个缓冲区就足够了
                   // driver will work in continuous mode.
    .grab_mode = CAMERA_GRAB_LATEST // 使用LATEST模式可以获取最新的图像，减少延迟
};

esp_err_t camera_init() {
  // power up the camera if PWDN pin is defined
  if (CAM_PIN_PWDN != -1) {
    pinMode(CAM_PIN_PWDN, OUTPUT);
    digitalWrite(CAM_PIN_PWDN, LOW);
  }

  // initialize the camera
  esp_err_t err = esp_camera_init(&camera_config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera Init Failed");
    return err;
  }

  return ESP_OK;
}
