#pragma once

#include "esp_camera.h"

namespace CameraModule {

/**
 * @brief 初始化摄像头
 * @return esp_err_t ESP_OK表示成功
 */
esp_err_t init();

/**
 * @brief 获取一帧摄像头图像
 * @return camera_fb_t* 指向帧缓冲区的指针，如果失败则为nullptr
 */
camera_fb_t* get_frame();

/**
 * @brief 归还帧缓冲区，以便摄像头可以重用它
 * @param fb 指向要归还的帧缓冲区的指针
 */
void return_frame(camera_fb_t *fb);

} // namespace CameraModule