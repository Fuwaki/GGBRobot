#pragma once

#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

namespace CameraModule {

/**
 * @brief 初始化摄像头并启动后台帧捕获任务
 * @return esp_err_t ESP_OK表示成功
 */
esp_err_t init();

/**
 * @brief 从队列中获取最新的摄像头图像 (non-blocking)
 * 
 * @return camera_fb_t* 指向帧缓冲区的指针。如果没有新的帧，则返回nullptr。
 *         使用者有责任在使用后调用 return_frame() 释放帧。
 */
camera_fb_t* get_frame();

/**
 * @brief 归还帧缓冲区，以便摄像头可以重用它
 * @param fb 指向要归还的帧缓冲区的指针
 */
void return_frame(camera_fb_t *fb);

} // namespace CameraModule
