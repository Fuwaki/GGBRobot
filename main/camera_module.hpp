#pragma once

#include "esp_camera.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

// Undefine the conflicting EPS macro before including OpenCV headers
#undef EPS
#include <opencv2/opencv.hpp>

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

/**
 * @brief 等待最多 timeout_ms 毫秒并获取最新帧（阻塞式，可设置为短超时）。
 * @param timeout_ms 最长等待时间（毫秒），为0表示立即返回（等同于 get_frame）。
 * @return camera_fb_t* 成功返回帧指针，超时或错误返回 nullptr。使用后必须调用 return_frame().
 */
camera_fb_t* blocking_get_frame(uint32_t timeout_ms);

/**
 * @brief 获取一帧灰度图像（自动完成JPEG解码和灰度转换）
 * @param timeout_ms 最大等待时间（毫秒）
 * @return cv::Mat 返回OpenCV Mat格式的灰度图像，如果失败则返回空Mat
 */
cv::Mat get_grayscale_frame(uint32_t timeout_ms);

} // namespace CameraModule
