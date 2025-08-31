#pragma once

#include "esp_camera.h"
#include "freertos/FreeRTOS.h"

// 在包含OpenCV头文件之前,取消定义可能冲突的EPS宏
#ifdef EPS
#undef EPS
#endif
#include <opencv2/opencv.hpp>

/**
 * @brief 摄像头模块,负责初始化和捕获图像
 */
namespace CameraModule {

/**
 * @brief 初始化摄像头模块。
 * 
 * 此函数会配置摄像头硬件引脚,设置图像格式(JPEG),并启动一个后台FreeRTOS任务,
 * 该任务持续从摄像头捕获图像帧,确保总是有最新的图像可供主程序使用。
 * 
 * @return esp_err_t ESP_OK 表示成功,其他值表示失败。
 */
esp_err_t init();

/**
 * @brief 获取一个已解码的灰度图像帧。
 * 
 * 此函数会阻塞式地等待新的一帧,然后自动完成JPEG到RGB的解码,
 * 并最终转换为灰度图像(cv::Mat)。内存管理已在此函数内部处理。
 * 
 * @param timeout_ms 获取帧的超时时间(毫秒)。
 * @return cv::Mat 返回一个OpenCV Mat格式的灰度图像。如果超时或发生错误,返回一个空的Mat。
 */
cv::Mat get_grayscale_frame(uint32_t timeout_ms);

// 注意: 以下函数用于获取原始JPEG帧,当前项目中优先使用 get_grayscale_frame。
// 如果需要直接处理JPEG数据(例如,用于网络流),这些函数会很有用。

/**
 * @brief (内部使用) 从队列中获取最新的摄像头原始图像 (非阻塞)
 * 
 * @return camera_fb_t* 指向帧缓冲区的指针。如果没有新的帧,则返回nullptr。
 *         使用者有责任在使用后调用 return_frame() 释放帧。
 */
camera_fb_t* get_frame();

/**
 * @brief (内部使用) 归还帧缓冲区,以便摄像头可以重用它
 * @param fb 指向要归还的帧缓冲区的指针
 */
void return_frame(camera_fb_t *fb);

/**
 * @brief (内部使用) 等待最多 timeout_ms 毫秒并获取最新原始帧（阻塞式）。
 * @param timeout_ms 最长等待时间（毫秒），为0表示立即返回（等同于 get_frame）。
 * @return camera_fb_t* 成功返回帧指针，超时或错误返回 nullptr。使用后必须调用 return_frame().
 */
camera_fb_t* blocking_get_frame(uint32_t timeout_ms);

} // namespace CameraModule