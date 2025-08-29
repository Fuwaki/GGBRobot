#pragma once

#include "esp_camera.h"


namespace ImageProcessor {

/**
 * @brief 对灰度图像帧进行二值化处理
 * @param fb 指向要处理的摄像头帧缓冲区的指针
 * @param threshold 二值化阈值 (0-255)
 */
void binarize(camera_fb_t *fb, uint8_t threshold);

// 未来可以添加更多处理函数，例如：
// void invert(camera_fb_t *fb);
// void edge_detect(camera_fb_t *fb);

} // namespace ImageProcessor