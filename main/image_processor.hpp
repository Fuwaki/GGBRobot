#pragma once

#include "esp_camera.h"
#include <opencv2/opencv.hpp>

namespace ImageProcessor {



/**
 * @brief 对灰度图像进行二值化处理
 * @param img 要处理的图像 (cv::Mat)
 * @param threshold 二值化阈值 (0-255)
 */
void binarize(cv::Mat &img, uint8_t threshold);

// 未来可以添加更多处理函数，例如：
// void invert(cv::Mat &img);
// void edge_detect(cv::Mat &img);

} // namespace ImageProcessor

