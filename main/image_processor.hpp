#pragma once

#include <opencv2/opencv.hpp>

/**
 * @brief 图像处理模块
 * 
 * 包含一系列用于图像预处理的函数。
 */
namespace ImageProcessor {

/**
 * @brief 对灰度图像进行二值化处理 (原地修改)
 * 
 * @param img 要处理的灰度图像 (cv::Mat)。函数将直接修改此图像。
 * @param threshold 二值化阈值 (0-255)。小于阈值的像素变为255, 大于等于的变为0 (反向二值化)。
 */
void binarize(cv::Mat &img, uint8_t threshold);

// 未来可以添加更多处理函数, 例如:
// void invert(cv::Mat &img);
// void edge_detect(cv::Mat &img);

} // namespace ImageProcessor