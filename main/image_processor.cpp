#include "image_processor.hpp"
#include <opencv2/opencv.hpp>
#include <cstdint>

namespace ImageProcessor {

void binarize(cv::Mat &img, uint8_t threshold) {
    if (img.empty() || img.channels() != 1) {
        return; // 只处理有效的灰度图像
    }

    cv::threshold(img, img, threshold, 255, cv::THRESH_BINARY_INV);
}

} // namespace ImageProcessor