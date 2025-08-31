#include "image_processor.hpp"
#include <cstdint>
#include <opencv2/opencv.hpp>

namespace ImageProcessor
{

void binarize(cv::Mat &img, uint8_t threshold)
{
    if (img.empty() || img.channels() != 1)
    {
        return; // 只处理有效的单通道灰度图像
    }

    // 使用反向二值化, 使目标(通常较暗)变为白色(255), 背景变为黑色(0)
    cv::threshold(img, img, threshold, 255, cv::THRESH_BINARY_INV);
}

} // namespace ImageProcessor
