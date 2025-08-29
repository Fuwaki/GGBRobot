#include "image_processor.hpp"

namespace ImageProcessor {

void binarize(camera_fb_t *fb, uint8_t threshold) {
    if (!fb || !fb->buf || fb->format != PIXFORMAT_GRAYSCALE) {
        return; // 只处理有效的灰度图像
    }

    uint8_t *image_buffer = fb->buf;
    size_t len = fb->width * fb->height;

    for (size_t i = 0; i < len; i++) {
        image_buffer[i] = (image_buffer[i] > threshold) ? 255 : 0;
    }
}

} // namespace ImageProcessor