#ifndef IMAGE_DETECTOR_HPP
#define IMAGE_DETECTOR_HPP

#include <opencv2/opencv.hpp>

class ImageDetector {
public:
    void test_all();

private:
    struct GroundTruth { int cx, cy, r; };

    cv::Mat create_test_image(int w, int h, GroundTruth& gt);
    void add_salt_and_pepper_noise(cv::Mat& img, int n);

    void test_findContours();
    void test_template_matching();
    void test_template_matching_fft();

    // FFT helpers
    void fft2d(float* data, int width, int height);
    void ifft2d(float* data, int width, int height);
};

#endif // IMAGE_DETECTOR_HPP
