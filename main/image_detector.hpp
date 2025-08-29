#ifndef IMAGE_DETECTOR_HPP
#define IMAGE_DETECTOR_HPP

// Undefine EPS to avoid conflict with OpenCV
#ifdef EPS
#undef EPS
#endif

#include <opencv2/opencv.hpp>

class ImageDetector {
public:
    void test_all();
    void detect_from_input();

private:
    struct GroundTruth { int cx, cy, r; };

    cv::Mat create_test_image(int w, int h, GroundTruth& gt);
    void add_salt_and_pepper_noise(cv::Mat& img, int n);

    void test_findContours();
    void test_template_matching();
    void detect_with_contours(const cv::Mat& img);
    void detect_with_template_matching(const cv::Mat& img);

    // FFT helpers
    void fft2d(float* data, int width, int height);
    void ifft2d(float* data, int width, int height);
    unsigned int nextPowerOf2(unsigned int n);
};

#endif // IMAGE_DETECTOR_HPP