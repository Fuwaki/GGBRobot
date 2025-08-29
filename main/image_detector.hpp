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
    struct Circle {
        cv::Point2f center;
        float radius;
        bool found;
    };

    cv::Mat create_test_image(int w, int h, GroundTruth& gt);
    void add_salt_and_pepper_noise(cv::Mat& img, int n);

    // Common function to find the largest contour and its enclosing circle
    Circle find_largest_contour_circle(const cv::Mat& processed_image);

    // Test functions
    void test_findContours();
    void test_findContours_no_noise(); // New test function using the common function
    void test_template_matching();
    
    // Detection methods
    void detect_with_contours(const cv::Mat& img);
    void detect_with_template_matching(const cv::Mat& img);
};

#endif // IMAGE_DETECTOR_HPP
