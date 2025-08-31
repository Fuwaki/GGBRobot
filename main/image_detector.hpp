#ifndef IMAGE_DETECTOR_HPP
#define IMAGE_DETECTOR_HPP

// Undefine EPS to avoid conflict with OpenCV
#ifdef EPS
#undef EPS
#endif

#include <opencv2/opencv.hpp>

namespace ImageDetector {
    void test_all();
    void detect_from_input();

    // --- Data Structures ---
    struct GroundTruth { int cx, cy, r; };

    struct Circle {
        cv::Point2f center;
        float radius;
        bool found;
    };

    struct ContourFindParams {
        double min_area = 0.0;
        double max_area = 1e9;
        double min_circularity = 0.0;
        double min_radius = 0.0;
        double max_radius = 1e9;

        enum class Selection {
            LARGEST_AREA,
            CLOSEST_TO_POINT
        };
        Selection selection_method = Selection::LARGEST_AREA;
        cv::Point2f expected_pos = {0, 0};
    };

    // --- Core Algorithm Functions ---
    Circle find_ball_by_contour(const cv::Mat& processed_img, const ContourFindParams& params);
    Circle find_ball_by_components(const cv::Mat& processed_img, const ContourFindParams& params);

    // --- Wrappers and Test Functions ---
    cv::Mat create_test_image(int w, int h, GroundTruth& gt);
    void add_salt_and_pepper_noise(cv::Mat& img, int n);

    void detect_with_contours(const cv::Mat& img);
    void detect_with_components(const cv::Mat& img);

    void run_contour_test(bool with_noise);
    void run_components_test(bool with_noise);
};

#endif // IMAGE_DETECTOR_HPP