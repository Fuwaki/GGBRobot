#include "image_detector.hpp"
#include "input.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include <cmath>

static const char *TAG = "ImageDetector";

using namespace cv;

namespace ImageDetector {

// =================================================================================================
// 核心算法函数部分
// =================================================================================================

Circle find_ball_by_contour(const cv::Mat& processed_img, const ContourFindParams& params) {
    std::vector<std::vector<cv::Point>> contours;
    findContours(processed_img.clone(), contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    if (contours.empty()) {
        return {cv::Point2f(0, 0), 0.0f, false};
    }

    // 策略1：查找最大轮廓。这很简单，忽略大多数参数。
    if (params.selection_method == ContourFindParams::Selection::LARGEST_AREA) {
        double max_area = 0;
        int max_idx = -1;
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = contourArea(contours[i]);
            if (area > max_area) {
                max_area = area;
                max_idx = i;
            }
        }
        
        if (max_idx != -1) {
            Circle c;
            minEnclosingCircle(contours[max_idx], c.center, c.radius);
            c.found = true;
            return c;
        }
        return {cv::Point2f(0, 0), 0.0f, false};
    }
    
    // 策略2：过滤候选轮廓并找到最接近某个点的轮廓。
    if (params.selection_method == ContourFindParams::Selection::CLOSEST_TO_POINT) {
        Point2f best_center;
        float best_radius = 0;
        double min_dist_to_expected = 1e9;
        bool found = false;

        for (const auto& contour : contours) {
            double area = contourArea(contour);
            if (area >= params.min_area && area <= params.max_area) {
                Point2f center;
                float radius;
                minEnclosingCircle(contour, center, radius);

                if (radius == 0) continue;
                double circularity = area / (CV_PI * radius * radius);

                if (circularity >= params.min_circularity && radius >= params.min_radius && radius <= params.max_radius) {
                    double dist = norm(center - params.expected_pos);
                    if (dist < min_dist_to_expected) {
                        min_dist_to_expected = dist;
                        best_center = center;
                        best_radius = radius;
                        found = true;
                    }
                }
            }
        }
        return {best_center, best_radius, found};
    }

    return {cv::Point2f(0, 0), 0.0f, false};
}

MatchResult find_ball_by_template(const cv::Mat& frame, const cv::Mat& templ, const cv::Rect& search_window, double threshold) {
    if (search_window.width < templ.cols || search_window.height < templ.rows || search_window.area() <= 0) {
        return {{}, 0.0, false};
    }

    Mat search_region = frame(search_window);
    Mat result;
    matchTemplate(search_region, templ, result, TM_CCORR_NORMED);

    double maxVal; Point maxLoc;
    minMaxLoc(result, NULL, &maxVal, NULL, &maxLoc);

    Rect box(maxLoc.x + search_window.x, maxLoc.y + search_window.y, templ.cols, templ.rows);
    return {box, maxVal, maxVal > threshold};
}

// =================================================================================================
// 公共入口点和高层次包装器部分
// =================================================================================================

void test_all() {
    run_contour_test(true);  // 带噪点测试
    run_contour_test(false); // 无噪点测试
    test_template_matching();
}

void detect_from_input() {
    ESP_LOGI(TAG, "开始处理input.h中的图像数据...");

    Mat img(120, 160, CV_8UC1, (void*)input);
    ESP_LOGI(TAG, "图像尺寸: %dx%d", img.cols, img.rows);

    ESP_LOGI(TAG, "\n=== 方案1: 轮廓检测 (findContours) ===");
    detect_with_contours(img);

    ESP_LOGI(TAG, "\n=== 方案2: 模板匹配 (Template Matching) ===");
    detect_with_template_matching(img);
}

void detect_with_contours(const Mat& img) {
    // 1. 对特定input.h图像进行预处理
    Mat inverted_img, morph;
    bitwise_not(img, inverted_img);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(inverted_img, morph, MORPH_OPEN, kernel);

    // 2. 为此特定检测任务配置参数
    ContourFindParams params;
    params.selection_method = ContourFindParams::Selection::CLOSEST_TO_POINT;
    params.expected_pos     = {71, 73};
    params.min_area         = 500;
    params.max_area         = 5000;
    params.min_circularity  = 0.5;
    params.min_radius       = 20;
    params.max_radius       = 40;

    // 3. 调用核心函数
    Circle ball = find_ball_by_contour(morph, params);

    // 4. 记录结果
    if (ball.found) {
        ESP_LOGI(TAG, "检测到圆形物体:");
        ESP_LOGI(TAG, "  中心坐标: (%.1f, %.1f)", ball.center.x, ball.center.y);
        ESP_LOGI(TAG, "  半径: %.1f", ball.radius);
    } else {
        ESP_LOGI(TAG, "未检测到符合条件的圆形物体");
    }
}

void detect_with_template_matching(const Mat& img) {
    // 1. 创建模板
    const int TEMPLATE_RADIUS = 30;
    Mat templ(TEMPLATE_RADIUS * 2 + 1, TEMPLATE_RADIUS * 2 + 1, CV_8UC1, Scalar(255));
    circle(templ, Point(TEMPLATE_RADIUS, TEMPLATE_RADIUS), TEMPLATE_RADIUS, Scalar(0), -1);

    // 2. 调用核心函数
    const double CONFIDENCE_THRESHOLD = 0.7;
    Rect search_window(0, 0, img.cols, img.rows);
    MatchResult match = find_ball_by_template(img, templ, search_window, CONFIDENCE_THRESHOLD);

    // 3. 记录结果
    ESP_LOGI(TAG, "模板匹配结果:");
    ESP_LOGI(TAG, "  最大相关系数: %.3f", match.score);
    if (match.found) {
        Point center(match.box.x + TEMPLATE_RADIUS, match.box.y + TEMPLATE_RADIUS);
        ESP_LOGI(TAG, "  检测到物体中心: (%d, %d)", center.x, center.y);
    } else {
        ESP_LOGI(TAG, "  未检测到符合条件的物体 (相关系数太低)");
    }
}

// =================================================================================================
// 测试实现部分
// =================================================================================================

void run_contour_test(bool with_noise) {
    const int W = 160, H = 120, N = 100;
    const int NOISE_LEVEL = 300;
    int ok = 0;
    int64_t t_total = 0;

    // 对于此测试，我们只需要最大轮廓，因此使用默认参数。
    ContourFindParams params;
    params.selection_method = ContourFindParams::Selection::LARGEST_AREA;

    for (int i = 0; i < N; ++i) {
        GroundTruth gt;
        Mat img = create_test_image(W, H, gt);
        Mat processed_img;

        if (with_noise) {
            add_salt_and_pepper_noise(img, NOISE_LEVEL);
            Mat bin, morph;
            threshold(img, bin, 0, 255, THRESH_BINARY | THRESH_OTSU);
            Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
            morphologyEx(bin, morph, MORPH_OPEN, kernel);
            processed_img = morph;
        } else {
            threshold(img, processed_img, 0, 255, THRESH_BINARY | THRESH_OTSU);
        }

        int64_t t0 = esp_timer_get_time();
        Circle circle = find_ball_by_contour(processed_img, params);
        t_total += esp_timer_get_time() - t0;

        const float tolerance = with_noise ? 3.0f : 1.0f;
        if (circle.found && norm(circle.center - Point2f(gt.cx, gt.cy)) < tolerance && fabs(circle.radius - gt.r) < tolerance) {
            ok++;
        }
    }

    float avg_ms = t_total / 1000.0f / N;
    ESP_LOGI(TAG, "findContours (%s): %.1f fps, 准确率 %.1f%%",
             with_noise ? "带噪点" : "无噪点",
             1000.0f / avg_ms, 100.0f * ok / N);
}

void test_template_matching() {
    const int W = 160, H = 120, N = 100, NOISE_LEVEL = 300;
    int ok = 0;
    int64_t t_total = 0;

    GroundTruth gt;
    Mat frame0 = create_test_image(W, H, gt);
    Rect roi(gt.cx - gt.r, gt.cy - gt.r, 2 * gt.r, 2 * gt.r);
    roi = roi & Rect(0, 0, W, H);
    if (roi.area() <= 0) {
        ESP_LOGE(TAG, "Template Matching: 初始ROI无效。");
        return;
    }
    Mat templ = frame0(roi).clone();
    Rect last_pos = roi;

    for (int i = 1; i < N; ++i) {
        Mat frame(H, W, CV_8UC1);
        gt.cx += (esp_random() % 7) - 3;
        gt.cy += (esp_random() % 7) - 3;
        gt.cx = std::max(gt.r, std::min(W - 1 - gt.r, gt.cx));
        gt.cy = std::max(gt.r, std::min(H - 1 - gt.r, gt.cy));
        frame.setTo(Scalar(0));
        circle(frame, Point(gt.cx, gt.cy), gt.r, Scalar(255), -1);
        add_salt_and_pepper_noise(frame, NOISE_LEVEL);

        Rect search_window = last_pos + Size(30, 30) - Point(15, 15);
        search_window &= Rect(0, 0, W, H);

        int64_t t0 = esp_timer_get_time();
        MatchResult match = find_ball_by_template(frame, templ, search_window, -1.0);
        t_total += esp_timer_get_time() - t0;

        if (match.box.area() > 0) {
            last_pos = match.box;
            Point2f center(match.box.x + match.box.width / 2.0f, match.box.y + match.box.height / 2.0f);
            if (norm(center - Point2f(gt.cx, gt.cy)) < 5.0f) {
                ++ok;
            }
        }
    }

    float avg_ms = t_total / 1000.0f / (N - 1);
    ESP_LOGI(TAG, "TemplateMatching (OpenCV): %.1f fps, 成功率 %.1f%%", 1000.0f / avg_ms, 100.0f * ok / (N - 1));
}

// =================================================================================================
// 低层次工具部分
// =================================================================================================

cv::Mat create_test_image(int w, int h, GroundTruth& gt) {
    Mat img(h, w, CV_8UC1, Scalar(0));
    gt.cx = w * (300 + esp_random() % 400) / 1000;
    gt.cy = h * (300 + esp_random() % 400) / 1000;
    gt.r  = 15 + esp_random() % 25;
    circle(img, Point(gt.cx, gt.cy), gt.r, Scalar(255), -1);
    return img;
}

void add_salt_and_pepper_noise(Mat& img, int n) {
    for (int k = 0; k < n; ++k) {
        int i = esp_random() % img.cols;
        int j = esp_random() % img.rows;
        if (img.type() == CV_8UC1) {
            img.at<uint8_t>(j, i) = (esp_random() % 2) * 255;
        }
    }
}

} // namespace ImageDetector
