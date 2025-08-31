#include "image_detector.hpp"
#include "esp_log.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "input.h"
#include <cmath>

static const char *TAG = "ImageDetector";

using namespace cv;

namespace ImageDetector
{

// =================================================================================================
// 核心算法函数部分
// =================================================================================================

Circle find_ball_by_contour(const cv::Mat &processed_img, const ContourFindParams &params)
{
    std::vector<std::vector<cv::Point>> contours;
    // findContours may modify the source image; we intentionally pass processed_img directly
    // to avoid an expensive clone() copy when the caller does not need the original preserved.
    findContours(processed_img, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    if (contours.empty())
    {
        return {cv::Point2f(0, 0), 0.0f, false};
    }

    // 策略1：查找最大轮廓。这很简单，忽略大多数参数。
    if (params.selection_method == ContourFindParams::Selection::LARGEST_AREA)
    {
        double max_area = 0;
        int max_idx = -1;
        for (size_t i = 0; i < contours.size(); ++i)
        {
            double area = contourArea(contours[i]);
            if (area > max_area)
            {
                max_area = area;
                max_idx = i;
            }
        }

        if (max_idx != -1)
        {
            Circle c;
            minEnclosingCircle(contours[max_idx], c.center, c.radius);
            c.found = true;
            return c;
        }
        return {cv::Point2f(0, 0), 0.0f, false};
    }

    // 策略2：过滤候选轮廓并找到最接近某个点的轮廓。
    if (params.selection_method == ContourFindParams::Selection::CLOSEST_TO_POINT)
    {
        Point2f best_center;
        float best_radius = 0;
        double min_dist_to_expected = 1e9;
        bool found = false;

        for (const auto &contour : contours)
        {
            double area = contourArea(contour);
            if (area >= params.min_area && area <= params.max_area)
            {
                Point2f center;
                float radius;
                minEnclosingCircle(contour, center, radius);

                if (radius == 0)
                    continue;
                double circularity = area / (CV_PI * radius * radius);

                if (circularity >= params.min_circularity && radius >= params.min_radius && radius <= params.max_radius)
                {
                    double dist = norm(center - params.expected_pos);
                    if (dist < min_dist_to_expected)
                    {
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

Circle find_ball_by_components(const cv::Mat &processed_img, const ContourFindParams &params)
{
    Mat labels, stats, centroids;
    int num_labels = connectedComponentsWithStats(processed_img, labels, stats, centroids, 8, CV_32S);

    if (num_labels <= 1)
    {
        return {cv::Point2f(0, 0), 0.0f, false};
    }

    Point2f best_center;
    float best_radius = 0;
    bool found = false;

    if (params.selection_method == ContourFindParams::Selection::LARGEST_AREA)
    {
        int max_area = 0;
        int max_label = -1;
        for (int i = 1; i < num_labels; ++i)
        {
            int area = stats.at<int>(i, CC_STAT_AREA);
            if (area > max_area)
            {
                max_area = area;
                max_label = i;
            }
        }
        if (max_label != -1)
        {
            best_center.x = centroids.at<double>(max_label, 0);
            best_center.y = centroids.at<double>(max_label, 1);
            int w = stats.at<int>(max_label, CC_STAT_WIDTH);
            int h = stats.at<int>(max_label, CC_STAT_HEIGHT);
            best_radius = (w + h) / 4.0f; // Approximate radius
            found = true;
        }
    }
    else if (params.selection_method == ContourFindParams::Selection::CLOSEST_TO_POINT)
    {
        double min_dist_to_expected = 1e9;
        for (int i = 1; i < num_labels; ++i)
        {
            int area = stats.at<int>(i, CC_STAT_AREA);
            if (area >= params.min_area && area <= params.max_area)
            {
                int w = stats.at<int>(i, CC_STAT_WIDTH);
                int h = stats.at<int>(i, CC_STAT_HEIGHT);
                float radius = (w + h) / 4.0f;
                float aspect_ratio = (float)w / h;

                // A better circularity check for components is aspect ratio of bounding box
                if (aspect_ratio > 0.75 && aspect_ratio < 1.33 && radius >= params.min_radius &&
                    radius <= params.max_radius)
                {
                    Point2f center(centroids.at<double>(i, 0), centroids.at<double>(i, 1));
                    double dist = norm(center - params.expected_pos);
                    if (dist < min_dist_to_expected)
                    {
                        min_dist_to_expected = dist;
                        best_center = center;
                        best_radius = radius;
                        found = true;
                    }
                }
            }
        }
    }

    return {best_center, best_radius, found};
}

// =================================================================================================
// 公共入口点和高层次包装器部分
// =================================================================================================

void test_all()
{
    run_contour_test(true);  // 带噪点测试
    run_contour_test(false); // 无噪点测试
    run_components_test(true);
    run_components_test(false);
}

void detect_from_input()
{
    ESP_LOGI(TAG, "开始处理input.h中的图像数据...");

    Mat img(120, 160, CV_8UC1, (void *)input);
    ESP_LOGI(TAG, "图像尺寸: %dx%d", img.cols, img.rows);

    ESP_LOGI(TAG, "=== 方案1: 轮廓检测 (findContours) ===");
    detect_with_contours(img);

    ESP_LOGI(TAG, "=== 方案2: 连通域检测 (connectedComponents) ===");
    detect_with_components(img);
}

void detect_with_contours(const Mat &img)
{
    // 1. 对特定input.h图像进行预处理
    Mat inverted_img, morph;
    bitwise_not(img, inverted_img);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(inverted_img, morph, MORPH_OPEN, kernel);

    // 2. 为此特定检测任务配置参数
    ContourFindParams params;
    params.selection_method = ContourFindParams::Selection::CLOSEST_TO_POINT;
    params.expected_pos = {71, 73};
    params.min_area = 500;
    params.max_area = 5000;
    params.min_circularity = 0.5;
    params.min_radius = 20;
    params.max_radius = 40;

    // 3. 调用核心函数
    Circle ball = find_ball_by_contour(morph, params);

    // 4. 记录结果
    if (ball.found)
    {
        ESP_LOGI(TAG, "检测到圆形物体:");
        ESP_LOGI(TAG, "  中心坐标: (%.1f, %.1f)", ball.center.x, ball.center.y);
        ESP_LOGI(TAG, "  半径: %.1f", ball.radius);
    }
    else
    {
        ESP_LOGI(TAG, "未检测到符合条件的圆形物体");
    }
}

void detect_with_components(const Mat &img)
{
    // 1. Pre-processing for the specific input.h image
    Mat inverted_img, morph;
    bitwise_not(img, inverted_img);
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    morphologyEx(inverted_img, morph, MORPH_OPEN, kernel);

    // 2. Configure parameters for this specific detection task
    ContourFindParams params;
    params.selection_method = ContourFindParams::Selection::CLOSEST_TO_POINT;
    params.expected_pos = {71, 73};
    params.min_area = 500;
    params.max_area = 5000;
    // min_circularity is not directly applicable, we use aspect ratio inside the function
    params.min_radius = 20;
    params.max_radius = 40;

    // 3. Call the core function
    Circle ball = find_ball_by_components(morph, params);

    // 4. Log the results
    if (ball.found)
    {
        ESP_LOGI(TAG, "检测到圆形物体:");
        ESP_LOGI(TAG, "  中心坐标: (%.1f, %.1f)", ball.center.x, ball.center.y);
        ESP_LOGI(TAG, "  半径: %.1f", ball.radius);
    }
    else
    {
        ESP_LOGI(TAG, "未检测到符合条件的圆形物体");
    }
}

// =================================================================================================
// 测试实现部分
// =================================================================================================

void run_contour_test(bool with_noise)
{
    const int W = 160, H = 120, N = 100;
    const int NOISE_LEVEL = 300;
    int ok = 0;
    int64_t t_total = 0;

    // 对于此测试，我们只需要最大轮廓，因此使用默认参数。
    ContourFindParams params;
    params.selection_method = ContourFindParams::Selection::LARGEST_AREA;

    for (int i = 0; i < N; ++i)
    {
        GroundTruth gt;
        Mat img = create_test_image(W, H, gt);
        Mat processed_img;

        if (with_noise)
        {
            add_salt_and_pepper_noise(img, NOISE_LEVEL);
            Mat bin, morph;
            threshold(img, bin, 0, 255, THRESH_BINARY | THRESH_OTSU);
            Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
            morphologyEx(bin, morph, MORPH_OPEN, kernel);
            processed_img = morph;
        }
        else
        {
            threshold(img, processed_img, 0, 255, THRESH_BINARY | THRESH_OTSU);
        }

        int64_t t0 = esp_timer_get_time();
        Circle circle = find_ball_by_contour(processed_img, params);
        t_total += esp_timer_get_time() - t0;

        const float tolerance = with_noise ? 3.0f : 1.0f;
        if (circle.found && norm(circle.center - Point2f(gt.cx, gt.cy)) < tolerance &&
            fabs(circle.radius - gt.r) < tolerance)
        {
            ok++;
        }
    }

    float avg_ms = t_total / 1000.0f / N;
    ESP_LOGI(TAG, "findContours (%s): %.1f fps, 准确率 %.1f%%", with_noise ? "带噪点" : "无噪点", 1000.0f / avg_ms,
             100.0f * ok / N);
}

void run_components_test(bool with_noise)
{
    const int W = 160, H = 120, N = 100;
    const int NOISE_LEVEL = 300;
    int ok = 0;
    int64_t t_total = 0;

    ContourFindParams params;
    params.selection_method = ContourFindParams::Selection::LARGEST_AREA;

    for (int i = 0; i < N; ++i)
    {
        GroundTruth gt;
        Mat img = create_test_image(W, H, gt);
        Mat processed_img;

        if (with_noise)
        {
            add_salt_and_pepper_noise(img, NOISE_LEVEL);
            Mat bin, morph;
            threshold(img, bin, 0, 255, THRESH_BINARY | THRESH_OTSU);
            Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
            morphologyEx(bin, morph, MORPH_OPEN, kernel);
            processed_img = morph;
        }
        else
        {
            threshold(img, processed_img, 0, 255, THRESH_BINARY | THRESH_OTSU);
        }

        int64_t t0 = esp_timer_get_time();
        Circle circle = find_ball_by_components(processed_img, params);
        t_total += esp_timer_get_time() - t0;

        const float tolerance = with_noise ? 3.0f : 1.0f;
        if (circle.found && norm(circle.center - Point2f(gt.cx, gt.cy)) < tolerance &&
            fabs(circle.radius - gt.r) < tolerance)
        {
            ok++;
        }
    }

    float avg_ms = t_total / 1000.0f / N;
    ESP_LOGI(TAG, "connectedComponents (%s): %.1f fps, 准确率 %.1f%%", with_noise ? "带噪点" : "无噪点",
             1000.0f / avg_ms, 100.0f * ok / N);
}

// =================================================================================================
// 低层次工具部分
// =================================================================================================

cv::Mat create_test_image(int w, int h, GroundTruth &gt)
{
    Mat img(h, w, CV_8UC1, Scalar(0));
    gt.cx = w * (300 + esp_random() % 400) / 1000;
    gt.cy = h * (300 + esp_random() % 400) / 1000;
    gt.r = 15 + esp_random() % 25;
    circle(img, Point(gt.cx, gt.cy), gt.r, Scalar(255), -1);
    return img;
}

void add_salt_and_pepper_noise(Mat &img, int n)
{
    for (int k = 0; k < n; ++k)
    {
        int i = esp_random() % img.cols;
        int j = esp_random() % img.rows;
        if (img.type() == CV_8UC1)
        {
            img.at<uint8_t>(j, i) = (esp_random() % 2) * 255;
        }
    }
}

} // namespace ImageDetector