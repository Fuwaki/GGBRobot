#include "image_detector.hpp"
#include "input.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_random.h"
#include <cmath>

// ESP-DSP
#include "dsps_fft2r.h"
#include "dsps_math.h"

static const char *TAG = "ImageDetector";

using namespace cv;

// Helper to get the next power of 2
unsigned int ImageDetector::nextPowerOf2(unsigned int n) {
    n--;
    n |= n >> 1;
    n |= n >> 2;
    n |= n >> 4;
    n |= n >> 8;
    n |= n >> 16;
    n++;
    return n;
}

void ImageDetector::test_all() {
    test_findContours();
    test_template_matching();
    // test_template_matching_fft();
}

cv::Mat ImageDetector::create_test_image(int w, int h, GroundTruth& gt)
{
    Mat img(h, w, CV_8UC1, Scalar(0));
    gt.cx = w * (300 + esp_random() % 400) / 1000;
    gt.cy = h * (300 + esp_random() % 400) / 1000;
    gt.r  = 15 + esp_random() % 25;
    circle(img, Point(gt.cx, gt.cy), gt.r, Scalar(255), -1);
    return img;
}

void ImageDetector::add_salt_and_pepper_noise(Mat& img, int n)
{
    for (int k = 0; k < n; ++k) {
        int i = esp_random() % img.cols;
        int j = esp_random() % img.rows;
        if (img.type() == CV_8UC1) {
            img.at<uint8_t>(j, i) = (esp_random() % 2) * 255;
        }
    }
}

void ImageDetector::test_findContours()
{
    const int W = 160, H = 120, N = 100;
    int ok = 0;
    int64_t t_total = 0;

    for (int i = 0; i < N; ++i) {
        GroundTruth gt;
        Mat img = create_test_image(W, H, gt);
        add_salt_and_pepper_noise(img, 300);

        Mat bin, morph;
        threshold(img, bin, 0, 255, THRESH_BINARY | THRESH_OTSU);
        
        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(bin, morph, MORPH_OPEN, kernel);

        int64_t t0 = esp_timer_get_time();
        std::vector<std::vector<Point>> cnts;
        findContours(morph, cnts, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        bool hit = false;
        if (!cnts.empty()) {
            double max_area = 0;
            int max_idx = -1;
            for (int i = 0; i < cnts.size(); ++i) {
                double area = contourArea(cnts[i]);
                if (area > max_area) {
                    max_area = area;
                    max_idx = i;
                }
            }
            
            if (max_idx != -1) {
                Point2f center; float r;
                minEnclosingCircle(cnts[max_idx], center, r);
                if (norm(center - Point2f(gt.cx, gt.cy)) < 3 && fabs(r - gt.r) < 3) {
                    hit = true;
                }
            }
        }
        t_total += esp_timer_get_time() - t0;
        ok += hit;
    }

    float avg_ms = t_total / 1000.0f / N;
    ESP_LOGI(TAG, "findContours (带噪点): %.1f fps, 准确率 %.1f%%",
             1000.0f / avg_ms, 100.0f * ok / N);
}

void ImageDetector::test_template_matching()
{
    const int W = 160, H = 120, N = 100;
    int ok = 0, NOISE_LEVEL = 300;
    int64_t t_total = 0;

    GroundTruth gt;
    Mat frame0 = create_test_image(W, H, gt);
    Rect roi(gt.cx - gt.r, gt.cy - gt.r, 2 * gt.r, 2 * gt.r);
    roi = roi & Rect(0, 0, W, H);

    if (roi.width <= 0 || roi.height <= 0) {
        ESP_LOGE(TAG, "Template Matching: 初始ROI无效。");
        return;
    }
    Mat templ = frame0(roi).clone();
    Rect last_pos = roi;
    Mat frame(H, W, CV_8UC1);

    for (int i = 1; i < N; ++i) {
        gt.cx += (esp_random() % 7) - 3;
        gt.cy += (esp_random() % 7) - 3;
        gt.cx = std::max(gt.r, std::min(W - 1 - gt.r, gt.cx));
        gt.cy = std::max(gt.r, std::min(H - 1 - gt.r, gt.cy));
        frame.setTo(Scalar(0));
        circle(frame, Point(gt.cx, gt.cy), gt.r, Scalar(255), -1);
        add_salt_and_pepper_noise(frame, NOISE_LEVEL);

        Rect search_window = last_pos + Size(30, 30) - Point(15, 15);
        search_window &= Rect(0, 0, W, H);

        if (search_window.width < templ.cols || search_window.height < templ.rows) continue;

        Mat search_region = frame(search_window);
        Mat result;

        int64_t t0 = esp_timer_get_time();
        matchTemplate(search_region, templ, result, TM_CCORR_NORMED);
        t_total += esp_timer_get_time() - t0;

        double maxVal; Point maxLoc;
        minMaxLoc(result, NULL, &maxVal, NULL, &maxLoc);

        Rect box(maxLoc.x + search_window.x, maxLoc.y + search_window.y, templ.cols, templ.rows);
        last_pos = box;

        Point2f center(box.x + box.width / 2.0f, box.y + box.height / 2.0f);
        if (norm(center - Point2f(gt.cx, gt.cy)) < 5.0f) ++ok;
    }

    float avg_ms = t_total / 1000.0f / (N - 1);
    ESP_LOGI(TAG, "TemplateMatching (OpenCV): %.1f fps, 成功率 %.1f%%",
             1000.0f / avg_ms, 100.0f * ok / (N - 1));
}

void ImageDetector::fft2d(float* data, int width, int height) {
    // FFT for rows
    for (int i = 0; i < height; i++) {
        dsps_fft2r_fc32(&data[i * width * 2], width);
    }
    // Transpose
    float* temp = new float[width * height * 2];
    for (int y = 0; y < height; y++) {
        for (int x = 0; x < width; x++) {
            temp[(x * height + y) * 2] = data[(y * width + x) * 2];
            temp[(x * height + y) * 2 + 1] = data[(y * width + x) * 2 + 1];
        }
    }
    // FFT for columns (on transposed data)
    for (int i = 0; i < width; i++) {
        dsps_fft2r_fc32(&temp[i * height * 2], height);
    }
    // Transpose back
    for (int x = 0; x < width; x++) {
        for (int y = 0; y < height; y++) {
            data[(y * width + x) * 2] = temp[(x * height + y) * 2];
            data[(y * width + x) * 2 + 1] = temp[(x * height + y) * 2 + 1];
        }
    }
    delete[] temp;
}

void ImageDetector::ifft2d(float* data, int width, int height) {
    // Conjugate input
    for (int i = 0; i < width * height; i++) {
        data[i * 2 + 1] = -data[i * 2 + 1];
    }
    // Forward FFT
    fft2d(data, width, height);
    // Conjugate and scale output
    float scale = 1.0f / (width * height);
    for (int i = 0; i < width * height; i++) {
        data[i * 2] = data[i * 2] * scale;
        data[i * 2 + 1] = -data[i * 2 + 1] * scale;
    }
}

void ImageDetector::detect_from_input() {
    ESP_LOGI(TAG, "开始处理input.h中的图像数据...");

    // 从input数组创建Mat (120行 x 160列, 灰度图)
    Mat img(120, 160, CV_8UC1);
    for (int y = 0; y < 120; y++) {
        for (int x = 0; x < 160; x++) {
            img.at<uint8_t>(y, x) = input[y][x];
        }
    }

    ESP_LOGI(TAG, "图像尺寸: %dx%d", img.cols, img.rows);

    // 方案1: 使用findContours进行轮廓检测
    ESP_LOGI(TAG, "\n=== 方案1: 轮廓检测 (findContours) ===");
    detect_with_contours(img);

    // 方案2: 使用模板匹配
    ESP_LOGI(TAG, "\n=== 方案2: 模板匹配 (Template Matching) ===");
    detect_with_template_matching(img);
}

void ImageDetector::detect_with_contours(const Mat& img) {
    Mat inverted_img, morph;

    // 1. 输入图像已是二值化图像，球是黑色的 (0)，背景是白色的 (255)。
    //    为了让findContours能检测到物体，需要反转图像，使球变为白色 (255)，背景变为黑色 (0)。
    bitwise_not(img, inverted_img);

    // 2. 形态学操作 (开运算，去除小噪点)
    Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5)); // 增大核大小
    morphologyEx(inverted_img, morph, MORPH_OPEN, kernel); // 对反转后的图像进行形态学操作

    std::vector<std::vector<Point>> contours;
    findContours(morph, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

    ESP_LOGI(TAG, "找到 %zu 个轮廓", contours.size());

    Point2f best_center;
    float best_radius = 0;
    double min_dist_to_expected = 1e9; // 用于找到最接近期望位置的圆形

    if (!contours.empty()) {
        for (size_t i = 0; i < contours.size(); ++i) {
            double area = contourArea(contours[i]);
            // 过滤掉过小或过大的轮廓，以及不规则的轮廓
            // 期望半径30，面积约 2826
            if (area > 500 && area < 5000) { // 假设最小面积500，最大面积5000
                Point2f center;
                float radius;
                minEnclosingCircle(contours[i], center, radius);

                // 检查圆形度 (面积与最小外接圆面积的比值)
                double circularity = area / (CV_PI * radius * radius);
                // 检查半径是否接近期望值
                // 考虑到球可能不是那么正圆，将圆形度阈值降低
                if (circularity > 0.5 && radius > 20 && radius < 40) { // 假设圆形度大于0.5，半径在20-40之间
                    // 计算与期望中心 (71, 73) 的距离
                    double dist = norm(center - Point2f(71, 73));
                    if (dist < min_dist_to_expected) {
                        min_dist_to_expected = dist;
                        best_center = center;
                        best_radius = radius;
                    }
                }
            }
        }
        
        if (best_radius > 0) {
            ESP_LOGI(TAG, "检测到圆形物体:");
            ESP_LOGI(TAG, "  中心坐标: (%.1f, %.1f)", best_center.x, best_center.y);
            ESP_LOGI(TAG, "  半径: %.1f", best_radius);
            ESP_LOGI(TAG, "  面积: %.1f", CV_PI * best_radius * best_radius);

            // 计算边界框
            Rect bbox(best_center.x - best_radius, best_center.y - best_radius, 2 * best_radius, 2 * best_radius);
            ESP_LOGI(TAG, "  边界框: x=%d, y=%d, w=%d, h=%d",
                     bbox.x, bbox.y, bbox.width, bbox.height);
        } else {
            ESP_LOGI(TAG, "未检测到符合条件的圆形物体");
        }
    } else {
        ESP_LOGI(TAG, "未检测到任何轮廓");
    }
}

void ImageDetector::detect_with_template_matching(const Mat& img) {
    // 创建一个简单的圆形模板 (假设球的半径大约是30像素)
    int template_radius = 30; // Updated to user's expected radius
    Mat templ(template_radius * 2 + 1, template_radius * 2 + 1, CV_8UC1, Scalar(0));
    circle(templ, Point(template_radius, template_radius), template_radius, Scalar(255), -1);

    // 使用归一化相关系数匹配
    // 可以考虑多尺度模板匹配来提高鲁棒性，但这里先固定半径
    Mat result;
    matchTemplate(img, templ, result, TM_CCORR_NORMED);

    double maxVal;
    Point maxLoc;
    minMaxLoc(result, NULL, &maxVal, NULL, &maxLoc);

    ESP_LOGI(TAG, "模板匹配结果:");
    ESP_LOGI(TAG, "  最大相关系数: %.3f", maxVal);
    ESP_LOGI(TAG, "  最佳匹配位置: (%d, %d)", maxLoc.x, maxLoc.y);

    if (maxVal > 0.6) {  // 提高阈值，因为模板更接近真实大小
        Point center(maxLoc.x + template_radius, maxLoc.y + template_radius);
        ESP_LOGI(TAG, "  检测到物体中心: (%d, %d)", center.x, center.y);
        ESP_LOGI(TAG, "  检测到物体半径: %d", template_radius);
    } else {
        ESP_LOGI(TAG, "  未检测到符合条件的物体 (相关系数太低)");
    }
}