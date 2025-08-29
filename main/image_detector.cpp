#include "image_detector.hpp"
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
