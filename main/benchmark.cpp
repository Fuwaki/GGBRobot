#include "benchmark.hpp"
#include "mat.h"
#include <opencv2/opencv.hpp>
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_random.h"
#include <array>

namespace Benchmark {

static const char* TAG = "性能测试";

// --- 辅助函数: 初始化矩阵 ---
void fill_dspm_matrix(dspm::Mat& m) {
    for (int r = 0; r < m.rows; ++r) {
        for (int c = 0; c < m.cols; ++c) {
            m(r, c) = (float)esp_random() / UINT32_MAX;
        }
    }
}

void fill_cv_matrix(cv::Mat& m) {
    cv::randu(m, cv::Scalar::all(0), cv::Scalar::all(1));
}

void run_matrix_benchmark() {
    const int ITERATIONS = 10000;

    ESP_LOGI(TAG, "--- 开始矩阵库性能测试 (迭代次数: %d) ---", ITERATIONS);

    // --- 1. 乘法与减法 (原始测试) ---
    ESP_LOGI(TAG, "--- 测试: 乘法与减法 ---");
    {
        dspm::Mat p(4, 4), r(4, 4), j(4, 4), s(4, 4);
        fill_dspm_matrix(p); fill_dspm_matrix(r); fill_dspm_matrix(j); fill_dspm_matrix(s);
        int64_t start_time = esp_timer_get_time();
        for (int i = 0; i < ITERATIONS; ++i) { dspm::Mat d = (p * r * j) - s; if (d(0,0) < -100) { ESP_LOGI(TAG, "Impossible"); } }
        int64_t total_time_us = esp_timer_get_time() - start_time;
        ESP_LOGI(TAG, "[dspm::Mat (DSP)] 平均每次迭代: %.2f us", (float)total_time_us / ITERATIONS);
    }
    {
        cv::Mat p(4, 4, CV_32F), r(4, 4, CV_32F), j(4, 4, CV_32F), s(4, 4, CV_32F);
        fill_cv_matrix(p); fill_cv_matrix(r); fill_cv_matrix(j); fill_cv_matrix(s);
        int64_t start_time = esp_timer_get_time();
        for (int i = 0; i < ITERATIONS; ++i) { cv::Mat d = (p * r * j) - s; if (d.at<float>(0,0) < -100) { ESP_LOGI(TAG, "Impossible"); } }
        int64_t total_time_us = esp_timer_get_time() - start_time;
        ESP_LOGI(TAG, "[OpenCV]           平均每次迭代: %.2f us", (float)total_time_us / ITERATIONS);
    }

    // --- 2. 加法 ---
    ESP_LOGI(TAG, "--- 测试: 加法 (4x4) ---");
    {
        dspm::Mat a(4, 4), b(4, 4);
        fill_dspm_matrix(a); fill_dspm_matrix(b);
        int64_t start_time = esp_timer_get_time();
        for (int i = 0; i < ITERATIONS; ++i) { dspm::Mat c = a + b; if (c(0,0) < -100) { ESP_LOGI(TAG, "Impossible"); } }
        int64_t total_time_us = esp_timer_get_time() - start_time;
        ESP_LOGI(TAG, "[dspm::Mat (DSP)] 平均每次迭代: %.2f us", (float)total_time_us / ITERATIONS);
    }
    {
        cv::Mat a(4, 4, CV_32F), b(4, 4, CV_32F);
        fill_cv_matrix(a); fill_cv_matrix(b);
        int64_t start_time = esp_timer_get_time();
        for (int i = 0; i < ITERATIONS; ++i) { cv::Mat c = a + b; if (c.at<float>(0,0) < -100) { ESP_LOGI(TAG, "Impossible"); } }
        int64_t total_time_us = esp_timer_get_time() - start_time;
        ESP_LOGI(TAG, "[OpenCV]           平均每次迭代: %.2f us", (float)total_time_us / ITERATIONS);
    }

    // --- 3. 转置 ---
    ESP_LOGI(TAG, "--- 测试: 转置 (4x4) ---");
    {
        dspm::Mat a(4, 4);
        fill_dspm_matrix(a);
        int64_t start_time = esp_timer_get_time();
        for (int i = 0; i < ITERATIONS; ++i) { dspm::Mat at = a.t(); if (at(0,0) < -100) { ESP_LOGI(TAG, "Impossible"); } }
        int64_t total_time_us = esp_timer_get_time() - start_time;
        ESP_LOGI(TAG, "[dspm::Mat]       平均每次迭代: %.2f us", (float)total_time_us / ITERATIONS);
    }
    {
        cv::Mat a(4, 4, CV_32F);
        fill_cv_matrix(a);
        int64_t start_time = esp_timer_get_time();
        for (int i = 0; i < ITERATIONS; ++i) { cv::Mat at = a.t(); if (at.at<float>(0,0) < -100) { ESP_LOGI(TAG, "Impossible"); } }
        int64_t total_time_us = esp_timer_get_time() - start_time;
        ESP_LOGI(TAG, "[OpenCV]           平均每次迭代: %.2f us", (float)total_time_us / ITERATIONS);
    }

    // --- 4. 求逆 ---
    ESP_LOGI(TAG, "--- 测试: 求逆 (4x4) ---");
    {
        dspm::Mat a(4, 4);
        int64_t total_time_us = 0;
        int count = 0;
        for (int i = 0; i < ITERATIONS; ++i) {
            fill_dspm_matrix(a);
            a(0,0) += 1; // 增加对角线元素以提高可逆性
            a(1,1) += 1;
            a(2,2) += 1;
            a(3,3) += 1;
            int64_t start_time = esp_timer_get_time();
            dspm::Mat inv = a.inverse();
            total_time_us += esp_timer_get_time() - start_time;
            if (inv.rows > 0) count++;
        }
        ESP_LOGI(TAG, "[dspm::Mat]       平均每次迭代: %.2f us (成功率: %d/%d)", (float)total_time_us / ITERATIONS, count, ITERATIONS);
    }
    {
        cv::Mat a(4, 4, CV_32F);
        int64_t total_time_us = 0;
        int count = 0;
        for (int i = 0; i < ITERATIONS; ++i) {
            fill_cv_matrix(a);
            a.at<float>(0,0) += 1;
            a.at<float>(1,1) += 1;
            a.at<float>(2,2) += 1;
            a.at<float>(3,3) += 1;
            int64_t start_time = esp_timer_get_time();
            cv::Mat inv = a.inv();
            total_time_us += esp_timer_get_time() - start_time;
            if (cv::sum(inv)[0] != 0) count++;
        }
        ESP_LOGI(TAG, "[OpenCV]           平均每次迭代: %.2f us (成功率: %d/%d)", (float)total_time_us / ITERATIONS, count, ITERATIONS);
    }

    ESP_LOGI(TAG, "--- 性能测试完成 ---");
}

} // namespace Benchmark
