

#include "driver/gpio.h"
#include "esp_log.h"
#include <array>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

// FreeRTOS
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Project Modules
#include "camera_module.hpp"
#include "image_detector.hpp"
#include "image_processor.hpp"
#include "kinematics_solver.hpp"
#include "servo_controller.hpp"
#include "stream_server.hpp"
#include "PID.hpp"
#include "timing_profiler.hpp"

// --- Configuration ---
// Image source: Choose one
#define IMAGE_SOURCE_CAMERA 1
#define IMAGE_SOURCE_GENERATED 2
#define IMAGE_SOURCE IMAGE_SOURCE_CAMERA

// Output target: Choose one
#define OUTPUT_REALTIME_CONTROL 1
#define OUTPUT_STREAM 2
#define OUTPUT_TARGET OUTPUT_REALTIME_CONTROL

// Enable timing profiling for performance measurement
#define ENABLE_TIMING_PROFILE
// -------------------

// --- Wi-Fi Credentials (only for STREAM output) ---
const char *ssid = "Fuwaki's";
const char *password = "114514qwq";
// -------------------

#define SERVER_PORT 8080

static const char *TAG = "MainApp";

// --- LED Indicator Pins ---
const gpio_num_t led_a_pin = GPIO_NUM_7;
const gpio_num_t led_b_pin = GPIO_NUM_15;

// --- Task Prototypes ---
void realtime_control_task(void *pvParameters);
void streaming_task(void *pvParameters);

// --- Servo Control ---
void apply_servo(std::array<float, 3> angles) {
    static ServoController::Servo servo_a(GPIO_NUM_1, LEDC_TIMER_0, LEDC_CHANNEL_0);
    static ServoController::Servo servo_b(GPIO_NUM_2, LEDC_TIMER_0, LEDC_CHANNEL_1);
    static ServoController::Servo servo_c(GPIO_NUM_3, LEDC_TIMER_0, LEDC_CHANNEL_2);

    float compensations[3] = {66, 85, 40};
    servo_a.change_angle(angles[2] + compensations[0]);
    servo_b.change_angle(angles[1] + compensations[1]);
    servo_c.change_angle(angles[0] + compensations[2]);
}

// --- Main Application Entry Point ---
extern "C" void app_main() {
    // --- Initialization ---
    gpio_set_direction(led_a_pin, GPIO_MODE_OUTPUT);
    gpio_set_direction(led_b_pin, GPIO_MODE_OUTPUT);
    gpio_set_level(led_a_pin, 1);
    gpio_set_level(led_b_pin, 0);

    ESP_LOGI(TAG, "Starting application...");

#if IMAGE_SOURCE == IMAGE_SOURCE_CAMERA || OUTPUT_TARGET == OUTPUT_REALTIME_CONTROL
    esp_err_t cam_err = CameraModule::init();
    if (cam_err != ESP_OK) {
        ESP_LOGE(TAG, "Camera initialization failed with error: 0x%x", cam_err);
        return;
    }
    ESP_LOGI(TAG, "Camera initialized successfully.");
#endif

#if OUTPUT_TARGET == OUTPUT_STREAM
    StreamServer::init_wifi_and_start_server(ssid, password, SERVER_PORT);
#endif

    ESP_LOGI(TAG, "Initialization complete. Creating tasks...");

    // --- Task Creation ---
#if OUTPUT_TARGET == OUTPUT_REALTIME_CONTROL
    xTaskCreate(realtime_control_task, "realtime_control_task", 16000, NULL, 5, NULL);
#elif OUTPUT_TARGET == OUTPUT_STREAM
    xTaskCreate(streaming_task, "streaming_task", 16000, NULL, 5, NULL);
#endif
}

// --- Real-time Control Task ---
void realtime_control_task(void *pvParameters) {
    pid_positional_controller x_controller(0.20, 0.001, 0.02);
    pid_positional_controller y_controller(0.20, 0.001, 0.02);
    x_controller.enable_limit(-30, 30);
    y_controller.enable_limit(-30, 30);

    const uint8_t threshold = 80;
    uint32_t b = 0;

#ifdef ENABLE_TIMING_PROFILE
    TimingProfiler profiler;
#endif

    static int frame_count = 0;
    static int64_t last_time = esp_timer_get_time();

    while (true) {
        b++;
#ifdef ENABLE_TIMING_PROFILE
        profiler.start();
#endif
        gpio_set_level(led_a_pin, b % 2);
#ifdef ENABLE_TIMING_PROFILE
        profiler.end("LED");
#endif

#ifdef ENABLE_TIMING_PROFILE
        profiler.start();
#endif
        cv::Mat grayscale_img = CameraModule::get_grayscale_frame(10);
#ifdef ENABLE_TIMING_PROFILE
        profiler.end("Decode");
#endif
        if (grayscale_img.empty()) {
            continue;
        }

        frame_count++;
        if (frame_count % 100 == 0) {
            int64_t current_time = esp_timer_get_time();
            float fps = 100.0f / ((current_time - last_time) / 1000000.0f);
            ESP_LOGI(TAG, "FPS: %.2f", fps);
#ifdef ENABLE_TIMING_PROFILE
            profiler.log_results(TAG, 100);
            profiler.reset();
#endif
            last_time = current_time;
        }

#ifdef ENABLE_TIMING_PROFILE
        profiler.start();
#endif
        ImageProcessor::binarize(grayscale_img, threshold);
#ifdef ENABLE_TIMING_PROFILE
        profiler.end("Binarize");
#endif

        ImageDetector::ContourFindParams params;
        params.selection_method = ImageDetector::ContourFindParams::Selection::CLOSEST_TO_POINT;
        params.expected_pos = cv::Point2f(grayscale_img.cols / 2.0f, grayscale_img.rows / 2.0f);
        params.min_area = 1000;
        params.max_area = 6000;
        params.min_circularity = 0.3;
        params.min_radius = 20;
        params.max_radius = 50;

#ifdef ENABLE_TIMING_PROFILE
        profiler.start();
#endif
        ImageDetector::Circle result = ImageDetector::find_ball_by_contour(grayscale_img, params);
#ifdef ENABLE_TIMING_PROFILE
        profiler.end("Detect");
#endif

        if (result.found) {
            gpio_set_level(led_b_pin, 1);
#ifdef ENABLE_TIMING_PROFILE
            profiler.start();
#endif
            float x_error = result.center.x - grayscale_img.cols / 2;
            float y_error = result.center.y - grayscale_img.rows / 2;
            
            static float filtered_x_error = 0, filtered_y_error = 0;
            const float alpha = 0.2;
            filtered_x_error = alpha * x_error + (1 - alpha) * filtered_x_error;
            filtered_y_error = alpha * y_error + (1 - alpha) * filtered_y_error;

            static int64_t last_set_error_time_us = 0;
            int64_t now_us = esp_timer_get_time();
            double dt_sec = (last_set_error_time_us == 0) ? 0.02 : (now_us - last_set_error_time_us) / 1000000.0;
            dt_sec = (dt_sec <= 0) ? 0.000001 : dt_sec;
            last_set_error_time_us = now_us;

            x_controller.set_dt(dt_sec);
            y_controller.set_dt(dt_sec);
            x_controller.set_error(-filtered_x_error);
            y_controller.set_error(-filtered_y_error);
#ifdef ENABLE_TIMING_PROFILE
            profiler.end("ErrorCalc");
#endif
        } else {
            gpio_set_level(led_b_pin, 0);
            x_controller.clear();
            y_controller.clear();
        }

#ifdef ENABLE_TIMING_PROFILE
        profiler.start();
#endif
        KinematicsSolver::ServoAngles angles = KinematicsSolver::move_platform(y_controller.get_adjustment(), x_controller.get_adjustment(), 70.0f);
#ifdef ENABLE_TIMING_PROFILE
        profiler.end("Kinematics");
#endif

#ifdef ENABLE_TIMING_PROFILE
        profiler.start();
#endif
        apply_servo({angles.angle_a, angles.angle_b, angles.angle_c});
#ifdef ENABLE_TIMING_PROFILE
        profiler.end("Servo");
#endif
    }
}

// --- Streaming Task ---
void streaming_task(void *pvParameters) {
    const int IMG_WIDTH = 160;
    const int IMG_HEIGHT = 120;
    const uint8_t threshold = 70;

    ImageDetector::ContourFindParams params;
#if IMAGE_SOURCE == IMAGE_SOURCE_GENERATED
    params.selection_method = ImageDetector::ContourFindParams::Selection::LARGEST_AREA;
#else // IMAGE_SOURCE_CAMERA
    params.selection_method = ImageDetector::ContourFindParams::Selection::CLOSEST_TO_POINT;
    params.expected_pos = cv::Point2f(IMG_WIDTH / 2.0f, IMG_HEIGHT / 2.0f);
    params.min_area = 500;
    params.min_circularity = 0.5;
    params.min_radius = 20;
    params.max_radius = 50;
#endif

    while (true) {
        if (!StreamServer::is_client_connected()) {
            ESP_LOGI(TAG, "Waiting for client connection...");
            while (!StreamServer::is_client_connected()) {
                vTaskDelay(pdMS_TO_TICKS(200));
            }
            ESP_LOGI(TAG, "Client connected. Starting stream.");
        }

        cv::Mat img;
#if IMAGE_SOURCE == IMAGE_SOURCE_GENERATED
        ImageDetector::GroundTruth gt;
        img = ImageDetector::create_test_image(IMG_WIDTH, IMG_HEIGHT, gt);
#else // IMAGE_SOURCE_CAMERA
        camera_fb_t *fb = CameraModule::get_frame();
        if (!fb) {
            ESP_LOGE(TAG, "Failed to get camera frame");
            vTaskDelay(pdMS_TO_TICKS(1000));
            continue;
        }
        cv::Mat grayscale_img(fb->height, fb->width, CV_8UC1, fb->buf);
        ImageProcessor::binarize(grayscale_img, threshold);
        img = grayscale_img;
        CameraModule::return_frame(fb); // Return frame early
#endif

        ImageDetector::Circle result = ImageDetector::find_ball_by_contour(img, params);

        if (!StreamServer::send_image(img)) {
            ESP_LOGE(TAG, "Failed to send image, connection may be closed.");
            vTaskDelay(pdMS_TO_TICKS(1100)); // Wait for socket cleanup
            continue;
        }

        if (!StreamServer::send_detection_result(result)) {
            ESP_LOGE(TAG, "Failed to send detection result.");
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // Limit frame rate
    }
}
