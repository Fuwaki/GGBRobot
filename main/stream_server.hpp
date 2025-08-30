#pragma once

#include "esp_camera.h"
#include "image_detector.hpp" 
#include <opencv2/opencv.hpp>

namespace StreamServer {

/**
 * @brief 初始化WiFi，连接到指定的SSID，并启动TCP流服务器。
 *
 * @param ssid WiFi名称。
 * @param password WiFi密码。
 * @param port TCP服务器端口。
 */
void init_wifi_and_start_server(const char* ssid, const char* password, uint16_t port);

/**
 * @brief 通过已建立的TCP连接发送一个OpenCV Mat对象。
 * 
 * @param img 要发送的图像。
 * @return 如果发送成功则为true，否则为false。
 */
bool send_image(const cv::Mat& img);

/**
 * @brief 发送检测结果。
 * 
 * @param result 检测到的圆。
 * @return 如果发送成功则为true，否则为false。
 */
bool send_detection_result(const ImageDetector::Circle& result);

/**
 * @brief 检查是否有客户端连接。
 * 
 * @return 如果有客户端连接则为true，否则为false。
 */
bool is_client_connected();

} // namespace StreamServer