#pragma once

#include "image_detector.hpp" 
#include <opencv2/opencv.hpp>

/**
 * @brief Wi-Fi和TCP视频流服务器模块
 */
namespace StreamServer {

/**
 * @brief 初始化WiFi,连接到指定的SSID,并启动TCP流服务器。
 *
 * @param ssid WiFi网络名称 (SSID)。
 * @param password WiFi密码。
 * @param port 用于监听客户端连接的TCP端口。
 */
void init_wifi_and_start_server(const char* ssid, const char* password, uint16_t port);

/**
 * @brief 通过已建立的TCP连接发送一个OpenCV Mat对象。
 * 
 * @param img 要发送的图像 (通常是BGR格式以便在客户端显示)。
 * @return true 如果发送成功。
 * @return false 如果未连接或发送失败。
 */
bool send_image(const cv::Mat& img);

/**
 * @brief (已弃用) 发送检测结果。
 * 
 * 注意: 当前的实现中, 检测结果直接绘制在图像上并通过 send_image 发送。
 * 这个函数保留用于未来可能的、将数据和图像分开传输的场景。
 * 
 * @param result 检测到的圆。
 * @return true 如果发送成功。
 * @return false 如果未连接或发送失败。
 */
bool send_detection_result(const ImageDetector::Circle& result);

/**
 * @brief 检查是否有客户端正连接到服务器。
 * 
 * @return true 如果有客户端连接。
 * @return false 如果没有客户端连接。
 */
bool is_client_connected();

} // namespace StreamServer
