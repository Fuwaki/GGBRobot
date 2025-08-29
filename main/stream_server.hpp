#pragma once

#include "esp_camera.h"

namespace StreamServer {

/**
 * @brief 初始化WiFi，连接到指定的SSID，并启动TCP流服务器。
 *
 * @param ssid WiFi名称。
 * @param password WiFi密码。
 * @param port TCP服务器端口。
 */
void init_wifi_and_start_server(const char* ssid, const char* password, uint16_t port);

} // namespace StreamServer