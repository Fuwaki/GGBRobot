/*
 * SPDX-FileCopyrightText: 2022-2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "stream_server.hpp"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <cstring>

namespace StreamServer {

static const char* TAG = "视频流服务器";

// 用于发出Wi-Fi连接信号的事件组
static EventGroupHandle_t wifi_event_group;
const int WIFI_CONNECTED_BIT = BIT0;

static uint16_t server_port;
static int client_socket = -1; // 将客户端套接字设为全局，以便在任务外部访问

// 函数前向声明
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void tcp_server_task(void* pvParameters);
static bool send_data(int sock, const void* data, size_t len);

void init_wifi_and_start_server(const char* ssid, const char* password, uint16_t port) {
    server_port = port;

    // 1. 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 2. 初始化网络协议栈
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_t *sta_netif = esp_netif_create_default_wifi_sta();
    assert(sta_netif);

    // 3. 初始化Wi-Fi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 4. 设置事件处理器
    wifi_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL));
    ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL));

    // 5. 配置并启动Wi-Fi
    wifi_config_t wifi_config = {};
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "等待WiFi连接...");
    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    ESP_LOGI(TAG, "WiFi已连接。");

    // 6. 启动TCP服务器任务
    xTaskCreate(tcp_server_task, "tcp_server", 4096, NULL, 5, NULL);
}

bool is_client_connected() {
    return client_socket != -1;
}

bool send_image(const cv::Mat& img) {
    if (client_socket < 0 || img.empty()) {
        return false;
    }

    uint32_t width = img.cols;
    uint32_t height = img.rows;
    uint32_t len = img.total() * img.elemSize();

    if (!send_data(client_socket, &width, sizeof(width))) return false;
    if (!send_data(client_socket, &height, sizeof(height))) return false;
    if (!send_data(client_socket, img.data, len)) return false;

    return true;
}

bool send_detection_result(const ImageDetector::Circle& result) {
    if (client_socket < 0) {
        return false;
    }
    // 手动序列化以避免结构体填充问题
    if (!send_data(client_socket, &result.center, sizeof(result.center))) return false; // 8 bytes (2x float)
    if (!send_data(client_socket, &result.radius, sizeof(result.radius))) return false; // 4 bytes (float)
    if (!send_data(client_socket, &result.found, sizeof(result.found))) return false;   // 1 byte (bool)
    return true;
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        ESP_LOGI(TAG, "WiFi连接已断开，正在尝试重新连接...");
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "获取到IP地址:" IPSTR, IP2STR(&event->ip_info.ip));
        xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

static void tcp_server_task(void* pvParameters) {
    char addr_str[128];
    int addr_family = AF_INET;
    int ip_protocol = IPPROTO_IP;
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(server_port);

    int listen_sock = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (listen_sock < 0) {
        ESP_LOGE(TAG, "无法创建套接字: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    ESP_LOGI(TAG, "套接字已创建");

    int err = bind(listen_sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "套接字无法绑定: errno %d", errno);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "套接字已绑定, 端口 %d", server_port);

    err = listen(listen_sock, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "监听时发生错误: errno %d", errno);
        goto CLEAN_UP;
    }

    while (1) {
        ESP_LOGI(TAG, "套接字正在监听");

        struct sockaddr_in source_addr;
        socklen_t addr_len = sizeof(source_addr);
        int sock = accept(listen_sock, (struct sockaddr *)&source_addr, &addr_len);
        if (sock < 0) {
            ESP_LOGE(TAG, "无法接受连接: errno %d", errno);
            break;
        }

        inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        ESP_LOGI(TAG, "套接字已接受来自IP地址的连接: %s", addr_str);
        client_socket = sock;

        // 保持连接打开，但在此任务中不执行任何操作。
        // 主循环将使用send_image()发送数据。
        // 我们通过检查套接字错误来检测断开连接。
        while (1) {
            int error = 0;
            socklen_t len = sizeof(error);
            int retval = getsockopt(client_socket, SOL_SOCKET, SO_ERROR, &error, &len);
            if (retval != 0 || error != 0) {
                ESP_LOGI(TAG, "客户端已断开连接。 error: %d", error);
                break; // 退出内部循环以接受新连接
            }
            vTaskDelay(pdMS_TO_TICKS(1000)); // 每秒检查一次连接
        }

        // 客户端断开连接，重置套接字并准备接受新连接
        shutdown(client_socket, 0);
        close(client_socket);
        client_socket = -1;
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

static bool send_data(int sock, const void* data, size_t len) {
    const char* p = (const char*)data;
    int to_write = len;
    while (to_write > 0) {
        int written = send(sock, p, to_write, 0);
        if (written < 0) {
            ESP_LOGE(TAG, "发送数据时出错: errno %d", errno);
            return false;
        }
        to_write -= written;
        p += written;
    }
    return true;
}

} // namespace StreamServer