#include "stream_server.hpp"
#include "camera_module.hpp"
#include "image_processor.hpp"
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

// 函数前向声明
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
static void tcp_server_task(void* pvParameters);
static bool send_frame_data(int sock, camera_fb_t *fb);

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
    int ip_protocol = 0;
    struct sockaddr_in dest_addr;

    dest_addr.sin_addr.s_addr = htonl(INADDR_ANY);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(server_port);
    ip_protocol = IPPROTO_IP;

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

        if (source_addr.sin_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr.s_addr, addr_str, sizeof(addr_str) - 1);
        }
        ESP_LOGI(TAG, "套接字已接受来自IP地址的连接: %s", addr_str);

        while (1) {
            camera_fb_t *fb = CameraModule::get_frame();
            if (!fb) {
                ESP_LOGE(TAG, "摄像头捕获失败");
                int error = 0;
                socklen_t len = sizeof(error);
                int retval = getsockopt(sock, SOL_SOCKET, SO_ERROR, &error, &len);
                if (retval != 0 || error != 0) {
                    ESP_LOGI(TAG, "客户端已断开连接。");
                    break;
                }
                vTaskDelay(100 / portTICK_PERIOD_MS);
                continue;
            }

            ImageProcessor::binarize(fb, 130);

            if (!send_frame_data(sock, fb)) {
                ESP_LOGI(TAG, "发送帧失败，客户端已断开连接。");
                CameraModule::return_frame(fb);
                break;
            }

            CameraModule::return_frame(fb);
        }

        shutdown(sock, 0);
        close(sock);
    }

CLEAN_UP:
    close(listen_sock);
    vTaskDelete(NULL);
}

static bool send_frame_data(int sock, camera_fb_t *fb) {
    uint32_t width = fb->width;
    uint32_t height = fb->height;
    
    int to_write = sizeof(width);
    char* p = (char*)&width;
    while (to_write > 0) {
        int written = send(sock, p, to_write, 0);
        if (written < 0) return false;
        to_write -= written;
        p += written;
    }

    to_write = sizeof(height);
    p = (char*)&height;
    while (to_write > 0) {
        int written = send(sock, p, to_write, 0);
        if (written < 0) return false;
        to_write -= written;
        p += written;
    }

    to_write = fb->len;
    p = (char*)fb->buf;
    while (to_write > 0) {
        int written = send(sock, p, to_write, 0);
        if (written < 0) return false;
        to_write -= written;
        p += written;
    }

    return true;
}

} // namespace StreamServer