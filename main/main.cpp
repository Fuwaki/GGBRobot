#include "Arduino.h"
#include "camera.hpp"
#include <WiFi.h>

// --- 请修改为您自己的Wi-Fi信息 ---
const char* ssid = "Fuwaki's";
const char* password = "114514qwq";
// ------------------------------------

#define SERVER_PORT 8080
WiFiServer server(SERVER_PORT);

/**
 * @brief 示例图像处理算法：反转图像颜色
 * 这是一个可以直接操作像素缓冲区的函数。
 * @param image_buffer 指向图像数据的指针 (uint8_t*)
 * @param width 图像宽度
 * @param height 图像高度
 */
void process_image_invert(uint8_t *image_buffer, int width, int height) {
    // 缓冲区是一个一维数组，大小为 width * height
    // 访问 (x, y) 处的像素: buffer[y * width + x]
    size_t len = width * height;
    for (size_t i = 0; i < len; i++) {
        image_buffer[i] = image_buffer[i]>130?255:0; 
    }
}

/**
 * @brief 捕获一帧图像，进行处理，并通过TCP客户端发送
 * @param client 连接的WiFi客户端
 */
void stream_image(WiFiClient &client) {
    if (!client.connected()) {
        return;
    }

    camera_fb_t * fb = esp_camera_fb_get();
    if (!fb) {
        ESP_LOGE(TAG, "摄像头捕获失败");
        return;
    }

    // --- 在这里运行你的图像处理算法 ---
    // fb->buf 是一个指向 uint8_t 数组的指针，即你的像素数据。
    // fb->width 和 fb->height 是图像的尺寸。
    // 因为我们设置了 PIXFORMAT_GRAYSCALE，每个像素是1个字节。
    process_image_invert((uint8_t *)fb->buf, fb->width, fb->height);

    // 新协议: 先发送4字节宽度，再发送4字节高度，最后发送原始图像数据
    uint32_t width = fb->width;
    uint32_t height = fb->height;

    // 1. 发送图像宽度 (4字节, little-endian)
    if (client.write((char *)&width, 4) != 4) {
        ESP_LOGE(TAG, "发送图像宽度失败");
        esp_camera_fb_return(fb);
        return;
    }

    // 2. 发送图像高度 (4字节, little-endian)
    if (client.write((char *)&height, 4) != 4) {
        ESP_LOGE(TAG, "发送图像高度失败");
        esp_camera_fb_return(fb);
        return;
    }

    // 3. 发送处理后的灰度图像数据
    if (client.write(fb->buf, fb->len) != fb->len) {
        ESP_LOGE(TAG, "发送图像数据失败");
    }

    esp_camera_fb_return(fb);
}

extern "C" void app_main()
{
    initArduino();
    pinMode(7, OUTPUT);
    digitalWrite(7, HIGH);

    WiFi.begin(ssid, password);
    ESP_LOGI(TAG, "正在连接到WiFi: %s", ssid);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);
    }
    Serial.println("");

    ESP_LOGI(TAG, "WiFi已连接. IP地址: %s", WiFi.localIP().toString().c_str());

    if (camera_init() == ESP_OK) {
        server.begin();
        ESP_LOGI(TAG, "TCP服务器已启动，端口: %d. 等待客户端连接...", SERVER_PORT);
        while(true) {
            WiFiClient client = server.available();
            if (client) {
                ESP_LOGI(TAG, "客户端已连接: %s", client.remoteIP().toString().c_str());
                while (client.connected()) {
                    stream_image(client);
                }
                client.stop();
                ESP_LOGI(TAG, "客户端已断开连接.");
            } else {
                // 当没有客户端连接时，添加一个短暂的延时。
                // 这可以防止主循环持续占用CPU，从而让IDLE任务有机会运行并重置看门狗定时器。
                delay(10);
            }
        }
    }
}