import cv2
import numpy as np
import socket
import struct
import sys
import math

# --- 配置 ---
# 在运行脚本时，将ESP32的IP地址作为第一个参数传入
# 例如: python receive_and_display.py 192.168.1.100
if len(sys.argv) < 2:
    print("用法: python receive_and_display.py <ESP32_IP_ADDRESS>")
    sys.exit(1)

HOST = sys.argv[1]
PORT = 8080
# ------------

def recv_all(sock, n):
    """帮助函数，确保从套接字接收到n个字节"""
    data = b''
    while len(data) < n:
        packet = sock.recv(n - len(data))
        if not packet:
            return None
        data += packet
    return data

def main():
    """主函数，用于连接、接收和显示图像流"""
    print(f"正在尝试连接到 {HOST}:{PORT}...")
    client_socket = None
    try:
        client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        client_socket.connect((HOST, PORT))
        print("连接成功！按 'q' 键退出窗口。")
    except socket.error as e:
        print(f"连接失败: {e}")
        return

    try:
        while True:
            # 1. 接收图像尺寸 (宽度和高度，各4字节, little-endian)
            header_data = recv_all(client_socket, 8)
            if header_data is None:
                raise ConnectionError("连接已断开 (接收尺寸失败)")
            
            width, height = struct.unpack('<II', header_data)
            img_size = width * height # 对于灰度图，每个像素1字节
            if img_size <= 0 or img_size > 50000: # 添加一个合理性检查
                print(f"收到无效尺寸: {width}x{height}，跳过此帧。")
                continue

            # 2. 根据大小接收完整的原始图像数据
            img_data = recv_all(client_socket, img_size)
            if img_data is None:
                raise ConnectionError("连接已断开 (接收数据失败)")

            # 3. 接收检测结果 (center.x, center.y, radius, found)
            # C++端手动发送 8 + 4 + 1 = 13 字节
            result_data = recv_all(client_socket, 13)
            if result_data is None:
                raise ConnectionError("连接已断开 (接收结果失败)")
            
            cx, cy, radius, found = struct.unpack('<ff f ?', result_data)

            # 4. 从原始数据构建图像
            np_array = np.frombuffer(img_data, dtype=np.uint8)
            frame = np_array.reshape((height, width))

            # 5. 可视化
            # 将灰度图转换为彩色图以便绘制彩色标记
            display_frame = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)

            # 在图像中心画一个标记
            img_center_x, img_center_y = width // 2, height // 2
            cv2.drawMarker(display_frame, (img_center_x, img_center_y), (0, 255, 255), cv2.MARKER_CROSS, 10, 1)

            # 如果检测到圆，则绘制它
            if found:
                # 检查值是否有效
                if (math.isnan(cx) or math.isnan(cy) or math.isnan(radius) or
                    math.isinf(cx) or math.isinf(cy) or math.isinf(radius) or
                    radius <= 0 or cx < 0 or cy < 0 or cx >= width or cy >= height):
                    print(f"检测结果无效: cx={cx}, cy={cy}, radius={radius}, 跳过绘制")
                    continue
                # 绘制圆心
                cv2.circle(display_frame, (int(cx), int(cy)), 3, (0, 0, 255), -1) # 红色圆心
                # 绘制圆轮廓
                cv2.circle(display_frame, (int(cx), int(cy)), int(radius), (0, 255, 0), 2) # 绿色轮廓

            # 6. 显示图像
            cv2.imshow('ESP32 Stream', display_frame)
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except (ConnectionError, socket.error, struct.error) as e:
        print(f"错误: {e}")
    finally:
        if client_socket:
            client_socket.close()
        cv2.destroyAllWindows()
        print("连接已关闭。")

if __name__ == '__main__':
    main()