import cv2
import numpy as np
import socket
import struct
import sys

# --- 配置 ---
# 在运行脚本时，将ESP32的IP地址作为第一个参数传入
# 例如: python receive_and_display.py 192.168.1.100
if len(sys.argv) < 2:
    print("用法: python receive_and_display.py <ESP32_IP_ADDRESS>")
    sys.exit(1)

HOST = sys.argv[1]
PORT = 8080
# ------------

def main():
    """主函数，用于连接、接收和显示图像流"""
    print(f"正在尝试连接到 {HOST}:{PORT}...")
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
            header_data = b''
            while len(header_data) < 8: # 4字节宽度 + 4字节高度
                chunk = client_socket.recv(8 - len(header_data))
                if not chunk:
                    raise ConnectionError("连接已断开 (接收尺寸失败)")
                header_data += chunk
            
            width, height = struct.unpack('<II', header_data)
            img_size = width * height # 对于灰度图，每个像素1字节
            if img_size == 0:
                print("收到无效尺寸，跳过此帧。")
                continue
            # print(f"准备接收图像: {width}x{height}, 大小: {img_size} 字节...")

            # 2. 根据大小接收完整的原始图像数据
            img_data = b''
            while len(img_data) < img_size:
                chunk = client_socket.recv(img_size - len(img_data))
                if not chunk:
                    raise ConnectionError("连接已断开 (接收数据失败)")
                img_data += chunk

            # 3. 从原始数据构建图像并显示
            np_array = np.frombuffer(img_data, dtype=np.uint8)
            frame = np_array.reshape((height, width))

            if frame is not None:
                # print("图像构建成功，正在显示。") # 注释掉以避免刷屏
                cv2.imshow('ESP32 Camera Stream', frame)
            else:
                print("图像构建失败！")
            
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except (ConnectionError, socket.error) as e:
        print(f"错误: {e}")
    finally:
        print("关闭连接。")
        client_socket.close()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
