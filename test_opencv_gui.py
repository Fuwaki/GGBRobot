import cv2
import numpy as np

print(f"OpenCV version: {cv2.__version__}")

# 创建一个黑色的空白图像
width, height = 400, 300
blank_image = np.zeros((height, width, 3), np.uint8)

# 在图像上写一些文字
font = cv2.FONT_HERSHEY_SIMPLEX
cv2.putText(blank_image, 'OpenCV GUI Test', (50, 50), font, 1, (255, 255, 255), 2, cv2.LINE_AA)
cv2.putText(blank_image, 'Press Q to quit', (50, 150), font, 1, (0, 255, 0), 2, cv2.LINE_AA)

# 尝试显示图像
try:
    cv2.imshow('Test Window', blank_image)
    print("\n'imshow' command executed without error.")
    print("If you see a window with text, your OpenCV GUI is working correctly.")
    print("Press 'q' in the window to close it.")
    
    # 等待按键，这是显示窗口所必需的
    while True:
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
    cv2.destroyAllWindows()
    print("Window closed successfully.")

except cv2.error as e:
    print("\n--- OpenCV Error ---")
    print(f"An error occurred: {e}")
    print("This often means OpenCV was compiled without GUI support (e.g., you installed 'opencv-python-headless').")
    print("Please try reinstalling the correct package: pip install opencv-python")
    print("--------------------")