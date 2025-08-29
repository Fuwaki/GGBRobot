#pragma once

namespace KinematicsSolver {

/**
 * @brief 用于保存三个舵机角度的结构体
 */
struct ServoAngles {
    float angle_a; // 对应舵机1 (0度方向)
    float angle_b; // 对应舵机2 (120度方向)
    float angle_c; // 对应舵机3 (240度方向)
};

/**
 * @brief Stewart平台运动学逆解算
 * 根据期望的平台姿态（俯仰、翻滚）和高度，计算出三个舵机需要转动的角度。
 *
 * @param pitch 期望的俯仰角 (绕Y轴)，单位：度
 * @param roll 期望的翻滚角 (绕X轴)，单位：度
 * @param h 期望的平台中心高度
 *
 * @return ServoAngles 包含三个舵机目标角度的结构体
 */
ServoAngles move_platform(float pitch, float roll, float h);

} // namespace KinematicsSolver