#include "kinematics_solver.hpp"
#include "mini_matrix.hpp" // 引入新的矩阵库
#include <cmath>
#include <vector>
#include <esp_dsp.h>

namespace KinematicsSolver {

constexpr float PI = 3.14159265358979323846f;

ServoAngles move_platform(float pitch, float roll, float h) {
    using namespace MiniMatrix; // 使用新的矩阵命名空间

    // --- 1. 定义平台和连杆的几何常量 ---
    constexpr float Rbase = 32.9f / sqrtf(3.0f); // 基础平台半径
    // const float Rplat = 50.0f;            // 运动平台半径 (在你的代码中未使用，但保留)
    const float R1 = 50.0f;                  // 舵机摇臂长度
    const float R2 = 39.2f;                  // 连杆长度

    pitch = pitch * PI / 180.0f;
    roll = roll * PI / 180.0f;

    // --- 2. 定义初始位置向量 ---
    std::vector<Matrix> servos;
    std::vector<Matrix> joints;
    float servo_angles[] = {0.0f, 2.0f * PI / 3.0f, 4.0f * PI / 3.0f};
    for (int i = 0; i < 3; ++i) {
        float angle = servo_angles[i];
        Matrix s(3, 1);
        s(0, 0) = Rbase * cosf(angle);
        s(1, 0) = Rbase * sinf(angle);
        s(2, 0) = 0;
        servos.push_back(s);

        Matrix j(3, 1);
        j(0, 0) = Rbase * cosf(angle);
        j(1, 0) = Rbase * sinf(angle);
        j(2, 0) = h;
        joints.push_back(j);
    }

    // --- 3. 创建姿态旋转矩阵 ---
    // 俯仰(Pitch)旋转矩阵 (绕Y轴)
    Matrix pitch_rot(3, 3);
    pitch_rot(0, 0) = cosf(pitch);  pitch_rot(0, 1) = 0; pitch_rot(0, 2) = sinf(pitch);
    pitch_rot(1, 0) = 0;            pitch_rot(1, 1) = 1; pitch_rot(1, 2) = 0;
    pitch_rot(2, 0) = -sinf(pitch); pitch_rot(2, 1) = 0; pitch_rot(2, 2) = cosf(pitch);

    // 翻滚(Roll)旋转矩阵 (绕X轴)
    Matrix roll_rot(3, 3);
    roll_rot(0, 0) = 1; roll_rot(0, 1) = 0;           roll_rot(0, 2) = 0;
    roll_rot(1, 0) = 0; roll_rot(1, 1) = cosf(roll);  roll_rot(1, 2) = -sinf(roll);
    roll_rot(2, 0) = 0; roll_rot(2, 1) = sinf(roll);  roll_rot(2, 2) = cosf(roll);

    // --- 4. 执行矩阵运算 ---
    Matrix MatrixR = pitch_rot * roll_rot;

    // --- 5. 迭代计算每个舵机的角度 ---
    ServoAngles result_angles;
    float* result_ptr = &result_angles.angle_a;

    for (int i = 0; i < 3; ++i) {
        Matrix new_joint = MatrixR * joints[i];
        Matrix d = new_joint - servos[i];

        // 将向量 d 投影到每个舵机自身的二维平面上
        float angle = servo_angles[i];
        float d_loc_x = d(0, 0) * cosf(angle) + d(1, 0) * sinf(angle); // 新的 x'
        float d_loc_y = d(2, 0);                                      // 新的 y' (就是原来的 z)

        // 计算投影后向量 d_loc 的模长
        float d_len = sqrtf(d_loc_x * d_loc_x + d_loc_y * d_loc_y);

        // 使用反余弦定理和 atan2 计算最终的舵机角度
        float acos_arg = (R1 * R1 - R2 * R2 + d_len * d_len) / (2.0f * R1 * d_len);
        float final_angle_rad = atan2f(d_loc_y, d_loc_x) - acosf(acos_arg);

        // 将结果从弧度转换为度，并存入
        result_ptr[i] = final_angle_rad * 180.0f / PI;
    }

    return result_angles;
}

} // namespace KinematicsSolver