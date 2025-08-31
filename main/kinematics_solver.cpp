#include "kinematics_solver.hpp"
#include "mini_matrix.hpp"
#include "config.hpp" // 引入配置文件以获取参数
#include "esp_log.h"
#include <cmath>
#include <array>

namespace KinematicsSolver {

static const char* TAG = "运动学求解器";
constexpr float PI = 3.14159265358979323846f;

ServoAngles move_platform(float pitch, float roll, float h) {
    using namespace MiniMatrix;

    // --- 1. 从config.hpp中获取几何常量 ---
    const float Rbase = Params::KINEMATICS_BASE_RADIUS;
    const float R1 = Params::KINEMATICS_SERVO_ARM_LENGTH;
    const float R2 = Params::KINEMATICS_LINK_LENGTH;

    // 将输入的角度从度转换为弧度
    pitch = pitch * PI / 180.0f;
    roll = roll * PI / 180.0f;

    // --- 2. 定义舵机和关节的初始位置向量 ---
    std::array<Matrix, 3> servos = { Matrix(3, 1), Matrix(3, 1), Matrix(3, 1) };
    std::array<Matrix, 3> joints = { Matrix(3, 1), Matrix(3, 1), Matrix(3, 1) };
    const float servo_angles[] = {0.0f, 2.0f * PI / 3.0f, 4.0f * PI / 3.0f};

    for (int i = 0; i < 3; ++i) {
        servos[i] = Matrix(3, 1);
        joints[i] = Matrix(3, 1);
        float angle = servo_angles[i];

        // 舵机在基座平台上的位置
        Matrix& s = servos[i];
        s(0, 0) = Rbase * cosf(angle);
        s(1, 0) = Rbase * sinf(angle);
        s(2, 0) = 0;

        // 连杆在运动平台上的初始连接点 (未旋转时)
        Matrix& j = joints[i];
        j(0, 0) = Rbase * cosf(angle);
        j(1, 0) = Rbase * sinf(angle);
        j(2, 0) = h;
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

    // 组合旋转矩阵
    Matrix MatrixR = pitch_rot * roll_rot;

    // --- 4. 迭代计算每个舵机的角度 ---
    ServoAngles result_angles;
    float* result_ptr = &result_angles.angle_a;

    for (int i = 0; i < 3; ++i) {
        // 计算旋转后关节的位置
        Matrix new_joint = MatrixR * joints[i];
        // 计算从舵机到新关节位置的向量
        Matrix d = new_joint - servos[i];

        // 将三维向量 d 投影到每个舵机各自的二维平面上进行计算
        float angle = servo_angles[i];
        float d_loc_x = d(0, 0) * cosf(angle) + d(1, 0) * sinf(angle); // 新的 x' 坐标
        float d_loc_y = d(2, 0);                                      // 新的 y' 坐标 (就是原来的 z)

        // 计算投影后向量 d_loc 的模长平方
        float d_len_sq = d_loc_x * d_loc_x + d_loc_y * d_loc_y;

        // 使用反余弦定理计算舵机摇臂和连杆之间的夹角
        float acos_arg = (R1 * R1 - R2 * R2 + d_len_sq) / (2.0f * R1 * sqrtf(d_len_sq));
        
        // 防止 acos 参数超出范围 [-1, 1]
        if (acos_arg > 1.0f) acos_arg = 1.0f;
        if (acos_arg < -1.0f) acos_arg = -1.0f;

        // 使用 atan2 计算最终的舵机角度
        float final_angle_rad = atan2f(d_loc_y, d_loc_x) - acosf(acos_arg);

        // 将结果从弧度转换为度
        result_ptr[i] = final_angle_rad * 180.0f / PI;
    }

    return result_angles;
}

} // namespace KinematicsSolver
