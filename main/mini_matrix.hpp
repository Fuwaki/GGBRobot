#pragma once

#include <cstddef>

namespace MiniMatrix {

/**
 * @brief 一个简单的矩阵类，封装了ESP-DSP的C语言API
 */
class Matrix {
public:
    // 构造函数
    Matrix(int rows, int cols);
    // 析构函数
    ~Matrix();

    // 拷贝构造函数
    Matrix(const Matrix& other);
    // 拷贝赋值运算符
    Matrix& operator=(const Matrix& other);

    // 访问维度
    int rows() const { return _rows; }
    int cols() const { return _cols; }

    // 元素访问
    float& operator()(int row, int col);
    const float& operator()(int row, int col) const;

    // 获取原始数据指针
    float* data() { return _data; }
    const float* data() const { return _data; }

private:
    int _rows;
    int _cols;
    float* _data;
};

// 为矩阵运算重载运算符
Matrix operator*(const Matrix& a, const Matrix& b);
Matrix operator-(const Matrix& a, const Matrix& b);

} // namespace MiniMatrix