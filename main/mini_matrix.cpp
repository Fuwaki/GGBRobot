#include "mini_matrix.hpp"
#include "dspm_mult.h"
#include "dspm_sub.h"
#include <cstdio>
#include <cstring>

namespace MiniMatrix {

// 构造函数
Matrix::Matrix(int rows, int cols) : _rows(rows), _cols(cols) {
    if (rows * cols > 0) {
        _data = new float[rows * cols];
    } else {
        _data = nullptr;
    }
}

// 析构函数
Matrix::~Matrix() {
    delete[] _data;
}

// 拷贝构造函数
Matrix::Matrix(const Matrix& other) : _rows(other._rows), _cols(other._cols) {
    if (_rows * _cols > 0) {
        _data = new float[_rows * _cols];
        memcpy(_data, other._data, _rows * _cols * sizeof(float));
    } else {
        _data = nullptr;
    }
}

// 拷贝赋值运算符
Matrix& Matrix::operator=(const Matrix& other) {
    if (this == &other) {
        return *this;
    }
    delete[] _data;
    _rows = other._rows;
    _cols = other._cols;
    if (_rows * _cols > 0) {
        _data = new float[_rows * _cols];
        memcpy(_data, other._data, _rows * _cols * sizeof(float));
    } else {
        _data = nullptr;
    }
    return *this;
}

// 元素访问
float& Matrix::operator()(int row, int col) {
    return _data[row * _cols + col];
}
const float& Matrix::operator()(int row, int col) const {
    return _data[row * _cols + col];
}

// 矩阵乘法
Matrix operator*(const Matrix& a, const Matrix& b) {
    if (a.cols() != b.rows()) {
        printf("错误：矩阵维度不兼容，无法相乘。\n");
        return Matrix(0, 0);
    }

    Matrix result(a.rows(), b.cols());

    // 针对本项目中使用的特定尺寸，调用优化的DSP函数
    if (a.rows() == 3 && a.cols() == 3 && b.rows() == 3 && b.cols() == 3) {
        dspm_mult_3x3x3_f32(a.data(), b.data(), result.data());
    } else if (a.rows() == 3 && a.cols() == 3 && b.rows() == 3 && b.cols() == 1) {
        dspm_mult_3x3x1_f32(a.data(), b.data(), result.data());
    } else {
        // 通用情况
        dspm_mult_f32(a.data(), b.data(), result.data(), a.rows(), a.cols(), b.cols());
    }
    return result;
}

// 矩阵减法
Matrix operator-(const Matrix& a, const Matrix& b) {
    if (a.rows() != b.rows() || a.cols() != b.cols()) {
        printf("错误：矩阵维度必须相同才能相减。\n");
        return Matrix(0, 0);
    }
    Matrix result(a.rows(), b.cols());
    // 对于无填充的连续矩阵，所有padd为0，所有step为1
    dspm_sub_f32(a.data(), b.data(), result.data(), a.rows(), a.cols(), 0, 0, 0, 1, 1, 1);
    return result;
}

} // namespace MiniMatrix