#pragma once

#include <cstddef>

/**
 * @brief 一个封装了ESP-DSP函数的极简矩阵库
 */
namespace MiniMatrix {

/**
 * @brief 专为本项目优化的简单矩阵类
 * 
 * 注意: 这个类没有实现完整的移动语义或复杂的内存管理,
 * 仅为满足本项目中3x3和3x1矩阵运算的需求。
 */
class Matrix {
public:
    /**
     * @brief 构造函数
     * @param rows 矩阵行数
     * @param cols 矩阵列数
     */
    Matrix(int rows, int cols);
    
    /**
     * @brief 析构函数
     */
    ~Matrix();

    /**
     * @brief 拷贝构造函数
     */
    Matrix(const Matrix& other);

    /**
     * @brief 拷贝赋值运算符
     */
    Matrix& operator=(const Matrix& other);

    // 访问维度
    int rows() const { return _rows; }
    int cols() const { return _cols; }

    /**
     * @brief 元素访问 (非常量)
     * @param row 行索引
     * @param col 列索引
     * @return float& 元素的引用
     */
    float& operator()(int row, int col);

    /**
     * @brief 元素访问 (常量)
     * @param row 行索引
     * @param col 列索引
     * @return const float& 元素的常量引用
     */
    const float& operator()(int row, int col) const;

    /**
     * @brief 获取指向矩阵数据的原始指针
     * @return float* 数据指针
     */
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
