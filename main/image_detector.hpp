#ifndef IMAGE_DETECTOR_HPP
#define IMAGE_DETECTOR_HPP

// 在包含OpenCV头文件之前,取消定义可能冲突的EPS宏
#ifdef EPS
#undef EPS
#endif

#include <opencv2/opencv.hpp>

/**
 * @brief 图像目标检测模块
 *
 * 包含用于在图像中寻找特定目标(如球体)的算法。
 */
namespace ImageDetector
{

// --- 数据结构 ---

/**
 * @brief 用于测试的地面真实数据(Ground Truth)
 */
struct GroundTruth
{
    int cx, cy, r;
};

/**
 * @brief 检测到的圆形目标的信息
 */
struct Circle
{
    cv::Point2f center; // 圆心坐标
    float radius;       // 半径
    bool found;         // 是否找到
};

/**
 * @brief 轮廓或连通域搜索的参数
 */
struct ContourFindParams
{
    double min_area = 0.0;        // 最小面积
    double max_area = 1e9;        // 最大面积
    double min_circularity = 0.0; // 最小圆度 (仅用于find_ball_by_contour)
    double min_radius = 0.0;      // 最小半径
    double max_radius = 1e9;      // 最大半径

    /**
     * @brief 目标选择策略
     */
    enum class Selection
    {
        LARGEST_AREA,    // 选择面积最大的目标
        CLOSEST_TO_POINT // 选择最接近某个期望位置的目标
    };
    Selection selection_method = Selection::LARGEST_AREA;
    cv::Point2f expected_pos = {0, 0}; // 期望位置 (当使用CLOSEST_TO_POINT时)
};

// --- 核心算法函数 ---

/**
 * @brief 通过查找轮廓来寻找球体
 * @param processed_img 经过预处理的二值化图像
 * @param params 搜索参数
 * @return Circle 检测到的圆形
 */
Circle find_ball_by_contour(const cv::Mat &processed_img, const ContourFindParams &params);

/**
 * @brief 通过连通域分析来寻找球体
 * @param processed_img 经过预处理的二值化图像
 * @param params 搜索参数
 * @return Circle 检测到的圆形
 */
Circle find_ball_by_components(const cv::Mat &processed_img, const ContourFindParams &params);

// --- 测试与辅助函数 ---
// (这些函数主要用于内部测试, Robot类不直接使用)

void test_all();
void detect_from_input();
cv::Mat create_test_image(int w, int h, GroundTruth &gt);
void add_salt_and_pepper_noise(cv::Mat &img, int n);
void detect_with_contours(const cv::Mat &img);
void detect_with_components(const cv::Mat &img);
void run_contour_test(bool with_noise);
void run_components_test(bool with_noise);
}; // namespace ImageDetector

#endif // IMAGE_DETECTOR_HPP
