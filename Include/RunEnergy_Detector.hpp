#ifndef RUNENERGY_DETECTOR_HPP_
#define RUNENERGY_DETECTOR_HPP_

#include <iostream>

// OpenCV
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

// STD
#include <utility>
#include <vector>

namespace rm_power_rune
{
    class Detector
    {
    private:
        cv::Mat src_;        // 原图像
        cv::Mat bin_;        // 二值化图
        cv::Mat pre_;        // 预处理图
        cv::Mat targetMask_; // 目标扇叶掩膜
        cv::Mat target_;     // 目标扇叶图

        cv::Point2f armor_;  // 击打目标中心
        cv::Point2f center_; // 中心点

        bool armorDetected_;  // 击打目标检测标识
        bool centerDetected_; // 中心R标检测标识

    public:
        enum class Color
        {
            RED,
            BLUE,
        } detect_color; // 能量机关颜色分类

        std::array<int, 3> lowerb1; // HSV阈值下界
        std::array<int, 3> lowerb2;
        std::array<int, 3> upperb1; // HSV阈值上界
        std::array<int, 3> upperb2;
        int bin_thresh; // 二值化阈值

        double min_strip_ratio, max_strip_ratio; // 扇叶长宽比
        double min_area_ratio, max_area_ratio;   // 目标扇叶凸包面积比

        Detector(Color color = Color::RED, int thresh = 50); // 构造函数
        ~Detector();
        cv::Mat binarize(const cv::Mat &src); // 二值化与形态学处理
        bool findArmor(cv::Point2f &armor, cv::RotatedRect &strip); // 检测目标击打位置
        bool findCenter(cv::Point2f &center); // 检测中心点
    };

} // namespace rm_power_rune

#endif // RUNE_DETECTOR__DETECTOR_HPP_
