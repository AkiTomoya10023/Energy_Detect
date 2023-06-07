#include <iostream>
#include <array>
#include <vector>
#include <cmath>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include "../Include/RunEnergy_Detector.hpp"

using namespace std;

namespace rm_power_rune
{

    Detector::Detector(Detector::Color color, int thresh)
    {
        detect_color = color;
        bin_thresh = thresh;

        if (detect_color == Color::RED)
        {
            lowerb1[0] = 0;
            lowerb1[1] = 34;
            lowerb1[2] = 255;

            upperb1[0] = 50;
            upperb1[1] = 255;
            upperb1[2] = 255;

            lowerb2[0] = 255;
            lowerb2[1] = 255;
            lowerb2[2] = 255;

            upperb2[0] = 255;
            upperb2[1] = 222;
            upperb2[2] = 255;
        }
        else
        {
            lowerb1[0] = 0;
            lowerb1[1] = 0;
            lowerb1[2] = 255;

            upperb1[0] = 95;
            upperb1[1] = 134;
            upperb1[2] = 255;

            lowerb2[0] = 255;
            lowerb2[1] = 255;
            lowerb2[2] = 255;

            upperb2[0] = 255;
            upperb2[1] = 255;
            upperb2[2] = 255;
        }
    }

    Detector::~Detector() {}

    cv::Mat Detector::binarize(const cv::Mat &src)
    {
        // 存储原图像
        src_ = src.clone();

        // 转换颜色空间
        cv::cvtColor(src, src, cv::COLOR_BGR2HSV);

        // 根据颜色进行通道提取
        cv::Mat diff, diff1, diff2;
        cv::inRange(src, cv::Scalar(lowerb1[0], lowerb1[1], lowerb1[2]), cv::Scalar(upperb1[0], upperb1[1], upperb1[2]), diff1);
        cv::inRange(src, cv::Scalar(lowerb2[0], lowerb2[1], lowerb2[2]), cv::Scalar(upperb2[0], upperb2[1], upperb2[2]), diff2);
        cv::bitwise_or(diff1, diff2, diff);

        // 显示通道提取结果
        // cv::imshow("diff", diff);

        // 对图像做阈值二值化
        cv::threshold(diff, bin_, bin_thresh, 255, cv::THRESH_BINARY);

        // 设置结构元
        cv::Mat kernel_bin = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(1, 1));
        cv::Mat kernel_pre = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));

        // 对二值图像做小结构元闭运算，连接中心断点
        cv::morphologyEx(bin_, bin_, cv::MORPH_CLOSE, kernel_bin);

        // 显示二值化结果
        // cv::imshow("bin", bin_);

        // 对预处理图像做大结构元的闭运算，填充装甲板中心
        cv::morphologyEx(bin_, pre_, cv::MORPH_CLOSE, kernel_pre);

        // 显示预处理结果
        // cv::imshow("pre", pre_);

        return pre_;
    }

    bool Detector::findArmor(cv::Point2f &armor, cv::RotatedRect &strip)
    {
        // 提取预处理图像轮廓
        vector<std::vector<cv::Point>> contours;
        vector<cv::Vec4i> hierarchy;
        cv::findContours(pre_, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        // TODO
        cv::Mat pre_clone = pre_.clone();
        cv::cvtColor(pre_clone, pre_clone, cv::COLOR_GRAY2BGR);

        int target_idx = -1; // 目标扇叶轮廓id初始化为-1

        targetMask_ = cv::Mat::zeros(pre_.size(), CV_8UC1); // 掩膜初始化
        target_ = cv::Mat::zeros(bin_.size(), CV_8UC1);     // 目标图像初始化

        // 对轮廓便利找目标扇叶
        for (int i = 0; i < contours.size(); i++)
        {
            // 排除有父轮廓的
            if (hierarchy[i][3] >= 0 || hierarchy[i][3] < contours.size())
                continue;

            // 求取轮廓面积与最小外接矩形
            auto area = cv::contourArea(contours[i]);
            auto rect = cv::minAreaRect(contours[i]);

            // 排除面积噪声
            if (area < 500 || area > 10000)
                continue;

            // 计算内接矩形长宽比
            float ratio = max(rect.size.width / rect.size.height, rect.size.height / rect.size.width);

            // 利用长宽比检查是否为扇叶
            if (min_strip_ratio < ratio && ratio < max_strip_ratio)
            {

                // 取轮廓凸包
                vector<cv::Point> hull;
                cv::convexHull(contours[i], hull);
                cv::drawContours(pre_clone, vector<vector<cv::Point>>(1, hull), 0, cv::Scalar(0, 255, 0), 1); // 在预处理图像中绘制凸包

                // 计算凸包面积与轮廓面积之比
                double hull_area, area_ratio;
                hull_area = cv::contourArea(hull);
                area_ratio = hull_area / area;

                // 利用凸包面积比检查是否为目标扇叶
                if (min_area_ratio < area_ratio && area_ratio < max_area_ratio)
                {
                    target_idx = i; // 目标轮廓id赋值
                    strip = rect;

                    cv::drawContours(pre_clone, contours, i, cv::Scalar(0, 255, 0)); // 在预处理图像中画出目标扇叶
                    cv::drawContours(targetMask_, contours, i, cv::Scalar(255), -1);     // 确定目标扇叶后绘制掩膜

                    cv::bitwise_and(targetMask_, bin_, target_); // 取图像交集为目标图像

                    // cout << "area:" << area << endl;
                    // cout << "ratio:" << ratio << endl;
                }
            }
        }

        // 显示目标图像
        // cv::imshow("strips", pre_clone);
        // cv::imshow("target", target_);

        // TODO
        cv::Mat target_clone = target_.clone();
        cv::cvtColor(target_clone, target_clone, cv::COLOR_GRAY2BGR);

        // 当检测到目标扇叶
        if (target_idx != -1)
        {
            // 提取目标图像轮廓
            cv::findContours(target_, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

            // 初始化圆心点集
            std::vector<cv::Point2f> circleCenters;

            // 对轮廓遍历查找符合条件的类圆
            for (int i = 0; i < contours.size(); i++)
            {
                // 排除没有父轮廓的
                if (hierarchy[i][3] < 0 || hierarchy[i][3] > contours.size())
                    continue;

                // 求取轮廓面积与周长
                auto area = cv::contourArea(contours[i]);
                auto perimeter = cv::arcLength(contours[i], true);

                // 排除面积噪声
                if (area < 10 || area > 1000)
                    continue;

                // 对每个轮廓检查是否为类圆形
                if (abs(perimeter * perimeter / (4 * M_PI * area - 0.001f) - 1) < 0.3f)
                {
                    // 这个轮廓符合圆形，找到最小包围圆
                    cv::Point2f center;
                    float radius;
                    cv::minEnclosingCircle(contours[i], center, radius);

                    // cv::drawContours(target_clone, contours, i, cv::Scalar(0, 255, 0), 1);

                    circleCenters.push_back(center);
                }
            }

            // 对全部类圆进行遍历求取圆心平均值
            for (int i = 0; i < circleCenters.size(); ++i)
            {
                for (int j = 0; j < i; ++j)
                {
                    if (cv::norm(circleCenters[i] - circleCenters[j]) < 10)
                    {
                        // 这个圆心与之前的圆心非常接近，认为是同一个圆
                        circleCenters[j] = (circleCenters[i] + circleCenters[j]) * 0.5f;
                    }
                }
            }

            // 判断是否检测到圆，并将结果赋值
            if (circleCenters.size())
            {
                cv::drawMarker(target_clone, circleCenters[circleCenters.size() - 1], cv::Scalar(0, 255, 0), cv::MARKER_CROSS);

                armor = armor_ = circleCenters[circleCenters.size() - 1];
                armorDetected_ = true;

                // 显示目标位置
                // cv::imshow("target", target_clone);

                return true;
            }
        }

        armorDetected_ = false;

        return false;
    }

    bool Detector::findCenter(cv::Point2f &center)
    {
        // TODO
        cv::Mat target_clone = pre_.clone();
        cv::cvtColor(target_clone, target_clone, cv::COLOR_GRAY2BGR);

        // 提取预处理图像轮廓
        vector<std::vector<cv::Point>> contours;
        cv::findContours(pre_, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        // 初始化中心R标轮廓id号
        int center_idx1 = -1;
        int center_idx2 = -1;

        // 根据扇叶找R标
        if (armorDetected_)
        {
            // 提取掩膜的轮廓
            vector<std::vector<cv::Point>> contour;
            cv::findContours(targetMask_, contour, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // 提取该轮廓的最小外接矩形
            auto rect = cv::minAreaRect(contour[0]);

            // 获取中心R标相对于击打中心方向
            float direction = atan2(rect.center.y - armor_.y, rect.center.x - armor_.x);

            // 计算中心R标大概位置
            double length = cv::norm(armor_ - rect.center) * 4.2;
            int center_x = armor_.x + length * cos(direction);
            int center_y = armor_.y + length * sin(direction);
            cv::Point center1 = cv::Point(center_x, center_y);

            // 根据R标位置找轮廓
            double minDist = DBL_MAX;
            for (int i = 0; i < contours.size(); i++)
            {

                double dist = norm(contours[i][0] - center1);
                if (dist < minDist)
                {
                    minDist = dist;
                    center_idx1 = i;
                }
            }
        }

        // 根据轮廓特征找R标
        double minArea = DBL_MAX;

        for (int i = 0; i < contours.size(); i++)
        {
            // 求取轮廓面积与周长
            auto area = cv::contourArea(contours[i]);
            auto perimeter = cv::arcLength(contours[i], true);

            // 排除面积噪声
            if (area < 10 || area > 1000)
                continue;

            // 对轮廓检查是否为类圆形
            if (abs(perimeter * perimeter / (4 * M_PI * area - 0.001f) - 1) < 0.8f)
            {
                // 这个轮廓符合圆形，找到最小面积的类圆
                if (area < minArea)
                {
                    minArea = area;
                    center_idx2 = i;
                }
            }
        }

        if (center_idx1 != -1 && center_idx2 != -1)
        {
            if (center_idx1 == center_idx2)
            {
                cv::Point2f center1, center2;
                float radius1, radius2;
                cv::minEnclosingCircle(contours[center_idx1], center1, radius1);
                cv::minEnclosingCircle(contours[center_idx2], center2, radius2);
                center = center_ = cv::Point((center1.x + center2.x) / 2, (center1.y + center2.y) / 2);
                centerDetected_ = true;
                return true;
            }
            else
            {
                cv::Point2f center1;
                float radius1;
                cv::minEnclosingCircle(contours[center_idx1], center1, radius1);
                center = center_ = center1;
                centerDetected_ = true;
                return true;
            }
        }
        else if (center_idx1 != -1 && center_idx2 == -1)
        {
            cv::Point2f center1;
            float radius1;
            cv::minEnclosingCircle(contours[center_idx1], center1, radius1);
            center = center_ = center1;
            centerDetected_ = true;
            return true;
        }
        else if (center_idx1 == -1 && center_idx2 != -1)
        {
            cv::Point2f center2;
            float radius2;
            cv::minEnclosingCircle(contours[center_idx2], center2, radius2);
            center = center_ = center2;
            centerDetected_ = true;
            return true;
        }

        centerDetected_ = false;
        return false;
    }

} // namespace rm_power_rune
