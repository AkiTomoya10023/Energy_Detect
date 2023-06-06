#include <iostream>
#include <vector>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include "Include/RunEnergy_Detector.hpp"
#include "Include/RunEnergy_Predictor.hpp"
#include "Include/RunEnergy_AngleSolve.hpp"

std::unique_ptr<rm_power_rune::Detector> detector;       // 初始化检测器对象
std::unique_ptr<rm_power_rune::Predictor> predictor;     // 初始化预测器对象
std::unique_ptr<rm_power_rune::AngleSolver> angleSolver; // 初始化角度解算对象

cv::Mat src;                 // 原图像存储位置
cv::Mat pre;                 // 预处理图像存储位置
cv::Point2f armor;           // 击打目标位置
cv::RotatedRect strip;       // 待击打扇叶的最小旋转矩阵
cv::Point2f center;          // 中心R标位置
bool armorDetected = false;  // 击打目标检测标识
bool centerDetected = false; // 中心R标检测标识

int main()
{

    cv::VideoCapture cap("/home/rm/Energy_Detect/能量机关视频素材（黑暗环境）/关灯-蓝方大能量机关-正在激活状态.MP4");

    // 检查视频文件是否成功打开
    if (!cap.isOpened())
    {
        std::cerr << "无法打开视频文件." << std::endl;
        return -1;
    }

    // 获取视频的帧率
    double frame_rate = cap.get(cv::CAP_PROP_FPS);

    // 计算每帧之间的时间间隔
    double frame_interval = 1.0 / frame_rate;

    // 设置延时
    int delay = static_cast<int>(frame_interval * 1000); // 将延时转换为毫秒

    // 读取图像6
    // src = cv::imread("D:\\CProject\\EnergyDetect\\RealRED.png");
    // cv::imshow("image", image);

    // 设置检测初始值
    detector = std::make_unique<rm_power_rune::Detector>(rm_power_rune::Detector::Color::BLUE, 0);
    detector->min_strip_ratio = 1.8;
    detector->max_strip_ratio = 2.1;
    detector->min_area_ratio = 1.4;
    detector->max_area_ratio = 2.1;

    // 读取图像
    // cap.read(src);

    // 创建角速度计算器
    predictor = std::make_unique<rm_power_rune::Predictor>();

    // 创建角度解算器
    angleSolver = std::make_unique<rm_power_rune::AngleSolver>();
    angleSolver->setCameraParam("../General/camera_params.xml", 1);
    angleSolver->setStripSize(723, 372);

    // 图像预处理
    // pre = detector->binarize(src);

    // 显示预处理图像
    // cv::imshow("bin", bin);

    // 提取击打中心
    // armorDetected = detector->findArmor(armor);

    // 提取中心R标
    // centerDetected = detector->findCenter(center);

    while (1)
    {
        bool ret = cap.read(src);

        if (!ret)
            break;

        int timeStamp = cap.get(cv::CAP_PROP_POS_MSEC);

        resize(src, src, cv::Size(src.cols * 0.35, src.rows * 0.35));

        cv::imshow("src", src);

        pre = detector->binarize(src);
        armorDetected = detector->findArmor(armor, strip);
        centerDetected = detector->findCenter(center);

        cv::cvtColor(pre, pre, cv::COLOR_GRAY2BGR);

        if (armorDetected)
        {
            cv::drawMarker(pre, armor, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 15, 1);
        }
        if (centerDetected)
        {
            cv::drawMarker(pre, center, cv::Scalar(0, 255, 0), cv::MARKER_CROSS, 15, 1);
        }
        if (armorDetected && centerDetected)
        {
            cv::circle(pre, center, cv::norm(center - armor), cv::Scalar(0, 255, 0), 1);

            double speed = 0;
            cv::Point2f targetPoint;
            predictor->speedCalculate(armor, center, timeStamp, speed);
            if (predictor->predict(speed, timeStamp))
            {
                targetPoint = predictor->calcAimingAngleOffset(timeStamp, 10, armor, center);
                cv::drawMarker(pre, targetPoint, cv::Scalar(0, 0, 255), cv::MARKER_CROSS, 15, 1);

                cv::Point2f rectPoints[4];
                strip.points(rectPoints);

                std::vector<cv::Point2f> points;
                for (int i = 0; i < 4; i++)
                {
                    points.push_back(rectPoints[i]);
                }

                double yaw = 0, pitch = 0, distance = 0;
                angleSolver->getAngle(points, targetPoint, yaw, pitch, distance);

                cout << "yaw:" << yaw << endl;
                cout << "pitch:" << pitch << endl;
            }
        }

        cv::imshow("result", pre);

        cv::waitKey(delay);
    }

    cv::waitKey(0);

    return 0;
}