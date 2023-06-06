#ifndef RUNENERGY_ANGLESOLVE_HPP_
#define RUNENERGY_ANGLESOLVE_HPP_

#include <iostream>

#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

#include <utility>
#include <vector>

namespace rm_power_rune
{

    class AngleSolver
    {
    private:
        cv::Mat camera_matrix_;                  // 相机内参矩阵
        cv::Mat distortion_coeffs_;              // 畸变系数
        std::vector<cv::Point3f> strips_;        // 能量机关扇叶世界坐标系下坐标
        std::vector<cv::Point2f> targetContour_; // 待击打扇叶外接矩形坐标
        cv::Point2f targetCenter_;               // 击打中心坐标
        cv::Mat rVec_;                           // 相机旋转向量
        cv::Mat tVec_;                           // 相机的平移向量

        double gun_cam_sidtance_y_; // 相机到枪口的距离mm
        double bullet_speed_ = 22000;  // 子弹弹速mm/s

        float yaw_;       // yaw轴角度信息
        float pitch_;     // pitch轴角度信息
        double distance_; // 到能量机关的深度

        float last_yaw_angle; // 上一次计算的yaw角度值
        float last_pit_angle; // 上一次计算的pitch角度值

    public:
        AngleSolver();
        ~AngleSolver();
        void setCameraParam(const cv::Mat &camera_matrix, const cv::Mat &distortion_coeff);
        int setCameraParam(const char *filePath, int camId);
        void setStripSize(double width, double height);
        void setTarget(std::vector<cv::Point2f> contourPoints, cv::Point2f centerPoint);
        void solveAngles();
        void PinHole_solver();
        void compensateAngle();
        void getAngle(std::vector<cv::Point2f> &contourPoints, cv::Point2f cenerPoint, double &yaw, double &pitch, double &evaluateDistance);
        float BulletModel(float x, float v, float angle);
        float GetPitch(float x, float y, float v);
    };

} // namespace rm_power_rune

#endif // RUNENERGY_ANGLESOLVE_HPP_