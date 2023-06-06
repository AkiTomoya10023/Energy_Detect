#ifndef ANGULARVELOCITY_CALCULATOR_HPP_
#define ANGULARVELOCITY_CALCULATOR_HPP_

#include <iostream>

// STD
#include <vector>

// OpenCV
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>

// Ceres
#include <ceres/ceres.h>
// Filter
#include "../Filter/particle_filter.h"

namespace rm_power_rune
{

    class Predictor
    {
    private:
        struct CURVE_FITTING_COST
        {
            CURVE_FITTING_COST(double x, double y) : _x(x), _y(y) {}
            // 残差的计算
            template <typename T>
            bool operator()(
                const T *params,   // 模型参数，有3维
                T *residual) const // 残差
            {
                residual[0] = T(_y) - params[0] * ceres::sin(params[1] * T(_x) + params[2]) - params[3]; // f(x) = a * sin(ω * t + θ) + b
                return true;
            }
            const double _x, _y; // x,y数据
        };                       // 第一阶段拟合函数

        struct CURVE_FITTING_COST_PHASE
        {
            CURVE_FITTING_COST_PHASE(double x, double y, double a, double omega, double dc) : _x(x), _y(y), _a(a), _omega(omega), _dc(dc) {}
            // 残差的计算
            template <typename T>
            bool operator()(
                const T *phase,    // 模型参数，有1维
                T *residual) const // 残差
            {
                residual[0] = T(_y) - T(_a) * ceres::sin(T(_omega) * T(_x) + phase[0]) - T(_dc); // f(x) = a * sin(ω * t + θ)
                return true;
            }
            const double _x, _y, _a, _omega, _dc; // x,y数据
        };                                        // 第二阶段拟合函数

        struct SpeedInfo
        {
            cv::Point2f point;  // 目标点坐标
            cv::Point2f center; // 中心R坐标
            long int timeStamp;      // 时间戳
        };                      // 速度计算信息结构体

        struct PredictorInfo
        {
            double speed;  // 扇叶旋转速度
            long int timeStamp; // 时间戳
        };                 // 位置预测信息结构体

        std::deque<SpeedInfo> _speed_info;          // 速度计算信息队列
        double _params[4];                          // 拟合参数数组
        double _bullet_speed = 22;                  // 弹速
        std::deque<PredictorInfo> _predict_info;    // 位置预测信息队列
        const int _max_timespan = 20000;            // 最大时间跨度，大于该时间重置预测器(ms)
        const double _max_rmse = 0.4;               // TODO:回归函数最大Cost
        const int _max_v = 3;                       // 设置最大速度,单位rad/s
        const int _history_deque_len_cos = 250;     // 大符全部参数拟合队列长度
        const int _history_deque_len_phase = 100;   // 大符相位参数拟合队列长度
        const int _history_deque_len_uniform = 100; // 小符转速求解队列长度
        const int _delay_small = 175;               // 小符发弹延迟
        const int _delay_big = 100;                 // 大符发弹延迟

    public:
        ParticleFilter pf;                                           // 滤波器
        const string pf_path = "../Params/filter/filter_param.yaml"; // 滤波器参数路径
        int mode = 1;                                                    // 预测器模式，0为小符，1为大符
        int last_mode;                                               // 上一次预测器的模式
        bool is_params_confirmed;                                    // 参数确定标识

        Predictor();
        ~Predictor();
        bool speedCalculate(cv::Point2f point, cv::Point2f center, long int timeStamp, double &speed);
        bool predict(double speed, long int timeStamp);
        double calculateFlightTime(double dist);
        cv::Point2f calcAimingAngleOffset(double t0, double dist, cv::Point2f point, cv::Point2f center);
        bool setBulletSpeed(double speed, int mode);
        bool setCameraParam(const char * filePath, int camId);
        double evalRMSE(double params[4]);
        double evalMAPE(double params[4]);

    }; // class AngularVelocityCalculator

} // namespace rm_power_rune

#endif // ANGULARVELOCITY_CALCULATOR_HPP_