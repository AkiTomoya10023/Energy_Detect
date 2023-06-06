#include <iostream>
#include <vector>
#include <cmath>
#include <opencv2/opencv.hpp>
#include "../Include/RunEnergy_Predictor.hpp"

using namespace std;

namespace rm_power_rune
{

    Predictor::Predictor()
    {
        is_params_confirmed = false;
        _params[0] = 0;
        _params[1] = 0;
        _params[2] = 0;
        _params[3] = 0;

        YAML::Node config = YAML::LoadFile(pf_path);
        pf.initParam(config, "buff");
    }

    Predictor::~Predictor() {}

    bool Predictor::speedCalculate(cv::Point2f point, cv::Point2f center, long int timeStamp, double &speed)
    {
        // 设置结构体保存当前点信息
        SpeedInfo speed_info = {point, center, timeStamp};

        // 保存当前点信息
        _speed_info.push_back(speed_info);

        // 初始化速度值
        double angularVelocity = 0;

        // 如果有足够的点来计算角速度
        if (_speed_info.size() > 1)
        {
            // 计算角速度
            cv::Point2f previousVector = _speed_info[_speed_info.size() - 2].point - _speed_info[_speed_info.size() - 2].center;
            cv::Point2f currentVector = point - center;

            double previousAngle = std::atan2(previousVector.y, previousVector.x);
            double currentAngle = std::atan2(currentVector.y, currentVector.x);

            double angleDifference = currentAngle - previousAngle;
            if (angleDifference > CV_PI)
                angleDifference -= 2 * CV_PI;
            else if (angleDifference < -CV_PI)
                angleDifference += 2 * CV_PI;

            angularVelocity = angleDifference / (static_cast<double>(timeStamp - _speed_info[_speed_info.size() - 2].timeStamp) / 1000);

            // 如果角速度的值不满足要求
            if (std::abs(angularVelocity) <= _max_v)
            {
                // 结果赋值
                // cout << angularVelocity << endl;
                speed = angularVelocity;

                if (_speed_info.size() > 3)
                {
                    _speed_info.pop_front();
                }

                return true;
            }
            else
            {
                _speed_info.pop_back();
                _speed_info.pop_back();
            }
        }

        return false;
    }

    bool Predictor::predict(double speed, long int timeStamp)
    {
        PredictorInfo predict_info = {speed, timeStamp};
        if (mode != last_mode)
        {
            last_mode = mode;
            _predict_info.clear();
            is_params_confirmed = false;
        }

        // 当时间跨度过长视作目标已更新，需清空历史信息队列
        if (_predict_info.size() == 0 || predict_info.timeStamp - _predict_info.front().timeStamp >= _max_timespan)
        {
            _predict_info.clear();
            _predict_info.push_back(predict_info);
            _params[0] = 0;
            _params[1] = 0;
            _params[2] = 0;
            _params[3] = 0;
            is_params_confirmed = false;
            return false;
        }

        // 输入数据前进行滤波
        auto is_ready = pf.is_ready;
        Eigen::VectorXd measure(1);
        measure << speed;
        pf.update(measure);

        if (is_ready)
        {
            auto predict = pf.predict();
            predict_info.speed = predict[0];
        }

        auto deque_len = 0;
        if (mode == 0)
        {
            deque_len = _history_deque_len_uniform;
        }
        else if (mode == 1)
        {
            if (!is_params_confirmed)
            {
                deque_len = _history_deque_len_cos;
            }
            else
            {
                deque_len = _history_deque_len_phase;
            }
        }

        if (_predict_info.size() < deque_len)
        {
            _predict_info.push_back(predict_info);
            return false;
        }
        else if (_predict_info.size() == deque_len)
        {
            _predict_info.pop_front();
            _predict_info.push_back(predict_info);
        }
        else if (_predict_info.size() > deque_len)
        {
            while (_predict_info.size() >= deque_len)
            {
                _predict_info.pop_front();
            }
            _predict_info.push_back(predict_info);
        }

        // 计算旋转方向
        double rotate_speed_sum = 0;
        int rotate_sign;
        for (auto target_info : _predict_info)
            rotate_speed_sum += target_info.speed;
        auto mean_velocity = rotate_speed_sum / _predict_info.size();
        // cout<<rotate_speed_sum<<endl;

        // TODO:小符模式不需要额外计算，也可增加判断，小符模式给定恒定转速进行击打
        if (mode == 0)
        {
            _params[3] = mean_velocity;
        }
        // 若为大符
        else if (mode == 1)
        {
            // 拟合函数: f(x) = a * sin(ω * t + θ) + b， 其中a， ω， θ需要拟合.
            // 参数未确定时拟合a， ω， θ
            if (!is_params_confirmed)
            {
                ceres::Problem problem;
                ceres::Solver::Options options;
                ceres::Solver::Summary summary; // 优化信息
                double params_fitting[4] = {1, 1, 1, mean_velocity};

                // 旋转方向，逆时针为正
                if (rotate_speed_sum / fabs(rotate_speed_sum) >= 0)
                    rotate_sign = 1;
                else
                    rotate_sign = -1;

                for (auto target_info : _predict_info)
                {
                    problem.AddResidualBlock( // 向问题中添加误差项
                                              // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST, 1, 4>(
                            new CURVE_FITTING_COST((float)(predict_info.timeStamp) / 1e3,
                                                   predict_info.speed * rotate_sign)),
                        new ceres::CauchyLoss(0.5),
                        params_fitting // 待估计参数
                    );
                }

                // 设置上下限
                // FIXME:参数需根据场上大符实际调整
                problem.SetParameterLowerBound(params_fitting, 0, 0.7);
                problem.SetParameterUpperBound(params_fitting, 0, 1.2);
                problem.SetParameterLowerBound(params_fitting, 1, 1.6);
                problem.SetParameterUpperBound(params_fitting, 1, 2.2);
                problem.SetParameterLowerBound(params_fitting, 2, -CV_PI);
                problem.SetParameterUpperBound(params_fitting, 2, CV_PI);
                problem.SetParameterLowerBound(params_fitting, 3, 0.5);
                problem.SetParameterUpperBound(params_fitting, 3, 2.5);

                ceres::Solve(options, &problem, &summary);
                double params_tmp[4] = {params_fitting[0] * rotate_sign, params_fitting[1], params_fitting[2], params_fitting[3] * rotate_sign};
                auto rmse = evalRMSE(params_tmp);
                if (rmse > _max_rmse)
                {
                    cout << summary.BriefReport() << endl;
                    LOG(INFO) << "[BUFF_PREDICT]RMSE is too high, Fitting failed!";
                    return false;
                }
                else
                {
                    LOG(INFO) << "[BUFF_PREDICT]Fitting Succeed! RMSE: " << rmse;
                    _params[0] = params_fitting[0] * rotate_sign;
                    _params[1] = params_fitting[1];
                    _params[2] = params_fitting[2];
                    _params[3] = params_fitting[3] * rotate_sign;
                    is_params_confirmed = true;
                }
            }
            // 参数确定时拟合θ
            else
            {
                ceres::Problem problem;
                ceres::Solver::Options options;
                ceres::Solver::Summary summary; // 优化信息
                double phase;

                for (auto target_info : _predict_info)
                {
                    problem.AddResidualBlock( // 向问题中添加误差项
                                              // 使用自动求导，模板参数：误差类型，输出维度，输入维度，维数要与前面struct中一致
                        new ceres::AutoDiffCostFunction<CURVE_FITTING_COST_PHASE, 1, 1>(
                            new CURVE_FITTING_COST_PHASE((float)(target_info.timeStamp) / 1e3,
                                                         (target_info.speed - _params[3]) * rotate_sign, _params[0], _params[1], _params[3])),
                        new ceres::CauchyLoss(1e1),
                        &phase // 待估计参数
                    );
                }

                // 设置上下限
                problem.SetParameterLowerBound(&phase, 0, -CV_PI);
                problem.SetParameterUpperBound(&phase, 0, CV_PI);

                ceres::Solve(options, &problem, &summary);
                double params_new[4] = {_params[0], _params[1], phase, _params[3]};
                auto old_rmse = evalRMSE(_params);
                auto new_rmse = evalRMSE(params_new);
                if (new_rmse < old_rmse)
                {
                    LOG(INFO) << "[BUFF_PREDICT]Params Updated! RMSE: " << new_rmse;
                    _params[2] = phase;
                }
                // cout << "RMSE:" << new_rmse << endl;
            }
        }

        return true;
    }

    bool Predictor::setBulletSpeed(double speed, int mode)
    {
        _bullet_speed = speed;
        this->mode = mode;
        return true;
    }

    double Predictor::calculateFlightTime(double dist)
    {
        const double g = 9.8;        // 重力加速度 (m/s^2)
        const double rho = 1.2;      // 空气密度 (kg/m^3)
        const double C = 0.47;       // 空气阻力系数
        const double A = 0.00028224; // 物体横截面积 (m^2)

        double totalTime = 0.0;
        double timeStep = 0.001; // 时间步长 (s)

        while (dist > 0)
        {
            double acceleration = g - (0.5 * rho * C * A * _bullet_speed * _bullet_speed) / dist;
            _bullet_speed += acceleration * timeStep;
            dist -= _bullet_speed * timeStep;
            totalTime += timeStep;
        }

        return totalTime * 1000;
    }

    /**
     * @brief 计算角度提前量
     * @param params 拟合方程参数
     * @param t0 积分下限
     * @param t1 积分上限
     * @param mode 模式
     * @return 角度提前量(rad)
     */
    cv::Point2f Predictor::calcAimingAngleOffset(double t0, double dist, cv::Point2f point, cv::Point2f center)
    {
        auto a = _params[0];
        auto omega = _params[1];
        auto theta = _params[2];
        auto b = _params[3];
        double theta1;
        double theta0;
        // f(x) = a * sin(ω * t + θ) + b
        // 对目标函数进行积分

        double t1 = (calculateFlightTime(dist) + t0) / 1000;
        t0 = t0 / 1000;
        // cout<<"t1: "<<t1<<endl;
        // cout<<"t0: "<<t0<<endl;
        if (mode == 0) // 适用于小符模式
        {
            theta0 = b * t0;
            theta1 = b * t1;
        }
        else
        {
            theta0 = (b * t0 - (a / omega) * cos(omega * t0 + theta));
            theta1 = (b * t1 - (a / omega) * cos(omega * t1 + theta));
        }
        // cout<<(theta1 - theta0) * 180 / CV_PI<<endl;

        // 计算点相对于圆心的向量
        double dx = point.x - center.x;
        double dy = point.y - center.y;

        // 计算点相对于圆心的距离
        double distance = std::sqrt(dx * dx + dy * dy);

        // 计算点相对于圆心的角度
        double currentAngle = std::atan2(dy, dx);

        // 计算在圆上的另一点的角度
        double newAngle = currentAngle + theta1 - theta0;

        // 计算在圆上的另一点的位置
        double newX = center.x + distance * std::cos(newAngle);
        double newY = center.y + distance * std::sin(newAngle);

        cv::Point newPoint;
        newPoint.x = newX;
        newPoint.y = newY;

        return newPoint;
    }

    /**
     * @brief 计算RMSE指标
     *
     * @param params 参数首地址指针
     * @return RMSE值
     */
    double Predictor::evalRMSE(double params[4])
    {
        double rmse_sum = 0;
        double rmse = 0;
        for (auto target_info : _predict_info)
        {
            auto t = (float)(target_info.timeStamp) / 1e3;
            auto pred = params[0] * sin(params[1] * t + params[2]) + params[3];
            auto measure = target_info.speed;
            rmse_sum += pow((pred - measure), 2);
        }
        rmse = sqrt(rmse_sum / _predict_info.size());
        return rmse;
    }

    /**
     * @brief 计算RMSE指标
     *
     * @param params 参数首地址指针
     * @return RMSE值
     */
    double Predictor::evalMAPE(double params[4])
    {
        double mape_sum = 0;
        double mape = 0;
        for (auto target_info : _predict_info)
        {
            auto t = (float)(target_info.timeStamp) / 1e3;
            auto pred = params[0] * sin(params[1] * t + params[2]) + params[3];
            auto measure = target_info.speed;

            mape_sum += abs((measure - pred) / measure);
            // cout<<abs((measure - pred) / measure)<<endl;
        }
        // mape = mape_sum / history_info.size() * 100;
        // cout<<mape<<endl;
        return mape;
    }

} // namespace rm_power_rune