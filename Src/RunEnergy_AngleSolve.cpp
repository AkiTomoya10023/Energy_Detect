#include "../Include/RunEnergy_AngleSolve.hpp"

namespace rm_power_rune
{

    AngleSolver::AngleSolver()
    {
    }

    AngleSolver::~AngleSolver()
    {
    }

    void AngleSolver::setCameraParam(const cv::Mat &camera_matrix, const cv::Mat &distortion_coeff)
    {
        camera_matrix.copyTo(camera_matrix_);
        distortion_coeff.copyTo(distortion_coeffs_);
    }

    int AngleSolver::setCameraParam(const char *filePath, int camId)
    {
        cv::FileStorage fsRead;
        fsRead.open(filePath, cv::FileStorage::READ);
        if (!fsRead.isOpened())
        {
            std::cout << "Failed to open xml" << std::endl;
            return -1;
        }

        fsRead["Y_DISTANCE_BETWEEN_GUN_AND_CAM"] >> gun_cam_sidtance_y_;

        cv::Mat camera_matrix;
        cv::Mat distortion_coeff;
        switch (camId)
        {
        case 1:
            fsRead["CAMERA_MATRIX_1"] >> camera_matrix;
            fsRead["DISTORTION_COEFF_1"] >> distortion_coeff;
            break;
        case 2:
            fsRead["CAMERA_MATRIX_2"] >> camera_matrix;
            fsRead["DISTORTION_COEFF_2"] >> distortion_coeff;
            break;
        case 3:
            fsRead["CAMERA_MATRIX_3"] >> camera_matrix;
            fsRead["DISTORTION_COEFF_3"] >> distortion_coeff;
            break;
        default:
            std::cout << "WRONG CAMID GIVEN!" << std::endl;
            break;
        }
        setCameraParam(camera_matrix, distortion_coeff);
        fsRead.release();
        return 0;
    }

    void AngleSolver::setStripSize(double width, double height)
    {
        double half_x = width / 2;
        double half_y = height / 2;

        strips_.push_back(cv::Point3f(-half_x, half_y, 0));  // tl top left
        strips_.push_back(cv::Point3f(half_x, half_y, 0));   // tr top right
        strips_.push_back(cv::Point3f(-half_x, -half_y, 0)); // br below right
        strips_.push_back(cv::Point3f(half_x, -half_y, 0));  // bl below left
    }

    void AngleSolver::setTarget(std::vector<cv::Point2f> contourPoints, cv::Point2f centerPoint)
    {
        targetContour_ = contourPoints;
        targetCenter_ = centerPoint;
    }

    void AngleSolver::solveAngles()
    {
        solvePnP(strips_, targetContour_, camera_matrix_, distortion_coeffs_, rVec_, tVec_, false, cv::SOLVEPNP_ITERATIVE);

        // gun_cam_sidtance_y_ = 0; // 相机中心相对于枪口的距离
        tVec_.at<double>(1, 0) -= gun_cam_sidtance_y_;

        double x_pos = tVec_.at<double>(0, 0);
        double y_pos = tVec_.at<double>(1, 0);
        double z_pos = tVec_.at<double>(2, 0);
        distance_ = sqrt(x_pos * x_pos + y_pos * y_pos + z_pos * z_pos);

        // Target is too far, using PinHole solver
        PinHole_solver();
    }

    void AngleSolver::PinHole_solver()
    {
        double fx = camera_matrix_.at<double>(0, 0);
        double fy = camera_matrix_.at<double>(1, 1);
        double cx = camera_matrix_.at<double>(0, 2);
        double cy = camera_matrix_.at<double>(1, 2);
        cv::Point2f pnt;
        std::vector<cv::Point2f> in;
        std::vector<cv::Point2f> out;
        in.push_back(targetCenter_);
        // 对像素点去畸变
        undistortPoints(in, out, camera_matrix_, distortion_coeffs_, cv::noArray(), camera_matrix_);
        pnt = out.front();

        // 去畸变后的比值
        double rxNew = (pnt.x - cx) / fx;
        double ryNew = (pnt.y - cy) / fy;

        yaw_ = atan(rxNew) / CV_PI * 180;
        pitch_ = -atan(ryNew) / CV_PI * 180;
    }

    void AngleSolver::compensateAngle()
    {
        // --FT--
        double x_pos = tVec_.at<double>(0, 0);
        double y_pos = tVec_.at<double>(1, 0);
        double z_pos = tVec_.at<double>(2, 0);

        float y_ac = 0.0f;
        float temp_dis = distance_ * 0.001;
        float temp_y = yaw_;
        float temp_p = pitch_;

        float temp_x_pos = (x_pos * 0.001);
        float temp_y_pos = -(y_pos * 0.001);
        float temp_z_pos = (z_pos * 0.001);
        float y_compensate = GetPitch(temp_dis, temp_y_pos, 20.0f);
        pitch_ = (float)atan2(y_compensate, temp_dis) / CV_PI * 180;

        float yaw_rate = yaw_ - last_yaw_angle;
        float pit_rate = pitch_ - last_pit_angle;
        float shoot_delay = 90.0f;
        float dt = 10.0f;
        if (yaw_rate > 10.0f || pit_rate > 8.0f)
        {
            dt = 0.0f;
            yaw_rate = 0;
            pit_rate = 0;
        }

        if (abs(yaw_) > 20.0f)
        {
            yaw_ = 0.0f;
        }
        // bullet fly cost compensate
        float fly_t = shoot_delay + distance_ / bullet_speed_;
        float yaw_compensate = atan2(fly_t * yaw_rate, distance_);

        yaw_ += yaw_compensate;

        if (abs(pitch_) > 20.0f)
        {
            pitch_ = 0.0f;
        }

        last_yaw_angle = yaw_;
        last_pit_angle = pitch_;
        printf("Before pre yaw:%f, pit compensate:%f, yaw compensate:%f, pre rate:%f\n", temp_y, pitch_ - temp_p, yaw_compensate, yaw_ - temp_y);
    }

    void AngleSolver::getAngle(std::vector<cv::Point2f> &contourPoints, cv::Point2f cenerPoint, double &yaw, double &pitch, double &evaluateDistance)
    {
        setTarget(contourPoints, cenerPoint);
        solveAngles();
        compensateAngle();
        yaw = yaw_;
        pitch = pitch_;
        evaluateDistance = distance_;
    }

    // air friction is considered
    float AngleSolver::BulletModel(float x, float v, float angle)
    { // x:m,v:m/s,angle:rad
        float t, y;
        // air friction factor
        float init_k_ = 0.020f;
        float GRAVITY = 9.8;
        t = (float)((exp(init_k_ * x) - 1) / (init_k_ * v * cos(angle)));
        y = (float)(v * sin(angle) * t - GRAVITY * t * t / 2);
        return y;
    }

    // x:distance , y: height
    float AngleSolver::GetPitch(float x, float y, float v)
    {
        float y_temp, y_actual, dy;
        float a;
        y_temp = y;
        // by iteration
        for (int i = 0; i < 20; i++)
        {
            a = (float)atan2(y_temp, x);
            y_actual = BulletModel(x, v, a);
            dy = y - y_actual;
            y_temp = y_temp + dy;
            if (fabsf(dy) < 0.001)
            {
                break;
            }
            // printf("iteration num %d: angle %f,temp target y:%f,err of y:%f\n",i+1,a*180/3.1415926535,yTemp,dy);
        }
        return y_temp;
    }

} // namespace rm_power_rune
