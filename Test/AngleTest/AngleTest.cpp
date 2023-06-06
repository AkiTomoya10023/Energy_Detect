#include <iostream>
#include "../../Include/RunEnergy_AngleSolve.hpp"

using namespace std;

int main()
{
    cv::Mat camera_matrix = (cv::Mat_<double>(3, 3) << 1055.91646081063, 18.5577572083904, 632.138219453034,
                             0, 1132.71376293414, 519.908557054770,
                             0, 0, 1);

    cv::Mat distortion_coeffs = (cv::Mat_<double>(1, 5) << -0.114878762544603, 0.349199593788434, -0.766650444280594,
                                 0.00565788982157328, -0.00888714980802018);

    double fixed_distance = 700.0; // 设置固定长度

    rm_power_rune::AngleSolver depth_calculator(camera_matrix, distortion_coeffs, fixed_distance);
    depth_calculator.setStripSize(372, 723);

    // 假设识别到的四个点坐标
    std::vector<cv::Point2f> points = {
        cv::Point2f(0, 0),
        cv::Point2f(0, 372),
        cv::Point2f(723, 372),
        cv::Point2f(723, 0)};

    double depth = depth_calculator.calculateDepth(points);
    std::cout << "Depth: " << depth << " mm" << std::endl;

    return 0;

    return 0;
}