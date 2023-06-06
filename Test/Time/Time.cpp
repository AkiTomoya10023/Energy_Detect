#include <opencv2/opencv.hpp>
#include <iostream>
#include <chrono>

int main() {

    // 获取程序启动的时间点
    std::chrono::steady_clock::time_point programStartTime = std::chrono::steady_clock::now();

    while (true) {

        // 计算相对于程序启动时间的时间戳
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        long long timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - programStartTime).count();

        std::cout << timestamp << std::endl;

        // 按下ESC键退出循环
        if (cv::waitKey(1) == 27)
            break;
    }

    return 0;
}
