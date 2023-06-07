#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/stat.h>
#include <chrono>

namespace fs = std::filesystem;

int main()
{
    // 创建保存图片的文件夹
    auto current_time = std::chrono::system_clock::now();
    auto current_time_t = std::chrono::system_clock::to_time_t(current_time);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&current_time_t), "%Y%m%d_%H%M%S");
    std::string save_dir = "./" + ss.str();

    // 在Linux上创建文件夹
    int status = mkdir(save_dir.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);
    if (status != 0) {
        std::cerr << "Failed to create directory." << std::endl;
        return 1;
    }

    // 初始化摄像头
    cv::VideoCapture camera(0); // 0表示默认摄像头，如果有多个摄像头可以尝试不同的索引
    if (!camera.isOpened())
    {
        std::cerr << "Failed to open camera." << std::endl;
        return 1;
    }

    cv::Mat frame;
    int count = 0;

    while (true)
    {
        camera.read(frame);

        // 在图像上显示一些文本（可选）
        cv::putText(frame, "Press 'q' to quit", cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 2);

        cv::imshow("Camera", frame);

        // 按下'q'键退出循环
        if (cv::waitKey(1) == 'q')
        {
            break;
        }

        // 保存图像
        std::stringstream img_name_ss;
        img_name_ss << save_dir << "/image" << count << ".jpg";
        std::string img_name = img_name_ss.str();
        cv::imwrite(img_name, frame);
        count++;
    }

    camera.release();
    cv::destroyAllWindows();

    return 0;
}
