#include <opencv2/opencv.hpp>

// 全局变量
cv::VideoCapture capture;
cv::Mat frame;
int hueMin, hueMax, satMin, satMax, valMin, valMax;

// 滑动条回调函数
void onTrackbar(int, void*) {
    // 将图像转换到HSV色彩空间
    cv::Mat frameHSV;
    cv::cvtColor(frame, frameHSV, cv::COLOR_BGR2HSV);

    // 根据滑动条值创建颜色范围
    cv::Scalar lower(hueMin, satMin, valMin);
    cv::Scalar upper(hueMax, satMax, valMax);

    // 使用颜色范围进行掩码操作
    cv::Mat mask;
    cv::inRange(frameHSV, lower, upper, mask);

    // 显示结果
    cv::imshow("Video", mask);
}

int main() {
    // 打开视频文件
    capture.open("/home/rm/Energy_Detect/能量机关视频素材（黑暗环境）/关灯-蓝方大能量机关-正在激活状态.MP4");

    if (!capture.isOpened()) {
        std::cout << "Failed to open video file!" << std::endl;
        return -1;
    }

    // 获取视频的帧率
    double frame_rate = capture.get(cv::CAP_PROP_FPS);

    // 计算每帧之间的时间间隔
    double frame_interval = 1.0 / frame_rate;

    // 设置延时
    int delay = static_cast<int>(frame_interval * 1000); // 将延时转换为毫秒

    // 创建窗口
    cv::namedWindow("Video");

    // 创建滑动条
    cv::createTrackbar("Hue Min", "Video", &hueMin, 179, onTrackbar);
    cv::createTrackbar("Hue Max", "Video", &hueMax, 179, onTrackbar);
    cv::createTrackbar("Saturation Min", "Video", &satMin, 255, onTrackbar);
    cv::createTrackbar("Saturation Max", "Video", &satMax, 255, onTrackbar);
    cv::createTrackbar("Value Min", "Video", &valMin, 255, onTrackbar);
    cv::createTrackbar("Value Max", "Video", &valMax, 255, onTrackbar);

    // 循环读取视频帧
    while (true) {
        capture >> frame;

        if (frame.empty()) {
            std::cout << "Video playback completed." << std::endl;
            break;
        }

        resize(frame, frame, cv::Size(frame.cols * 0.35, frame.rows * 0.35));

        // 显示视频帧
        cv::imshow("Video", frame);

        // 按下空格键以暂停/继续视频播放
        int key = cv::waitKey(delay);
        if (key == ' ') {
            cv::waitKey(0);
        }

        // 按下ESC键以退出循环
        if (key == 27) {
            break;
        }
    }

    // 释放视频捕获对象和窗口
    capture.release();
    cv::destroyAllWindows();

    return 0;
}
