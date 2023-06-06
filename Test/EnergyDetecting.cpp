#include "../General/general.hpp"
#include "../Include/RunEnergy_Detector.hpp"
#include "../Include/RunEnergy_Predictor.hpp"
#include "../Serial/Serial.hpp"

// 全局变量
cv::VideoCapture capture;
cv::Mat frame;
int hueMin, hueMax, satMin, satMax, valMin, valMax;

// 滑动条回调函数
void onTrackbar(int, void *)
{
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

void *energyDetectingThread(void *PARAM)
{
    usleep(1000000); // 开始暂停一段时间等待相机初始化

    // 创建窗口
    cv::namedWindow("Video");

    // 创建滑动条
    cv::createTrackbar("Hue Min", "Video", &hueMin, 179, onTrackbar);
    cv::createTrackbar("Hue Max", "Video", &hueMax, 179, onTrackbar);
    cv::createTrackbar("Saturation Min", "Video", &satMin, 255, onTrackbar);
    cv::createTrackbar("Saturation Max", "Video", &satMax, 255, onTrackbar);
    cv::createTrackbar("Value Min", "Video", &valMin, 255, onTrackbar);
    cv::createTrackbar("Value Max", "Video", &valMax, 255, onTrackbar);

    while (1)
    {
        // consumer gets image
        pthread_mutex_lock(&Globalmutex);
        while (!imageReadable)
        {
            pthread_cond_wait(&GlobalCondCV, &Globalmutex);
        }

        // 展示原图片
        cv::imshow("src", src);

        imageReadable = false;
        pthread_mutex_unlock(&Globalmutex);

        // 按下空格键以暂停/继续视频播放
        int key = cv::waitKey(1);
        if (key == ' ') {
            cv::waitKey(0);
        }

        // 按下ESC键以退出循环
        if (key == 27) {
            break;
        }
    }
}
