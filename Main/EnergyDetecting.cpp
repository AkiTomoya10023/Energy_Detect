#include "../General/general.hpp"
#include "../Include/RunEnergy_Detector.hpp"
#include "../Include/RunEnergy_Predictor.hpp"
#include "../Include/RunEnergy_AngleSolve.hpp"
#include "../Serial/Serial.hpp"

std::unique_ptr<rm_power_rune::Detector> detector;       // 初始化检测器对象
std::unique_ptr<rm_power_rune::Predictor> predictor;     // 初始化预测器对象
std::unique_ptr<rm_power_rune::AngleSolver> angleSolver; // 初始化角度解算对象

Color enemyColor = GREEN;
cv::Mat pre;                 // 预处理图像存储位置
cv::Point2f armor;           // 击打目标位置
cv::RotatedRect strip;       // 待击打扇叶的最小旋转矩阵
cv::Point2f center;          // 中心R标位置
bool armorDetected = false;  // 击打目标检测标识
bool centerDetected = false; // 中心R标检测标识
char last_mode = 0;          // 上一次模式

Test_receive ser_recv; // 串口接收变量
Test_send ser_send;    // 串口发送变量

void *energyDetectingThread(void *PARAM)
{
    angleSolver = std::make_unique<rm_power_rune::AngleSolver>();
    angleSolver->setCameraParam("../General/camera_params.xml", 1);
    angleSolver->setStripSize(723, 372);
    usleep(1000000); // 开始暂停一段时间等待相机初始化

    // 获取程序启动的时间点
    std::chrono::steady_clock::time_point programStartTime = std::chrono::steady_clock::now();

    
    Serial ser_obj;
    ser_obj.openPort("/dev/ttyTHS2");
    do
    {
        if (enemyColor == GREEN)
        {
            while (1)
            {
                // 接收串口数据
                ser_recv = ser_obj.receive();

                char robot_id = ser_recv.robot_id;
                if (robot_id == 1 || robot_id == 3 || robot_id == 4 || robot_id == 5 || robot_id == 7)
                {
                    printf("Robot id: %d\n", robot_id);
                    printf("target Armor is BLUE");
                    enemyColor = BLUE;

                    // 实例化检测器与预测器
                    detector = std::make_unique<rm_power_rune::Detector>(rm_power_rune::Detector::Color::BLUE, 0);
                    detector->min_strip_ratio = 1.8;
                    detector->max_strip_ratio = 2.1;
                    detector->min_area_ratio = 1.4;
                    detector->max_area_ratio = 2.1;

                    predictor = std::make_unique<rm_power_rune::Predictor>();
                    predictor->setBulletSpeed(22, 0);

                    break;
                }
                if (robot_id == 101 || robot_id == 103 || robot_id == 104 || robot_id == 105 || robot_id == 107)
                {
                    printf("Robot id%d\n", robot_id);
                    printf("target Armor is RED");
                    enemyColor = RED;

                    // 实例化检测器与预测器
                    detector = std::make_unique<rm_power_rune::Detector>(rm_power_rune::Detector::Color::RED, 0);
                    detector->min_strip_ratio = 1.8;
                    detector->max_strip_ratio = 2.1;
                    detector->min_area_ratio = 1.4;
                    detector->max_area_ratio = 2.1;

                    predictor = std::make_unique<rm_power_rune::Predictor>();
                    predictor->setBulletSpeed(22, 0);

                    break;
                }
                usleep(300000);
            }
        }

        ser_recv = ser_obj.receive();
        char mode = ser_recv.mode;

        predictor->setBulletSpeed(22, mode);

        // consumer gets image
        pthread_mutex_lock(&Globalmutex);
        while (!imageReadable)
        {
            pthread_cond_wait(&GlobalCondCV, &Globalmutex);
        }

        // 计算相对于程序启动时间的时间戳
        std::chrono::steady_clock::time_point currentTime = std::chrono::steady_clock::now();
        long timeStamp = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - programStartTime).count();

        imageReadable = false;
        pthread_mutex_unlock(&Globalmutex);

        // 对图像进行预处理
        pre = detector->binarize(src);

        cv::cvtColor(pre, pre, cv::COLOR_GRAY2BGR);

        armorDetected = detector->findArmor(armor, strip);
        centerDetected = detector->findCenter(center);

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

                // std::cout << "yaw: " << yaw << std::endl;
                // std::cout << "pitch: " << pitch << std::endl;

                ser_send.sof = 0xAA;
                ser_send.eof = 0x55;
                ser_send.yaw_angle = -(float)yaw;
                ser_send.pitch_angle = -(float)pitch;
                ser_obj.send(ser_send);
            }
        }

        cv::imshow("result", pre);

    } while (1);
}
