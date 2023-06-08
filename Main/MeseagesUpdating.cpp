#include "../General/general.hpp"

void *meseagesUpdatingThread(void *PARAM)
{
    sleep(2000000); // 开始暂停一段时间等待相机与检测器初始化

    Serial ser_obj;
    ser_obj.openPort("/dev/ttyTHS2");

    while (1)
    {
        if (enemyColor != GREEN)
        {
            // 接收串口数据
            ser_recv = ser_obj.receive();

            pthread_mutex_lock(&Globalmutex);
            mode = ser_recv.mode;
            pthread_mutex_unlock(&Globalmutex);

            last_mode = mode;
            usleep(300000);
        }
    }
}