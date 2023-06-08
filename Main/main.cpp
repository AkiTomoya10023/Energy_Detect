#include "../GxCamera/GxCamera.hpp"
#include "../General/general.hpp"
#include <X11/Xlib.h>

pthread_t thread1;
pthread_t thread2;
pthread_t thread3;

// muti-threads control variables
pthread_mutex_t Globalmutex;                     // threads conflict due to image-updating
pthread_cond_t GlobalCondCV;                     // threads conflict due to image-updating
bool imageReadable = false;                      // threads conflict due to image-updating
cv::Mat src = cv::Mat::zeros(600, 800, CV_8UC3); // Transfering buffer
int mode = 0;                                    // 打符模式，0为小符，1为大符
int last_mode  = 0;                              // 上一次打符的模式

int main(int argc, char **argv)
{
    // For MutiTHread
    XInitThreads();
    // Init mutex
    pthread_mutex_init(&Globalmutex, NULL);
    // Init cond
    pthread_cond_init(&GlobalCondCV, NULL);
    // Create thread 1 -- image acquisition thread
    pthread_create(&thread1, NULL, imageUpdatingThread, NULL);
    // Create thread 2 -- armor Detection thread
    pthread_create(&thread2, NULL, energyDetectingThread, NULL);
    // Create thread 3 -- Meseages Updating thread
    pthread_create(&thread3, NULL, meseagesUpdatingThread, NULL);
    // Wait for children thread
    pthread_join(thread1, NULL);
    pthread_join(thread2, NULL);
    pthread_join(thread3, NULL);
    pthread_mutex_destroy(&Globalmutex);
    return 0;
}
