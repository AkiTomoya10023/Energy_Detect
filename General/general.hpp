/*
 *	@Author: Qunshan He,mountain.he@qq.com
 *	@Date:	 2020.07.13
 *	@Brief:  This header file include the common head files and define the common structure as well as the functions ect.
 */

#ifndef GENERAL_H
#define GENERAL_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>

// #define DEBUG_MODE

// extern variables
extern pthread_mutex_t Globalmutex; // threads conflict due to image-updating
extern pthread_cond_t GlobalCondCV; // threads conflict due to image-updating
extern bool imageReadable;          // threads conflict due to image-updating
extern cv::Mat src;                 // Transfering buffer

/**
 * @brief: imageUpdating thread
 */
void *imageUpdatingThread(void *PARAM);

/**
 * @brief: energyDetecting thread
 */
void *energyDetectingThread(void *PARAM);

/**
* @brief: colors in order B G R 颜色B蓝 G绿 R红
*/
enum Color
{
    BLUE = 0,
    GREEN = 1,
    RED = 2
};

#endif // GENERAL_H
