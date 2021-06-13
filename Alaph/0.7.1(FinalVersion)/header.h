#ifndef HEADER_H
#define HEADER_H

#include <thread>
#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <time.h>
#include <sys/timeb.h>
#include <math.h>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/core/core.hpp>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <dirent.h>
#include <vector>
#include <string.h>
#include <fstream>
#include "serial.h"
#define THRESHOLD_CIRCLECENTER_BIGEST 1000 //480:600
#define THRESHOLD_CIRCLECENTER_SMALLEST 20

#define THRESHOLD_ROTATEDARM_BIGEST 5000 //480: 8000
#define THRESHOLD_ROTATEDARM_SMALLEST 1000

using namespace std;
using namespace cv;

union data_send_float
{
    float data_float;
    char data_uint8[4];
};

struct Energy_Agency_Rotated_Arm
{
    cv::RotatedRect extern_rotatedrect;               //能量机关单个旋转臂最外围旋转矩形
    std::vector<cv::RotatedRect> inside_rotatedrects; //能量机关内单个旋转臂内部旋转矩形集合
    std::vector<cv::Point2f> perhaps_vertices;        //单个旋转臂需送入svm分类的四个点坐标
    cv::RotatedRect perhaps_target;                   //单个旋转臂可能需击打的目标
};

struct EnergyResult
{
    cv::RotatedRect TargetRotatedRect; //装甲板旋转矩阵
    cv::Point2f RotatedArmCenter;      //外壁中心点
    bool flag = 0;
};

struct Perhaps_scores
{
    double distance; //与该旋转臂最外围轮廓中心点的距离
    std::vector<cv::Point2f> sort_vertices;
    cv::RotatedRect target; //此旋转矩形（是一个内部轮廓的旋转矩形）
};

#endif // HEADER_H
