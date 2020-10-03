#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>
#include <time.h>
#include <sys/timeb.h>
//#include <sys/time.h>
//#include <sys\utime.h>
#include <math.h>

using namespace std;
using namespace cv;


cv::Point2f getRotatePoint(cv::Mat srcImage, cv::Point2f Points, const cv::Point2f rotate_center, const double angle)
{
    cv::Point2f dstPoints;
    int x1 = 0, y1 = 0;
    int row = srcImage.rows;
    x1 = Points.x;
    y1 = row - Points.y;
    int x2 = rotate_center.x;
    int y2 = row - rotate_center.y;
    int x = cvRound((x1 - x2)*cos(CV_PI / 180.0 * angle) - (y1 - y2)*sin(CV_PI / 180.0 * angle) + x2);
    int y = cvRound((x1 - x2)*sin(CV_PI / 180.0 * angle) + (y1 - y2)*cos(CV_PI / 180.0 * angle) + y2);

//    int x = cvRound((x1 - x2)*cos(angle) - (y1 - y2)*sin(angle) + x2);//弧度
//    int y = cvRound((x1 - x2)*sin(angle) + (y1 - y2)*cos(angle) + y2);//弧度

    x=x;
    y = row - y;
    dstPoints.x=x;
    dstPoints.y=y;
    return dstPoints;
}


int main()
{
    Mat clk(480, 640, CV_8UC3,Scalar(0,0,0)); //Mat to store clock image
    Mat back_up(480, 640, CV_8UC3, Scalar(0, 0, 0)); //Mat to store backup image

    Point cent(320, 240);
    Point perim(320.240);
    int rad = clk.rows/2 -30;
    float sec_angle = 270;

    circle(clk, cent, 2, Scalar(0, 255, 0, 0), 5, CV_AA, 0); //Draw inner circle

    back_up = clk.clone(); // Clone to backup image

    time_t rawtime;
    struct tm * timeinfo;
    float second;
    float millisec;
    struct timeb tmb;


    while (1){
        //获取本地时间
        ftime(&tmb);
        rawtime = tmb.time;
        timeinfo = localtime (&rawtime);

        second = timeinfo->tm_sec;
        millisec = tmb.millitm;

        second = second + millisec / 1000;
        sec_angle = (second * 60) + 270;   //Convert second to angle

        if (sec_angle>360)sec_angle = sec_angle - 360;

        //画秒针
        perim.x = (int)cvRound(cent.x + (rad ) * cos(sec_angle * CV_PI / 180.0));
        perim.y = (int)cvRound(cent.y + (rad ) * sin(sec_angle * CV_PI / 180.0));

        circle(clk, perim, 3, Scalar(0, 255, 255),-1);

        cv::Point2f Prediction=getRotatePoint(clk,perim,cent,-30);

        circle(clk, Prediction, 3, Scalar(255, 255, 0),-1);

        imshow("SmallEnergy", clk); //Show result in a window
//        clk.setTo(0);         // set clk image to zero for next drawing
        clk = back_up.clone();// Clone the previously drawned markings from back-up image

        char c = waitKey(1); // 这里如果参数为10，则看到的是秒针连续地转动；如果是1000，则效果是秒针一秒一秒地跳动
        if (c == 27)break;
    }

    return 0;
}
