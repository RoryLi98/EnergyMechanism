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

float CalculateDistance(Point2f a, Point2f b)
{
    return sqrt((a.x-b.x) * (a.x-b.x) + (a.y - b.y) * (a.y - b.y));
}

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

    cv::Point2f OldPoint(0,0);

    float millisec;
    struct timeb tmb;

    int bias1=12;
    int bias2=82;
    namedWindow("bias");
    createTrackbar("bias1","bias",&bias1,33,NULL);
    createTrackbar("bias2","bias",&bias2,360,NULL);

    int couter=0;
    float Oldsecond =0;
    float Oldrad = 0;

    int CorrectFlag = 0;
    float TimeNow;
    while (1){
        //获取本地时间
        ftime(&tmb);
        rawtime = tmb.time;
        timeinfo = localtime (&rawtime);

        second = timeinfo->tm_sec;
        millisec = tmb.millitm;

        second = second + millisec / 1000;

//        cout<<second<<endl;
//        sec_angle = (second * 30) + 270;   //Convert second to angle
//        sec_angle =(second*(1.305+0.785*sin(1.884*second)))*180/CV_PI;
//        sec_angle = (second*1.305)*180/CV_PI+0.785*sin(1.884*second)*180/CV_PI;

        sec_angle = (-(250*cos((471*second)/250)-783*second)/600 + 25/60)*180/CV_PI;


//        cout<<(1.305+0.785*sin(1.884*second))<<endl;
//        cout<<sec_angle<<endl;
        if (sec_angle>360)sec_angle = sec_angle-360 ;

        perim.x = (int)cvRound(cent.x + (rad ) * cos(sec_angle * CV_PI / 180.0));
        perim.y = (int)cvRound(cent.y + (rad ) * sin(sec_angle * CV_PI / 180.0));

        circle(clk, perim, 3, Scalar(0, 255, 255),-1);

//        couter=1;
          couter++;
//        if(couter%20==0)
//        {
//        float Distant = CalculateDistance(OldPoint,perim);
//        cout<<Distant<<endl;
//        Mat speed(100, 200, CV_8UC3,Scalar(0,0,0));
//        circle(speed, cv::Point2f(50,Distant+20), 3, Scalar(255, 255, 0),-1);
//        imshow("Speed",speed);
//        OldPoint = perim;
//        }

        if(couter%23==0)
        {
            if(CorrectFlag==0)
            {
        float rad = atan2(perim.y-cent.y,perim.x-cent.x);
        float RotateSpeed;
//        cout<<rad-Oldrad<<endl;
//        RotateSpeed=(rad-Oldrad)/(second-Oldsecond);
        if(rad-Oldrad>3.14159)
            RotateSpeed=(rad-Oldrad-3.14159*2)/(second-Oldsecond);
        if(rad-Oldrad<-3.14159)
            RotateSpeed=(rad-Oldrad+3.14159*2)/(second-Oldsecond);
        if(rad-Oldrad>-3.14159&&rad-Oldrad<3.14159)
            RotateSpeed=(rad-Oldrad)/(second-Oldsecond);

//        if(abs(RotateSpeed)>5)continue;

//        cout<<RotateSpeed<<":"<<1.305+0.785*sin(1.884*second)<<":"<<RotateSpeed-(1.305+0.785*sin(1.884*second))<<endl;
//        cout<<(second-Oldsecond)<<endl;

        Mat speed(100, 200, CV_8UC3,Scalar(0,0,0));
        circle(speed, cv::Point2f(50,RotateSpeed*20+50), 3, Scalar(255, 255, 0),-1);
        circle(speed, cv::Point2f(100,(1.305+0.785*sin(1.884*second))*20+50), 3, Scalar(255, 255, 0),-1);
        imshow("Speed",speed);
        Oldrad = rad;
        Oldsecond= second;


        // 周期：3.335
        TimeNow = 250*asin((200*RotateSpeed-261)/157)/471 ;
        //cout<<TimeNow<<":"<< ((int(second*1000))%3335)/1000.0<<endl;
        cout<<TimeNow+0.833<<endl;
            }
        }

        cv::Point2f Prediction=getRotatePoint(clk,perim,cent,-(1.305+0.785*sin(1.884*second+bias1/10))*180/CV_PI+bias2-100);//18 0
//        cv::Point2f Prediction=getRotatePoint(clk,perim,cent,((250*cos((471*second)/250)-783*second)/600 + 25/60)*180/CV_PI);

        circle(clk, Prediction, 3, Scalar(255, 255, 0),-1);

        imshow("BigEnergy", clk);   //Show result in a window
//        clk.setTo(0);         // set clk image to zero for next drawing
        clk = back_up.clone();  // Clone the previously drawned markings from back-up image

        char c = waitKey(1); // 这里如果参数为10，则看到的是秒针连续地转动；如果是1000，则效果是秒针一秒一秒地跳动
        if (c == 27)break;
    }

    return 0;
}
