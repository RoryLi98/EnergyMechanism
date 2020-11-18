#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"
#include <time.h>
#include <sys/timeb.h>
#include <math.h>
#include <stdio.h>
#include <vector>
#include "header.h"

using namespace std;
using namespace cv;

int main()
{
    VideoCapture cap;
    Mat src,mask_bright,mask_color;
    cap.open("/home/link/能量机关/test.mp4");

    time_t rawtime;
    struct tm * timeinfo;
    float second;
    float millisec;
    struct timeb tmb;

    int FLAG = 0;//0小能量 1大能量


    //============大能量
    int couter=0;
    float Oldsecond =0;
    float Oldrad = 0;
    bool CalibrationFlag = 0;
    float TimeNow;
    float OldTimeNow;

    while(1)
    {
        //获取本地时间
        ftime(&tmb);
        rawtime = tmb.time;
        timeinfo = localtime (&rawtime);
        second = timeinfo->tm_sec;
        millisec = tmb.millitm;
        second = second + millisec / 1000;

        cap>>src;

        if(src.rows==0)
        {
            destroyAllWindows();
            break;
        }

        Energy_Agency ee;
        vector<Point2f> prehaps_circle_centers_set;
        RotatedRect final_armor;
        int rotatedArmsIndex;
        vector<Energy_Agency_Rotated_Arm> rotated_arms;
        ee.EnergyProcess(src, mask_bright, mask_color,1);
        EnergyResult detect_center = ee.Energy_Agency_choose_final_target(prehaps_circle_centers_set, src, mask_bright, rotated_arms, final_armor,rotatedArmsIndex);
        circle(src, detect_center.TargetRotatedRect.center, 3, Scalar(0, 255, 255),-1);
        cv::Point cc = FindCircleCenter(detect_center,79);
        circle(src, detect_center.TargetRotatedRect.center, 3, Scalar(0, 255, 255),-1);
//        cout<<detect_center.RotatedArmCenter<<endl;
//        cout<<detect_center.TargetRotatedRect.center<<endl;


        if(FLAG==0)
        {
             cv::Point2f Prediction=getRotatePoint(src,detect_center.TargetRotatedRect.center,cc,-50);
             circle(src, Prediction, 3, Scalar(255, 255, 0),-1);
        }

        if(FLAG==1)
        {
             float start,now,guessnow,mark;
             couter++;
             if(couter%1==0)//no waitkey150 have waitkey 22
             {
                 if(CalibrationFlag==0)
                 {
                  float rad = atan2(detect_center.TargetRotatedRect.center.y-cc.y,detect_center.TargetRotatedRect.center.x-cc.x);
                  float RotateSpeed;
                  float BiasSecond=second-Oldsecond;
                  if(BiasSecond<0)BiasSecond+=60;

                 if(rad-Oldrad>3.14159)
                     RotateSpeed=(rad-Oldrad-3.14159*2)/BiasSecond;
                 if(rad-Oldrad<-3.14159)
                     RotateSpeed=(rad-Oldrad+3.14159*2)/BiasSecond;
                 if(rad-Oldrad>-3.14159&&rad-Oldrad<3.14159)
                     RotateSpeed=(rad-Oldrad)/BiasSecond;


                 // 周期：3.335
                 TimeNow = 250*asin((200*abs(RotateSpeed)-261)/157)/471 ;
                 if(isnan(TimeNow)&&abs(RotateSpeed)>2)TimeNow = 0.83375;
                 if(isnan(TimeNow)&&abs(RotateSpeed)<0.8)TimeNow = -0.83375;

     //            TimeNow +=0.83375;


               cout<<TimeNow<<"="<<TimeNow-OldTimeNow<<"="<<BiasSecond<<endl;
     //            cout<<abs(TimeNow-OldTimeNow-BiasSecond)<<endl;
     //            cout<<RotateSpeed<<"-----"<<TimeNow<<endl;

     //            list.push_back(RotateSpeed);

     //            if(list.size()>20)
     //            {
     //                Clockwise = judge_direction(list);
     ////                cout<<Clockwise<<endl;
     //            }

     //            if(abs(TimeNow-OldTimeNow-BiasSecond)<0.04)

                 if(abs(TimeNow) > 0.8 )
                 {
                     CalibrationFlag = 1;
                     start = int(second*1000)%3335/1000.0;
                     if(rad<Oldrad)mark = TimeNow+1.667;
                     else mark = TimeNow;

                 }

                 Oldrad = rad;
                 Oldsecond= second;
                 OldTimeNow = TimeNow;
                 }
     //            else cout<<"Calibration Done"<<endl;
             }
             now = int(second*1000)%3335/1000.0;



             if(CalibrationFlag==1)
             {
             if(now>start)guessnow = int((mark+now-start)*1000)%3335/1000.0;
     //        else guessnow = int((mark+3.335-abs(now-start))*1000)%3335/1000.0;
             else guessnow = int((mark+3.335-abs(now-start))*1000)%3335/1000.0;

             cv::Point2f Prediction=getRotatePoint(src,detect_center.TargetRotatedRect.center,cc,((250*cos((471*(guessnow+1))/250)-783*(guessnow+1))/600 + 25/60)*180/CV_PI-((250*cos((471*(guessnow))/250)-783*(guessnow))/600 + 25/60)*180/CV_PI);//18 0
             circle(src, Prediction, 3, Scalar(255, 255, 0),-1);
             }


        }




        char c = (char)waitKey(1);  //x for calibrate s for stop c for continue
        if(c == 27 )break;
        if(c == 120 )CalibrationFlag=0;
        if(c == 115 )
        {
            while(true)
            {
                char c = (char)waitKey(1);
                if(c == 99 )break;
            }
        }

        if(src.rows==0)
        {
            destroyAllWindows();
            break;
        }

        imshow("RESULT", src);
//    waitKey(0);
   }

    return 0;
}
