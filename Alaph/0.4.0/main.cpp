#include "header.h"
using namespace std;
using namespace cv;

int main()
{
    VideoCapture cap;
    Mat src,mask_bright,mask_color;
    cap.open("/home/link/能量机关/test.mp4");
    union data_send_float data_send[2];
//    cap.open(0);

    int Radius = 79;   //单位：像素
    int CalculateSpeedPer = 10; //单位：帧数
    int DelayAngle = 50;  //单位：角度
    int DelayMillisecond = 1000;  //单位：毫秒
    namedWindow("Parameter");
    createTrackbar("Radius/Pixel","Parameter",&Radius,480,NULL);
    createTrackbar("CalculateSpeedPer","Parameter",&CalculateSpeedPer,50,NULL);
    createTrackbar("DelayAngle/Degree","Parameter",&DelayAngle,180,NULL);
    createTrackbar("DelayMillisecond","Parameter",&DelayMillisecond,1500,NULL);



    time_t rawtime;
    struct tm * timeinfo;
    float second;
    float millisec;
    struct timeb tmb;

    int FLAG = 0;//0小能量 1大能量


    //============大能量
    int BigCounter = 0;
    int SmallCounter = 0;
    float Oldsecond =0;
    float Oldrad = 0;
    bool  BigCalibrationFlag = 0;
    bool  SmallCalibrationFlag = 0;
    float TimeNow;
    float OldTimeNow;
//    double angle=50;
    vector<float>list;
    int Clockwise = 0;
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

        cv::Point cc = FindCircleCenter(detect_center,Radius );
        circle(src, cc, 3, Scalar(0, 255, 255),-1);
//        cout<<detect_center.RotatedArmCenter<<endl;
//        cout<<detect_center.TargetRotatedRect.center<<endl;


        if(FLAG==0)
        {
             SmallCounter++;
             if(SmallCounter%CalculateSpeedPer==0)        //no waitkey150 have waitkey 22
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
                 if(SmallCounter >50 )
                 {
                     Clockwise = judge_direction(list);
                 }
                 else
                     list.push_back(RotateSpeed);

//                 cout<<RotateSpeed<<endl;
//                 cout<<Clockwise<<endl;
                 Oldrad=rad ;

             }

             if(Clockwise!=0)
             {
                 int RotatedAngle;
                 if( Clockwise ==1)
                      RotatedAngle = -DelayAngle;
                 else
                      RotatedAngle = DelayAngle;
                 cv::Point2f Prediction=getRotatePoint(src,detect_center.TargetRotatedRect.center,cc,RotatedAngle);
                 circle(src, Prediction, 3, Scalar(255, 255, 0),-1);
             }

        }

        if(FLAG==1)
        {
             float start,now,guessnow,mark;
             BigCounter++;
             if(BigCounter%CalculateSpeedPer==0)        //no waitkey150 have waitkey 22
             {
                 if(BigCalibrationFlag==0)
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
                     BigCalibrationFlag = 1;
                     start = int(second*1000)%3335/1000.0;
                     if(rad<Oldrad)mark = TimeNow+1.667;
                     else mark = TimeNow;
                 }

                 Oldrad = rad;
                 Oldsecond= second;
                 OldTimeNow = TimeNow;
                 }

             }
             now = int(second*1000)%3335/1000.0;

             if(BigCalibrationFlag==1)
             {
                 if(now>start)guessnow = int((mark+now-start)*1000)%3335/1000.0;
         //        else guessnow = int((mark+3.335-abs(now-start))*1000)%3335/1000.0;
                 else guessnow = int((mark+3.335-abs(now-start))*1000)%3335/1000.0;

                 cv::Point2f Prediction=getRotatePoint(src,detect_center.TargetRotatedRect.center,cc,((250*cos((471*(guessnow+DelayMillisecond/1000.0))/250)-783*(guessnow+DelayMillisecond/1000.0))/600 + 25/60)*180/CV_PI-((250*cos((471*(guessnow))/250)-783*(guessnow))/600 + 25/60)*180/CV_PI);//18 0
                 circle(src, Prediction, 3, Scalar(255, 255, 0),-1);
                 //*************最终点：Prediction******************

             }

        }

        char c = (char)waitKey(1);  //x for calibrate s for stop c for continue
        if(c == 27 )break;
        if(c == 32 )
        {
            BigCalibrationFlag=0;
            list.clear();
        }
        if(c == 115 )                      //S 暂停
        {
            while(true)
            {
                char c = (char)waitKey(1);
                if(c == 99 )break;         //C 继续
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
