#include "header.h"
using namespace std;
using namespace cv;

int main()
{
    VideoCapture cap;
    Mat src,mask_bright,mask_color;
//    cap.open("/home/link/能量机关/test.mp4");

    cap.open(0);


    int fd2car=openPort("/dev/RMCWB");  //开串口
    char buff[10];
    bzero(buff, 10);
    int nRet = 0;
    cout<<"fd2car:"<<fd2car<<endl;

    if(fd2car!=-1)
    {
        configurePort(fd2car);
    }

    union data_send_float data_send[2];
    int cnt=0;

    int Radius = 150;   //单位：像素
    int CalculateSpeedPer = 5; //单位：帧数
    int DelayAngle = 50;  //单位：角度
    int DelayMillisecond = 1000;  //单位：毫秒
    int SmallCounterValue = 50;   //单位：帧数
    int k = 1;
    int b = 0;
    namedWindow("Parameter");
    createTrackbar("Radius/Pixel","Parameter",&Radius,480,NULL);
    createTrackbar("CalculateSpeedPer","Parameter",&CalculateSpeedPer,50,NULL);
    createTrackbar("SmallCounterValue","Parameter",&SmallCounterValue,50,NULL);
    createTrackbar("DelayAngle/Degree","Parameter",&DelayAngle,180,NULL);
    createTrackbar("DelayMillisecond","Parameter",&DelayMillisecond,1500,NULL);
    createTrackbar("k","Parameter",&k,20,NULL);
    createTrackbar("b","Parameter",&b,30,NULL);



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

    int CalibrationTime = 0;

    vector<float>list;

    int Clockwise = -10; //0未确定 1顺 2逆

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

        cv::Point2f Prediction = Point2f(0,0);
        int ShootTime = 0;
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

        cv::Point cc = FindCircleCenter(detect_center,Radius);
        circle(src, cc, 3, Scalar(0, 255, 255),-1);
//        cout<<detect_center.RotatedArmCenter<<endl;
//        cout<<detect_center.TargetRotatedRect.center<<endl;

        if(FLAG==0)
        {
             Clockwise++;
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
                 if(Clockwise == 0 )
                 {
                     Clockwise = judge_direction(list);
                 }
                 else if(Clockwise < 0)
                 {
                     Clockwise++;
                     list.push_back(RotateSpeed);
                 }
//                 cout<<RotateSpeed<<endl;
//                 cout<<Clockwise<<endl;
                 Oldrad=rad ;

             }

             if(Clockwise > 0)
             {
                 int RotatedAngle;
                 if( Clockwise ==1)
                      RotatedAngle = -DelayAngle;
                 else
                      RotatedAngle = DelayAngle;
                 Prediction = getRotatePoint(src,detect_center.TargetRotatedRect.center,cc,RotatedAngle);
                 circle(src, Prediction, 3, Scalar(255, 255, 0),-1);
//                 cout<<"//"<<Prediction.x<<Prediction.y<<endl;


                 int HeightBias = Prediction.y-cc.y;
                 cout<<HeightBias<<endl;
//                 int HeightCal=k*HeightBias+b;



             }

        }

        if(FLAG==1)
        {
             float start,now,guessnow,mark;
             BigCounter++;
             if(BigCounter%CalculateSpeedPer==0)        //no waitkey150 have waitkey 22
             {
                 if(BigCalibrationFlag == 0)
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

                  if(Clockwise == 0)
                  {
                      Clockwise = judge_direction(list);
                  }
                  else if(Clockwise < 0)
                  {
                      Clockwise++;
                      list.push_back(RotateSpeed);
                  }

                  RotateSpeed=abs(RotateSpeed);
                  cout<<RotateSpeed<<":RotateSpeed"<<endl;
                 // 周期：3.335
                  TimeNow = 250*asin((200*abs(RotateSpeed)-261)/157)/471 ;
                  if(isnan(TimeNow)&&abs(RotateSpeed)>2)TimeNow = 0.83375;
                  if(isnan(TimeNow)&&abs(RotateSpeed)<0.8)TimeNow = -0.83375;

     //            TimeNow +=0.83375;

                   cout<<TimeNow<<":TimeNow"<<endl;
     //            cout<<abs(TimeNow-OldTimeNow-BiasSecond)<<endl;
     //            cout<<RotateSpeed<<"-----"<<TimeNow<<endl;

                 if(abs(TimeNow) > 0.8 )
                 {
                     if(CalibrationTime > 5)
                     {
                         BigCalibrationFlag = 1;
                         start = int(second*1000)%3335/1000.0;
                         if(rad<Oldrad)mark = TimeNow+1.667;
                         else mark = TimeNow;
                         cout<<"done"<<endl;
                     }
                     else CalibrationTime = 0;
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


                 double angle;
                 if(Clockwise == 1)
                     angle = ((250*cos((471*(guessnow+DelayMillisecond/1000.0))/250)-783*(guessnow+DelayMillisecond/1000.0))/600 + 25/60)*180/CV_PI-((250*cos((471*(guessnow))/250)-783*(guessnow))/600 + 25/60)*180/CV_PI;
                 if(Clockwise == 2)
                 {
                     guessnow +=1.667;
                     angle =-(((250*cos((471*(guessnow+DelayMillisecond/1000.0))/250)-783*(guessnow+DelayMillisecond/1000.0))/600 + 25/60)*180/CV_PI-((250*cos((471*(guessnow))/250)-783*(guessnow))/600 + 25/60)*180/CV_PI);
                 }


                 Prediction=getRotatePoint(src,detect_center.TargetRotatedRect.center,cc,angle);//18 0
                 circle(src, Prediction, 3, Scalar(255, 255, 0),-1);
//                 cout<<"//"<<Prediction.x<<Prediction.y<<endl;
                 //*************最终点：Prediction******************


                 int HeightBias = Prediction.y-cc.y;
                 cout<<HeightBias<<endl;
//                 int HeightCal=k*HeightBias+b;


             }

             char c = (char)waitKey(1);  //x for calibrate s for stop c for continue

             if(c == 's' )
             {
                 ShootTime = 1;
             }

      }

                 //------------------------------------------------------------------
                 data_send[0].data_float = Prediction.x;                  //Yaw
                 data_send[1].data_float = Prediction.y;                  //Pitch

                 cout<<"// 1:"<<data_send[0].data_float<<endl;                //唯一COUT
                 cout<<"// 2:"<<data_send[1].data_float<<endl;
                 cout<<"// 3:"<<ShootTime<<endl;
//                 if(guessnow<2.6&&guessnow>2.4)ShootTime=1;      //自动击打
//                 else ShootTime=0;

                 sendXYZ(fd2car,data_send,ShootTime,1,cnt);      //发送数据
                 cnt=(cnt+1)%255;                      //累加
                 //------------------------------------------------------------------

        imshow("RESULT", src);

        char c = (char)waitKey(1);  //x for calibrate s for stop c for continue
        if(c == 27 )break;
        if(c == 'x' )  // x
        {
            BigCalibrationFlag=0;
            CalibrationTime = 0;
//            list.clear();
        }
//        if(c == 115 )                      //S 暂停
//        {

//             while(true)
//             {
//                 char c = (char)waitKey(1);
//                 if(c == 99 )break;         //C 继续
//             }
//        }

   }

    return 0;
}
