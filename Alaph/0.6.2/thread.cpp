#include "thread.h"
#include "CameraApi.h" //相机SDK的API头文件
#include "energy.h"
#define BUFFER_SIZE 5
Mat data[BUFFER_SIZE];

volatile unsigned int prdIdx;
volatile unsigned int csmIdx;

//功能函数
//比较中心点的横坐标
bool comp_center_x(RotatedRect a, RotatedRect b)
{
    return a.center.x < b.center.x;
}

//计算角度
float calculate_angle(Point2f s1, Point2f s2)
{
    return abs(atan2(s2.y - s1.y, s2.x - s1.x) * 180.0 / CV_PI);
}
//计算点到点距离
float GetDistance(Point2f a, Point2f b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

//计算点到目标的距离
double calculate_dot_to_line_distance(Point2f p0, double k, double b, int flag)
{
    if (flag == 0)
        return fabs(k * p0.x - p0.y + b) / sqrt(k * k + 1);
    else
        return fabs(k - p0.x);
}

//通过左上、左下、右上、右下四个点画出四边形方框
void drawBoxBy4Points(Point2f pt[4], Mat &img)
{
    line(img, pt[0], pt[1], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pt[1], pt[2], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pt[2], pt[3], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pt[3], pt[0], CV_RGB(0, 255, 0), 2, 8, 0);
}

cv::Point FindCircleCenter(EnergyResult DetectResult, int r, vector<Point2f> &circle_center)
{
    cv::Point PreCircleCenter;
    cv::Point CircleCenter;
    if (DetectResult.flag == 0)
    {
        return cv::Point(0, 0);
    }
    if (DetectResult.flag == 1)
    {
        double Theta;
        cv::Point2f RotatedArmCenter = DetectResult.RotatedArmCenter;
        RotatedRect TargetArm = DetectResult.TargetRotatedRect;
        cv::Point2f TargetCenter = DetectResult.TargetRotatedRect.center;
        Point2f vertices[4], distance_points1, distance_points2;
        TargetArm.points(vertices); //vertices[0]为最下方的点，若有两个最下方的点（四个边分别与x、y轴平行）则取左下方。顺时针vertices为最外大臂四点

        if (TargetArm.size.width < TargetArm.size.height) //右上方的边为height：顺时针旋转，碰到的第一条边长，叫做height
        {
            distance_points1.x = (vertices[2].x + vertices[3].x) / 2;
            distance_points1.y = (vertices[2].y + vertices[3].y) / 2;
            distance_points2.x = (vertices[0].x + vertices[1].x) / 2;
            distance_points2.y = (vertices[0].y + vertices[1].y) / 2;
            //           cout<<"TargetArm.size.width > TargetArm.size.height"<<endl;

            double distance_1 = sqrt((distance_points1.x - RotatedArmCenter.x) * (distance_points1.x - RotatedArmCenter.x) + (distance_points1.y - RotatedArmCenter.y) * (distance_points1.y - RotatedArmCenter.y));
            double distance_2 = sqrt((distance_points2.x - RotatedArmCenter.x) * (distance_points2.x - RotatedArmCenter.x) + (distance_points2.y - RotatedArmCenter.y) * (distance_points2.y - RotatedArmCenter.y));
            //           cout<<distance_1<<":"<<distance_2<<endl;
            if (distance_1 > distance_2) //youshang
            {
                Theta = (atan2(vertices[0].x - vertices[3].x, vertices[0].y - vertices[3].y) + atan2(vertices[1].x - vertices[2].x, vertices[1].y - vertices[2].y)) / 2;
            }
            if (distance_1 < distance_2) //zuoxia
            {
                Theta = (atan2(vertices[3].x - vertices[0].x, vertices[3].y - vertices[0].y) + atan2(vertices[2].x - vertices[1].x, vertices[2].y - vertices[1].y)) / 2;
            }
            //cout<<Theta*180/3.14159<<endl;
            PreCircleCenter.x = (int)cvRound(r * cos(Theta - 1.57079) + TargetCenter.x);
            PreCircleCenter.y = (int)cvRound(-r * sin(Theta - 1.57079) + TargetCenter.y);
        }
        else if (TargetArm.size.width > TargetArm.size.height)
        {
            distance_points1.x = (vertices[1].x + vertices[2].x) / 2;
            distance_points1.y = (vertices[1].y + vertices[2].y) / 2;
            distance_points2.x = (vertices[0].x + vertices[3].x) / 2;
            distance_points2.y = (vertices[0].y + vertices[3].y) / 2;
            //           cout<<"TargetArm.size.width < TargetArm.size.height"<<endl;
            double distance_1 = sqrt((distance_points1.x - RotatedArmCenter.x) * (distance_points1.x - RotatedArmCenter.x) + (distance_points1.y - RotatedArmCenter.y) * (distance_points1.y - RotatedArmCenter.y));
            double distance_2 = sqrt((distance_points2.x - RotatedArmCenter.x) * (distance_points2.x - RotatedArmCenter.x) + (distance_points2.y - RotatedArmCenter.y) * (distance_points2.y - RotatedArmCenter.y));
            //            cout<<distance_1<<":"<<distance_2<<endl;
            if (distance_1 > distance_2) //zuoshang
            {
                Theta = (atan2(vertices[0].x - vertices[1].x, vertices[0].y - vertices[1].y) + atan2(vertices[3].x - vertices[2].x, vertices[3].y - vertices[2].y)) / 2;
            }
            if (distance_1 < distance_2) //youxia
            {
                Theta = (atan2(vertices[1].x - vertices[0].x, vertices[1].y - vertices[0].y) + atan2(vertices[2].x - vertices[3].x, vertices[2].y - vertices[3].y)) / 2;
            }
            //cout<<Theta*180/3.14159<<endl;
            PreCircleCenter.x = (int)cvRound(r * cos(Theta - 1.57079) + TargetCenter.x);
            PreCircleCenter.y = (int)cvRound(-r * sin(Theta - 1.57079) + TargetCenter.y);
        }

        int MinDistance = 888;
        int MinNumber = -1;
        //cout<<"circle_center.size():"<<circle_center.size()<<endl;
        for (int k = 0; k < circle_center.size(); k++)
        {

            if (MinDistance > GetDistance(circle_center[k], PreCircleCenter))
            {
                MinDistance = GetDistance(circle_center[k], PreCircleCenter);
                MinNumber = k;
            }
        }
        if (circle_center.size() != 0 && GetDistance(circle_center[MinNumber], PreCircleCenter) < 50)
        {
            CircleCenter = circle_center[MinNumber];
            cout<<"锁住圆心了！CircleTarget LOCK "<<GetDistance(circle_center[MinNumber],PreCircleCenter)<<endl;
        }
        else
        {
            cout << "circle_center.size():" << circle_center.size() << endl;
            CircleCenter = PreCircleCenter;
            if (circle_center.size() != 0)
                cout << "GetDistance(circle_center[MinNumber],PreCircleCenter):" << GetDistance(circle_center[MinNumber], PreCircleCenter) << endl;
        }
        return CircleCenter;
    }
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
    int x = cvRound((x1 - x2) * cos(CV_PI / 180.0 * angle) - (y1 - y2) * sin(CV_PI / 180.0 * angle) + x2);
    int y = cvRound((x1 - x2) * sin(CV_PI / 180.0 * angle) + (y1 - y2) * cos(CV_PI / 180.0 * angle) + y2);
    x = x;
    y = row - y;
    dstPoints.x = x;
    dstPoints.y = y;
    return dstPoints;
}

float CalculateDistance(Point2f a, Point2f b)
{
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

int judge_direction(vector<float> list)
{
    float num = 0;
    for (int i = 0; i < list.size(); i++)
        num += list[i];
    if (num > 0)
        return 1; //顺时针
    if (num < 0)
        return 2; //逆时针
    return 0;
}

Image_Thread::Image_Thread(int fd2car)
{

    this->fd2car = fd2car;
}

void Image_Thread::Image_Thread_energy()
{
    Mat src, mask_bright, mask_color;


    union data_send_float data_send[2];
    int cnt = 0;

    int Radius = 77;             //单位：像素
    int CalculateSpeedPer = 5;   //单位：帧数
    int DelayAngle = 30;         //单位：角度
    int DelayMillisecond = 270;  //单位：毫秒
    int SmallCounterValue = 500; //单位：帧数
    int k = 1;
    int b = 0;

    int yaw = 33;
    int pitch = 8;
    namedWindow("Parameter");
    createTrackbar("Radius/Pixel", "Parameter", &Radius, 480, NULL);
    createTrackbar("CalculateSpeedPer", "Parameter", &CalculateSpeedPer, 50, NULL);
    // createTrackbar("SmallCounterValue","Parameter",&SmallCounterValue,50,NULL);
    createTrackbar("DelayAngle/Degree", "Parameter", &DelayAngle, 180, NULL);
    createTrackbar("DelayMillisecond", "Parameter", &DelayMillisecond, 1500, NULL);
    // createTrackbar("k","Parameter",&k,20,NULL);
    // createTrackbar("b","Parameter",&b,30,NULL);
    createTrackbar("yaw", "Parameter", &yaw, 100, NULL);
    createTrackbar("pitch", "Parameter", &pitch, 100, NULL);

    time_t rawtime;
    struct tm *timeinfo;
    float second;
    float millisec;
    struct timeb tmb;

    int FLAG = 0; //0小能量 1大能量

    //============大能量
    int BigCounter = 0;
    int SmallCounter = 0;
    float Oldsecond = 0;
    float Oldrad = 0;
    bool BigCalibrationFlag = 0;
    //    bool  SmallCalibrationFlag = 0;
    float TimeNow;
    float OldTimeNow;
    float Oldradmark = 0;
    int CalibrationTime = 0;

    vector<float> list;

    int Clockwise = -20; //0未确定 1顺 2逆
    Energy_Agency ee;
    while (true)
    {
        while (prdIdx - csmIdx == 0)
            ;
        data[csmIdx % BUFFER_SIZE].copyTo(src);
        ++csmIdx;

        cv::Point2f Prediction = Point2f(0, 0);
        int ShootTime = 0;
        if (src.rows == 0)
        {
            destroyAllWindows();
            break;
        }

        //                    clock_t start,finish;
        //                    double duration;
        //                    start = clock();
        //                    finish = clock();
        //                    duration = double(finish - start)/CLOCKS_PER_SEC;
        //                    cout<<"Duration:"<<duration<<endl;

        vector<Point2f> PrehapsCircleCenters;
        RotatedRect final_armor;
        int rotatedArmsIndex;
        vector<Energy_Agency_Rotated_Arm> rotated_arms;
        ee.EnergyProcess(src, mask_bright, mask_color, 1);
        EnergyResult detect_center = ee.Energy_Agency_choose_final_target(PrehapsCircleCenters, src, mask_bright, rotated_arms, final_armor, rotatedArmsIndex);
        circle(src, detect_center.TargetRotatedRect.center, 3, Scalar(0, 255, 255), -1);
        cv::Point cc = FindCircleCenter(detect_center, Radius, PrehapsCircleCenters);
        circle(src, cc, 3, Scalar(0, 255, 255), -1);

        if (cc.x != 0 || detect_center.RotatedArmCenter.x != 0)
        {
            if (FLAG == 0)
            {
                //获取本地时间
                ftime(&tmb);
                rawtime = tmb.time;
                timeinfo = localtime(&rawtime);
                second = timeinfo->tm_sec;
                millisec = tmb.millitm;
                second = second + millisec / 1000;
                SmallCounter++;
                if (SmallCounter % CalculateSpeedPer == 0)
                {
                    float rad = atan2(detect_center.TargetRotatedRect.center.y - cc.y, detect_center.TargetRotatedRect.center.x - cc.x);
                    float RotateSpeed;
                    float BiasSecond = second - Oldsecond;
                    if (BiasSecond < 0)
                        BiasSecond += 60;
                    if (rad - Oldrad > 3.14159)
                        RotateSpeed = (rad - Oldrad - 3.14159 * 2) / BiasSecond;
                    if (rad - Oldrad < -3.14159)
                        RotateSpeed = (rad - Oldrad + 3.14159 * 2) / BiasSecond;
                    if (rad - Oldrad > -3.14159 && rad - Oldrad < 3.14159)
                        RotateSpeed = (rad - Oldrad) / BiasSecond;
                    //                   cout<<"RotateSpeed"<<RotateSpeed<<endl;
                    if (Clockwise == 0)
                    {
                        Clockwise = judge_direction(list);
                    }
                    else if (Clockwise < 0)
                    {
                        // cout<<RotateSpeed<<endl;
                        Clockwise++;
                        list.push_back(RotateSpeed);
                    }

                    Oldrad = rad;
                }

                if (Clockwise > 0)
                {
                    int RotatedAngle;
                    if (Clockwise == 1)
                        RotatedAngle = -DelayAngle;
                    else
                        RotatedAngle = DelayAngle;
                    Prediction = getRotatePoint(src, detect_center.TargetRotatedRect.center, cc, RotatedAngle);
                    circle(src, Prediction, 3, Scalar(255, 255, 0), -1);

                    Prediction.x = Prediction.x - 1280 / 2;
                    Prediction.y = Prediction.y - 1024 / 2;
                    //int HeightBias = Prediction.y - cc.y;
                    //cout<<HeightBias<<endl;
                    //int HeightCal=k*HeightBias+b;
                }
            }

            if (FLAG == 1)
            {
                //获取本地时间
                ftime(&tmb);
                rawtime = tmb.time;
                timeinfo = localtime(&rawtime);
                second = timeinfo->tm_sec;
                millisec = tmb.millitm;
                second = second + millisec / 1000;

                float start, now, guessnow, mark;
                BigCounter++;
                if (BigCounter % CalculateSpeedPer == 0) //no waitkey150 have waitkey 22
                {
                    if (BigCalibrationFlag == 0 || Clockwise <= 0)
                    {
                        float rad = atan2(detect_center.TargetRotatedRect.center.y - cc.y, detect_center.TargetRotatedRect.center.x - cc.x);
                        float RotateSpeed;
                        float BiasSecond = second - Oldsecond;
                        if (BiasSecond < 0)
                            BiasSecond += 60;

                        if (rad - Oldrad > 3.14159)
                            RotateSpeed = (rad - Oldrad - 3.14159 * 2) / BiasSecond;
                        if (rad - Oldrad < -3.14159)
                            RotateSpeed = (rad - Oldrad + 3.14159 * 2) / BiasSecond;
                        if (rad - Oldrad > -3.14159 && rad - Oldrad < 3.14159)
                            RotateSpeed = (rad - Oldrad) / BiasSecond;

                        if (Clockwise == 0)
                        {
                            Clockwise = judge_direction(list);
                        }
                        else if (Clockwise < 0)
                        {
                            Clockwise++;
                            list.push_back(RotateSpeed);
                        }

                        RotateSpeed = abs(RotateSpeed);
                        //             cout<<RotateSpeed<<":RotateSpeed"<<endl;
                        // 周期：3.335
                        TimeNow = 250 * asin((200 * abs(RotateSpeed) - 261) / 157) / 471;
                         if (isnan(TimeNow) && abs(RotateSpeed) > 2)
                             TimeNow = 0.83375;
                         if (isnan(TimeNow) && abs(RotateSpeed) < 0.8)
                             TimeNow = -0.83375;
                        cout << TimeNow << endl;

                        if(BigCounter % (CalculateSpeedPer*5) == 0)
                        {
                            Oldradmark = rad;
                            cout<<"HAHAHAHHA"<<endl;


                        }
//                        if(abs(TimeNow) > 0.8 && Oldradmark!=0 )
//                        {
//                            if(rad<Oldradmark)mark = TimeNow+1.667;
//                            if(rad>Oldradmark)mark = TimeNow;
//                            BigCalibrationFlag = 1;
//                            start = int(second*1000)%3335/1000.0;
//                        }


                        Oldrad = rad;
                        Oldsecond = second;
                        OldTimeNow = TimeNow;
                    }
                }
                now = int(second * 1000) % 3335 / 1000.0;

                if (BigCalibrationFlag == 1 && Clockwise > 0)
                {

                    if (now > start)
                        guessnow = int((mark + now - start) * 1000) % 3335 / 1000.0;
                    else
                        guessnow = int((mark + 3.335 - abs(now - start)) * 1000) % 3335 / 1000.0;
                    double angle;
                    if (Clockwise == 1)
                    {
                        angle = ((250 * cos((471 * (guessnow + DelayMillisecond / 1000.0)) / 250) - 783 * (guessnow + DelayMillisecond / 1000.0)) / 600 + 25 / 60) * 180 / CV_PI - ((250 * cos((471 * (guessnow)) / 250) - 783 * (guessnow)) / 600 + 25 / 60) * 180 / CV_PI;
                        //cout<<"大能量 顺时针"<<endl;
                    }
                    if (Clockwise == 2)
                    {
                        guessnow += 1.667;
                        angle = -(((250 * cos((471 * (guessnow + DelayMillisecond / 1000.0)) / 250) - 783 * (guessnow + DelayMillisecond / 1000.0)) / 600 + 25 / 60) * 180 / CV_PI - ((250 * cos((471 * (guessnow)) / 250) - 783 * (guessnow)) / 600 + 25 / 60) * 180 / CV_PI);
                        //cout<<"大能量 逆时针"<<endl;
                    }
                    if (guessnow < 2.45 && guessnow > 2.35)
                        ShootTime = 1; //自动击打
                    else
                        ShootTime = 0;
                    Prediction = getRotatePoint(src, detect_center.TargetRotatedRect.center, cc, angle); //18 0
                    circle(src, Prediction, 3, Scalar(255, 255, 0), -1);
                    //                 cout<<"//"<<Prediction.x<<Prediction.y<<endl;
                    //*************最终点：Prediction******************

                    //                     int HeightBias = Prediction.y-cc.y;
                    //                    // cout<<HeightBias<<endl;
                    //    //                 int HeightCal=k*HeightBias+b;
                    Prediction.x = Prediction.x - 1280 / 2;
                    Prediction.y = Prediction.y - 1024 / 2;
                }
            }
        }
        //------------------------------------------------------------------
        data_send[0].data_float = Prediction.x + yaw - 50;   //Yaw
        data_send[1].data_float = Prediction.y + pitch - 50; //Pitch

        //                     cout<<"// 1:"<<data_send[0].data_float<<endl;                //唯一COUT
        //                     cout<<"// 2:"<<data_send[1].data_float<<endl;
        //                     cout<<"// 3:"<<ShootTime<<endl;

        sendXYZ(fd2car, data_send, ShootTime, 1, cnt); //发送数据
        cnt = (cnt + 1) % 255;                         //累加
                                                       //------------------------------------------------------------------
        resize(src,src,Size(src.cols/2,src.rows/2),0,0);

        imshow("RESULT", src);

        char c = (char)waitKey(1); //x for calibrate s for stop c for continue
        if (c == 27)
            break;
        if (c == 'x') // x
        {
            BigCalibrationFlag = 0;
            CalibrationTime = 0;
        }
    }
}

void Image_Thread::Image_Thread_read_from_cam()
{

    unsigned char *g_pRgbBuffer;

    int iCameraCounts = 1;
    int iStatus = -1;
    tSdkCameraDevInfo tCameraEnumList;
    int hCamera;
    tSdkCameraCapbility tCapability; //设备描述信息
    tSdkFrameHead sFrameInfo;
    tSdkImageResolution sResolution = {0};
    BYTE *pbyBuffer;
    int iDisplayFrames = 10000;
    IplImage *iplImage = NULL;
    int channel = 3;

    CameraSdkInit(1);

    //枚举设备，并建立设备列表
    iStatus = CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
    printf("state = %d\n", iStatus);

    printf("count = %d\n", iCameraCounts);
    //没有连接设备
    if (iCameraCounts == 0)
    {
        return;
    }

    //相机初始化。初始化成功后，才能调用任何其他相机相关的操作接口
    iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera);

    //初始化失败
    printf("state = %d\n", iStatus);
    if (iStatus != CAMERA_STATUS_SUCCESS)
    {
        return;
    }

    //获得相机的特性描述结构体。该结构体中包含了相机可设置的各种参数的范围信息。决定了相关函数的参数
    CameraGetCapability(hCamera, &tCapability);

    //
    g_pRgbBuffer = (unsigned char *)malloc(tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3);
    //g_readBuf = (unsigned char*)malloc(tCapability.sResolutionRange.iHeightMax*tCapability.sResolutionRange.iWidthMax*3);

    /*让SDK进入工作模式，开始接收来自相机发送的图像
        数据。如果当前相机是触发模式，则需要接收到
        触发帧以后才会更新图像。    */
    CameraPlay(hCamera);

    /*其他的相机参数设置
        例如 CameraSetExposureTime   CameraGetExposureTime  设置/读取曝光时间
             CameraSetImageResolution  CameraGetImageResolution 设置/读取分辨率
             CameraSetGamma、CameraSetConrast、CameraSetGain等设置图像伽马、对比度、RGB数字增益等等。
             更多的参数的设置方法，，清参考MindVision_Demo。本例程只是为了演示如何将SDK中获取的图像，转成OpenCV的图像格式,以便调用OpenCV的图像处理函数进行后续开发
        */

    if (tCapability.sIspCapacity.bMonoSensor)
    {
        channel = 1;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);
    }
    else
    {
        channel = 3;
        CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);
    }

    //        sResolution.iIndex=0xff;

    //        sResolution.iWidthFOV=1280;
    //        sResolution.iHeightFOV=960;
    //        sResolution.iHeight=480;
    //        sResolution.iWidth=640;

    //        sResolution.iVOffsetFOV=0;
    //        sResolution.iHOffsetFOV=0;

    //        sResolution.iWidthZoomHd=0;
    //        sResolution.iHeightZoomHd=0;

    //        sResolution.uBinAverageMode=0;
    //        sResolution.uBinSumMode=0;
    //        sResolution.uResampleMask=0;
    //        sResolution.uSkipMode=0;

    // CameraSetImageResolution(hCamera,&sResolution);
    CameraSetAeState(hCamera, 0);
    CameraSetExposureTime(hCamera, 800);

    while (true)
    {
        //TIME_START(produce)
        while (prdIdx - csmIdx >= BUFFER_SIZE)
            ; //cout<<"wait for process!!\n";
        //for change resolution of camera
        if (CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 1000) == CAMERA_STATUS_SUCCESS)
        {
            CameraImageProcess(hCamera, pbyBuffer, g_pRgbBuffer, &sFrameInfo);

            cv::Mat matImage(
                cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight),
                sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
                g_pRgbBuffer);
            resize(matImage, matImage, Size(1280, 1024));
            //imshow("Opencv Demo", matImage);
            matImage.copyTo(data[prdIdx % BUFFER_SIZE]);

            CameraReleaseImageBuffer(hCamera, pbyBuffer);
        }
        ++prdIdx;
        //TIME_END(produce)
    }

    CameraUnInit(hCamera);
    //注意，现反初始化后再free
    free(g_pRgbBuffer);
}
