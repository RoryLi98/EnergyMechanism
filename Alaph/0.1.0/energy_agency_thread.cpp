#include "energy_agency_thread.h"
#include <stdio.h>
#include <cstdlib>

using namespace std;
using namespace cv;

//#define FINAL_DETECT
//#define CAN_NOT_FIND_TARGET
//139-140 Line
//VideoWriter vw(to_string(rand())+".avi", CV_FOURCC('M','J','P','G'),60.0, Size(1280,720));
energy_agency_thread::energy_agency_thread()
{
    
}

//====================EA_initDetector=====================================
//dectectMode
//-1 debug计算延时以及云台延时
//0  激光瞄准
//1  打击静止(处理下坠) ;
//2  打击运动(处理下坠+预瞄运动)

//Mat src_EA;
    Mat  mask_bright, mask_color;

//======================Cam===============================================

void energy_agency_thread::EA_initCam(char* file_dev, int exp_t)
{
    for(int i=0;i<BUFFER_SIZE;i++ )
    {
        this->camBufferData.EA_Cam_dataUsingFlags[i]=0;
    }
    #ifdef EA_USECAM
        this->readFromCamInfo.cap=new RMVideoCapture(file_dev, 3);
        this->readFromCamInfo.cap->setVideoFormat(1280, 720, true);   //1280 720
        this->readFromCamInfo.cap->setExposureTime(0, exp_t);//8
        this->readFromCamInfo.cap->startStream();
        this->readFromCamInfo.cap->info();
    #else
        this->readFromCamInfo.cap=new VideoCapture("/home/sinh/pics/BigEnergy.mp4");
        if(!readFromCamInfo.cap->isOpened())
        {
            cout<<"can not open"<<endl;
        }

    #endif
}


void energy_agency_thread::EA_writeBuffer(int exp_t)
{
    if( camBufferData.EA_Cam_dataUsingFlags[camBufferData.writerIdx]==0  )
    {
        camBufferData.EA_Cam_dataUsingFlags[camBufferData.writerIdx]=1;           //置为我在使用
        Mat tempMat;
        *(this->readFromCamInfo.cap)>>tempMat;
        if( tempMat.empty() )
        {
            cout<<"empty"<<endl;
            this->readFromCamInfo.cap->closeStream();
            this->readFromCamInfo.cap->restartCapture();
            this->readFromCamInfo.cap->setVideoFormat(1280, 720, true);
            this->readFromCamInfo.cap->setExposureTime(0, exp_t);//8
            this->readFromCamInfo.cap->startStream();
            this->readFromCamInfo.cap->info();
            usleep(500000);
        }
        else
        {
            //*(this->readFromCamInfo.cap)>>camBufferData.EA_Cam_data[(camBufferData.writerIdx) % BUFFER_SIZE].src;
            tempMat.copyTo( camBufferData.EA_Cam_data[(camBufferData.writerIdx) % BUFFER_SIZE].src );
        }
        camBufferData.EA_Cam_dataUsingFlags[camBufferData.writerIdx]=0;        //用完了
        camBufferData.writerIdx=(camBufferData.writerIdx+1)%BUFFER_SIZE;
    }
    else
    {
        if(camBufferData.writerIdx==0) camBufferData.writerIdx=5;
        camBufferData.writerIdx=(camBufferData.writerIdx-1)%BUFFER_SIZE;
        //cout<<"**writer-1"<<endl;
    }
}


void energy_agency_thread::EA_initDetector()
{
    //fd2car = input_fd2car;
    ee.Energy_Agency_loadSVM_Model(this->EA_parameter.svm_Model_Path);
    ee.set_roll_prevaricate(this->EA_parameter.roll_prevaricate);
    //ee.create_debug_image();
    //读取相机内参
    Mat cam_matrix_1024, distortion_coeff_1024;

    FileStorage fs(this->EA_parameter.calibration_Path, FileStorage::READ); //标定文件千万注意不要开错
    fs["intrinsic_matrix"] >> cam_matrix_1024;
    fs["distortion_coeffs"] >> distortion_coeff_1024;
    //构造相机坐标系到云台坐标系的转移矩阵

    //----------------平移--------------
    const double barrel_ptz_offset_y = 0;
    const double ptz_camera_x = 12;  //cm
    const double ptz_camera_y = 0;   //cm    相机在云台上面8.5cm时:8.5
    const double ptz_camera_z = 0;   //cm-4
    //double theta = -atan((ptz_camera_y + barrel_ptz_offset_y)/overlap_dist);
    //----------------旋转--------------
    double theta_y = (this->EA_parameter.theta_y)*(pi/180);        //yaw 越负激光点越右    -0.6               //-3.5
    double theta_x = (this->EA_parameter.theta_x)*(pi/180);        //pitch 原本激光点偏下时调为-4   越小激光越上 //-0.5

    double r_data_y[] = {cos(theta_y),0,-sin(theta_y),
                         0,1,0,
                         sin(theta_y), 0, cos(theta_y)};
    double r_data_x[] = {1,0,0,
                         0 , cos(theta_x) , sin(theta_x),
                         0 , -sin(theta_x) , cos(theta_x)};

    double t_data[] = {ptz_camera_x, ptz_camera_y, ptz_camera_z}; // ptz org position in camera coodinate system

    Mat t_camera_ptz(3,1, CV_64FC1, t_data);

    Mat r_camera_ptz_x(3,3, CV_64FC1,r_data_x);
    Mat r_camera_ptz_y(3,3, CV_64FC1,r_data_y);
    Mat r_camera_ptz(3,3, CV_64FC1);
    r_camera_ptz=r_camera_ptz_x*r_camera_ptz_y;
    Energy_solver=new AngleSolver(cam_matrix_1024, distortion_coeff_1024, 50.0, 1000.0);
    Energy_solver->setRelationPoseCameraPTZForEnergy(r_camera_ptz, t_camera_ptz, barrel_ptz_offset_y);
}

void energy_agency_thread::EA_readFromBuffer(cv::Mat &operateSrc)
{

    while(camBufferData.EA_Cam_dataUsingFlags[camBufferData.readerIdx]==1);  //等待至无人使用
    camBufferData.EA_Cam_dataUsingFlags[camBufferData.readerIdx]=1;           //置为我在使用
    camBufferData.EA_Cam_data[(camBufferData.readerIdx) % BUFFER_SIZE].src.copyTo(operateSrc);
//    if(!operateSrc.empty())
//        vw << operateSrc;
    camBufferData.EA_Cam_dataUsingFlags[camBufferData.readerIdx]=0;        //用完了
    camBufferData.readerIdx=(camBufferData.readerIdx+1)%BUFFER_SIZE;
}

/*
 *矫正弹道，测试时用
int energy_agency_thread::correctTheBullet()
{
    int returnNum;
    //矫正弹道用，比赛不要用！

    ea_correctBullet correctTheBullet(this->Energy_solver);
    returnNum=correctTheBullet.correctBullet(src_EA,angle_x,angle_y);
    angle_y=angle_y-EA_parameter.angleDrop;

    cout<<"In eat correctBullet anglex"<<angle_x<<" "<<angle_y<<endl;

    return returnNum;
}
*/

int energy_agency_thread::EA_detector(int dectectMode, cv::Mat &src_EA)
{
    vector<Point2f> prehaps_circle_centers_set;
    Point2f circle_center=Point2f(-1,-1);
    RotatedRect final_armor;
    int rotatedArmsIndex;

    //==============input img and pretreatment===============

    //EA_readFromBuffer(src_EA);
    if(src_EA.empty())
    {
        //cout<<"empty!"<<endl;
        return -1;
    }

    //reciver cont
    // image thread 1\2
    // =====!bao guang 30 !!!!
    //==========correct
//        cout<<"correctBullet!"<<endl;
//        int correctFind=this->correctTheBullet();
//        cout<<"return: "<<correctFind<<endl;
//        return correctFind;
    //correct---------

    //若要更改，记得更改： refresh笛卡尔坐标系转换
    ee.Energy_Agency_process(src_EA, mask_bright, mask_color);

    //==============find target==============================
    //返回值为两个量，第一个是装甲板中心点，第二个是该旋转臂的中心点
    vector<Point2f> detect_center = ee.Energy_Agency_choose_final_target(prehaps_circle_centers_set, src_EA, mask_bright, rotated_arms, final_armor,rotatedArmsIndex);
    //circle_Center_set是所有可能是圆心的外轮廓的中心点集合

    if(detect_center.size() == 0)
    {
#ifdef CAN_NOT_FIND_TARGET
        imshow("src", src_EA);
#endif
        waitKey(1);
        cout<<"can not find the armor,NO ARMOR NO LEAF "<<endl;
        return -1;
    }
    //cout<<"TEST0= "<<prehaps_circle_centers_set.size()<<endl;
    //==========================find circle_centers====================
    if(prehaps_circle_centers_set.size() > 0)
    {
        circle_center = ee.calculate_probably_circle_center(detect_center, prehaps_circle_centers_set,final_armor);
        //detect_center[0]=perhaps_target.center ;  detect_center[1]=extern_rotatedrect.center;
        circle(src_EA,circle_center,5,Scalar(255,180,180),3,8,0);
    }

    if(prehaps_circle_centers_set.size() > 0 && circle_center.x!=-1)
    {
        circle(src_EA,circle_center,5,Scalar(180,180,180),3,8,0);
    }
    else
    {
        //cout<<"can find the target,but no circle center!"<<endl;
        //waitKey(0);
    }

    //==================Judge Finally ShootPoint===================================
    Point2f shootPoint=Point2f(-1,-1);
    Mat finallyShootPointMat=src_EA.clone();

    if(dectectMode==-1)//debug 用于调参，观察 计算以及云台延时
    {
        shootPoint.x=final_armor.center.x;
        shootPoint.y=final_armor.center.y;

        ee.refreshStructInfo(detect_center[0],circle_center,detect_center[1],final_armor);
        circle(src_EA,circle_center,5,Scalar(0,255,255),3,8,0);//圆心

        Point2f forecastSeeFinalArmorCenter=Point2f(-1,-1);
        ee.shootRollEnergyAgencyDebug(forecastSeeFinalArmorCenter,8);
        //circle(finallyShootPointMat,shootPoint,5,Scalar(0,255,0),5,8);

        vector<Point2f> final_armor4Points;
        ee.getRotatedRect4Points(final_armor,final_armor4Points);
        ee.translateRect( Point2f(final_armor.center.x,final_armor.center.y),forecastSeeFinalArmorCenter,final_armor4Points );
        circle(src_EA,forecastSeeFinalArmorCenter,5,Scalar(0,255,255),3,8,0);

        //cout<<"out getRotatedRect4Points:"<< final_armor4Points[0]<< final_armor4Points[1]<< final_armor4Points[2]<< final_armor4Points[3]<<endl;
        line(finallyShootPointMat, final_armor4Points[0], final_armor4Points[1], CV_RGB(255, 255, 0), 2, 8, 0);
        line(finallyShootPointMat, final_armor4Points[1], final_armor4Points[2], CV_RGB(255, 255, 0), 2, 8, 0);
        line(finallyShootPointMat, final_armor4Points[2], final_armor4Points[3], CV_RGB(255, 255, 0), 2, 8, 0);
        line(finallyShootPointMat, final_armor4Points[3], final_armor4Points[0], CV_RGB(255, 255, 0), 2, 8, 0);

        AngleSolver::TargetType type = AngleSolver::TARGET_ENERGY;
        //能量机关置false
        Energy_solver->getAngle(final_armor4Points, angle_x, angle_y, type, false);
        angle_y=angle_y+0;

    }
    else if(dectectMode==0)//激光完全瞄准
    {
        shootPoint.x=final_armor.center.x;
        shootPoint.y=final_armor.center.y;
        //circle(finallyShootPointMat,shootPoint,5,Scalar(255,255,255),5,8);

        vector<Point2f> final_armor4Points;
        ee.getRotatedRect4Points(final_armor,final_armor4Points);
        AngleSolver::TargetType type = AngleSolver::TARGET_ENERGY;
        //能量机关, false
        Energy_solver->getAngle(final_armor4Points, angle_x, angle_y, type, false);

        //======sizeTest
        //vector<Point2f> final_armor4PointsSizeTest;
        //debugTools debugTools1;
        //vector<double> sideBox;
        //ee.getRotatedRect4Points(final_armor,final_armor4PointsSizeTest);
        //debugTools1.calTheSide4(final_armor4PointsSizeTest,sideBox);
        //ee.getRotatedRect4Points(rotated_arms[rotatedArmsIndex].extern_rotatedrect,final_armor4PointsSizeTest);
        //debugTools1.calTheSide4(final_armor4PointsSizeTest,sideBox);
        //======sizeTest
    }
    else if(dectectMode==1)//打击静止能量机关 （有判断下坠）
    {
        shootPoint.x=final_armor.center.x;
        shootPoint.y=final_armor.center.y;

        //页码，5.4.1  桥头到能量机关水平距离 7100mm  + 350mm =7450 mm
        //能量机关页码：5.5.4 ；中心R字到地面距离：2700mm
        //桥页码：5.4.1      ;桥高：1200mm   or 1125mm+400mm=1525mm        //1125+350=1475
        //步兵摄像头高 350mm  摄像头远离300mm+50mm=350mm
        //装甲板尺寸：  长230mm 宽124mm


        //摄像头距离枪管 x距离 12cm

        //h     see      v       d      v水平     t
        //1975  14.84   17.55   2.71    26.69   0.279
        //1175  8.96    11.65   2.69    27.42   0.271
        //375   2.88    5.55    2.67    27.86   0.267

        //2025  15.20   17.91   2.71    26.64   0.279
        //1225  9.33    12.02   2.69    27.38   0.272
        //425   3.26    5.94    2.68    27.84   0.267


        vector<Point2f> final_armor4Points;
        ee.getRotatedRect4Points(final_armor,final_armor4Points);
        AngleSolver::TargetType type = AngleSolver::TARGET_ENERGY;
        Energy_solver->getAngle(final_armor4Points, angle_x, angle_y, type, false);
        angle_y = angle_y - EA_parameter.angleDrop;   //5.3//2.69//pitch// 28m/s:2.69 ; 29m/s:2.51
        ee.deleteClockWise1History();//删掉一个旋转记录
    }
    else if(dectectMode==2)//打击运动能量机关   (判断下坠+预瞄能量机关运动)
    {
        shootPoint.x=final_armor.center.x;
        shootPoint.y=final_armor.center.y;

        //圆心点只用于计算半径方向，半径长度由装甲板比例计算。
        ee.refreshStructInfo(detect_center[0],circle_center,detect_center[1],final_armor);
        //circle(finallyShootPointMat,circle_center,5,Scalar(255,0,255),3,8,0);//圆心
        circle(src_EA,circle_center,5,Scalar(255,0,255),3,8,0);//圆心    //===============================

        cout<<"circle_center="<<circle_center<<endl;       //===========================

        Point2f forecastSeeFinalArmorCenter=Point2f(-1,-1);
        ee.shootRollEnergyAgency(forecastSeeFinalArmorCenter);
        //circle(finallyShootPointMat,shootPoint,5,Scalar(0,255,0),5,8);
        //circle(src_EA,shootPoint,5,Scalar(0,255,0),5,8);
        vector<Point2f> final_armor4Points;
        ee.getRotatedRect4Points(final_armor,final_armor4Points);
        ee.translateRect( Point2f(final_armor.center.x,final_armor.center.y),forecastSeeFinalArmorCenter,final_armor4Points );
        circle(src_EA,forecastSeeFinalArmorCenter,5,Scalar(0,255,255),3,8,0); //===========================
        //cout<<"out getRotatedRect4Points:"<< final_armor4Points[0]<< final_armor4Points[1]<< final_armor4Points[2]<< final_armor4Points[3]<<endl;
//        line(finallyShootPointMat, final_armor4Points[0], final_armor4Points[1], CV_RGB(255, 255, 0), 2, 8, 0);
//        line(finallyShootPointMat, final_armor4Points[1], final_armor4Points[2], CV_RGB(255, 255, 0), 2, 8, 0);
//        line(finallyShootPointMat, final_armor4Points[2], final_armor4Points[3], CV_RGB(255, 255, 0), 2, 8, 0);
//        line(finallyShootPointMat, final_armor4Points[3], final_armor4Points[0], CV_RGB(255, 255, 0), 2, 8, 0);
        AngleSolver::TargetType type = AngleSolver::TARGET_ENERGY;
        Energy_solver->getAngle(final_armor4Points, angle_x, angle_y, type, false);
        angle_y=angle_y-EA_parameter.angleDrop;//2.69;   //pitch

    }
    else
    {
        cout<<"dectectMode ERROR!"<<endl;
    }
    //imshow("finallyShootPointMat",finallyShootPointMat);
#ifdef FINAL_DETECT
    imshow("src", src_EA);
#endif
    rotated_arms.clear();
    waitKey(1);
    if (shootPoint.x>0 && shootPoint.x<src_EA.cols && shootPoint.y>0 &&shootPoint.y<src_EA.rows)
        return 1;
    else
    {
        if(shootPoint.x>=src_EA.cols || shootPoint.y>=src_EA.rows)
            cout<<"shootPoint is out of src! "<<endl;
        return 0;
    }
}
