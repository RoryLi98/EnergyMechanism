#ifndef ENERGY_AGENCY_THREAD_H
#define ENERGY_AGENCY_THREAD_H

#include "header.h"
#include "energy_agency.h"
#include <fstream>
#include "AngleSolver.h"
#include <thread>
#include <atomic>
#include <ctime>

#include "debugTools.h" //used to debug ,will be deleted finally


#define BUFFER_SIZE 5
#define pi 3.141592653


#define EA_USECAM

class energy_agency_thread
{
public:
    energy_agency_thread();

    void EA_initCam(char *file_dev, int exp_t);
    void EA_writeBuffer(int exp_t);

    void EA_readFromBuffer(cv::Mat &operateSrc);
    void EA_initDetector();
    int EA_detector(int dectectMode, cv::Mat &src_EA);

public:
    //int fd2car;    //串口
    int detect_count=0;

    Energy_Agency ee;
    std::vector<Energy_Agency_Rotated_Arm> rotated_arms;

    double angle_x, angle_y;
    AngleSolver* Energy_solver;

    struct struct_EA_readFromCamInfo{
        #ifdef EA_USECAM
            RMVideoCapture* cap;
        #else
            cv::VideoCapture* cap;
        #endif
    };

    struct struct_EA_imageData{
        cv::Mat src;
        unsigned int frame_count;
    };

    struct struct_EA_camBufferData{
        struct_EA_imageData EA_Cam_data[BUFFER_SIZE];

        std::atomic_int EA_Cam_dataUsingFlags[BUFFER_SIZE];

        volatile unsigned int writerIdx=0;  //read from cam and write
        volatile unsigned int readerIdx=0;
    };

    struct_EA_readFromCamInfo readFromCamInfo;
    struct_EA_camBufferData   camBufferData;


//===================下面是调参数部分===================================
//===================下面是调参数部分===================================

    struct struct_parameter{

        //旋转:
        const double theta_y = (-3.6);       //yaw   越小激光点越右                     //-0.6 //-3.5
        const double theta_x = (3.5);     //pitch 越小激光越上 原本激光点偏下时调为-4   //-0.5


        // -3.4 3.5
        //用于矫正激光与弹道的偏角:
        const double angleDrop=5.7;
        //6.7

        //向电控发送撒谎信息，从而使枪管匀速沿z轴旋转，用于在大能量机关时抵消电控的PID、替代电控的卡尔曼滤波
        const double roll_prevaricate=11.5;

        //相机标定参数位置:
        const std::string calibration_Path=
                "/home/link/能量机关/Energy/EA_calibration_result/calibrationUsed/EA_03_calibration_result_720.xml";

        //以上参数，每个参数对于每台步兵都不一样，注意区分，谢谢
        //SVM 模型位置:
        const std::string svm_Model_Path=
                "/home/link/能量机关/Energy/EA_SVM_Model/modelUsed/hog_model_small.xml";

    };
    struct_parameter EA_parameter;

//====================

    //参数备忘录：
    /*
    */
    //=====数据======

    /*---------
     *1号摄像头
     *
     *
     *---------
     *2号摄像头(三号步兵)
     *  theta_y =    -3.0
     *  theta_x =    5
     *  angleDrop=   9.2
     *  roll_prevaricate+   =  8
     *
     *
     *---------
     *3号摄像头(四号步兵)
     *  --旧参数--
     *  theta_y =   -4
     *  theta_x =   -4.5
     *  angleDrop=  -2.3
     *  roll_prevaricate+   =  13
     *
     *  --新参数--
     *  theta_y =   -4
     *  theta_x =   4.3
     *  angleDrop=  6.5
     *  roll_prevaricate+   =  14
     *
     *---------
    */

    //==============

private:
    int correctTheBullet();
};

#endif // ENERGY_AGENCY_THREAD_H
