#ifndef ENERGY_AGENCY_H
#define ENERGY_AGENCY_H
#include "header.h"

class Energy_Agency
{
public:
    Energy_Agency();
    void Energy_Agency_loadSVM_Model(std::string svm_Path);
    void set_roll_prevaricate(int roll_prevaricate);

    void create_debug_image();
    void Energy_Agency_process(cv::Mat &src, cv::Mat &mask_bright, cv::Mat& mask_color);
    std::vector<Energy_Agency_Rotated_Arm> Energy_Agency_choose_perhaps_targets(std::vector<cv::Point2f> &circle_center, cv::Mat &mask_bright);
    int Energy_Agency_target_filters(std::vector<cv::Point2f> &circle_center, cv::Mat &mask_bright, std::vector<Energy_Agency_Rotated_Arm> &rotated_arms);
    std::vector<cv::Point2f> Energy_Agency_choose_final_target(std::vector<cv::Point2f> &circle_center, cv::Mat &src, cv::Mat &mask_bright, std::vector<Energy_Agency_Rotated_Arm> &rotated_arms, cv::RotatedRect & final_armor, int &rotatedArmsIndex);
    cv::Point2f calculate_probably_circle_center(std::vector<cv::Point2f> armor_center, std::vector<cv::Point2f> match_centers,cv::RotatedRect final_armor);
    cv::HOGDescriptor *train_hog;
    void makeVideo_PushFrame(const cv::Mat src);

    int judgeTheClockWise();
    void deleteClockWise1History();
    void refreshStructInfo(cv::Point2f targetCenterImg_notD, cv:: Point2f& circleCenter_notD, cv:: Point2f rotatedArmCenter, cv::RotatedRect final_armor);
    void raiseHighShoot(cv::Point2f targetPoint_notD, cv::Point2f &shootPoint);
    void shootRollEnergyAgency(cv::Point2f &forecastSeePoint);

    void shootRollEnergyAgencyDebug(cv::Point2f &forecastSeePoint,double angle);

    void translateRect(cv::Point2f beforeCenter,cv::Point2f afterCenter,std::vector<cv::Point2f> &IORect4Points );
    void getRotatedRect4Points(cv::RotatedRect InputRect,std::vector<cv::Point2f>& get4Points);

private:

    int energy_agency_color_thresh;
    int energy_agency_bright_thresh;
    cv::Ptr<cv::ml::SVM> predict_model;

    //距离参数
    struct struct_Energy3dReal
    {
        const double PI=3.1415926535;       //PI
        const double gravity=9790;          //  mm/s2

        const double distance=7450;         //7100mm  7100+350=7450
        const double bulletSpeed=27429;     //
        const double shoott=0.272;          // shootTime= distance/bulletSpeed
        const double distanceToR=7542;      //mm
        const double energyAgencyRollSpeed=60;          // 60度/s
        double energyAgencyRollDegree=16.32+0;  // 27m/s:28.92; 28m/s:16.26 ; 29m/s:15.72 ;

        //energyAgencyRollDegree:
        //16.32不要改
        //+=
        //原始:   16.32+7
        //1号摄像头 energyAgencyRollDegree=16.32+
        //2号摄像头 energyAgencyRollDegree=16.32+8
        //3号摄像头 energyAgencyRollDegree=16.32+13

    };

    struct struct_Energy2dImgD//笛卡尔坐标系
    {
        const double focalx=942.349167;
        const double focaly=942.57345455164159;

        cv::Point2f targetCenterD;
        cv::Point2f circleCenterD;

        cv::Point2f targetInRSystem;            //目标在以R圆心 或者打击臂坐标系下的坐标 主要用于计算角度
        //double targetInRSystemAtan;

    };

    struct struct_Energy2dReal
    {
        double targetInRSystemAtan;

      //  const double radiusInReal_PIX=180;    //比赛时要改！===========================================
        const double radiusInReal_PIX=80;    //比赛时要改！
        //lab193.5 , 比赛197.7~198

    };

    struct struct_EnergyHistory
    {
        std::vector<int> judgeClockWiseHistory;
        std::vector<double> previousTargetInRSystemAtan;
        std::vector<cv::Point2f> previousTargetInRSystem;
    };

    struct struct_makeTestVideoInfo
    {
        const bool makeVideo=false;
        const std::string saveVideoPath="/home/link/energyvideo";

        cv::VideoWriter energyAgencyVideoWriter;
        bool energyAgencyVideoWriterCreatedAlready=false;
    };

    struct_Energy3dReal     energy3dReal;
    struct_Energy2dImgD     energy2dImgD;//笛卡尔坐标系
    struct_Energy2dReal     energy2dReal;
    struct_EnergyHistory    energyHistory;

    struct_makeTestVideoInfo makeTestVideoInfo;

};

#endif // ENERGY_AGENCY_H
