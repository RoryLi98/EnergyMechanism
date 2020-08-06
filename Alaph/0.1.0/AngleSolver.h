#ifndef ANGLESOLVER_H
#define ANGLESOLVER_H

#include "header.h"

class AngleSolver
{
public:
    typedef enum {TARGET_ARMOR, TARGET_SAMLL_ARMOR, TARGET_ENERGY} TargetType;
    AngleSolver(cv::Mat cam_matrix, cv::Mat distortion_coeff, double min_distance,double max_distance);
    void solvePnP4Points(const std::vector<cv::Point2f> & points2d, cv::Mat & rot, cv::Mat & trans);
    void getTarget2dPoinstion(std::vector<cv::Point2f> points, std::vector<cv::Point2f> & target2d);
    void setTargetSize(TargetType type);
    void setRelationPoseCameraPTZ(const cv::Mat & rot_camera_y_ptz,
                                  const cv::Mat &rot_camera_x_ptz,
                                  const cv::Mat & trans_camera_ptz,
                                  double y_offset_barrel_ptz);
    void adjustRect2FixedRatio(cv::RotatedRect & rect, double wh_ratio);
    bool getAngle(std::vector<cv::Point2f> & points,
                  double & angle_x,
                  double & angle_y,
                  TargetType type,
                  bool isAttack_armor);
    void tranformationCamera2PTZ(const cv::Mat & pos, cv::Mat & transed_pos, bool is_attack_armor);
    void adjustPTZ2Barrel(const cv::Mat & pos_in_ptz, double & angle_x, double & angle_y);

    void setRelationPoseCameraPTZForEnergy(const cv::Mat & rot_camera_ptz,
                                           const cv::Mat & trans_camera_ptz,
                                           double y_offset_barrel_ptz);
public:
    cv::Mat _cam_matrix;
    cv::Mat _distortion_coeff;
    //单位均为cm
    double _width_target;       //目标值宽
    double _height_target;      //目标值高
    double _min_distance;       //检测的最小距离
    double _max_distance;       //检测是最大距离
    double _small_armor_height; //小装甲板的高
    double _small_armor_weight; //小装甲板的宽
    double _big_armor_height;   //大装甲板的高
    double _big_armor_weight;   //大装甲板的宽

    double _energy_armor_height; //能量机关装甲板的高
    double _energy_armor_weight; //能量机关装甲板的宽

    cv::Mat _position_in_camera;//目标在相机坐标系下的坐标
    cv::Mat _position_in_ptz;   //目标在云台坐标系下的坐标
    cv::Mat _trans_camera2ptz;  //相机坐标系到云台坐标系的平移向量
    cv::Mat _rot_y_camera2ptz;    //相机坐标系到云台坐标系的旋转矩阵
    cv::Mat _rot_x_camera2ptz;

    //能量机关
    cv::Mat _rot_camera2ptz;
    double _offset_y_barrel_ptz;
    bool calculate_result;
};

#endif // ANGLESOLVER_H
