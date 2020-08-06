#ifndef HEADER_H
#define HEADER_H

#include <opencv2/opencv.hpp>
#include <iostream>
#include <opencv2/highgui.hpp>
#include <opencv2/ml/ml.hpp>
#include <opencv2/core/core.hpp>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <termios.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <dirent.h>
#include <vector>
#include "math.h"
#include <string.h>
#include <fstream>
#include "RMVideoCapture.hpp"
#include "ImgUtils.h"


struct Armor_Param
{
    //预处理二值化部分 RGB
         int blue_bright_max;
         int blue_color_max;
         int red_bright_max;
         int red_color_max;
    //预处理二值化部分 HSV
         int red_h_min1;
         int red_h_max1;
         int red_h_min2;
         int red_h_max2;
         int red_s_min;
         int red_s_max;
         int red_v_min;
         int red_v_max;

         int blue_h_min;
         int blue_h_max;
         int blue_s_min;
         int blue_s_max;
         int blue_v_min;
         int blue_v_max;
    //筛选灯柱部分
         int min_light_bright_contours_num;           //亮度二值化后的轮廓点数最小值
         double min_rect_area ;                       //限制灯条亮度面积的最小值
         double max_rect_area ;                       //限制灯条亮度面积的最大值
         double contour_angle_threshold ;             //单个灯条包围的旋转矩形最大偏角
         double min_bright_rrect_hw;                  //单个灯条亮度旋转矩形最小长宽比（rotatedRect）
         double max_bright_rrect_hw;                  //单个灯条亮度旋转矩形最大长宽比（rotatedRect）
    //匹配装甲板部分
         double center_angle_threshold;               //灯柱中心连线与水平夹角最大阈值
         double diff_angle_threshold;                 //灯柱间角度差最大阈值
         double diff_height_ratio_threshold ;         //匹配灯条的最长边的比例，用来判断是否过偏
         double min_height_and_x_threshold;           //最小的高与装甲板两灯条x的比值阈值
         double max_height_and_x_threshold;           //最大的高与装甲板两灯条x的比例阈值
         double big_armor_min_height_and_x_threshold;
         double big_armor_max_height_and_x_threshold;
         int detect_mode;                             //1:蓝色，2：红色
};
//该结构体表示装甲板每一项的打分
struct General_Score
{
    float diff_height_radio;  //反映正对程度，表示最长灯条与最短灯条的差与最长灯条的比值，归一化
    float armor_area;         //整个装甲板面积，归一化
    float armor_true_area;   //原来的装甲板面积，后面用面积限制识别距离有作用
    float armor_angle;        //装甲板与水平的夹角,2019.3.9，归一化
    float diff_angle;
    float total_light_score;  //装甲板整体打分, 该分数确定最终应该打击的装甲板，比例1:1
};

struct Light_Group
{
    std::vector<cv::RotatedRect> group_rects;
    std::vector<std::vector<cv::Point2f>> group_rects_points;
    bool is_big_armor;
    float diff_angle;
    General_Score general_score;
    int num_type = 0; //数字类型，0代表没有识别到数字，其余表示识别到的数字
    bool is_equal_lastnum = false;
    float distance_with_last;//与上一帧最佳匹配装甲板的距离
    cv::Point2f group_center = cv::Point2f(-1,-1); //当前两灯条的中心
};

struct Perhaps_scores
{
    double distance;                        //与该旋转臂最外围轮廓中心点的距离
    std::vector<cv::Point2f> sort_vertices;
    cv::RotatedRect target;                 //此旋转矩形（是一个内部轮廓的旋转矩形）
};

//针对单个能量机关旋转臂的结构体
struct Energy_Agency_Rotated_Arm
{
    cv::RotatedRect extern_rotatedrect;//能量机关单个旋转臂最外围旋转矩形
    std::vector<cv::RotatedRect> inside_rotatedrects; //能量机关内单个旋转臂内部旋转矩形集合
    std::vector<cv::Point2f> perhaps_vertices; //单个旋转臂需送入svm分类的四个点坐标
    cv::RotatedRect perhaps_target; //单个旋转臂可能需击打的目标
};

struct sort_center
{
    cv::Point2f center;
    double distance;
};


bool comp_center_x(cv::RotatedRect a, cv::RotatedRect b);
bool comp_total_score(Light_Group a, Light_Group b);
bool comp_total_score_max(Light_Group a, Light_Group b);
float calculate_angle(cv::Point2f s1, cv::Point2f s2);
float calculate_distance(cv::Point2f a, cv::Point2f b);
float comp_area(Light_Group a, Light_Group b);

//by wzx
double calculate_dot_to_line_distance(cv::Point2f p0, double k, double b, int flag);
void drawBox(cv::RotatedRect box, cv::Mat &img);
void drawBoxBy4Points(cv::Point2f pt[4], cv::Mat &img);
bool comp_distance(Perhaps_scores a, Perhaps_scores b);
bool comp_dot_distance(sort_center a, sort_center b);

#endif // HEADER_H
