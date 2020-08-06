#include "header.h"

using namespace  std;
using namespace  cv;

//比较中心点的横坐标
bool comp_center_x(RotatedRect a, RotatedRect b)
{
    return a.center.x < b.center.x;
}

//比较总分，总分是用来评判相似度
bool comp_total_score(Light_Group a, Light_Group b)
{
    return a.general_score.total_light_score < b.general_score.total_light_score;
}

//计算角度
float calculate_angle(Point2f s1, Point2f s2)
{
    return abs(atan2(s2.y - s1.y, s2.x - s1.x) * 180.0 / CV_PI);
}

//计算距离
float calculate_distance(Point2f a, Point2f b)
{
    return sqrt((a.x-b.x) * (a.x-b.x) + (a.y - b.y) * (a.y - b.y));
}

//比较区域面积的大小
float comp_area(Light_Group a, Light_Group b){
    return a.general_score.armor_true_area > b.general_score.armor_true_area;
}

//比较总分的最大值
bool comp_total_score_max(Light_Group a, Light_Group b)
{
    return a.general_score.total_light_score > b.general_score.total_light_score;
}

//计算点到目标的距离
double calculate_dot_to_line_distance(Point2f p0, double k, double b, int flag)
{
    if(flag == 0)
        return fabs(k * p0.x - p0.y + b)/sqrt(k * k + 1);
    else
        return fabs(k - p0.x);
}

//画矩形：框出需要处理、击打的目标
void drawBox(RotatedRect box, Mat &img)
{
    Point2f pt[4];
    int i;
    for (i = 0; i<4; i++)
    {
        pt[i].x = 0;
        pt[i].y = 0;
    }
    box.points(pt); //计算二维盒子顶点  0右下角的点，顺时针，3右上角的点
    line(img, pt[0], pt[1], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pt[1], pt[2], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pt[2], pt[3], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pt[3], pt[0], CV_RGB(0, 255, 0), 2, 8, 0);
}

//通过左上、左下、右上、右下四个点画出四边形方框
void drawBoxBy4Points(Point2f pt[4], Mat &img)
{
    line(img, pt[0], pt[1], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pt[1], pt[2], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pt[2], pt[3], CV_RGB(0, 255, 0), 2, 8, 0);
    line(img, pt[3], pt[0], CV_RGB(0, 255, 0), 2, 8, 0);
}

//比较距离
bool comp_distance(Perhaps_scores a, Perhaps_scores b)
{
    return a.distance < b.distance;
}

//比较点的距离
bool comp_dot_distance(sort_center a, sort_center b)
{
    return a.distance < b.distance;
}
