#ifndef ENERGY_H
#define ENERGY_H
#include "header.h"
class Energy_Agency
{
public:
    Energy_Agency();
    void EnergyProcess(Mat &src, Mat &mask_bright, Mat &mask_color, int ColorNumber, int debug);
    vector<Energy_Agency_Rotated_Arm> Energy_Agency_choose_perhaps_targets(vector<Point2f> &circle_center, Mat &mask_bright);
    int Energy_Agency_target_filters(vector<Point2f> &circle_center, Mat &mask_bright, vector<Energy_Agency_Rotated_Arm> &rotated_arms);
    EnergyResult Energy_Agency_choose_final_target(vector<Point2f> &circle_center, Mat &src, Mat &mask_bright, vector<Energy_Agency_Rotated_Arm> &rotated_arms, cv::RotatedRect &final_armor, int &rotatedArmsIndex);
    HOGDescriptor *train_hog;
    cv::Ptr<cv::ml::SVM> predict_model;
private:
};

class timecap
{
public:
    float second = 0;
    float coefficient = 0;
private:
};


#endif // ENERGY_H
