#include <iostream>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/imgproc/imgproc.hpp"

#include <stdio.h>
#include <vector>
#include "header.h"

using namespace std;
using namespace cv;

int main()
{
    VideoCapture cap;
    Mat src,mask_bright,mask_color;
    cap.open("/home/link/能量机关/test.mp4");


    while(1)
    {
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
        cv::Point cc = FindCircleCenter(detect_center,79);
        circle(src, cc, 3, Scalar(0, 255, 255),-1);
//        cout<<detect_center.RotatedArmCenter<<endl;
//        cout<<detect_center.TargetRotatedRect.center<<endl;
        char c = (char)waitKey(1);
        if(c == 27 )break;

        if(c == 115 )
        {
            while(true)
            {
                char c = (char)waitKey(1);
                if(c == 99 )break;
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
