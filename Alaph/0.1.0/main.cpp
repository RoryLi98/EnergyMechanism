#include "energy_agency.h"
#include "energy_agency_thread.h"
#include <thread>
#include <QString>
#include <QProcess>
#include <fstream>
#include <QCoreApplication>
#include <cstdlib>
#include <time.h>
#include "header.h"

#define RELEASE

using namespace std;
using namespace cv;

int main()
{
    VideoCapture cap;
    Mat src,dst,roi;
    cap.open("/home/link/能量机关/test.mp4");

    while(1)
    {

        char c = (char)waitKey(1);
        if(c == 27 )break;
        if(c == 32 )
        {
            while(true)
            {
                char c = (char)waitKey(1);
                if(c == 99 )break;
            }
        }

        cap>>src;
        if(src.rows==0)
        {
            destroyAllWindows();
            break;
        }

    double send_data[4] = { 0 };//定义发送数据

    energy_agency_thread energyAgencyThread_object;
    energyAgencyThread_object.EA_initDetector();

    int ret = energyAgencyThread_object.EA_detector(2,src);
    double angle_x, angle_y;
    if(ret == 1)
   {
        energyAgencyThread_object.detect_count++;
        energyAgencyThread_object.detect_count %= 256;
        cout<<"angle_x: "<<angle_x<<" ;angle_y :" <<angle_y<<endl;
        //应用PnP算法角度结算后得到的角度，×100是为了保留小数。把数据发给电控之后，他们除以100就可以了
        send_data[0] = energyAgencyThread_object.angle_y * 100; //pitch
        send_data[1] = energyAgencyThread_object.angle_x * 100; //yaw
        send_data[2] = 0;            //solver->_position_in_camera.at<double>(2, 0) * 100;
        send_data[3] = energyAgencyThread_object.detect_count % 256;
        cout<<"send_data[0]: "<<send_data[0]<<" ;send_data[1] :" <<send_data[1]<<endl;

    }
    else if(ret == 0)
    {
        send_data[0] = 0.0; //pitch
        send_data[1] = 0.0; //yaw
        send_data[2] = 0.0;
        send_data[3] = 0.0;
        cout<<"send_data[0]: "<<send_data[0]<<" ;send_data[1] :" <<send_data[1]<<endl;
    }
    imshow("RESULT", src);
//    waitKey(0);
   }

    return 0;
}
