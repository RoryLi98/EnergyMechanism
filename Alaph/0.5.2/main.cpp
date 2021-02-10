#include "CameraApi.h" //相机SDK的API头文件
#include "thread.h"
using namespace std;
using namespace cv;
int main()
{

    int fd2car=openPort("/dev/RMCWB");  //开串口

    cout<<"fd2car:"<<fd2car<<endl;

    if(fd2car!=-1)
    {
        configurePort(fd2car);
    }

    Image_Thread image_task(fd2car);
    thread t1(&Image_Thread::Image_Thread_read_from_cam, &image_task);
    thread t2(&Image_Thread::Image_Thread_energy, &image_task);

    t1.join();
    t2.join();


    if(fd2car != -1)
       close(fd2car);
    return 0;
}
