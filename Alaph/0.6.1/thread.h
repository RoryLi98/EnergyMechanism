#ifndef THREAD_H
#define THREAD_H

#include "header.h"

class Image_Thread
{
    public:
        Image_Thread(int fd2car);
//        virtual ~Image_Thread();
        void Image_Thread_read_from_cam();
        void Image_Thread_energy();
    public:
        int fd2car;
};


#endif // THREAD_H
