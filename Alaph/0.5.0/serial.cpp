/*******************************************************************************************************************
Copyright 2017 Dajiang Innovations Technology Co., Ltd (DJI)

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated
documentation files(the "Software"), to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense, and / or sell copies of the Software, and
to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of
the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO
THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF
CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
IN THE SOFTWARE.
*******************************************************************************************************************/

#include "serial.h"
#include <math.h>
#include <stdio.h>      // standard input / output functions
#include <string.h>     // string function definitions
#include <unistd.h>     // UNIX standard function definitions
#include <fcntl.h>      // File control definitions
#include <errno.h>      // Error number definitions
#include <termios.h>    // POSIX terminal control definitionss
#include <iostream>
#include <termios.h>
#include <sys/ioctl.h>
using namespace std;
union data_send_float
{
    float data_float;
    char data_uint8[4];
};

int openPort(const char * dev_name){
    int fd; // file description for the serial port
    cout<<dev_name<<endl;
    fd = open(dev_name, O_RDWR | O_NOCTTY | O_NDELAY);
    if(fd == -1){ // if open is unsucessful
        printf("open_port: Unable to open\n");
    }
    else
    {
        fcntl(fd, F_SETFL, FNDELAY);//设置非堵塞，原来是设置堵塞，电控有时收不到数据是因为缓冲区爆炸，但是又清不了，最后一个参数设置为0代表堵塞
        tcflush(fd, TCIOFLUSH); //清空发送接收缓冲区数据
        printf("port is open.\n");
    }
    return(fd);
}

int configurePort(int fd){                      // 配置端口configure the port
    struct termios port_settings;               // structure to store the port settings in
    cfsetispeed(&port_settings, B115200);       // set baud rates
    cfsetospeed(&port_settings, B115200);

    port_settings.c_cflag &= ~PARENB;           // set no parity, stop bits, data bits （x&=~x 取消x功能）
    port_settings.c_cflag &= ~CSTOPB;           //每个字符使用一停止位
    port_settings.c_cflag &= ~CSIZE;            //数据位屏蔽   c_cflag成员控制着波特率、数据位、奇偶校验、停止位以及流控制
    port_settings.c_cflag |= CS8;               //8位数据

    //修改控制模式，保证程序不会占用串口
    port_settings.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    port_settings.c_cflag |= CREAD;
    //修改输出模式，原始数据输出
    port_settings.c_oflag &= ~OPOST;

    port_settings.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

    tcsetattr(fd, TCSANOW, &port_settings);     // apply the settings to the port tcsetattr函数用于设置终端参数，TCSANOW：不等数据传输完毕就立即改变属性
    return(fd);
}


bool sendXYZ(int fd,union data_send_float *data_send,char ShootTime, int cur_mode,char cnt)
{

     unsigned char send_bytes[11] = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

     if(cur_mode == 1)
         send_bytes[10] = 0xcc;
     else if(cur_mode == 2)
         send_bytes[10] = 0xdc;
     else
         send_bytes[10] = 0xec;

     send_bytes[0]=(char)data_send[0].data_uint8[0];
     send_bytes[1]=(char)data_send[0].data_uint8[1];
     send_bytes[2]=(char)data_send[0].data_uint8[2];
     send_bytes[3]=(char)data_send[0].data_uint8[3];

     send_bytes[4]=(char)data_send[1].data_uint8[0];
     send_bytes[5]=(char)data_send[1].data_uint8[1];
     send_bytes[6]=(char)data_send[1].data_uint8[2];
     send_bytes[7]=(char)data_send[1].data_uint8[3];
     send_bytes[8]=ShootTime;
     send_bytes[9]=cnt;

     if (11 == write(fd, send_bytes, 11))      //Send data，发不出去，缓冲区爆了，所以清空下缓冲区
         return true;
     else
     {
         tcflush(fd, TCOFLUSH);              //write不出那么多个字节的话就清空缓冲区
         return false;
     }

}

//bool sendOver(int fd,int *cmd){
//     unsigned char send_bytes[4]{0xfe,0x00,0x00,0xff};//
//    //cout<<send_bytes<<endl;
//     if(NULL == cmd){
//         /*if (4 == write(fd, send_bytes, 4))  //Send data
//             return true;*/
//         return false;
//     }
//     send_bytes[1]=(short)cmd[0];
//     send_bytes[2]=(short)cmd[1];
//     tcflush(fd, TCOFLUSH);
//     if (4 == write(fd, send_bytes, 4))      //Send data
//         return true;
//     else
//     {
//         tcflush(fd, TCOFLUSH); //write不出那么多个字节的话就清空缓冲区
//         return false;
//     }
//}

//bool sendData(int fd, char * data, int size, DATATYPE data_type){
//    unsigned char header[5];
//    int * _size = (int *)header;
//    _size[0] = size;
//    unsigned char * _type = header+4;
//    _type[0] = data_type;
//    if (5 != write(fd, header, 5)){
//        printf("send Data error\n");
//        return false;
//    }
//    if (size == write(fd, data, size)){
//        printf("send Data success\n");
//        return true;
//    }
//    return false;
//}



