# EnergyMechanism

<div align=center><img src="https://github.com/LinkLiar/ImageStorage/blob/master/SchoolBadge.png" width="200" height="265"/><img src="https://github.com/LinkLiar/ImageStorage/blob/master/CollegeBadge.png" width="200" height="265"/></div>

Qmake Setting：

    INCLUDEPATH += /usr/local/include \
                   /usr/local/include/opencv \
                   /usr/local/include/opencv2 \
                   /home/link/linuxSDK_V2.1.0.20/include

    LIBS += `pkg-config opencv --cflags --libs`
    LIBS += /home/link/linuxSDK_V2.1.0.20/lib/x64/libMVSDK.so

    SOURCES += main.cpp \
               serial.cpp \
               thread.cpp \
               energy.cpp
               
模拟校准程序：/EnergyMechanism/EnergySimulation/  
赛前就绪版本：/EnergyMechanism/tree/master/Alaph/0.7.1(FinalVersion)  
旧版大能量瞄准效果：https://github.com/LinkLiar/ImageStorage/blob/master/BigEnergyAiming.MP4  
新版大能量瞄准效果：https://github.com/LinkLiar/ImageStorage/blob/master/Competition.mp4  
小能量模拟效果：https://github.com/LinkLiar/ImageStorage/blob/master/SmallEnergy.gif  
大能量模拟效果：https://github.com/LinkLiar/ImageStorage/blob/master/BigEnergy.gif  
