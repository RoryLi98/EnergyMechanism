# EnergyMechanism

<div align=center><img src="https://github.com/LinkLiar/ImageStorage/blob/master/SchoolBadge.png" width="200" height="265"/><img src="https://github.com/LinkLiar/ImageStorage/blob/master/CollegeBadge.png" width="200" height="265"/></div>

Qmake Setting：

    INCLUDEPATH += /usr/local/include \
                   /usr/local/include/opencv \
                  /usr/local/include/opencv2
                 
    LIBS += `pkg-config opencv --cflags --libs`

    SOURCES += main.cpp \
               RMVideoCapture.cpp \
               energy_agency.cpp \
               energy_agency_thread.cpp \
               function_tool.cpp \
               AngleSolver.cpp
                 
    LHEADERS += RMVideoCapture.hpp \
                energy_agency.h \
                energy_agency_thread.h \
                AngleSolver.h \
                header.h


