TEMPLATE = app
CONFIG += console c++14
QT += network
CONFIG -= app_bundle
#CONFIG -= qt
LIBS=-ldl

SOURCES += \
        Myserver.cpp \
        Mythread.cpp \
        Server.cpp \
        main.cpp

#ros
INCLUDEPATH += /opt/ros/noetic/include

LIBS += -L/opt/ros/noetic/lib -lroscpp -lpthread -lrosconsole\
         -lroscpp_serialization -lrostime  -lcpp_common \
        -lroslib -ltf  -lboost_system \
        -ltf2 -lxmlrpcpp -lrosconsole -lrosconsole_backend_interface -lrosconsole_log4cxx

#VTK
INCLUDEPATH += /usr/include/vtk-7.1

#PCL
INCLUDEPATH += /usr/include/pcl-1.10\
               /usr/include/eigen3


LIBS += /usr/lib/x86_64-linux-gnu/libpcl*

#boost
LIBS += /usr/lib/x86_64-linux-gnu/libvtk*.so

LIBS += /usr/lib/x86_64-linux-gnu/libboost*

LIBS += /usr/lib/x86_64-linux-gnu/libpthread*

#basler
INCLUDEPATH += /opt/Basler_blaze/include
LIBS += /opt/Basler_blaze/library/CPP/lib/Linux64_x64/lib*
LIBS += /opt/Basler_blaze/lib/lib*

HEADERS += \
    Head.h \
    Myserver.h \
    Mythread.h \
    Server.h
