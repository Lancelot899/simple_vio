TEMPLATE = app
CONFIG += console

QMAKE_CXXFLAGS += -std=c++11

INCLUDEPATH += /usr/local/include/eigen3

SOURCES += main.cpp \
    IMU/IMU.cpp \
    IMU/test/Test_IMU.cpp \
    IO/IMUIO.cpp

HEADERS += \
    IMU/IMU.h \
    IO/IOBase.h \
    IO/IMUIO.h

