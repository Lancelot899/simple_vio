TEMPLATE = app
CONFIG += console

QMAKE_CXXFLAGS += -std=c++11

INCLUDEPATH += /usr/include/eigen3
INCLUDEPATH += /usr/include/python2.7

SOURCES += main.cpp \
    IMU/IMU.cpp \
    IMU/test/Test_IMU.cpp \
    IO/IMUIO.cpp \
    IMU/Implement/IMUImpl.cpp \
    IMU/Implement/IMUImplOKVIS.cpp \
    util/util.cpp

HEADERS += \
    IMU/IMU.h \
    IO/IOBase.h \
    IO/IMUIO.h \
    DataStructure/Measurements.h \
    DataStructure/IMUMeasure.h \
    IMU/Implement/IMUImpl.h \
    IMU/Implement/IMUImplOKVIS.h \
    util/util.h

