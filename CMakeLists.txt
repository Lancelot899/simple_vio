cmake_minimum_required(VERSION 3.5)
project(simple_vio)


IF(CMAKE_COMPILER_IS_GNUCC)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()

include_directories("/usr/local/eigen3")

FIND_PACKAGE(OpenCV REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)
FIND_PACKAGE(fast REQUIRED)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system regex)
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

INCLUDE_DIRECTORIES(
        include
        ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${fast_INCLUDE_DIRS}
)

LIST(APPEND LINK_LIBS
        ${OpenCV_LIBS}
        ${Sophus_LIBRARIES}
        ${Boost_LIBRARIES}
        ${fast_LIBRARIES}
        )

set(SOURCE_FILES
        cv/Feature_Detector.cpp
        cv/Feature_Detector.h
        DataStructure/cv/Camera.cpp
        DataStructure/cv/Camera.h
        DataStructure/cv/cvFrame.cpp
        DataStructure/cv/cvFrame.h
        DataStructure/cv/Feature.h
        DataStructure/cv/Point.cpp
        DataStructure/cv/Point.h
        DataStructure/imu/imuFactor.cpp
        DataStructure/imu/imuFactor.h
        DataStructure/imu/imumeasure.cpp
        DataStructure/imu/IMUMeasure.h
        DataStructure/Measurements.h
        DataStructure/viFrame.cpp
        DataStructure/viFrame.h
        IMU/Implement/IMUImpl.cpp
        IMU/Implement/IMUImpl.h
        IMU/Implement/IMUImplOKVIS.cpp
        IMU/Implement/IMUImplOKVIS.h
        IMU/Implement/IMUImplPRE.cpp
        IMU/Implement/IMUImplPRE.h
        IMU/test/Test_IMUPRE.cpp
        IMU/IMU.cpp
        IMU/IMU.h
        IO/imu/test/test_IMUIO.cpp
        IO/imu/IMUIO.cpp
        IO/imu/IMUIO.h
        IO/IOBase.h
        ThirdParty/okvis_time/include/implementation/Duration.hpp
        ThirdParty/okvis_time/include/implementation/Time.hpp
        ThirdParty/okvis_time/include/Duration.hpp
        ThirdParty/okvis_time/include/Time.hpp
        ThirdParty/okvis_time/src/Duration.cpp
        ThirdParty/okvis_time/src/Time.cpp
        ThirdParty/sophus/ensure.hpp
        ThirdParty/sophus/rxso3.hpp
        ThirdParty/sophus/se2.hpp
        ThirdParty/sophus/se3.hpp
        ThirdParty/sophus/sim3.hpp
        ThirdParty/sophus/so2.hpp
        ThirdParty/sophus/so3.hpp
        ThirdParty/sophus/sophus.hpp
        util/setting.cpp
        util/setting.h
        util/util.cpp
        util/util.h
        main.cpp)



add_executable(simple_vio ${SOURCE_FILES})

TARGET_LINK_LIBRARIES(simple_vio ${LINK_LIBS})
target_link_libraries(simple_vio ${Sophus_LIBRARIES})
target_link_libraries(simple_vio gtest)
target_link_libraries(simple_vio ceres.a)

