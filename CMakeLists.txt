cmake_minimum_required(VERSION 2.5)
project(simple_vio)


IF(CMAKE_COMPILER_IS_GNUCC)
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -std=c++0x")
ELSE()
    SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
ENDIF()


INCLUDE(ExternalProject)
LIST(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

#glog
EXECUTE_PROCESS(COMMAND sudo apt-get install libgoogle-glog-dev)
FIND_PACKAGE(Glog REQUIRED QUIET)
INCLUDE_DIRECTORIES(BEFORE ${GLOG_INCLUDE_DIRS})

#suitesparse
EXECUTE_PROCESS(COMMAND sudo apt-get install libsuitesparse-dev)
FIND_PACKAGE(SuiteSparse)

#eigen3
EXECUTE_PROCESS(COMMAND sudo apt-get install libeigen3-dev)
FIND_PACKAGE(Eigen)

#g2o
FIND_PACKAGE(G2O REQUIRED QUIET)

#ceres
FIND_PACKAGE(Ceres REQUIRED QUIET)
#if(NOT CERES_FOUND)
#    ExternalProject_Add(ceres-solver
#            GIT_REPOSITORY https://github.com/ceres-solver/ceres-solver.git
#            UPDATE_COMMAND ""
#            PATCH_COMMAND mkdir ${CMAKE_CURRENT_BINARY_DIR}/ceres-solver/build
#            SOURCE_DIR  ${CMAKE_CURRENT_BINARY_DIR}/ceres-solver
#            BINARY_DIR  ${CMAKE_CURRENT_BINARY_DIR}/ceres-solver/build
#            CMAKE_ARGS
#                -DBUILD_EXAMPLES:BOOL=OFF
#                -DBUILD_TESTING:BOOL=OFF
#            BUILD_COMMAND make -j4
#            INSTALL_COMMAND ""
#            )
#    SET(CERES_INCLUDE_DIRS ${CMAKE_CURRENT_BINARY_DIR}/ceres-solver/build/include)
#    SET(CERES_LIBRARIES ${CMAKE_CURRENT_BINARY_DIR}/ceres-solver/build/lib/libceres.a)
#endif()

FIND_PACKAGE(OpenCV REQUIRED QUIET)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system regex)


INCLUDE_DIRECTORIES(
        .
        ./ThirdParty
        ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
)

LIST(APPEND LINK_LIBS
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${CERES_LIBRARIES}
        )

LIST(APPEND LINK_LIBS
        ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_SOLVER_CHOLMOD} ${G2O_SOLVER_CSPARSE} ${G2O_SOLVER_DENSE}
        ${G2O_SOLVER_PCG} ${G2O_TYPES_SBA} ${G2O_TYPES_SLAM3D} cholmod cxsparse
        )

set(SOURCE_FILES
        cv/AbstractDetector.cpp
        cv/AbstractDetector.h
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
        IO/imu/test/Test_IMUIO.cpp
        IO/imu/IMUIO.cpp
        IO/imu/IMUIO.h
        IO/camera/test/Test_CameraIO.cpp
        IO/camera/CameraIO.cpp
        IO/camera/CameraIO.h
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
        main.cpp cv/test/Test_fast.cpp DataStructure/GraphFactor/imuG2OType.h DataStructure/GraphFactor/imuG2OType.cpp cv/FastDetector.cpp cv/FastDetector.h cv/EdgeDetector.cpp cv/EdgeDetector.h)


add_executable(simple_vio ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(simple_vio ${LINK_LIBS})
