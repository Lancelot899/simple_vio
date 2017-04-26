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


FIND_LIBRARY(GFLAGS_LIB
        NAMES flags libgflags.so
        PATH
        /usr/lib/x86_64-linux-gnu
        /usr/lib
        /usr/local/lib
        )

FIND_LIBRARY(GOMP_LIB
        NAMES gomp libgomp.so.1
        PATH
        /usr/lib/x86_64-linux-gnu
        /usr/lib
        /usr/local/lib
        )

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
FIND_PACKAGE(QGLViewer REQUIRED QUIET)
FIND_PACKAGE(OpenCV REQUIRED QUIET)
FIND_PACKAGE(Boost REQUIRED COMPONENTS thread system regex)
FIND_PACKAGE(BLAS REQUIRED QUIET)
FIND_PACKAGE(LAPACK REQUIRED QUIET)
FIND_PACKAGE(SuiteSparse REQUIRED)
FIND_PACKAGE(Cholmod REQUIRED QUIET)

INCLUDE_DIRECTORIES(
        .
        ./ThirdParty
        ${OpenCV_INCLUDE_DIRS}
        ${Sophus_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${GLOG_INCLUDE_DIRS}
        ${EIGEN_INCLUDE_DIRS}
        ${CERES_INCLUDE_DIRS}
        ${QGLVIEWER_INCLUDE_DIR}
)

if(Qt4_FOUND)
    INCLUDE(${QT_USE_FILE})
    QT4_WRAP_CPP(UI_SOURCES)

    INCLUDE_DIRECTORIES(${QT_INCLUDES})
    SET(MY_QT_LIBRARIES ${QT_LIBRARIES})
endif()

if(Qt5_FOUND)
    QT5_WRAP_CPP(UI_SOURCES)

    INCLUDE_DIRECTORIES(${Qt5Core_INCLUDE_DIRS} ${Qt5Xml_INCLUDE_DIRS} ${Qt5Gui_INCLUDE_DIRS} ${Qt5Widgets_INCLUDE_DIRS} ${Qt5OpenGL_INCLUDE_DIRS})
    SET(MY_QT_LIBRARIES ${Qt5Widgets_LIBRARIES} ${Qt5Core_LIBRARIES} ${Qt5Gui_LIBRARIES} ${Qt5Xml_LIBRARIES} ${Qt5OpenGL_LIBRARIES})
endif()

FIND_LIBRARY(GFLAGS_LIB
        NAMES flags libgflags.so
        PATH
        /usr/lib/x86_64-linux-gnu
        /usr/lib
        /usr/local/lib
        )

FIND_LIBRARY(GL_LIB
        NAMES GL gl
        PATH
        /usr/lib/x86_64-linux-gnu
        /usr/lib
        /usr/local/lib
        )

FIND_LIBRARY(
        GLU_LIB
        NAMES glu GLu GlU GLU
        PATH
        /usr/lib/x86_64-linux-gnu
        /opt/local/lib
        /usr/lib
        /usr/local/lib
)



LIST(APPEND LINK_LIBS
        ${OpenCV_LIBS}
        ${Boost_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${CERES_LIBRARIES}
        ${GFLAGS_LIB}
        ${GOMP_LIB}
        ${SUITESPARSE_LIBRARIES}
        ${CHOLMOD_LIBRARIES}
        ${LAPACK_LIBRARIES}
        ${BLAS_LIBRARIES}
        ${MY_QT_LIBRARIES}
        ${FREEGLUT_LIBRARY}
        ${GLU_LIB}
        ${GL_LIB}
        ${QGLVIEWER_LIBRARY_RELEASE}
        ${GLEW_LIBRARY}
        ${GLU_LIB}
        ${GL_LIB}
        ${OPENGL_LIBRARY}
        )


set(SOURCE_FILES
        cv/FeatureDetector/AbstractDetector.cpp
        cv/FeatureDetector/AbstractDetector.h
        DataStructure/cv/Camera/AbstractCamera.h
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
        IO/image/ImageIO.h
        IO/image/ImageIO.cpp
        IO/image/test/Test_ImageIO.cpp
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
        main.cpp
        cv/FeatureDetector/test/Test_fast.cpp
        cv/FeatureDetector/test/Test_edge.cpp
        cv/FeatureDetector/FastDetector.cpp
        cv/FeatureDetector/FastDetector.h
        cv/FeatureDetector/EdgeDetector.cpp
        cv/FeatureDetector/EdgeDetector.h
        DataStructure/cv/Camera/PinholeCamera.cpp
        DataStructure/cv/Camera/PinholeCamera.h
        DataStructure/cv/Camera/VIOPinholeCamera.cpp
        DataStructure/cv/Camera/VIOPinholeCamera.h
        vio/Initialize.cpp
        vio/Initialize.h
        vio/Implement/InitialImpl.cpp
        vio/Implement/InitialImpl.h
        cv/FeatureDetector/Detector.cpp
        cv/FeatureDetector/Detector.h
        cv/Tracker/Tracker.cpp
        cv/Tracker/Tracker.h
        cv/Tracker/test/Test_Tracker.cpp
        util/ThreadReduce.cpp
        util/ThreadReduce.h
        cv/FeatureDetector/test/Test_Detector.cpp
        cv/Triangulater/Triangulater.h
        cv/Triangulater/Triangulater.cpp
        cv/Triangulater/test/Test_Triangulater.cpp
        DataStructure/cv/test/Test_cvFrame.cpp
        vio/test/Test_initial.cpp
        vio/test/Test_initial.h
        vio/system.h
        vio/system.cpp
        vio/BA/BundleAdjustemt.h
        vio/BA/BundleAdjustemt.cpp
        vio/BA/Implement/BABase.h
        vio/BA/Implement/BABase.cpp
        vio/BA/Implement/SimpleBA.h
        vio/BA/Implement/SimpleBA.cpp
        vio/BA/Implement/test/Test_SimpleBA.h
        vio/BA/Implement/test/Test_SimpleBA.cpp
        )


add_executable(simple_vio ${SOURCE_FILES})
TARGET_LINK_LIBRARIES(simple_vio ${LINK_LIBS})
