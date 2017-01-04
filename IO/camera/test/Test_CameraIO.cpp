#include <opencv2/ts.hpp>

#include "../CameraIO.h"

TEST(TestCamIO, CAMERA_IO) {
    std::string camDatafile = "../testData/mav0/cam1/data.csv";
    std::string camParamfile ="../testData/mav0/cam1/sensor.yaml";
    CameraIO camTest(camDatafile,camParamfile);

#if 0
    printf("Finished class Construct,then begin test TBS!\n");
    /// T_BS & Rate_hz :
    Eigen::Matrix<double, 4, 4>     se3d;
    se3d << 0.0125552670891, -0.999755099723, 0.0182237714554, -0.0198435579556,
            0.999598781151, 0.0130119051815, 0.0251588363115, 0.0453689425024,
            -0.0253898008918, 0.0179005838253, 0.999517347078, 0.00786212447038,
            0.0, 0.0, 0.0, 1.0;


//    GTEST_ASSERT_EQ(camTest.camParam->getTBS().matrix(),se3d);


    std::cout<<"diff = \n"<<camTest.getCamera()->getTBS().matrix()-se3d<<"\n";
    GTEST_ASSERT_EQ(camTest.camParam->getRate(),20);
    /// Mode:
    std::string cameraMode = "pinhole", distortion_model = "radial-tangential";
    GTEST_ASSERT_EQ(camTest.camParam->getCameraMode(),cameraMode);
    GTEST_ASSERT_EQ(camTest.camParam->getDistorMode(),distortion_model);
    /// Resolution:
    GTEST_ASSERT_EQ(camTest.camParam->width(),752); GTEST_ASSERT_EQ(camTest.camParam->height(),480);
    /// Intrinsics
    GTEST_ASSERT_EQ(camTest.camParam->fx(),457.587); GTEST_ASSERT_EQ(camTest.camParam->fy(),456.134);
    GTEST_ASSERT_EQ(camTest.camParam->cx(),379.999); GTEST_ASSERT_EQ(camTest.camParam->cy(),255.238);
    /// Distortion_coefficients
    GTEST_ASSERT_EQ(camTest.camParam->d0(),-0.28368365); GTEST_ASSERT_EQ(camTest.camParam->d1(),0.07451284);
    GTEST_ASSERT_EQ(camTest.camParam->d2(),-0.00010473);GTEST_ASSERT_EQ(camTest.camParam->d3(),-3.55590700e-05);

    double timestamp;
    for (int i = 0; i < 10; ++i) {
        std::string fileName = camTest.getNextFrame(timestamp);
        std::cout<<"["<<i<<"]: "<<std::fixed<<timestamp<<" <===> "<<fileName<<"\n";
    }

#endif


}
