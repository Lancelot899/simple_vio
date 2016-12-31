#include <opencv2/ts.hpp>

#include "IO/imu/IMUIO.h"

TEST(TestImuIO, IMU_IO) {
    std::string imuDatafile("../testData/mav0/imu0/data.csv");
    std::string imuParamfile("../testData/mav0/imu0/sensor.yaml");
    IMUIO imuIO(imuDatafile, imuParamfile);
    IMUIO::pImuParam imuParam = imuIO.getImuParam();

    GTEST_ASSERT_NE(imuParam.get(), nullptr);
    GTEST_ASSERT_EQ(imuParam->T_BS.matrix(), Eigen::Matrix4d::Identity());
    GTEST_ASSERT_EQ(imuParam->rate, 200);
    GTEST_ASSERT_EQ(imuParam->sigma_ba, 3.0000e-3);
    GTEST_ASSERT_EQ(imuParam->sigma_bg, 1.9393e-05);
    GTEST_ASSERT_EQ(imuParam->sigma_a_c, 2.0000e-3);
    GTEST_ASSERT_EQ(imuParam->sigma_g_c, 1.6968e-04);

    IMUIO::pData_t data = imuIO.pop();
    GTEST_ASSERT_NE(imuParam.get(), nullptr);
    Eigen::Vector3d gyr(-0.0020943951023931952,0.017453292519943295,0.07749261878854824);
    Eigen::Vector3d acc(9.0874956666666655,0.13075533333333333,-3.6938381666666662);

    GTEST_ASSERT_EQ(data->measurement.acceleration, acc);
    GTEST_ASSERT_EQ(data->measurement.gyroscopes, gyr);
    int i = 1;
    data = imuIO.pop();
    while(data.get() != nullptr && i < 150) {
        i++;
        if(i == 100) {
            Eigen::Vector3d gyr(-0.19198621771937624,0.0034906585039886592,0.12077678423800758);
            Eigen::Vector3d acc(9.2100787916666658,0.15527195833333335,-3.6448049166666663);
            GTEST_ASSERT_EQ(data->measurement.acceleration, acc);
            GTEST_ASSERT_EQ(data->measurement.gyroscopes, gyr);
        }

        data = imuIO.pop();
    }


}



