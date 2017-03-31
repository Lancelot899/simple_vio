#include <opencv2/ts/ts.hpp>

#include "IO/imu/IMUIO.h"

TEST(TestImuIO, IMU_IO) {
    std::string imuDatafile("../testData/mav0/imu0/data.csv");
    std::string imuParamfile("../testData/mav0/imu0/sensor.yaml");
    IMUIO imuIO(imuDatafile, imuParamfile);
    IMUIO::pImuParam imuParam = imuIO.getImuParam();
	printf(" --test start!\n");
    GTEST_ASSERT_NE(imuParam.get(), nullptr);
    GTEST_ASSERT_EQ(imuParam->T_BS.matrix(), Eigen::Matrix4d::Identity());
    GTEST_ASSERT_EQ(imuParam->rate, 200);
    GTEST_ASSERT_EQ(imuParam->sigma_ba, 3.0000e-3);
    GTEST_ASSERT_EQ(imuParam->sigma_bg, 1.9393e-05);
    GTEST_ASSERT_EQ(imuParam->sigma_a_c, 2.0000e-3);
    GTEST_ASSERT_EQ(imuParam->sigma_g_c, 1.6968e-04);

    IMUIO::pData_t data = imuIO.pop();
    GTEST_ASSERT_NE(imuParam.get(), nullptr);

    int i = 1;
    data = imuIO.pop();
    while(data.get() != nullptr && i < 150) {
        data = imuIO.pop();
    }


}



