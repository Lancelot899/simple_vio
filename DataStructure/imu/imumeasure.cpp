#include "IMUMeasure.h"
#include "util/setting.h"

bool IMUMeasure::ImuMeasureDeque::addImuMeasurement(int sensorID, const okvis::Time &stamp, const Eigen::Vector3d &alpha, const Eigen::Vector3d &omega) {
    IMUMeasure_Ptr imu_measurement = std::make_shared<IMUMeasure>(sensorID, stamp, alpha, omega);
    push_back(imu_measurement);
    return true;
}


