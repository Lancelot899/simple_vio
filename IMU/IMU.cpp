#include "IMU.h"
#include "Implement/IMUImplOKVIS.h"

IMU::IMU()
{
    impl = std::make_shared<IMUImplOKVIS>();
}

int IMU::propagation(const ImuMeasureDeque &imuMeasurements,
                     const ImuParamenters &imuParams,
                     Transformation &T_WS,
                     SpeedAndBias &speedAndBiases,
                     double &t_start,
                     double &t_end,
                     covariance_t *covariance,
                     jacobian_t *jacobian)
{
    return impl->propagation(imuMeasurements, imuParams, T_WS, speedAndBiases, t_start, t_end, covariance, jacobian);
}
