#include "IMU.h"
#include "Implement/IMUImplOKVIS.h"
#include "Implement/IMUImplPRE.h"

IMU::IMU(IntegalType type)
{
    switch(type) {
    case OKVIS_INTEGRATION:
        impl = std::make_shared<IMUImplOKVIS>();
        break;
    case PRE_INTEGRATION:
        impl = std::make_shared<IMUImplPRE>();
        break;
    }


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

int IMU::error(const imuFrame &frame_i, const imuFrame &frame_j, Error_t &err, void *info)
{
    return impl->error(frame_i, frame_j, err, info);
}

int IMU::Jacobian(const error_t &err, const imuFrame &frame_i, jacobian_t &jacobian_i, const imuFrame &frame_j, jacobian_t &jacobian_j)
{
    return impl->Jacobian(err, frame_i, jacobian_i, frame_j, jacobian_j);
}
