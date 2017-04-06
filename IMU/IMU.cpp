#include "IMU.h"
#include "Implement/IMUImplOKVIS.h"
#include "Implement/IMUImplPRE.h"

#ifndef IMUTYPE_DEF_
#define IMUTYPE_DEF_

typedef IMUMeasure::ImuMeasureDeque ImuMeasureDeque;
typedef IMUMeasure::Transformation  Transformation;
typedef IMUMeasure::SpeedAndBias    SpeedAndBias;
typedef IMUMeasure::covariance_t    covariance_t;
typedef IMUMeasure::jacobian_t      jacobian_t;
typedef IMUMeasure::Error_t         Error_t;
typedef Eigen::Vector3d             speed_t;
typedef Eigen::Matrix<double, 6, 1> bias_t;

#endif // IMUTYPE_DEF_


IMU::IMU(IntegalType type)
{
    switch(type) {
    case OKVIS_INTEGRATION:
        impl = std::make_shared<IMUImplOKVIS>();
//            printf("okvis!\n");
        break;
    case PRE_INTEGRATION:
        impl = std::make_shared<IMUImplPRE>();
//            printf("preIntegation !\n");
        break;
    }


}

int IMU::propagation(const ImuMeasureDeque &imuMeasurements,
                     const ImuParameters &imuParams,
                     Transformation &T_WS,
                     SpeedAndBias &speedAndBiases,
                     okvis::Time &t_start,
                     okvis::Time &t_end,
                     covariance_t *covariance,
                     jacobian_t *jacobian)
{
    return impl->propagation(imuMeasurements, imuParams, T_WS, speedAndBiases, t_start, t_end, covariance, jacobian);
}

int IMU::error(const pViFrame &frame_i, const pViFrame &frame_j, Error_t &err, void *info) {
    return impl->error(frame_i, frame_j, err, info);
}

int IMU::Jacobian(const Error_t &err, const pViFrame &frame_i, jacobian_t &jacobian_i, const pViFrame &frame_j, jacobian_t &jacobian_j, void *info) {
    return impl->Jacobian(err, frame_i, jacobian_i, frame_j, jacobian_j, info);
}
