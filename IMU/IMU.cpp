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
typedef Eigen::Matrix<double, 9, 1> bias_t;

#endif // IMUTYPE_DEF_


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

int IMU::error(const viFrame &frame_i, const viFrame &frame_j, Error_t &err, void *info)
{
    return impl->error(frame_i, frame_j, err, info);
}

int IMU::Jacobian(const error_t &err, const viFrame &frame_i, jacobian_t &jacobian_i, const viFrame &frame_j, jacobian_t &jacobian_j)
{
    return impl->Jacobian(err, frame_i, jacobian_i, frame_j, jacobian_j);
}
