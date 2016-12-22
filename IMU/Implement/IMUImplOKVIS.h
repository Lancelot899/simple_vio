#ifndef IMUIMPLOKVIS_H
#define IMUIMPLOKVIS_H

#include "IMUImpl.h"


class IMUImplOKVIS : public IMUImpl {
    int propagation(const ImuMeasureDeque &imuMeasurements,
                             const ImuParamenters &imuParams,
                             Transformation &T_WS,
                             SpeedAndBias &speedAndBiases,
                             double &t_start,
                             double &t_end,
                             covariance_t *covariance,
                             jacobian_t *jacobian);

    int error(const IMU::pViFrame &frame_i, const IMU::pViFrame &frame_j, Error_t &err, void *info);
    int repropagation();
    int Jacobian(const Error_t& err, const IMU::pViFrame& frame_i, jacobian_t& jacobian_i, const IMU::pViFrame& frame_j, jacobian_t& jacobian_j, void *info);

};


#endif // IMUIMPLOKVIS_H

