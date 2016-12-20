#ifndef IMUIMPL_H
#define IMUIMPL_H

#include "IMU/IMU.h"

class IMUImpl {
public:
    IMUImpl();

    virtual int propagation(const ImuMeasureDeque & imuMeasurements,
                    const ImuParamenters & imuParams,
                    Transformation& T_WS,
                    SpeedAndBias & speedAndBiases,
                    double & t_start,
                    double & t_end,
                    covariance_t* covariance,
                    jacobian_t* jacobian) = 0;

    virtual int error(const imuFrame& frame_i, const imuFrame& frame_j, Error_t &err/* out */, void *info) = 0;
    virtual int repropagation();
    virtual int Jacobian(const error_t& err, const imuFrame& frame_i, jacobian_t& jacobian_i, const imuFrame& frame_j, jacobian_t& jacobian_j) = 0;
};

#endif // IMUIMPL_H
