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

    virtual int error(imuFrame& frame_i, imuFrame& frame_j, error_t &err/* out */) = 0;
    virtual int repropagation();
    virtual int Jacobian(error_t& err, imuFrame& frame_i, jacobian_t& jacobian_i, imuFrame& frame_j, jacobian_t& jacobian_j) = 0;
};

#endif // IMUIMPL_H
