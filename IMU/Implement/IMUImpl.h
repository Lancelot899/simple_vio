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

};

#endif // IMUIMPL_H
