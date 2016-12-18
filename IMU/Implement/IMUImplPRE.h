#ifndef IMUIMPLPRE_H
#define IMUIMPLPRE_H

#include "IMUImpl.h"

class IMUImplPRE : public IMUImpl
{
public:
    IMUImplPRE();
    int propagation(const ImuMeasureDeque & imuMeasurements,
                    const ImuParamenters & imuParams,
                    Transformation& T_WS,
                    SpeedAndBias & speedAndBiases,
                    double & t_start,
                    double & t_end,
                    covariance_t* covariance,
                    jacobian_t* jacobian);
};

#endif // IMUIMPLPRE_H
