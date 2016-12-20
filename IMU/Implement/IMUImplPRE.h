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

    int error(imuFrame &frame_i, imuFrame &frame_j, error_t &err);
    int Jacobian(error_t& err, imuFrame& frame_i, jacobian_t& jacobian_i, imuFrame& frame_j, jacobian_t& jacobian_j);




};

#endif // IMUIMPLPRE_H
