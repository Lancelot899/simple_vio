#ifndef IMUIMPL_H
#define IMUIMPL_H

class IMUImpl;
class ImuMeasureDeque;
class ImuParamenters;
class Transformation;
class SpeedAndBias;
class jacobian_t;
class covariance_t;

class IMUImpl
{
public:
    IMUImpl() {

    }

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
