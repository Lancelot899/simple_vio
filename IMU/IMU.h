#ifndef IMU_H
#define IMU_H

#include <memory>

class IMUImpl;
class ImuMeasureDeque;
class ImuParamenters;
class Transformation;
class SpeedAndBias;
class jacobian_t;
class covariance_t;

class IMU
{
public:
    IMU();
    int propagation(const ImuMeasureDeque & imuMeasurements,
                    const ImuParamenters & imuParams,
                    Transformation& T_WS,
                    SpeedAndBias & speedAndBiases,
                    double & t_start,
                    double & t_end,
                    covariance_t* covariance,
                    jacobian_t* jacobian);

private:
    std::shared_ptr<IMUImpl> impl;
};

#endif // IMU_H
