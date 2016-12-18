#ifndef IMU_H
#define IMU_H

#include <memory>
#include "DataStructure/IMUMeasure.h"

class IMUImpl;

typedef IMUMeasure::ImuMeasureDeque ImuMeasureDeque;
typedef IMUMeasure::Transformation  Transformation;
typedef IMUMeasure::SpeedAndBias    SpeedAndBias;
typedef IMUMeasure::covariance_t    covariance_t;
typedef IMUMeasure::jacobian_t      jacobian_t;

class IMU
{
public:
    enum IntegalType {
        PRE_INTEGRATION,
        OKVIS_INTEGRATION
    };

public:
    IMU(IntegalType type = PRE_INTEGRATION);
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
