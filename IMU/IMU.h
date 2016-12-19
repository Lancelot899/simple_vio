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
typedef IMUMeasure::error_t         error_t;

class imuFrame;

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

    int repropagation();
    int error(imuFrame& frame_i, imuFrame& frame_j, error_t &err/* out */);
    int Jacobian(error_t& err, imuFrame& frame_i, jacobian_t& jacobian_i, imuFrame& frame_j, jacobian_t& jacobian_j);

private:
    std::shared_ptr<IMUImpl> impl;
};

#endif // IMU_H
