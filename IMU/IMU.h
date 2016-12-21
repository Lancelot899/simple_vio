#ifndef IMU_H
#define IMU_H

#include <memory>
#include "DataStructure/IMUMeasure.h"

class IMUImpl;

class viFrame;

class IMU
{
public:
    typedef std::shared_ptr<viFrame> pViFrame;

public:
    enum IntegalType {
        PRE_INTEGRATION,
        OKVIS_INTEGRATION
    };

public:
    IMU(IntegalType type = PRE_INTEGRATION);
    int propagation(const IMUMeasure::ImuMeasureDeque & imuMeasurements,
                    const ImuParamenters & imuParams,
                    IMUMeasure::Transformation& T_WS,
                    IMUMeasure::SpeedAndBias & speedAndBiases,
                    double & t_start,
                    double & t_end,
                    IMUMeasure::covariance_t* covariance,
                    IMUMeasure::jacobian_t* jacobian);

    int repropagation();
    int error(const pViFrame& frame_i, const pViFrame& frame_j, IMUMeasure::Error_t &err/* out */, void *info = NULL);
    int Jacobian(const error_t& err, const pViFrame& frame_i, IMUMeasure::jacobian_t& jacobian_i, const pViFrame& frame_j, IMUMeasure::jacobian_t& jacobian_j);

private:
    std::shared_ptr<IMUImpl> impl;
};

#endif // IMU_H
