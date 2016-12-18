#ifndef IMUIMPLOKVIS_H
#define IMUIMPLOKVIS_H

#include "IMUImpl.h"


class IMUImplOKVIS : public IMUImpl {
    __inline__ Eigen::Matrix3d rightJacobian(const Eigen::Vector3d & PhiVec);

    int propagation(const ImuMeasureDeque &imuMeasurements,
                             const ImuParamenters &imuParams,
                             Transformation &T_WS,
                             SpeedAndBias &speedAndBiases,
                             double &t_start,
                             double &t_end,
                             covariance_t *covariance,
                             jacobian_t *jacobian);
};


#endif // IMUIMPLOKVIS_H

