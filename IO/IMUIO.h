#ifndef IMUIO_H
#define IMUIO_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "IOBase.h"
#include "DataStructure/imu/IMUMeasure.h"

class IMUIO : public IOBase<IMUMeasure>
{
public:
    IMUIO();
    std::shared_ptr<IMUMeasure> pop();
};

#endif // IMUIO_H
