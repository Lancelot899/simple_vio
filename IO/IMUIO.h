#ifndef IMUIO_H
#define IMUIO_H

#include "IOBase.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>


struct IMUData {
    int                      ID;
    Eigen::Vector3f          acceleration;
    Eigen::Vector3f          angular_velocity;
    Eigen::Quaternion<float> orientation;
    Eigen::Vector3f          geomagnetism;
    float                    atmospheric_pressure;
    float                    time;
};

class IMUIO : public IOBase<IMUData>
{
public:
    IMUIO();
    std::shared_ptr<IMUData> pop();
};

#endif // IMUIO_H
