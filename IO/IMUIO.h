#ifndef IMUIO_H
#define IMUIO_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "IOBase.h"
#include "DataStructure/imu/IMUMeasure.h"

class IMUIO : public IOBase<IMUMeasure>
{
public:
    IMUIO(std::string &file);
    std::shared_ptr<IMUMeasure> pop(okvis::Time& start, okvis::Time& end);

private:
    int                             number_of_lines;
    IMUMeasure::ImuMeasureDeque     imuMeasureDeque;
};

#endif // IMUIO_H
