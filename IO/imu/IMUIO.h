#ifndef IMUIO_H
#define IMUIO_H

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "../IOBase.h"
#include "DataStructure/imu/IMUMeasure.h"


namespace okvis {
    class Time;
}

class IMUIO : public IOBase<IMUMeasure> {
public:
    typedef data_t::ImuMeasureDeque          dataDeque_t;
    typedef std::shared_ptr<ImuParameters>   pImuParam;

public:
    IMUIO(std::string &imufile, std::string &imuParamfile);
    dataDeque_t pop(okvis::Time& start, okvis::Time& end);
    pData_t pop();
    const pImuParam& getImuParam();

private:
    dataDeque_t                     imuMeasureDeque;
    pImuParam                       imuParam;
};

#endif // IMUIO_H
