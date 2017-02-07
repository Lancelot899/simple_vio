//
// Created by lancelot on 1/17/17.
//

#ifndef SIMPLE_VIO_INITIALIMPL_H
#define SIMPLE_VIO_INITIALIMPL_H

#include <memory>
#include "DataStructure/imu/IMUMeasure.h"
#include "DataStructure/viFrame.h"
#include "IMU/IMU.h"

class InitialImpl {
public:
    InitialImpl();
    bool init(std::vector<std::shared_ptr<viFrame>>& VecFrames,
              std::vector<std::shared_ptr<imuFactor>>& VecImuFactor,
              std::shared_ptr<ImuParameters> &imuParam, int n_iter);

private:
    std::shared_ptr<IMU> imu_;
};


#endif //SIMPLE_VIO_INITIALIMPL_H
