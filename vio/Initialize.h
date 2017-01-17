//
// Created by lancelot on 1/15/17.
//

#ifndef SIMPLE_VIO_INITIALIZE_H
#define SIMPLE_VIO_INITIALIZE_H

#include <memory>
#include "DataStructure/imu/IMUMeasure.h"
#include "DataStructure/viFrame.h"
#include "IMU/IMU.h"

class InitialImpl;

class Initialize {
public:
    Initialize();
    bool init(std::vector<std::shared_ptr<viFrame>>& VecFrames,
              std::vector<std::shared_ptr<imuFactor>>& VecImuFactor,
              ImuParameters& imuParam, int n_iter = 30);

private:
    std::shared_ptr<InitialImpl> impl_;
};


#endif //SIMPLE_VIO_INITIALIZE_H
