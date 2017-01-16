//
// Created by lancelot on 1/15/17.
//

#ifndef SIMPLE_VIO_INITIALIZE_H
#define SIMPLE_VIO_INITIALIZE_H

#include <memory>
#include "DataStructure/imu/IMUMeasure.h"
#include "DataStructure/viFrame.h"
#include "IMU/IMU.h"

class Initialize {
public:
    Initialize(IMU::IntegalType type = IMU::PRE_INTEGRATION);
    bool init(std::shared_ptr<viFrame> &firstFrame, std::shared_ptr<viFrame>&frame,
              IMUMeasure::ImuMeasureDeque &imuMeasures, ImuParameters& imuParam);
    std::shared_ptr<IMU> imu_;
};


#endif //SIMPLE_VIO_INITIALIZE_H
