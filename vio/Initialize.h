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
namespace feature_detection {
    class Detector;
}

class Initialize {
public:
    Initialize(std::shared_ptr<feature_detection::Detector>& detector);
    void setFirstFrame(std::shared_ptr<cvFrame> &cvframe);
    void pushcvFrame(std::shared_ptr<cvFrame> &cvframe, std::shared_ptr<imuFactor> &imufactor);
    std::vector<std::shared_ptr<viFrame>>* getInitialViframe() {
        if(isInitialed)
            return &VecFrames;

        return nullptr;
    }

    bool init(std::shared_ptr<ImuParameters> &imuParam, int n_iter = 30);

private:
    bool initImu(std::shared_ptr<ImuParameters> &imuParam, int n_iter);

private:
    std::shared_ptr<feature_detection::Detector> detector;
    std::shared_ptr<InitialImpl> impl_;
    std::vector<std::shared_ptr<viFrame>> VecFrames;
    bool isInitialed;
    std::vector<std::shared_ptr<imuFactor>> VecImuFactor;
};


#endif //SIMPLE_VIO_INITIALIZE_H
