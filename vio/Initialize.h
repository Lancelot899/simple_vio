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
    std::shared_ptr<InitialImpl> impl_;
    std::vector<std::shared_ptr<viFrame>> VecFrames;
    bool isInitialed;
    std::vector<std::shared_ptr<imuFactor>> VecImuFactor;
};


#endif //SIMPLE_VIO_INITIALIZE_H
