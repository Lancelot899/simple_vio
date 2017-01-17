//
// Created by lancelot on 1/17/17.
//

#include <limits>

#include "InitialImpl.h"
#include "DataStructure/cv/cvFrame.h"
#include "DataStructure/imu/imuFactor.h"

InitialImpl::InitialImpl() {
    imu_ =  std::make_shared<IMU>();
}


bool InitialImpl::init(std::vector<std::shared_ptr<viFrame>> &VecFrames,
                       std::vector<std::shared_ptr<imuFactor>>& VecImuFactor,
                       ImuParameters &imuParam, int n_iter) {
    assert(VecImuFactor.size() + 1 == VecFrames.size());

    size_t size = VecFrames.size();

    if(size < 3) {
        printf("frames are too few\n");
        return false;
    }

    if(size < 6) {
        printf("warning: the information got from initializing steps may not be ensured!\n");
    }

    Eigen::Vector3d gbias(0.0, 0.0, 0.0);
    Eigen::Vector3d Err;
    Eigen::Matrix3d Jac;
    double errOld;

    for(int iter = 0; iter < n_iter; ++iter) {
        if(iter == 0)
            errOld =;
        Err.setZero();
        Jac.setZero();
        for (int i = 0; i < size - 1; ++i) {
            const IMUMeasure::Transformation &deltaPose = VecImuFactor[i]->deltaPose;
            const imuFactor::FacJBias_t &facJac = VecImuFactor[i]->getJBias();
            Eigen::Vector3d err = Sophus::SO3d::log(
                    deltaPose.so3() * Sophus::SO3d::exp(facJac.block<3, 3>(0, 0) * gbias).inverse()
                    * VecFrames[i]->getPose().so3() * VecFrames[i + 1]->getPose().so3().inverse());



            Err += err;
        }
    }



}

