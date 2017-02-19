//
// Created by lancelot on 1/15/17.
//

#include "Initialize.h"
#include "Implement/InitialImpl.h"
#include "DataStructure/cv/cvFrame.h"

Initialize::Initialize() {
    impl_ = std::make_shared<InitialImpl>();
    isInitialed = false;
}

void Initialize::setFirstFrame(std::shared_ptr<cvFrame> &cvframe) {
    std::shared_ptr<viFrame> firstFrame = std::make_shared<viFrame>(0, cvframe);
    Sophus::SE3d ie;
    cvframe->setPose(ie);

}

void Initialize::pushcvFrame(std::shared_ptr<cvFrame> &cvframe, std::shared_ptr<imuFactor> &imufactor) {

}

bool Initialize::init(std::shared_ptr<ImuParameters> &imuParam, int n_iter) {
    if(VecFrames.size() < 3)
        return false;
    return this->initImu(imuParam, n_iter);
}

bool Initialize::initImu(std::shared_ptr<ImuParameters> &imuParam, int n_iter) {
    isInitialed = impl_->init(VecFrames, VecImuFactor, imuParam, n_iter);
    return isInitialed;
}


