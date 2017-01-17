//
// Created by lancelot on 1/15/17.
//

#include "Initialize.h"
#include "Implement/InitialImpl.h"

Initialize::Initialize() {
    impl_ = std::make_shared<InitialImpl>();
}

bool Initialize::init(std::vector<std::shared_ptr<viFrame>>& VecFrames,
                      std::vector<std::shared_ptr<imuFactor>>& VecImuFactor,
                      ImuParameters& imuParam, int n_iter) {

    return impl_->init(VecFrames, VecImuFactor, imuParam, n_iter);

}


