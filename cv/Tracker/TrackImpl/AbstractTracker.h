//
// Created by lancelot on 2/20/17.
//

#ifndef SIMPLE_VIO_ABSTRACTTRACKER_H
#define SIMPLE_VIO_ABSTRACTTRACKER_H

#include "ThirdParty/sophus/se3.hpp"

#include <memory>

class viFrame;

namespace direct_tracker {
    typedef Sophus::SE3d Translation;

    class AbstractTracker {
    public:
        AbstractTracker();
        virtual ~AbstractTracker();
        virtual bool tracking(std::shared_ptr<viFrame> &cvframe_i, std::shared_ptr<viFrame> &cvframe_j, Translation &T_ij) = 0;
    };
}


#endif //SIMPLE_VIO_ABSTRACTTRACKER_H
