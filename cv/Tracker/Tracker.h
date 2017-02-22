//
// Created by lancelot on 2/20/17.
//

#ifndef SIMPLE_VIO_TRACKER_H
#define SIMPLE_VIO_TRACKER_H

#include "ThirdParty/sophus/se3.hpp"

class cvFrame;
class viFrame;

namespace direct_tracker {

    class Tracker {
    public:
        Tracker();
        bool Tracking(std::shared_ptr<viFrame>&viframe_i, std::shared_ptr<viFrame>&viframe_j, Sophus::SE3d &T_ji, int n_iter);

    private:

    };
}


#endif //SIMPLE_VIO_TRACKER_H
