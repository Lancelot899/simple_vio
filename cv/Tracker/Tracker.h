//
// Created by lancelot on 2/20/17.
//

#ifndef SIMPLE_VIO_TRACKER_H
#define SIMPLE_VIO_TRACKER_H

#include <memory>
#include "ThirdParty/sophus/se3.hpp"

class cvFrame;
class viFrame;

namespace direct_tracker {

    class Tracker {
    public:
        Tracker();
        bool Tracking(std::shared_ptr<viFrame>&viframe_i, std::shared_ptr<viFrame>&viframe_j,
                      Sophus::SE3d &Tij, Eigen::Matrix<double, 6, 6>& infomation, int n_iter = 30);
        int  reProject(std::shared_ptr<viFrame>&viframe_i, std::shared_ptr<viFrame>&viframe_j,
                       Sophus::SE3d &Tij, Eigen::Matrix<double, 6, 6>& infomation);
    private:

    };
}


#endif //SIMPLE_VIO_TRACKER_H
