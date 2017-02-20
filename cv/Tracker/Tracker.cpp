//
// Created by lancelot on 2/20/17.
//


#include <memory>
#include "Tracker.h"
#include "DataStructure/cv/cvFrame.h"
#include "DataStructure/viFrame.h"
#include "DataStructure/cv/Feature.h"

namespace direct_tracker {

    typedef Eigen::Matrix<double, 1, 6> Jac_t;

    void compute_EdgeJac(std::shared_ptr<cvFrame> &cvframe_j, std::shared_ptr<Feature> &ft, Jac_t &jac) {

    }

    void compute_PointJac(std::shared_ptr<cvFrame> &cvframe_j, std::shared_ptr<Feature> &ft, Jac_t &jac) {

    }


    Tracker::Tracker() {}
}

