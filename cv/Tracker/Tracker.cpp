//
// Created by lancelot on 2/20/17.
//


#include <memory>
#include "Tracker.h"
#include "DataStructure/cv/cvFrame.h"
#include "DataStructure/viFrame.h"
#include "DataStructure/cv/Feature.h"
#include "DataStructure/cv/Point.h"
#include "ThirdParty/sophus/se3.hpp"

namespace direct_tracker {

typedef Eigen::Matrix<double, 1, 6> Jac_t;

void compute_EdgeJac(std::shared_ptr<cvFrame> &cvframe_j, std::shared_ptr<Feature> &ft, Jac_t &jac) {

}

void compute_PointJac(std::shared_ptr<cvFrame> &cvframe_j, std::shared_ptr<Feature> &ft,Sophus::SE3d T_ji,Jac_t &jac) {
    typedef Eigen::Vector3d Point3d;

    cvMeasure::cam_t camera = cvframe_j->getCam();
    double fx = camera->fx(), fy = camera->fy(), cx = camera->cx(), cy = camera->cy();
    Point3d point = T_ji * ft->point->pos_;
    double X_ = point(0), Y_ = point(1), Z_ = point(2);
    double u = fx*X_/Z_ + cx;
    double v = fy*Y_/Z_ + cy;

    Eigen::Vector2d grad;cvframe_j->getGrad(u,v,grad);
    double dI_du = grad(0) ,dI_dv = grad(1);

    double zInverse = 1.0/Z_,  fx_z = fx*zInverse, fy_z = fy*zInverse, fx_zz = fx_z*zInverse, fy_zz = fy_z*zInverse;
    jac(0,0) = dI_du * fx_zz * X_ * Y_ + dI_dv * fy + dI_dv * fy_zz * Y_ * Y_;
    jac(0,1) = -dI_du * fx - dI_du * fx_zz * X_ * X_ - dI_dv * fy_zz * X_ * Y_;
    jac(0,2) = dI_du * fx_z * Y_ - dI_dv * fy_z * X_;
    jac(0,3) = dI_du * fx_z;
    jac(0,4) = dI_dv * fy_z;
    jac(0,5) = -dI_du * fx_zz * X_ - dI_dv * fy_zz * Y_;
}


Tracker::Tracker() {}
}

