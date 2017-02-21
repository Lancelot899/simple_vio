//
// Created by lancelot on 2/20/17.
//


#include <memory>
#include "Tracker.h"
#include "DataStructure/cv/cvFrame.h"
#include "DataStructure/viFrame.h"
#include "DataStructure/cv/Feature.h"
#include "DataStructure/cv/Point.h"

namespace direct_tracker {

    typedef Eigen::Matrix<double, 1, 6> Jac_t;
    bool compute_EdgeJac(std::shared_ptr<viFrame> &viframe_j, std::shared_ptr<Feature> &ft, const Sophus::SE3d T_SB, const Sophus::SE3d& Ti,  Sophus::SE3d& T_ji, Jac_t &jac, double &w) {
        std::shared_ptr<Point>& p = ft->point;
        Eigen::Vector2d& dir = ft->grad;
        Eigen::Vector2d grad;
        Eigen::Vector3d P = T_ji * Ti * p->pos_;
        const viFrame::cam_t & cam = viframe_j->getCam();
        Eigen::Vector3d Pj = T_SB * P;
        if(Pj(2) < 0.0000001)
            return false;

        double u = cam->fx() * Pj(0) / Pj(2) + cam->cx();
        if(u < 0 || u >= viframe_j->getCVFrame()->getWidth(ft->level))
            return false;

        double v = cam->fy() * Pj(1) / Pj(2) + cam->cy();

        if(v < 0 || v >= viframe_j->getCVFrame()->getWidth(ft->level))
            return false;

        if(!viframe_j->getCVFrame()->getGrad(u, v, grad, ft->level))
            return false;

        w = 1.0 / viframe_j->getCVFrame()->getGradNorm(u, v, ft->level);
        if(w < 0.00000001 || std::isinf(w))
            return false;

        double Ix = dir(1) * dir(0);
        double Iy = Ix * grad(0) + dir(1) * dir(1) * grad(1);
        Ix *= grad(1);
        Ix += dir(0) * dir(0) * grad(0);
        jac(0, 0) =  cam->fx() /  Pj(2);
        jac(0, 1) = cam->fy() / Pj(2);
        jac(0, 2) = -jac(0, 0) * Pj(0) / Pj(2);
        jac(0, 3) = -jac(0, 1) * Pj(1) / Pj(2);
        jac(0, 0) *= Ix;
        jac(0, 1) *= Iy;
        jac(0, 2) = jac(0, 2) * Ix + jac(0, 3) * Iy;
        Eigen::Matrix<double, 3, 6> JacR;
        JacR.block<3, 3>(0, 0) = T_SB.so3().matrix();
        JacR.block<3, 3>(0, 3) = - Sophus::SO3d::hat(P) * JacR.block<3, 3>(0, 0);

        return true;
    }

    bool compute_PointJac(std::shared_ptr<cvFrame> &cvframe_j, std::shared_ptr<Feature> &ft, Jac_t &jac) {

    }


    Tracker::Tracker() {}
}

