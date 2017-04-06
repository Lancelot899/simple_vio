//
// Created by lancelot on 2/20/17.
//


#include <memory>

#include <ceres/ceres.h>
#include <boost/thread/shared_mutex.hpp>

#include "Tracker.h"
#include "DataStructure/cv/cvFrame.h"
#include "DataStructure/viFrame.h"
#include "DataStructure/cv/Feature.h"
#include "DataStructure/cv/Point.h"
#include "util/setting.h"

#define PHOTOMATRICERROR 100.0


namespace direct_tracker {

class TrackingErr : public ceres::SizedCostFunction<1, 6> {
public:
    TrackingErr(const std::shared_ptr<Feature> &ft, std::shared_ptr<viFrame> &viframe_i,
                std::shared_ptr<viFrame> &viframe_j) {
        this->ft = ft;
        this->viframe_i = viframe_i;
        this->viframe_j = viframe_j;
        T_SB = viframe_i->getT_BS().inverse();
    }

    virtual bool Evaluate(double const *const *parameters,
                          double *residuals,
                          double **jacobians) const {
        Eigen::Vector3d so3, trans_ji;
        for (int i = 0; i < 3; ++i) {
            so3(i) = parameters[0][i];
            trans_ji(i) = parameters[0][3 + i];
        }

        Sophus::SO3d R_ji = Sophus::SO3d::exp(so3);
        const Eigen::Vector3d &p = ft->point->pos_;
        Eigen::Vector3d pi = viframe_i->getCVFrame()->getPose() * p;
        if (pi(2) > 0.0000000001) {
            Eigen::Vector3d pj = T_SB * (R_ji * (viframe_j->getT_BS() * pi) + trans_ji);

            if (pj(2) > 0.0000000001) {
                const viFrame::cam_t &cam = viframe_j->getCam();
                double u = cam->fx() * (pj(0) / pj(2)) + cam->cx();
                if (u >= 0 && u < viframe_j->getCVFrame()->getWidth()) {
                    double v = cam->fy() * (pj(1) / pj(2)) + cam->cy();
                    if (v >= 0 && v < viframe_j->getCVFrame()->getHeight()) {

                        Eigen::Vector2d px = cam->world2cam(pi);
                        if (px(0) >= 0 && px(0) < viframe_i->getCVFrame()->getWidth()
                                && px(1) >= 0 && px(1) < viframe_i->getCVFrame()->getHeight()) {

                            for (int i = 0; i < ft->level; ++i) {
                                u /= 2.0;
                                v /= 2.0;
                                px /= 2.0;
                            }

                            double err = viframe_j->getCVFrame()->getIntensityBilinear(u, v, ft->level)
                                       - viframe_i->getCVFrame()->getIntensityBilinear(px(0), px(1), ft->level);

                            if (err < PHOTOMATRICERROR && err > -PHOTOMATRICERROR) {
                                double w = 1.0 / viframe_j->getCVFrame()->getGradNorm(u, v, ft->level);

                                if (w > 0.0000001 && !std::isinf(w)) {
                                    *residuals = w * err;
                                    Eigen::Vector2d grad;

                                    if (viframe_j->getCVFrame()->getGrad(u, v, grad, ft->level)) {
                                        Eigen::Vector2d &dir = ft->grad;
                                        w = std::sqrt(w);
                                        if (jacobians && jacobians[0]) {
                                            double Ix, Iy;
                                            if (ft->type == Feature::EDGELET) {
                                                Ix = dir(1) * dir(0);
                                                Iy = Ix * grad(0) + dir(1) * dir(1) * grad(1);
                                                Ix *= grad(1);
                                                Ix += dir(0) * dir(0) * grad(0);
                                            } else {
                                                Ix = grad(0);
                                                Iy = grad(1);
                                            }
                                            Eigen::Matrix<double, 1, 3> Jac;
                                            Jac(0, 0) = Ix * cam->fx() / pj(2);
                                            Jac(0, 1) = Iy * cam->fy() / pj(2);
                                            Jac(0, 2) = -Ix * cam->fx()  * pj(0) / pj(2) / pj(2) -
                                                    Iy * cam->fy() * pj(1) / pj(2) / pj(2);
                                            Jac = w * Jac * T_SB.rotationMatrix();
                                            jacobians[0][3] = Jac(0, 0);
                                            jacobians[0][4] = Jac(0, 1);
                                            jacobians[0][5] = Jac(0, 2);
                                            Jac = -Jac * Sophus::SO3d::hat(R_ji * pi);
                                            jacobians[0][0] = Jac(0, 0);
                                            jacobians[0][1] = Jac(0, 1);
                                            jacobians[0][2] = Jac(0, 2);
                                        }
                                    }
                                    return true;
                                }
                            }
                        }
                    }
                }
            }
        }
        *residuals = 0;
        if(jacobians && jacobians[0])
            memset(jacobians[0], 0, sizeof(double) * 6);
        return true;
    }

private:
    std::shared_ptr<Feature> ft;
    std::shared_ptr<viFrame> viframe_i;
    std::shared_ptr<viFrame> viframe_j;
    Sophus::SE3d T_SB;
};

class CERES_EXPORT SE3Parameterization : public ceres::LocalParameterization {
public:
    virtual ~SE3Parameterization() {}
    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const;
    virtual bool ComputeJacobian(const double* x,
                                 double* jacobian) const;
    virtual int GlobalSize() const { return 6; }
    virtual int LocalSize() const { return 6; }
};

bool SE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const {
    ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6, 6);
    return true;
}

bool SE3Parameterization::Plus(const double* x,
                               const double* delta,
                               double* x_plus_delta) const {
    Eigen::Vector3d origin_x, delta_x;
    for(int i = 0; i < 3; ++i) {
        origin_x(i) = x[i];
        delta_x(i) = delta[i];
        x_plus_delta[3 + i] = x[3 + i] + delta[3 + i];
    }

    Sophus::SO3d R = Sophus::SO3d::exp(origin_x);
    Sophus::SO3d delta_R = Sophus::SO3d::exp(delta_x);
    Eigen::Matrix<double, 3, 1> x_plus_delta_lie = (delta_R * R).log();

    for(int i = 0; i < 3; ++i) x_plus_delta[i] = x_plus_delta_lie(i, 0);
    return true;

}

Tracker::Tracker() {}

int Tracker::reProject(std::shared_ptr<viFrame> &viframe_i, std::shared_ptr<viFrame> &viframe_j, Sophus::SE3d &T_ji) {
    const cvMeasure::features_t &fts = viframe_i->getCVFrame()->getMeasure().fts_;
    int width = viframe_j->getCVFrame()->getWidth();
    int height = viframe_j->getCVFrame()->getHeight();
    int cellwidth = viframe_j->getCVFrame()->getWidth() / detectCellWidth;
    int chellheight = viframe_j->getCVFrame()->getHeight() / detectCellHeight;
    int cntCell = 0;
    for (auto &ft : fts) {
        Eigen::Vector2d uv = viframe_j->getCam()->world2cam(viframe_j->getCVFrame()->getPose() * ft->point->pos_);
        if (uv(0) < width && uv(1) < height && uv(0) > 0 && uv(1) > 0) {
            std::shared_ptr<Feature> ft_ = std::make_shared<Feature>(viframe_j->getCVFrame(),
                                                                     ft->point, uv, ft->point->pos_, ft->level);
            ft_->isBAed = ft->isBAed;
            viframe_j->getCVFrame()->addFeature(ft_);
            int u = int(uv(0) / cellwidth);
            int v = int(uv(1) / chellheight);
            if (!viframe_j->getCVFrame()->checkCell(u, v)) {
                viframe_j->getCVFrame()->setCellTrue(u, v);
                cntCell++;
            }
            ft->point->n_succeeded_reproj_++;
        }
        else {
            ft->point->n_failed_reproj_++;
        }
    }

    return cntCell;
}

bool Tracker::Tracking(std::shared_ptr<viFrame> &viframe_i, std::shared_ptr<viFrame> &viframe_j,
                       Sophus::SE3d &T_ji_, int n_iter) {

    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    const cvMeasure::features_t& fts = viframe_i->getCVFrame()->getMeasure().fts_;
    std::cout << "size of fts :" << fts.size() << std::endl;
    Eigen::Vector3d so3 = T_ji_.so3().log();
    Eigen::Vector3d& t = T_ji_.translation();
    double t_ji[6];
    for(int i = 0; i < 3; ++i) {
        t_ji[i] = so3(i);
        t_ji[3 + i] = t(i);
    }

    for(auto &f : fts) {
        problem.AddResidualBlock(new TrackingErr(f, viframe_i, viframe_j), new ceres::HuberLoss(0.5), t_ji);
    }
    problem.SetParameterization(t_ji, new SE3Parameterization);

    options.max_num_iterations = n_iter;
    options.minimizer_type = ceres::TRUST_REGION;
    options.linear_solver_type = ceres::DENSE_QR;

    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << std::endl;
    for(int i = 0; i < 3; ++i) {
        so3(i) = t_ji[i];
        t(i) = t_ji[3 + i];
    }

    T_ji_.so3() = Sophus::SO3d::exp(so3);
    if(summary.termination_type == ceres::CONVERGENCE)
        return true;

    return false;
}

}

