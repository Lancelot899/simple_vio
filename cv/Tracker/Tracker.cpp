//
// Created by lancelot on 2/20/17.
//


#include <memory>
#include "Tracker.h"
#include "DataStructure/cv/cvFrame.h"
#include "DataStructure/viFrame.h"
#include "DataStructure/cv/Feature.h"
#include "DataStructure/cv/Point.h"
#include "util/setting.h"

#define PHOTOMATRICERROR 15.0

namespace direct_tracker {

typedef Eigen::Matrix<double, 1, 6> Jac_t;
bool compute_PointJac(std::shared_ptr<viFrame> &viframe_i,std::shared_ptr<viFrame> &viframe_j,
                      const std::shared_ptr<Feature> &ft,const Sophus::SE3d T_SB,
                      const Sophus::SE3d& Ti,  Sophus::SE3d& T_ji,
                      Jac_t &jac, double &w, double &err) {
    typedef Eigen::Vector3d Point3d;

    cvMeasure::cam_t camera = viframe_j->getCam();
    double fx = camera->fx(), fy = camera->fy(), cx = camera->cx(), cy = camera->cy();
    Point3d pointj = T_SB * T_ji * Ti  * ft->point->pos_;
    if(pointj(2) < 0.0000001)
        return false;

    double X_ = pointj(0), Y_ = pointj(1), Z_ = pointj(2);
    double u = fx*X_/Z_ + cx;
    double v = fy*Y_/Z_ + cy;

    Point3d pointI =  Ti * ft->point->pos_;

    if(u < 0 || u >= viframe_j->getCVFrame()->getWidth() ||
            v < 0 || v >= viframe_j->getCVFrame()->getHeight())
        return false;

    w = 1.0 / viframe_j->getCVFrame()->getGradNorm(u, v, ft->level);
    if(w < 0.00000001 || std::isinf(w))
        return false;

    err = viframe_i->getCVFrame()->getIntensityBilinear(fx*pointI(0)/pointI(2)+cx, fy*pointI(1)/pointI(2)+cy)
            -viframe_j->getCVFrame()->getIntensityBilinear(u,v);
    if(err>PHOTOMATRICERROR ||err<-PHOTOMATRICERROR)
    {
        printf("wrong match!\n");
        return false;
    }
    //    else printf("right match!\n");

    err = err>0? err:-err;

    Eigen::Vector2d grad;   viframe_j->getCVFrame()->getGrad(u,v,grad);
    double dI_du = grad(0), dI_dv = grad(1);

    double zInverse = 1.0/Z_,  fx_z = fx*zInverse, fy_z = fy*zInverse, fx_zz = fx_z*zInverse, fy_zz = fy_z*zInverse;
    Eigen::Matrix<double, 3, 6> JacR;
    JacR.block<3, 3>(0, 0) = T_SB.so3().matrix();
    JacR.block<3, 3>(0, 3) = - JacR.block<3, 3>(0, 0) * Sophus::SO3d::hat(pointj);
    Eigen::Matrix<double,2,3> a;
    a<<fx_z,0,-fx_zz*Z_,0,fy_z,-fy_zz*Y_;
    jac.block<1,6>(0,0) = Eigen::Matrix<double,1,2>(dI_du,dI_dv) *
            a * JacR;
    return true;
}


bool compute_EdgeJac(std::shared_ptr<viFrame> &viframe_i,
                     std::shared_ptr<viFrame> &viframe_j,
                     const std::shared_ptr<Feature> &ft,
                     const Sophus::SE3d T_SB,
                     const Sophus::SE3d& Ti,
                     Sophus::SE3d& T_ji,
                     Jac_t &jac,
                     double &w,
                     double &err) {
    std::shared_ptr<Point>& p = ft->point;
    Eigen::Vector2d& dir = ft->grad;
    Eigen::Vector2d grad;
    Eigen::Vector3d P = T_ji * Ti  * p->pos_;
    const viFrame::cam_t & cam = viframe_j->getCam();
    Eigen::Vector3d Pj = T_SB * P;
    if(Pj(2) < 0.001)
        return false;
    std::cout << "pos_ = \n" << p->pos_ << "\nPj = \n" << Pj  << "\n <<<<<<<<<<<<<<<<<<<<<<<" << std::endl;
    double u = cam->fx() * (Pj(0) / Pj(2)) + cam->cx();
    if(u < 0 || u >= viframe_j->getCVFrame()->getWidth())
        return false;

    double v = cam->fy() * (Pj(1) / Pj(2)) + cam->cy();

    if(v < 0 || v >= viframe_j->getCVFrame()->getHeight())
        return false;

    std::cout << u << "\t" << v;

    for(int i = 0; i < ft->level; ++i) {
        u /= 2.0;
        v /= 2.0;
    }

    if(!viframe_j->getCVFrame()->getGrad(u, v, grad, ft->level))
        return false;

    w = 1.0 / viframe_j->getCVFrame()->getGradNorm(u, v, ft->level);
    if(w < 0.001 || std::isinf(w))
        return false;
    Eigen::Vector2d px = viframe_i->getCam()->world2cam(p->pos_);
    std::cout << "\t"  << px(0) << '\t' << px(1) << "\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n";
    err = viframe_i->getCVFrame()->getIntensityBilinear(px(0), px(1), ft->level)
            - viframe_j->getCVFrame()->getIntensityBilinear(u, v, ft->level);

    if(err>PHOTOMATRICERROR ||err<-PHOTOMATRICERROR)     {
        //        printf("wrong match!\n");
        return false;
    }
    //    else printf("right match!\n");
    //! if err is to large, should I compute this point?

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


Tracker::Tracker() {}

int Tracker::reProject(std::shared_ptr<viFrame> &viframe_i, std::shared_ptr<viFrame> &viframe_j, Sophus::SE3d &T_ji) {
    const cvMeasure::features_t& fts = viframe_i->getCVFrame()->getMeasure().fts_;
    int width = viframe_j->getCVFrame()->getWidth();
    int height = viframe_j->getCVFrame()->getHeight();
    int cellwidth = viframe_j->getCVFrame()->getWidth() / detectCellWidth;
    int chellheight = viframe_j->getCVFrame()->getHeight() / detectCellHeight;
    int cntCell = 0;
    for(auto &ft : fts) {
        Eigen::Vector2d uv = viframe_j->getCam()->world2cam(ft->point->pos_);
        if(uv(0) < width && uv(1) < height && uv(0) > 0 && uv(1) > 0) {
            std::shared_ptr<Feature> ft_ = std::make_shared<Feature>(viframe_j->getCVFrame(),
                                                                     ft->point,uv, ft->point->pos_, ft->level);
            viframe_j->getCVFrame()->addFeature(ft_);
            int u = int(uv(0) / cellwidth);
            int v = int(uv(1) / chellheight);
            if(!viframe_j->getCVFrame()->checkCell(u, v)) {
                viframe_j->getCVFrame()->setCellTrue(u, v);
                cntCell++;
            }
        }
    }

    return cntCell;
}

bool Tracker::Tracking(std::shared_ptr<viFrame> &viframe_i, std::shared_ptr<viFrame> &viframe_j,
                       Sophus::SE3d &T_ji_, int n_iter) {
    Sophus::SE3d T_ji = T_ji_;
    Sophus::SE3d T_ji_new = T_ji;
    bool converge = false;
    int cnt = 0;
    int iter = 0;
    Eigen::Matrix<double, 6, 6> H;
    Eigen::Matrix<double, 6, 1> b;
    const cvMeasure::features_t& fts = viframe_i->getCVFrame()->getMeasure().fts_;
    Jac_t jac;
    int cnt_min = int(fts.size()) / 3;
    Sophus::SE3d T_SB = viframe_j->getT_BS().inverse();
    double w = 0;
    double e = 0;
    Eigen::Matrix<double, 6, 1> xi;
    double chi = 0;
    double chi_new = 0;

    for(; iter < n_iter; iter++) {
        printf("iter = %d:", iter);
        chi_new = 0;
        H.setZero();
        b.setZero();
        cnt = 0;
        jac.setZero();

        int cntf = 0;
        int cntP = 0, cntE = 0;

        for(auto& ft : fts) {
            if(ft->type == Feature::EDGELET) {
                if(!direct_tracker::compute_EdgeJac(viframe_i, viframe_j, ft, T_SB, viframe_i->getPose(), T_ji_new, jac, w, e))
                    continue;
                cnt++; cntE++;
                H += jac.transpose() * jac * w;
                b += jac.transpose() * e * w;
                chi_new += e;

            } else {
                if(!direct_tracker::compute_PointJac(viframe_i, viframe_j, ft, T_SB, viframe_i->getPose(), T_ji_new, jac, w, e))
                    continue;
                cnt++; cntP++;
                H += jac.transpose() * jac * w;
                b += jac.transpose() * e * w;
                chi_new += e;
            }
        }

        std::cout<<"cntP = "<<cntP<<" cntE = "<<cntE;
        if(cnt <= cnt_min && cnt <= 12)
            return false;

        if(iter == 0) {
            chi = chi_new;
            xi = H.ldlt().solve(-b);
            std::cout<<"\n xi = \n"<<xi<<"\n";
            printf("\tinitial err: %lf\n", chi);
            T_ji_new = Sophus::SE3d::exp(xi) * T_ji;
            int cntf = 0;
            continue;
        }

        xi = H.ldlt().solve(-b);
        std::cout<<"\n xi = "<<xi<<"\n";

        printf("\tnew err: %lf\n", chi_new);
        if(chi_new < chi) {
            T_ji = T_ji_new;
            if(chi - chi_new < 1.0e-2) {
                converge = true;
                break;
            }
            chi = chi_new;
            T_ji_new = Sophus::SE3d::exp(xi) * T_ji_new;
        }
        else {
            if(chi_new - chi < 1.0e-2) {
                converge = true;
            }
            break;

        }
    }
    T_ji_ = T_ji;
    return converge;
}

}

