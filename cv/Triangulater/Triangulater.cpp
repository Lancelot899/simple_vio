#include <ceres/ceres.h>
#include "Triangulater.h"

#include "../DataStructure/viFrame.h"
#include "../DataStructure/cv/cvFrame.h"
#include "../DataStructure/cv/Feature.h"
#include "../DataStructure/cv/Point.h"


class depthErr : public ceres::SizedCostFunction<1, 1> {
public:
    virtual ~depthErr() {}
    depthErr(std::shared_ptr<viFrame> nextFrame, double I_k,
             const Sophus::SE3d &T_nk, const cvMeasure::features_t::value_type &f)
            : T_nk_(nextFrame->getT_BS().inverse() * T_nk * nextFrame->getT_BS()), f_(f) {
        this->nextFrame = nextFrame;
        this->I_k = I_k;
    }

    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const {
        double d = **parameters;
        Eigen::Vector3d P_next = T_nk_ * (d * f_->f);
        auto &cam = nextFrame->getCam();
        auto uv = cam->world2cam(P_next);
        for(int i = 0; i < f_->level; ++i) {
            uv /= 2.0;
        }
        *residuals = nextFrame->getCVFrame()->getIntensityBilinear(uv(0), uv(1), f_->level) - I_k;
        if (jacobians && jacobians[0]) {
            Eigen::Vector2d grad = nextFrame->getCVFrame()->getGradBilinear(uv(0), uv(1), f_->level);
            double Ix, Iy;
            if (f_->type == Feature::EDGELET) {
                Eigen::Vector2d &dir = f_->grad;
                Ix = dir(1) * dir(0);
                Iy = Ix * grad(0) + dir(1) * dir(1) * grad(1);
                Ix *= grad(1);
                Ix += dir(0) * dir(0) * grad(0);
            } else {
                Ix = grad(0);
                Iy = grad(1);
            }

            Eigen::Matrix<double, 1, 3> Jac;
            Jac(0, 0) = Ix * cam->fx() / P_next(2);
            Jac(0, 1) = Iy * cam->fy() / P_next(2);
            Jac(0, 2) = -Ix * cam->fx()  * P_next(0) / P_next(2) / P_next(2) -
                        Iy * cam->fy() * P_next(1) / P_next(2) / P_next(2);
            Jac =  Jac * T_nk_.rotationMatrix();
            jacobians[0][0] = Jac * f_->f;
        }


    }

private:
    const Sophus::SE3d T_nk_;
    const cvMeasure::features_t::value_type f_;
    std::shared_ptr<viFrame> nextFrame;
    double I_k;
};

Triangulater::Triangulater()
{}

bool Triangulater::triangulate(std::shared_ptr<viFrame> &keyFrame,
                               std::shared_ptr<viFrame> &nextFrame, const Sophus::SE3d &T_nk)
{
    typedef Eigen::Vector3d     PointType;

    int height_ = keyFrame->getCVFrame()->getHeight();
    int width_  = keyFrame->getCVFrame()->getWidth();

    const cvMeasure::features_t& fts = keyFrame->getCVFrame()->getMeasure().fts_;
    for(auto& ftKey : fts) {
        if(ftKey->point->pos_[2]!=1) continue;    //! Already had depth
        Eigen::Matrix<double, 3, 3> Rotation       = T_nk.rotationMatrix();

        ///< search in the epipoleline on next frame:  menthod: GN
        double depthInit = 1.0; int iter = 12;
        bool foundGoodDepth = false; double old_err = 99999;
        while (iter--) {
            PointType point = nextFrame->getCVFrame()->getCam()->K() * Rotation *
                    PointType(ftKey->point->pos_(0),ftKey->point->pos_(1),depthInit) + T_nk.translation();

            if(point(2)==0) break;
            double u = point(0)/point(2);   if(u<0 || u>width_) break;
            double v = point(1)/point(2);   if(v<0 || v>height_) break;

            cvFrame::grad_t  grad;  nextFrame->getCVFrame()->getGrad(u,v,grad);
            Eigen::Vector3d uv_d = nextFrame->getCVFrame()->getCam()->K() * Rotation * ftKey->point->pos_;
            if(uv_d(2)==0) break;
            uv_d /= uv_d(2);

            double Jacob = grad.transpose() * Eigen::Vector2d(uv_d(0),uv_d(1));
            if(Jacob<1e-3) break;

            double err = keyFrame->getCVFrame()->getIntensity(ftKey->px(0),ftKey->px(1)) - nextFrame->getCVFrame()->getIntensity(u,v);
            err = err>0?err:-err;
            if(err<old_err) old_err = err;
            else {
                foundGoodDepth = true;
                break;
            }

            double H = Jacob * Jacob;
            double b = Jacob * err;
            double deltaDepth = -b/H;
            depthInit += deltaDepth;
        }   // end of while
    }   //end of auto ftKey: fts

#ifdef ORB_TRIANGULAR
        Eigen::Matrix<double, 3, 3> Rotation       = T_nk.rotationMatrix();
        Eigen::Matrix<double, 3, 3> translationHat = Sophus::SO3d::hat(T_nk.translation());
        Eigen::Matrix<double, 1, 3> epipoleLine    = ftKey->point->pos_.transpose() * translationHat * Rotation;

        double &a = epipoleLine(0,0); double &b = epipoleLine(0,1); double &c = epipoleLine(0,2);

        double denominator = sqrt(a*a + b*b);
        if(denominator < 1e-4) continue;
        denominator = 1.0/denominator;

        double    yBoundary,xBoundary;
        if(b && a){
            double y_x0 = -c/b;
            double x_y0 = -c/a;
            double y_xMax = (-c+a*width_)/b;
            double x_yMax = (-c+b*height_)/a;
            xBoundary =
        }


        for (int v = 0; v < height_; ++v) {
            for (int u = 0; u < width_; ++u) {
                double temp_ = a*u + b*v + c ;
                double factorA = temp_ + epilolineThreshold*denominator;
                double factorB = temp_ - epilolineThreshold*denominator;
                if(factorA*factorB>0) continue;  //!<  far away from the epiloline, discard
                cvFrame::grad_t  grad;  nextFrame->getCVFrame()->getGrad(u,v,grad);
                Eigen::Vector3d uv_d = nextFrame->getCVFrame()->getCam()->K() * Rotation * ftKey->point->pos_;
                uv_d /= uv_d(2);
                double Jacob = grad.transpose() * Eigen::Vector2d(uv_d(0),uv_d(1));
                double err = keyFrame->getCVFrame()->getIntensity(ftKey->px(0),ftKey->px(1)) - nextFrame->getCVFrame()->getIntensity(u,v) ;
                double H = Jacob * Jacob;
                double b = Jacob * err;

            }
        }

        double numerator = ;
        double distance = ;
#endif

}
