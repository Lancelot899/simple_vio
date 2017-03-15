#include "Triangulater.h"

#include "../DataStructure/viFrame.h"
#include "../DataStructure/cv/cvFrame.h"
#include "../DataStructure/cv/Feature.h"
#include "../DataStructure/cv/Point.h"

Triangulater::Triangulater()
{}

bool Triangulater::triangulate(std::shared_ptr<viFrame> &keyFrame, std::shared_ptr<viFrame> &nextFrame, Sophus::SE3d T_nk)
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
