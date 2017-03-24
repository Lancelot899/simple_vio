#include <glog/logging.h>
#include <ceres/ceres.h>

#include "Triangulater.h"

#include "../DataStructure/viFrame.h"
#include "../DataStructure/cv/cvFrame.h"
#include "../DataStructure/cv/Feature.h"
#include "../DataStructure/cv/Point.h"

class TriangulaterPhotometrixResidual: public ceres::SizedCostFunction<1,1> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    TriangulaterPhotometrixResidual(const std::shared_ptr<cvFrame> keyframe_,
                                    const std::shared_ptr<cvFrame> nextframe_,
                                    const Sophus::SE3d T_nk_,
                                    double u_,double v_)
        :keyFrame(keyframe_),nextFrame(nextframe_),T_nk(T_nk_),u_Key(u_),v_Key(v_)
    {}

    virtual bool Evaluate(double const* const* parameters,double* residuals,double** jacobians) const
    {
        double depth = *parameters[0];
        Eigen::Vector3d posNF = T_nk.rotationMatrix()* nextFrame->getCam()->K_inv()* Eigen::Vector3d(u_Key*depth,v_Key*depth,depth) + T_nk.translation();
        Eigen::Vector3d pos_ = nextFrame->getCam()->K() * posNF;

        double depth2 = pos_(2);
        if(depth2<1e-5) {
            printf("TriangulaterPhotometrixResidual with wrong depth!\n");
            exit(-1);
        }

        double fx = nextFrame->getCam()->fx();  double fy = nextFrame->getCam()->fy();

        Eigen::Matrix<double,2,3> uv_XYZ;
        uv_XYZ<<fx/posNF(2), 0, -fx*pos_(0)/(posNF(2)*posNF(2)), 0, fy/posNF(2), -fy*posNF(1)/(posNF(2)*posNF(2));

        pos_ /= depth2;
        *residuals = keyFrame->getIntensityBilinear(u_Key,v_Key) - nextFrame->getIntensityBilinear(pos_(0),pos_(1));

        *jacobians[0] = (keyFrame->getGradBilinear(u_Key,v_Key).transpose() * uv_XYZ *
                     (T_nk.rotationMatrix() * (nextFrame->getCam()->K_inv() * Eigen::Vector3d(u_Key,v_Key,1)) + T_nk.translation()))(0);
    }

private:
    std::shared_ptr<cvFrame>  keyFrame,nextFrame;
    Sophus::SE3d              T_nk;
    double                    u_Key,v_Key;
};

class CERES_EXPORT DepthParameter : public ceres::LocalParameterization {
public:
    virtual ~DepthParameter() {}
    virtual bool Plus(const double* x,
                      const double* delta,
                      double* x_plus_delta) const
    {
        *x_plus_delta = *x + * delta;
    }
    virtual bool ComputeJacobian(const double* x,
                                 double* jacobian) const
    {
        ceres::MatrixRef(jacobian, 1, 1) = ceres::Matrix::Identity(1, 1);
        return true;
    }
    virtual int GlobalSize() const { return 1; }
    virtual int LocalSize() const { return 1; }
};

struct TriangulaterSolver
{
    TriangulaterSolver()
    {
        m_options.max_num_iterations = 25;
        m_options.linear_solver_type = ceres::SPARSE_SCHUR;
        m_options.trust_region_strategy_type = ceres::DOGLEG;
        m_options.dogleg_type = ceres::SUBSPACE_DOGLEG;
        /// m_options.minimizer_progress_to_stdout = true;
        m_bInitParameters = false;
    }

    bool SolveParameters()
    {
        if(keyFrame == true || keyFrame == true)
            return false;

        ceres::Problem problem;
        double depthInit = 1.0;

        for(auto &ft: fts) {
            problem.AddResidualBlock(new TriangulaterPhotometrixResidual(keyFrame,nextFrame,T_nk,ft->px(0),ft->px(1)),
                                     new ceres::HuberLoss(0.5),depthInit);
        }
        problem.SetParameterization(&depthInit,new DepthParameter);

        ceres::Solve(m_options,&problem,&m_summary);
        return true;
    }

    bool                               m_bInitParameters;
    double                             m_EllipsoidParameters[6];
    std::shared_ptr<cvFrame>           keyFrame,nextFrame;
    cvMeasure::features_t              fts;
    Sophus::SE3d                       T_nk;
    ceres::Solver::Options             m_options;
    ceres::Solver::Summary             m_summary;
};


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
            PointType Pnext(ftKey->point->pos_(0),ftKey->point->pos_(1),depthInit);
            PointType point = nextFrame->getCVFrame()->getCam()->K() * Rotation * Pnext
                    + T_nk.translation();

            if(point(2)==0) break;
            double u = point(0)/point(2);   if(u<0 || u>width_) break;
            double v = point(1)/point(2);   if(v<0 || v>height_) break;

            cvFrame::grad_t  grad;  nextFrame->getCVFrame()->getGrad(u,v,grad);
            Eigen::Vector3d uv_d = nextFrame->getCVFrame()->getCam()->K() * Rotation * Pnext;
            if(uv_d(2)==0) break;
            uv_d /= uv_d(2);

            double Jacob = grad.transpose() * Eigen::Vector2d(uv_d(0),uv_d(1));
            if(Jacob<1e-3) break;

            double err = keyFrame->getCVFrame()->getIntensityBilinear(ftKey->px(0),ftKey->px(1))
                       - nextFrame->getCVFrame()->getIntensityBilinear(u,v);

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
