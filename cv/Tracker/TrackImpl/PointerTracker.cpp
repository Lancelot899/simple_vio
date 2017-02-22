//
// Created by lancelot on 2/20/17.
//
#include <ceres/ceres.h>
#include "DataStructure/cv/cvFrame.h"
#include "PointerTracker.h"

namespace direct_tracker {

struct PhotometricError:public ceres::SizedCostFunction<1, 6>{
    PhotometricError(cvframePtr_t frameI): frame_i(frameI){}

    virtual ~PhotometricError() {}
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const
    {

    }


    template <typename T>
    bool operator ()(Sophus::SE3d translation,
                     cvframePtr_t frame_j,
                     T* residual) const
    {
        double gray_i , gray_j;
        residual[0] = gray_i - gray_j;
        residual[0] *= ( residual[0]>0? (1./gray):(-1./gray) );
        return true;
    }
    const cvframePtr_t  frame_i;
};

PointerTracker::PointerTracker()
{

}

PointerTracker::~PointerTracker()
{

}

bool PointerTracker::tracking(std::shared_ptr<viFrame> &viframe_i, std::shared_ptr<viFrame> &viframe_j, Translation &T_ij)
{
    typedef Sophus::SE3d     pose_t;
    const std::shared_ptr<cvFrame>& cvframe_i = viframe_i->getCVFrame();
    const std::shared_ptr<cvFrame>& cvframe_j = viframe_j->getCVFrame();

    const cvMeasure& measureI = cvframe_i->getMeasure(); const cvMeasure& measureJ = cvframe_j->getMeasure();
    ceres::Problem problem;
    for(auto it : measureI.fts_)
    {
        problem.AddResidualBlock(new ceres::NumericDiffCostFunction<PhotometricError,new ceres::HuberLoss(1.0),,>,
                                 PhotometricError);
    }
}

}
