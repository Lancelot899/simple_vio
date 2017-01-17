#include "viFrame.h"
#include "imu/IMUMeasure.h"
#include "cv/cvFrame.h"

#ifndef IMUTYPE_DEF_
#define IMUTYPE_DEF_

typedef IMUMeasure::ImuMeasureDeque ImuMeasureDeque;
typedef IMUMeasure::Transformation  Transformation;
typedef IMUMeasure::SpeedAndBias    SpeedAndBias;
typedef IMUMeasure::covariance_t    covariance_t;
typedef IMUMeasure::jacobian_t      jacobian_t;
typedef IMUMeasure::Error_t         Error_t;
typedef Eigen::Vector3d             speed_t;
typedef Eigen::Matrix<double, 6, 1> bias_t;

#endif // IMUTYPE_DEF_



viFrame::viFrame()
{

}

viFrame::~viFrame()
{

}

const viFrame::pose_t &viFrame::getPose() {
    return dynamic_cast<VIOPinholeCamera*>(cvframe.get())->getT_BS() * cvframe->getPose();
}

const cvframePtr_t &viFrame::getCVFrame() {
    return cvframe;
}

const IMUMeasure::SpeedAndBias &viFrame::getSpeedAndBias() {
    return spbs;
}

const okvis::Time& viFrame::getTimeStamp() {
    return cvframe->getTimestamp();
}

