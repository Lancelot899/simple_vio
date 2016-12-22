#include "viFrame.h"
#include "IMUMeasure.h"
#include "cvFrame.h"

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
    return cvframe->getPose();
}

const std::shared_ptr<cvFrame> &viFrame::getCVFrame() {
    return cvframe;
}

const IMUMeasure::SpeedAndBias &viFrame::getSpeedAndBias() {
    return spbs;
}

double viFrame::getTimeStamp() {
    return cvframe->getTimestamp();
}
