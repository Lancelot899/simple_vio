#include "viFrame.h"
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


void viFrame::updatePose(Sophus::SE3d &pose) {
    auto se3 = getT_BS().inverse() * pose;
    cvframe->setPose(se3);
}

viFrame::viFrame(int id, std::shared_ptr<cvFrame>& cvframe) {
    this->id = id;
    this->cvframe = cvframe;
    spbs.setZero();
}

const viFrame::cam_t& viFrame::getCam() {
    return cvframe->getCam();
}

viFrame::~viFrame()
{
}

viFrame::pose_t viFrame::getT_BS() {
    return cvframe->getCam()->getT_BS();
}

int viFrame::ID = 0;

viFrame::pose_t viFrame::getPose() {
    return  cvframe->getCam()->getT_BS() * cvframe->getPose();
}

cvframePtr_t &viFrame::getCVFrame() {
    return cvframe;
}

const IMUMeasure::SpeedAndBias &viFrame::getSpeedAndBias() {
    return spbs;
}

const okvis::Time& viFrame::getTimeStamp() {
    return cvframe->getTimestamp();
}

