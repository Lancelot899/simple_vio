#include "cvFrame.h"

cvFrame::cvFrame(std::shared_ptr<AbstractCamera> &cam) {
    cam_ = cam;
}

cvFrame::~cvFrame() {

}

int cvFrame::getID() {
    return cvData.id;
}

double cvFrame::getTimestamp() {
    return cvData.timeStamp;
}

int cvFrame::getSensorID() {
    return cvData.sensorId;
}

const cvFrame::cam_t cvFrame::getCam() {
    return cam_;
}

const cvFrame::cov_t &cvFrame::getCovariance() {
    return Cov_;
}

const pose_t &cvFrame::getPose() {
    return pose_;
}

void cvFrame::setPose(pose_t &pose) {
    pose_ = pose;
}

void cvFrame::setCovariance(cvFrame::cov_t &Cov) {
    Cov_ = Cov;
}

bool cvFrame::isKeyframe() {
    return is_keyframe_ == true;
}

void cvFrame::setKey() {
    is_keyframe_ = true;
}

void cvFrame::cancelKeyframe() {
    is_keyframe_ = false;
}

int cvFrame::getPublishTimestamp() {
    return last_published_ts_;
}

const cvFrame::position_t &cvFrame::pos() {
    return pose_.translation();
}

