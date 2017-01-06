#include "cvFrame.h"

const cvMeasure& cvFrame::getMeasure() {
    return cvData;
}

const cvFrame::Pic_t& cvFrame::getPicture() {
    return cvData.measurement.pic;
}

int cvFrame::getWidth(int level) {
    return cvData.measurement.width[level];
}

int cvFrame::getHeight(int level) {
    return cvData.measurement.height[level];
}

double cvFrame::getIntensity(int u, int v, int level) {
    if(u < 0 || v < 0 || level > IMG_LEVEL)
        return -1.0;

    int rows = cvData.measurement.height[level];
    int cols = cvData.measurement.width[level];

    if(u >= rows || v >= cols)
        return -1.0;

    return cvData.measurement.imgPyr[level][v * rows + u][0];
}

bool cvFrame::getGrad(int u, int v, cvFrame::grad_t&  out, int level) {
    if(u < 0 || v < 0 || level > IMG_LEVEL)
        return false;

    int rows = cvData.measurement.height[level];
    int cols = cvData.measurement.width[level];

    if(u >= rows || v >= cols)
        return false;

    out =  cvData.measurement.imgPyr[level][v * rows + u].segment<2>(1);
    return true;
}

cvFrame::cvFrame(const std::shared_ptr<AbstractCamera> &cam, Pic_t &pic) {
    cam_ = cam;
    cvData.measurement.pic = pic;
    int rows = pic.rows;
    int cols = pic.cols;

    for(int i = 0; i < IMG_LEVEL; ++i) {
        if(i != 0) {
            rows /= 2;
            cols /= 2;
        }

        cvData.measurement.width[i]  = rows;
        cvData.measurement.height[i] = cols;
        cvData.measurement.imgPyr[i].assign(cvData.measurement.width[i] * cvData.measurement.height[i], Eigen::Vector3d::Zero());
        cvData.measurement.gradNormPyr[i].assign(cvData.measurement.width[i] * cvData.measurement.height[i], 0.0);

        if(i == 0) {
            for(int p = 0; p < rows; p++) {
                for(int q = 0; q < cols; q++) {
                    cvData.measurement.imgPyr[i][q * rows + p][0] = double(pic.at<u_char>(p, q));
                }
            }
        }

        else {
            for(int p = 0; p < rows; p++) {
                for(int q = 0; q < cols; q++)
                    cvData.measurement.imgPyr[i][q * rows + p][0]
                            = 0.25 * (cvData.measurement.imgPyr[i - 1][p * 2 + (q * 2) * rows * 2][0]
                                      + cvData.measurement.imgPyr[i - 1][p * 2 + (q * 2 + 1) * rows * 2][0]
                                      + cvData.measurement.imgPyr[i - 1][p * 2 + 1 + (q * 2 + 1) * rows * 2][0]
                                       + cvData.measurement.imgPyr[i - 1][p * 2 + 1 + (q * 2) * rows * 2][0]);
            }
        }

        for(int p = 1; p < rows - 1; ++p) {
            for (int q = 1; q < cols - 1; ++q) {
                cvData.measurement.imgPyr[i][q * rows + p][1]
                        = 0.5 * (cvData.measurement.imgPyr[i][q * rows + p + 1][0]
                                 - cvData.measurement.imgPyr[i][q * rows + p - 1][0]);
                cvData.measurement.imgPyr[i][q * rows + p][2]
                        = 0.5 * (cvData.measurement.imgPyr[i][q * rows + p + rows][0]
                                 - cvData.measurement.imgPyr[i][q * rows + p - rows][0]);

                cvData.measurement.gradNormPyr[i][q * rows + p]
                        = std::sqrt(cvData.measurement.imgPyr[i][q * rows + p][1] * cvData.measurement.imgPyr[i][q * rows + p][1]
                                  + cvData.measurement.imgPyr[i][q * rows + p][2] * cvData.measurement.imgPyr[i][q * rows + p][2]);

            }
        }
    }
}

double cvFrame::getGradNorm(int u, int v, int level) {
    if(u < 0 || v < 0 || level > IMG_LEVEL)
        return -1.0;

    int rows = cvData.measurement.height[level];
    int cols = cvData.measurement.width[level];

    if(u >= rows || v >= cols)
        return -1.0;

    return cvData.measurement.gradNormPyr[level][v * rows + u];
}

cvFrame::~cvFrame() {

}

int cvFrame::getID() {
    return cvData.id;
}

const okvis::Time &cvFrame::getTimestamp() {
    return cvData.timeStamp;
}

int cvFrame::getSensorID() {
    return cvData.sensorId;
}

const cvFrame::cam_t& cvFrame::getCam() {
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

