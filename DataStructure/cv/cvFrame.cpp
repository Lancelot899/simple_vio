#include "cvFrame.h"

const cvMeasure& cvFrame::getMeasure() {
    return cvData;
}

bool cvFrame::checkCellOccupy(int u, int v, int level)
{
    return occupy[cellNumbel[level] + v*cellRowNumbel[level] + u];
}

bool cvFrame::setCellOccupy(int u, int v, int level, bool occupied)
{
    occupy[cellNumbel[level] + v*cellRowNumbel[level] + u] = occupied;
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

    if(u >= cols || v >= rows)
        return -1.0;

    return cvData.measurement.imgPyr[level][v * cols + u][0];
}

bool cvFrame::getGrad(int u, int v, cvFrame::grad_t&  out, int level) {
    if(u < 0 || v < 0 || level > IMG_LEVEL)
        return false;

    int rows = cvData.measurement.height[level];
    int cols = cvData.measurement.width[level];

    if(u >= cols || v >= rows)
        return false;

    out =  cvData.measurement.imgPyr[level][v * cols + u].segment<2>(1);
    return true;
}

cvFrame::cvFrame(const std::shared_ptr<AbstractCamera> &cam, Pic_t &pic) {
    cam_ = cam;
    cvData.measurement.pic = pic;
    int rows = pic.rows;
    int cols = pic.cols;

    // 341 = 1 + 4 + 16 + 64 + 256 !>> cell's numbel for each level
    // for a point(u,v) in cell(on the l level): (u,v,l) , occupy[(4^l-1)/3 + v*2^l + u]
    occupy.assign(341, false);
    for(int i = 0; i < IMG_LEVEL; ++i) {
        if(i != 0) {
            rows /= 2;
            cols /= 2;
        }

        cvData.measurement.width[i]  = cols;
        cvData.measurement.height[i] = rows;
        cvData.measurement.imgPyr[i].assign(cvData.measurement.width[i] * cvData.measurement.height[i], Eigen::Vector3d::Zero());
        cvData.measurement.gradNormPyr[i].assign(cvData.measurement.width[i] * cvData.measurement.height[i], 0.0);

        if(i == 0) {
            for(int p = 0; p < rows; p++) {
                for(int q = 0; q < cols; q++) {
                    cvData.measurement.imgPyr[i][p * cols + q][0] = double(pic.at<u_char>(p, q));
                }
            }
        }

        else {
            for(int p = 0; p < rows; p++) {
                for(int q = 0; q < cols; q++)
                    cvData.measurement.imgPyr[i][p * cols + q][0]
                            = 0.25 * (  cvData.measurement.imgPyr[i - 1][p * cols * 4          + q * 2][0]
                                      + cvData.measurement.imgPyr[i - 1][p * cols * 4          + q * 2 + 1][0]
                                      + cvData.measurement.imgPyr[i - 1][p * cols * 4 + 2*cols + q * 2 + 1][0]
                                      + cvData.measurement.imgPyr[i - 1][p * cols * 4 + 2*cols + q * 2][0]);
            }
        }

        for(int p = 1; p < rows - 1; ++p) {
            for (int q = 1; q < cols - 1; ++q) {
                cvData.measurement.imgPyr[i][p * cols + q][1]
                        = 0.5 * (cvData.measurement.imgPyr[i][p * cols + q + 1][0]
                               - cvData.measurement.imgPyr[i][p * cols + q - 1][0]);
                cvData.measurement.imgPyr[i][p * cols + q][2]
                        = 0.5 * (cvData.measurement.imgPyr[i][p * cols + q + cols][0]
                               - cvData.measurement.imgPyr[i][p * cols + q - cols][0]);

                cvData.measurement.gradNormPyr[i][p * cols + q]
                        = std::sqrt(cvData.measurement.imgPyr[i][p * cols + q][1] * cvData.measurement.imgPyr[i][p * cols + q][1]
                                  + cvData.measurement.imgPyr[i][p * cols + q][2] * cvData.measurement.imgPyr[i][p * cols + q][2]);

            }
        }
    }
}

double cvFrame::getGradNorm(int u, int v, int level) {
    if(u < 0 || v < 0 || level > IMG_LEVEL)
        return -1.0;

    int rows = cvData.measurement.height[level];
    int cols = cvData.measurement.width[level];

    if(u >= cols || v >= rows)
        return -1.0;

    return cvData.measurement.gradNormPyr[level][v * cols + u];
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

