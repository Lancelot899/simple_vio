#ifndef viFrame_H
#define viFrame_H

#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

#include "util/setting.h"
#include "DataStructure/Measurements.h"

typedef Sophus::SE3d pose_t;

struct cvData {
    cv::Mat img[IMG_LEVEL];
};


struct cvMeasure : public MeasurementBase<cvData> {
    typedef cv::Mat     Img_t;
    int id;
};

class cvFrame
{
public:
    cvFrame();
    ~cvFrame();

    void setPose(pose_t pose) {
        this->pose = pose;
    }

    const pose_t getPose() {
        return pose;
    }

    int getID() {
        return cvData.id;
    }

    double getTimestamp() {
        return cvData.timeStamp;
    }

    int getSensorID() {
        return cvData.sensorId;
    }

private:
    cvMeasure  cvData;
    pose_t     pose;
};

#endif // viFrame_H
