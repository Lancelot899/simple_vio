#ifndef CAMERAIO_H
#define CAMERAIO_H
#include <map>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include "../IOBase.h"
#include "DataStructure/cv/cvFrame.h"

class CameraIO : public IOBase<cvMeasure>
{
    typedef std::shared_ptr<AbstractCamera>   pCamereParam;
    typedef std::map<double,cv::Mat>          CameraData;
public:
    CameraIO();


protected:
    pCamereParam        camParam;
    CameraData          camData;
};

#endif // CAMERAIO_H
