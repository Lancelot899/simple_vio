#ifndef CAMERAIO_H
#define CAMERAIO_H
#include <deque>
#include <utility>
#include <memory>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>

#include "../IOBase.h"
#include "../../DataStructure/cv/cvFrame.h"

class CameraIO : public IOBase<cvMeasure> {
public:
    typedef std::shared_ptr<AbstractCamera>        pCamereParam;
    typedef std::deque<std::pair<double, std::string> >        CameraData;
public:
    CameraIO(std::string imageFile, std::string cameraParamfile);
    CameraIO(int device, std::string cameraParamfile);

    ///  for device
    /// \param device
    /// \return
    int getNextFrame(int device);
    ///  for dataset
    /// \param timestamp
    /// \return
    std::string getNextFrame(double &timestamp);

    const pCamereParam& getCamera();

/*
    pose_t          getTBS(void) {return  camParam->getTBS();}
    int             getRate(void) {return  camParam->getRate();}
    std::string     getCameraMode(void) {return  camParam->getCameraMode();}
    std::string     getDistorMode(void) {return  camParam->getDistorMode();}
*/
private:
    int parseParamFile(const std::string &cameraParamfile);
    int getDataSet(const std::string &imageFile);

    pCamereParam        camParam;
    CameraData          camData;
};

#endif // CAMERAIO_H
