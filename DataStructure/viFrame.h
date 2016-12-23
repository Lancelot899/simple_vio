#ifndef VIFRAME_H
#define VIFRAME_H

#include <memory>

#include "imu/IMUMeasure.h"

class cvFrame;
class imuFactor;

class viFrame
{
public:

    typedef std::shared_ptr<imuFactor>        linked_t;
    typedef std::shared_ptr<viFrame>          imuConnection_t;
    typedef std::shared_ptr<ImuParameters>    ImuParam;
    typedef Sophus::SE3d                      pose_t;

public:
    viFrame();
    ~viFrame();
    int getID() {
        return id;
    }

    const pose_t& getPose();
    const std::shared_ptr<cvFrame>& getCVFrame();
    const IMUMeasure::SpeedAndBias &getSpeedAndBias();
    const ImuParam& getImuParam() {
        return imuParam;
    }

    double getTimeStamp();

private:
    int                      id;
    std::shared_ptr<cvFrame> cvframe;
    IMUMeasure::SpeedAndBias spbs;
    imuConnection_t          from;
    imuConnection_t          to;
    linked_t                 from_link;
    linked_t                 to_link;
    ImuParam                 imuParam;
};



#endif // VIFRAME_H
