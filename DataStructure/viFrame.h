#ifndef VIFRAME_H
#define VIFRAME_H

#include <memory>
#include <boost/noncopyable.hpp>

#include "DataStructure/cv/Camera/AbstractCamera.h"
#include "imu/IMUMeasure.h"

class cvFrame;
class imuFactor;


class viFrame : boost::noncopyable {
public:
    friend class InitialImpl;

public:
    typedef std::shared_ptr<imuFactor>        linked_t;
    typedef std::shared_ptr<viFrame>          imuConnection_t;
    typedef std::shared_ptr<ImuParameters>    ImuParam;
    typedef Sophus::SE3d                      pose_t;
    typedef std::shared_ptr<AbstractCamera>   cam_t;

public:
    viFrame(int id, std::shared_ptr<cvFrame>& cvframe);
    ~viFrame();
    int getID() {
        return id;
    }



    pose_t getT_BS();
    pose_t getPose();
    const std::shared_ptr<cvFrame>& getCVFrame();
    const IMUMeasure::SpeedAndBias &getSpeedAndBias();
    const ImuParam& getImuParam() {
        return imuParam;
    }

    const cam_t& getCam();
    const okvis::Time &getTimeStamp();

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
