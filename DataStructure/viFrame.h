#ifndef VIFRAME_H
#define VIFRAME_H

#include <memory>

#include "IMUMeasure.h"

class cvFrame;
class imuFactor;

class viFrame
{
public:
    typedef std::shared_ptr<imuFactor> linked_t;
    typedef std::shared_ptr<viFrame>   imuConnection_t;
    typedef std::shared_ptr<ImuParamenters>   ImuParam;

public:
    viFrame();
    ~viFrame();
    int getID() {
        return id;
    }

    const std::shared_ptr<cvFrame>& getCVFrame();
    const IMUMeasure::SpeedAndBias &getSpeedAndBias();
    const ImuParam& getImuParam() {
        return imuParam;
    }

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
