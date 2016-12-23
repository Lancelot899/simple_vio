#include "cvFrame.h"

cvFrame::cvFrame(std::shared_ptr<AbstractCamera> &cam)
{
    cam_ = cam;
}

cvFrame::~cvFrame()
{

}

