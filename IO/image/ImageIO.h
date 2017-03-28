#ifndef IMAGEIO_H
#define IMAGEIO_H

#include <deque>
#include <string.h>
#include "../IOBase.h"


typedef std::deque<std::string> ImageIOData;

class ImageIO : public IOBase<ImageIOData>
{
public:
    ImageIO(std::string &imagefile);
    std::string pop();

private:
    ImageIOData       imageDeque;
    ImageIOData       timestampDeque;
};



#endif
