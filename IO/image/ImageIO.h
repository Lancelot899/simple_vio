#ifndef IMAGEIO_H
#define IMAGEIO_H

#include <deque>
#include <string.h>
#include <opencv2/opencv.hpp>
#include "../IOBase.h"


typedef std::deque<std::string> ImageIOData;

class ImageIO : public IOBase<ImageIOData>
{
public:
    ImageIO(std::string &imagefile,std::string dataDirectory_);
    std::string popName();
    cv::Mat  popImage();

private:
    ImageIOData       imageDeque;
    ImageIOData       timestampDeque;
    std::string       dataDirectory;
};



#endif
