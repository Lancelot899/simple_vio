#ifndef IMAGEIO_H
#define IMAGEIO_H

#include <deque>
#include <string.h>
#include <opencv2/opencv.hpp>
#include "../IOBase.h"

namespace IO {

}
typedef std::deque<std::pair<double, std::string>> ImageIOData;

class ImageIO : public IOBase<ImageIOData>
{
public:
    ImageIO(std::string &imagefile,std::string cameraParamFile,std::string dataDirectory_);
    std::string popName();
    int parseParamFile(std::string ParamFile);
    cv::Mat  popImage();
    std::pair<double, cv::Mat> popImageAndTimestamp();

private:
    ImageIOData       imageDeque;
    std::string       dataDirectory;
    double            distortion_coefficients[4];
    double            intrinsics[4];
};



#endif
