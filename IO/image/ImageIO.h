#ifndef IMAGEIO_H
#define IMAGEIO_H

#include <deque>
#include <string.h>
#include <opencv2/opencv.hpp>
#include "../IOBase.h"

class AbstractCamera;

typedef std::deque<std::pair<double, std::string>> ImageIOData;

class ImageIO : public IOBase<ImageIOData>
{
public:
    ImageIO(std::string &imagefile, std::string dataDirectory_);
	ImageIO(std::string &imagefile, std::string dataDirectory_,
	        std::shared_ptr<AbstractCamera> cam);

    std::string popName();
    cv::Mat  popImage();
    std::pair<double, cv::Mat> popImageAndTimestamp();

private:
    ImageIOData       imageDeque;
    std::string       dataDirectory;
//    double            distortion_coefficients[4];
//    double            intrinsics[4];
	bool isUndistortion;
	std::shared_ptr<AbstractCamera> cam_;
};



#endif
