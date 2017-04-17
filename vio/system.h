//
// Created by lancelot on 4/12/17.
//

#ifndef SIMPLE_VIO_SYSTEM_H
#define SIMPLE_VIO_SYSTEM_H

#include <string>

#include "Initialize.h"

class ImageIO;
class IMUIO;

class system {
public:
	system(std::string &imuDatafile,
	       std::string &imuParamfile,
	       std::string &camDatafile,
	       std::string &camParamfile,
	       std::string &imageFile,
	       std::string &dataDirectory);

private:
	std::shared_ptr<Initialize> initialier;
	std::shared_ptr<feature_detection::Detector> detector;
	std::shared_ptr<direct_tracker::Tracker> tracker;
	std::shared_ptr<Triangulater> triangulate;
	std::shared_ptr<IMU> imu;
	std::shared_ptr<ImageIO> imgIO;
	std::shared_ptr<IMUIO> imuIO;
};


#endif //SIMPLE_VIO_SYSTEM_H
