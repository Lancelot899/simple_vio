//
// Created by lancelot on 4/12/17.
//

#include "system.h"
#include "IMU/IMU.h"
#include "IO/imu/IMUIO.h"
#include "IO/camera/CameraIO.h"
#include "IO/image/ImageIO.h"
#include "IMU/IMU.h"
#include "DataStructure/cv/Camera"
#include "DataStructure/cv/Feature.h"
#include "DataStructure/cv/cvFrame.h"
#include "DataStructure/viFrame.h"
#include "DataStructure/imu/imuFactor.h"
#include "cv/FeatureDetector"
#include "cv/Triangulater"
#include "cv/Tracker"

system::system(std::string &imuDatafile, std::string &imuParamfile, std::string &camDatafile, std::string &camParamfile,
               std::string &imageFile, std::string &dataDirectory) {
	std::shared_ptr<CameraIO> camIO = std::make_pair<CameraIO>(camDatafile, camParamfile);
}