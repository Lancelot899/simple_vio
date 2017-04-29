//
// Created by lancelot on 4/12/17.
//

#ifndef SIMPLE_VIO_SYSTEM_H
#define SIMPLE_VIO_SYSTEM_H

#include <string>
#include <mutex>
#include <thread>
#include <condition_variable>
#include <atomic>

#include "Initialize.h"

class Point;
class ImageIO;
class IMUIO;
class AbstractCamera;
class BundleAdjustemt;

namespace vio {


class system {
public:
	system(std::string &imuDatafile,
	       std::string &imuParamfile,
	       std::string &camDatafile,
	       std::string &camParamfile,
	       std::string &imageFile,
	       std::string &dataDirectory,
	       const int img_width,
	       const int img_height);

	void run();

private:
	void workLoop();
	bool runBA();
    bool isInsertKeyframe(int num,Sophus::SE3d T);


private:
	int id;
	std::shared_ptr<Initialize> initialier;
	std::shared_ptr<BundleAdjustemt> BA;
	std::shared_ptr<feature_detection::Detector> detector;
	std::shared_ptr<direct_tracker::Tracker> tracker;
	std::shared_ptr<Triangulater> triangulater;
	std::shared_ptr<IMU> imu;
	std::shared_ptr<ImageIO> imgIO;
	std::shared_ptr<IMUIO> imuIO;
	std::shared_ptr<AbstractCamera> cam;
	std::shared_ptr<ImuParameters> imuParam;
	std::vector<std::shared_ptr<viFrame>> keyFrames;
	std::vector<std::shared_ptr<imuFactor>> imuFactors;
	std::thread BAThread;
	std::mutex BAMutex;
	std::condition_variable callBA;
	std::atomic_bool BARunning;
	std::atomic_bool BAResult;
    std::deque<std::shared_ptr<viFrame>> QueKeyFrames;
    bool      isInsert;
    std::shared_ptr<viFrame>   curframe;
};
}


#endif //SIMPLE_VIO_SYSTEM_H
