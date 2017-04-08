//
// Created by lancelot on 4/6/17.
//

#include <QGLViewer/qglviewer.h>

#include "../Initialize.h"
#include "opencv2/ts.hpp"
#include "IO/camera/CameraIO.h"
#include "IO/image/ImageIO.h"
#include "IO/imu/IMUIO.h"
#include "DataStructure/cv/cvFrame.h"
#include "DataStructure/imu/imuFactor.h"
#include "IMU/IMU.h"
#include "DataStructure/cv/Point.h"

class drPoint {
public:
	drPoint(std::shared_ptr<Point>& point_) : point(point_) {}
	void draw() {
		glColor3d(0.3, 0.6, 0.9);
		point->pos_mutex.lock_shared();
		glVertex3d(point->pos_[0] * 100, point->pos_[1] * 100, point->pos_[2] * 100);
		point->pos_mutex.unlock_shared();
	}

private:
	std::shared_ptr<Point> point;
};

class drPose {
public:
	drPose(Sophus::SE3d pose_) : pose(pose_) {}
	void draw() {
		Eigen::Vector3d arrow_ = pose.so3() * arrow;
		glColor3d(0, 0, 1);

	}

private:
	static const Eigen::Vector3d arrow;
private:
	Sophus::SE3d pose;
};

const Eigen::Vector3d drPose::arrow = Eigen::Vector3d(1, 1, 1);

class Viewer : public QGLViewer {
protected :
	virtual void draw();
	virtual void animate();
	void pushPoint(std::shared_ptr<Point> point) {
		drPoints.push_back(drPoint(point));
	}

private:
	std::vector<drPose> drPoses;
	std::vector<drPoint> drPoints;
};

TEST(INITIALIZE, INITIALIZE) {
	std::string imuDatafile("../testData/mav0/imu0/data.csv");
	std::string imuParamfile("../testData/mav0/imu0/sensor.yaml");
	IMUIO imuIO(imuDatafile, imuParamfile);
	IMUIO::pImuParam imuParam = imuIO.getImuParam();
	std::string camDatafile = "../testData/mav0/cam1/data.csv";
	std::string camParamfile = "../testData/mav0/cam1/sensor.yaml";
	CameraIO camIO(camDatafile, camParamfile);
	std::shared_ptr<AbstractCamera> cam = camIO.getCamera();
	std::string imageFile = "../testData/mav0/cam0/data.csv";
	ImageIO imageIO(imageFile, "../testData/mav0/cam0/data/", cam);


}