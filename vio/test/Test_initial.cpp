//
// Created by lancelot on 4/6/17.
//
#include <qt4/QtGui/QApplication>
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
		Eigen::Matrix3d rot = pose.so3().matrix();
		Eigen::Vector3d p1 = pose.translation();
		Eigen::Vector3d p2 = p1 + rot.block<3, 1>(0, 1);
		Eigen::Vector3d p3 = p1 + rot.block<3, 1>(0, 2);
		p1 += rot.block<3, 1>(0, 0);
		glColor3d(0, 0, 1);
		glVertex3d(p1[0], p1[1], p1[2]);
		glColor3d(1, 0, 1);
		glVertex3d(p2[0], p2[1], p2[2]);
		glVertex3d(p3[0], p3[1], p3[2]);
	}

private:
	Sophus::SE3d pose;
};

class Viewer : public QGLViewer {
public:
	Viewer() {}
	~Viewer() {}
	virtual void draw() {}
	virtual void animate() {}
	void pushPoint(std::shared_ptr<Point> point) {
		drPoints.push_back(drPoint(point));
	}

	void pushPose(std::shared_ptr<viFrame> viframe) {
		drPoses.push_back(drPose(viframe->getPose()));
	}

private:
	std::vector<drPose> drPoses;
	std::vector<drPoint> drPoints;
};

void testInitial(int argc, char **argv) {
	QApplication app(argc, argv);
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
	Viewer viewer;



	viewer.show();
	app.exec();
}