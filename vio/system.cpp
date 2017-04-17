//
// Created by lancelot on 4/12/17.
//

#include <Eigen/Core>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>


#include "system.h"
#include "IMU/IMU.h"
#include "IO/imu/IMUIO.h"
#include "IO/camera/CameraIO.h"
#include "IO/image/ImageIO.h"
#include "IMU/IMU.h"
#include "DataStructure/cv/Camera/AbstractCamera.h"
#include "DataStructure/cv/Feature.h"
#include "DataStructure/cv/cvFrame.h"
#include "DataStructure/viFrame.h"
#include "DataStructure/imu/imuFactor.h"
#include "cv/FeatureDetector"
#include "cv/Triangulater"
#include "cv/Tracker"
#include "util/util.h"
#include "util/setting.h"
#include "DataStructure/GraphFactor/imuG2OType.h"



system::system(std::string &imuDatafile, std::string &imuParamfile, std::string &camDatafile, std::string &camParamfile,
               std::string &imageFile, std::string &dataDirectory,  const int img_width, const int img_height) {
	std::shared_ptr<CameraIO> camIO = std::make_pair<CameraIO>(camDatafile, camParamfile);
	cam = camIO->getCamera();
	BARunning = false;
	BAResult = true;
	imgIO = std::make_shared<ImageIO>(imageFile, dataDirectory, cam);
	imuIO = std::make_shared<IMUIO>(imuDatafile, imuParamfile);
	detector = std::make_shared<feature_detection::Detector>(img_width, img_width, 25, IMG_LEVEL);
	tracker = std::make_shared<direct_tracker::Tracker>();
	triangulate = std::make_shared<Triangulater>();
	imu = std::make_shared<IMU>();
	initialier = std::make_shared<Initialize>(detector, tracker, triangulater, imu);
	BAThread = std::thread(&system::workLoop, this);
}

void system::workLoop() {
		std::unique_lock<std::mutex> lock(BAMutex);
		while(true) {
			if(BARunning == false)
				callBA.wait(lock);

			BAResult = runBA();
			BARunning = false;
		}
}

bool system::runBA() {
	if(keyFrames.size() < widowSize)
		return false;

	assert(keyFrames.size() == imuFactors.size() + 1);

	typedef g2o::BlockSolver<g2o::BlockSolverTraits<15, 15>> Block15_15;
	typedef g2o::BlockSolver<g2o::BlockSolverTraits<15, 3>> Block6_3;

	Block::LinearSolverType* linearSolver15_15 = new g2o::LinearSolverCholmod<Block15_15::PoseMatrixType>();
	Block* solver_ptr15_15 = new Block15_15 (linearSolver);
	g2o::OptimizationAlgorithmDogleg* solver15_15 = new g2o::OptimizationAlgorithmLevenberg (solver_ptr15_15);

	g2o::SparseOptimizer optimizer;
	optimizer.setAlgorithm (solver15_15);

}

void system::run() {

}
