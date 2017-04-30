//
// Created by lancelot on 1/15/17.
//

#include "Initialize.h"
#include "Implement/InitialImpl.h"
#include "DataStructure/cv/cvFrame.h"
#include "cv/FeatureDetector/Detector.h"
#include "DataStructure/imu/imuFactor.h"
#include "cv/Tracker/Tracker.h"
#include "cv/Triangulater/Triangulater.h"

Initialize::Initialize(std::shared_ptr<feature_detection::Detector>& detector,
                       std::shared_ptr<direct_tracker::Tracker>& tracker,
                       std::shared_ptr<Triangulater> &triangulater,
                       std::shared_ptr<IMU> &imu) {
    impl_ = std::make_shared<InitialImpl>();
    isInitialed = false;
    this->detector = detector;
    this->tracker = tracker;
    this->triangulater = triangulater;
	this->imu  = imu;
}

void Initialize::setFirstFrame(std::shared_ptr<cvFrame> &cvframe, std::shared_ptr<ImuParameters> imuParam) {
    std::shared_ptr<viFrame> firstFrame = std::make_shared<viFrame>(viFrame::ID++, cvframe, imuParam);
    Sophus::SE3d ie;
    cvframe->setPose(ie);
    feature_detection::features_t features;
    detector->detect(cvframe, cvframe->getMeasure().measurement.imgPyr, features);
    cvframe->cvData.fts_.swap(features);
    VecFrames.push_back(firstFrame);
}

void Initialize::pushcvFrame(std::shared_ptr<cvFrame> &cvframe,
                             std::shared_ptr<imuFactor> &imufactor,
                             std::shared_ptr<ImuParameters> imuParam) {
    Sophus::SE3d T = imufactor->getPoseFac();
	//std::cout << "imu data pose = \n" << T.matrix3x4() << std::endl;

    std::shared_ptr<viFrame> viframe = std::make_shared<viFrame>(viFrame::ID++, cvframe, imuParam);
	Eigen::Matrix<double, 6, 6> information = Eigen::Matrix<double, 6, 6>::Identity();
//	if(VecFrames.size() == 1) {
		int pointNum = triangulater->triangulate(VecFrames.back(), viframe, T, information, 100);
		std::cout << "initial point num = " << pointNum << std::endl;
//	}

    if(!tracker->Tracking(VecFrames.back(), viframe, T, information)) {
	    std::cout << "now the fts of last frame is : " << VecFrames.back()->getCVFrame()->cvData.fts_.size() << "\n";
	//    return;
    }
	//std::cout << "delta pose = \n" << T.matrix3x4() << std::endl;
	//std::cout << "pose = \n" << T.matrix3x4() << std::endl;
	Sophus::SE3d approxT = VecFrames.back()->getPose() * T;
	viframe->updatePose(approxT);
//	int ptNum = triangulater->triangulate(VecFrames.back(), viframe, T, information ,100);
//	if((VecFrames.back()->getCVFrame()->cvData.fts_.size() / 10 > ptNum) || ptNum < 30) {
//		std::cout << "the size of fts is : " << VecFrames.back()->getCVFrame()->cvData.fts_.size()
//		          << "\nnew point is to less! the number of them is : " << ptNum << std::endl;
//		return;
//	}
//	std::cout << "refine or get point : " << ptNum << std::endl;
    tracker->reProject(VecFrames.back(), viframe, T, information);
	std::cout << "get new obs : " << viframe->getCVFrame()->cvData.fts_.size() << std::endl;
	feature_detection::features_t features;
    detector->detect(viframe->getCVFrame(), viframe->getCVFrame()->getMeasure().measurement.imgPyr, features);
	std::cout << "new feature : " << features.size() << std::endl;
    for(auto &feat : features)
        viframe->getCVFrame()->cvData.fts_.push_back(feat);
	VecFrames.push_back(viframe);
	VecImuFactor.push_back(imufactor);
	std::cout << "add a frame!\n\n";
}

bool Initialize::init(std::shared_ptr<ImuParameters> &imuParam, int n_iter) {
    if(VecFrames.size() < 3)
        return false;
    return this->initImu(imuParam, n_iter);
}

bool Initialize::initImu(std::shared_ptr<ImuParameters> &imuParam, int n_iter) {
    isInitialed = impl_->init(VecFrames, VecImuFactor, imuParam, n_iter);
    return isInitialed;
}


