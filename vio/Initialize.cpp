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
	//std::cout << T.matrix3x4() << std::endl;
    std::shared_ptr<viFrame> viframe = std::make_shared<viFrame>(viFrame::ID++, cvframe, imuParam);
    if(!tracker->Tracking(VecFrames.back(), viframe, T))
	    return;
	//std::cout << T.matrix3x4() << std::endl;
    T = VecFrames.back()->getPose() * T;
	viframe->updatePose(T);
	if(VecFrames.back()->getCVFrame()->cvData.fts_.size() / 4 >
		    triangulater->triangulate(VecFrames.back(), viframe, T, 30))
	    return;

    tracker->reProject(VecFrames.back(), viframe);

	feature_detection::features_t features;
    detector->detect(viframe->getCVFrame(), viframe->getCVFrame()->getMeasure().measurement.imgPyr, features);
	//std::cout << "new feature : " << features.size() << std::endl;
    for(auto &feat : features)
        viframe->getCVFrame()->cvData.fts_.push_back(feat);
	VecFrames.push_back(viframe);
	VecImuFactor.push_back(imufactor);
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


