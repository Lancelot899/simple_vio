#include <iostream>

#include <opencv2/ts/ts.hpp>

#include "DataStructure/cv/cvFrame.h"
#include "DataStructure/viFrame.h"
#include "IO/imu/IMUIO.h"
#include "IO/image/ImageIO.h"
#include "IO/camera/CameraIO.h"
#include "cv/Tracker/Tracker.h"
#include "IMU/IMU.h"
#include "cv/FeatureDetector/Detector.h"
#include "util/util.h"

#ifndef IMUTYPE_DEF_
#define IMUTYPE_DEF_

typedef IMUMeasure::ImuMeasureDeque ImuMeasureDeque;
typedef IMUMeasure::Transformation  Transformation;
typedef IMUMeasure::SpeedAndBias    SpeedAndBias;
typedef IMUMeasure::covariance_t    covariance_t;
typedef IMUMeasure::jacobian_t      jacobian_t;
typedef IMUMeasure::Error_t         Error_t;
typedef Eigen::Vector3d             speed_t;
typedef Eigen::Matrix<double, 6, 1> bias_t;

#endif // IMUTYPE_DEF_


TEST(test_ImuPRE, IMU_PRE) {
//	std::string imuDatafile("../testData/mav0/imu0/data.csv");
//	std::string imuParamfile("../testData/mav0/imu0/sensor.yaml");
//	IMUIO imuIO(imuDatafile, imuParamfile);
//	IMUIO::pImuParam imuParam = imuIO.getImuParam();
//	std::string camDatafile = "../testData/mav0/cam1/data.csv";
//	std::string camParamfile = "../testData/mav0/cam1/sensor.yaml";
//	CameraIO camIO(camDatafile, camParamfile);
//	std::shared_ptr<AbstractCamera> cam = camIO.getCamera();
//	std::string imageFile = "../testData/mav0/cam0/data.csv";
//	ImageIO imageIO(imageFile, "../testData/mav0/cam0/data/", cam);

//    auto time_image1 = imageIO.popImageAndTimestamp();
//    time_image1 = imageIO.popImageAndTimestamp();
//    time_image1 = imageIO.popImageAndTimestamp();
//    time_image1 = imageIO.popImageAndTimestamp();
//    auto time_image2 = imageIO.popImageAndTimestamp();

//    std::shared_ptr<cvFrame> cvframe1 = std::make_shared<cvFrame>(cam, time_image1.second);
//	cvMeasure::features_t fts;
//	feature_detection::Detector detector(time_image1.second.cols, time_image1.second.rows, 25, IMG_LEVEL);
//	detector.detect(cvframe1, cvframe1->getMeasure().measurement.imgPyr, fts);
//	for(auto &ft : fts)
//		cvframe1->addFeature(ft);
//    std::shared_ptr<cvFrame> cvframe2 = std::make_shared<cvFrame>(cam, time_image2.second);

//	std::shared_ptr<viFrame> viframe1 = std::make_shared<viFrame>(1, cvframe1, imuParam);
//	std::shared_ptr<viFrame> viframe2 = std::make_shared<viFrame>(2, cvframe2, imuParam);

//	direct_tracker::Tracker tracker;


//	std::cout << "img1 time:" << time_image1.first << "; img2 time:" << time_image2.first << std::endl;
//    IMUIO::dataDeque_t imuData = imuIO.pop(time_image1.first, time_image2.first);
//	std::cout << "  -- imuMeasure size:" << imuData.size() << std::endl;
//    auto imuparam = imuIO.getImuParam();
//    auto imu = std::make_shared<IMU>();

//    Sophus::SE3d T_WS;
//    SpeedAndBias speedAndBiases;
////    std::cout<<"1>>"<<time_image1.first<<" "<<imuData.front()->timeStamp<<"\n";
//    imu->propagation(imuData, *imuparam, T_WS, speedAndBiases, time_image1.first, time_image2.first, nullptr, nullptr);
//	printf("  -- integration:\n");
//	std::cout << T_WS.rotationMatrix() << std::endl;
//	std::cout << T_WS.translation() << std::endl;

//	Sophus::SE3d T_WS_;

//	bool isTracked = tracker.Tracking(viframe1, viframe2, T_WS_, 50);
//	printf("  -- tracking!\n");
//	std::cout << T_WS_.rotationMatrix() << std::endl;
//	std::cout << T_WS_.translation() << std::endl;

//	Eigen::Matrix<double, 6, 1> err = T_WS.log() - T_WS_.log();
//	double err_ = std::sqrt(err.transpose() * err);
//	std::cout << "  -- err:" << err_ << std::endl;
//	T_WS_ = T_WS;
//	isTracked = tracker.Tracking(viframe1, viframe2, T_WS_, 50);
//	err = T_WS.log() - T_WS_.log();
//	err_ = std::sqrt(err.transpose() * err);
//	std::cout << "  -- err:" << err_ << std::endl;
    printf("-- End of test_ImuPRE_IMU_PRE_Test\n");
}
