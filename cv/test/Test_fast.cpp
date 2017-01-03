//
// Created by lancelot on 1/3/17.
//

#include <memory>
#include <opencv2/ts.hpp>
#include <IO/camera/CameraIO.h>
#include "cv/Feature_Detector.h"
#include "DataStructure/cv/cvFrame.h"
#include "DataStructure/cv/Camera.h"

TEST(fast_detector, fast_detector) {
    std::string camDatafile = "../testData/mav0/cam1/data.csv";
    std::string camParamfile ="../testData/mav0/cam1/sensor.yaml";
    CameraIO camTest(camDatafile,camParamfile);
    const CameraIO::pCamereParam cam = camTest.getCamera();
    cv::Mat pic = cv::imread("..testData/mav0/cam0/data/1403715273262142976.png");
    GTEST_ASSERT_NE(pic.empty(), true);
    std::shared_ptr<cvFrame> frame = std::make_shared<cvFrame>(cam, pic);
    feature_detection::AbstractDetector *detector = new feature_detection::FastDetector(pic.cols, pic.rows, 25, IMG_LEVEL);
    feature_detection::FastDetector::features_t features;
    detector->detect(frame, frame->getMeasure().measurement.imgPyr, 20,features);
    GTEST_ASSERT_NE(features.empty(), true);
    delete detector;

}

