//
// Created by lancelot on 1/3/17.
//

#include <memory>
#include <opencv2/ts/ts.hpp>

#include "IO/camera/CameraIO.h"
#include "DataStructure/cv/Feature.h"
#include "cv/FeatureDetector/FastDetector.h"

#define SHOW_FAST

TEST(fast_detector, fast_detector) {
    std::string camDatafile = "../testData/mav0/cam1/data.csv";
    std::string camParamfile ="../testData/mav0/cam1/sensor.yaml";
    CameraIO camTest(camDatafile,camParamfile);
    const CameraIO::pCamereParam cam = camTest.getCamera();
    cv::Mat pic = cv::imread("../testData/mav0/cam0/data/1403715273562142976.png");
    cv::Mat pic1 = cv::imread("../testData/mav0/cam0/data/1403715310362142976.png");
    GTEST_ASSERT_NE(pic.empty(), true);
    std::shared_ptr<cvFrame> frame = std::make_shared<cvFrame>(cam, pic);
    std::shared_ptr<cvFrame> frame1 = std::make_shared<cvFrame>(cam, pic1);

    GTEST_ASSERT_NE(frame.get(), nullptr);
    feature_detection::AbstractDetector *detector = new feature_detection::FastDetector(pic.cols, pic.rows, 25, IMG_LEVEL);
    GTEST_ASSERT_NE(detector, nullptr);
    feature_detection::FastDetector::features_t features, features1;
//  printf("-- detector start!\n");
#ifdef SHOW_FAST
    detector->detect(frame, frame->getMeasure().measurement.imgPyr, 5,features);
    detector->detect(frame1, frame1->getMeasure().measurement.imgPyr, 5, features1);
    GTEST_ASSERT_NE(features.empty(), true);


    std::vector<cv::KeyPoint> keypoints;
    cv::Mat cvPic = pic.clone();
    cv::FAST(cvPic, keypoints, 5);

    for(auto it : keypoints) {
        cv::circle(cvPic, it.pt, 3, cv::Scalar(255, 0, 0), 1);
    }

    for (auto it : features) {
        std::cout << it->px << std::endl;
        printf("\n");
        cv::Point2i point(int((it)->px(0)), int((it)->px(1)));
        cv::circle(pic, point, 4 * (it->level + 1), cv::Scalar(0, 255, 0), 1);
    }


    for (auto it : features1) {
        cv::Point2i point(int((it)->px(0)), int((it)->px(1)));
        cv::circle(pic1, point, 4 * (it->level + 1), cv::Scalar(0, 255, 0), 1);
    }

    cv::imshow("cv", cvPic);
    cv::imshow("fast",pic);
    cv::imshow("fast1", pic1);
    cv::waitKey();

#else
    detector->detect(frame1, frame1->getMeasure().measurement.imgPyr, 5, features1);
    GTEST_ASSERT_NE(features1.empty(), true);


#endif

    delete detector;

}

