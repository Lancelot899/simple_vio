//
// Created by lancelot on 3/1/17.
//

#include <opencv2/ts/ts.hpp>
#include "IO/camera/CameraIO.h"
#include "DataStructure/cv/Feature.h"
#include "../Detector.h"

TEST(Detector, Detector) {
    std::string camDatafile = "../testData/mav0/cam1/data.csv";
    std::string camParamfile ="../testData/mav0/cam1/sensor.yaml";
    CameraIO camTest(camDatafile,camParamfile);
    const CameraIO::pCamereParam cam = camTest.getCamera();
    cv::Mat pic = cv::imread("../testData/mav0/cam0/data/1403715278762142976.png", 0);
    GTEST_ASSERT_NE(pic.empty(), true);
    std::shared_ptr<cvFrame> frame = std::make_shared<cvFrame>(cam, pic);
    feature_detection::Detector detector(pic.cols, pic.rows, 25, IMG_LEVEL);
    feature_detection::features_t fts;

// #define SHOW_FEATURE
#ifdef SHOW_FEATURE

    double t = (double)cvGetTickCount();
    detector.detect(frame, frame->getMeasure().measurement.imgPyr, fts);
    t = (double)cvGetTickCount() - t;
    printf("\t--detecting time:%d\n", t / (cvGetTickFrequency() * 1000));

    GTEST_ASSERT_NE(fts.size(), 0);

    int levelTimes[5] = {1,2,4,8,16};
    cv::Scalar color[3] = {cv::Scalar(255,0,0),cv::Scalar(255,255,0),cv::Scalar(0,0,255)};
    cv::Mat resultline = cv::imread("../testData/mav0/cam0/data/1403715273562142976.png");
    cv::Mat resultpt = resultline.clone();
    cv::Mat result = resultline.clone();

    for (auto it : fts) {
        if(it->type == Feature::EDGELET) {
            cv::Point2i point0((it)->px(0)*levelTimes[it->level]+it->grad(1)*0.75, (it)->px(1)*levelTimes[it->level]-it->grad(0)*0.75);
            cv::Point2i point1((it)->px(0)*levelTimes[it->level]-it->grad(1)*0.75, (it)->px(1)*levelTimes[it->level]+it->grad(0)*0.75);
            cv::line(resultline, point0, point1, color[it->level]);
            cv::line(result, point0, point1, color[it->level]);
        }

        if(it->type == Feature::CORNER) {
            cv::Point2i point(int((it)->px(0)), int((it)->px(1)));
            cv::circle(resultpt, point, 4 * (it->level + 1), cv::Scalar(0, 255, 0), 1);
            cv::circle(result, point, 4 * (it->level + 1), cv::Scalar(0, 255, 0), 1);
        }

    }

    cv::imshow("result", result);
    cv::imshow("result line", resultline);
    cv::imshow("result point", resultpt);
    cv::waitKey();

#else
    {
        TimeUse time(__FUNCTION__,__LINE__);
        detector.detect(frame, frame->getMeasure().measurement.imgPyr, fts);
    }

#endif

}
