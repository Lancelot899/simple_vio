//
// Created by ZHANGZHENWEI on 1/17/17.
//

#include <memory>
#include <opencv2/ts.hpp>

#include "IO/camera/CameraIO.h"
#include "DataStructure/cv/Feature.h"
#include "cv/FeatureDetector/EdgeDetector.h"

TEST(edge_detector, edge_detector) {
    std::string camDatafile = "../testData/mav0/cam1/data.csv";
    std::string camParamfile ="../testData/mav0/cam1/sensor.yaml";
    CameraIO camTest(camDatafile,camParamfile);
    const CameraIO::pCamereParam cam = camTest.getCamera();
    cv::Mat pic = cv::imread("../testData/mav0/cam0/data/1403715273562142976.png");
    cv::Mat pic1 = cv::imread("../testData/mav0/cam0/data/1403715310362142976.png");
    cv::Mat picCanny; cv::Canny(pic,picCanny,100,150);
    cv::Mat picCanny1; cv::Canny(pic1,picCanny1,100,150);

    cv::imshow("pic",pic);
    cv::imshow("pic1", pic1);
    cv::imshow("Edge0",picCanny);
    cv::imshow("Edge01",picCanny1);
    GTEST_ASSERT_NE(pic.empty(), true);
    std::shared_ptr<cvFrame> frame = std::make_shared<cvFrame>(cam, pic);
    std::shared_ptr<cvFrame> frame1 = std::make_shared<cvFrame>(cam, pic1);

    cv::Mat grad(pic.rows,pic.cols,CV_8UC1,cv::Scalar(0)),grad1(pic.rows,pic.cols,CV_8UC1,cv::Scalar(0));

    for (int v = 0; v < pic.rows; ++v) {
        for (int u = 0; u < pic.cols; ++u) {
            grad.at<u_char>(v,u) = (u_char)frame->getGradNorm(u,v,0);
            grad1.at<u_char>(v,u) = (u_char)frame1->getGradNorm(u,v,0);
        }
    }

    cv::imshow("grad",grad);
    cv::imshow("grad1", grad1);
    cv::waitKey();
    //    GTEST_ASSERT_NE(frame.get(), nullptr);
    //    feature_detection::AbstractDetector *detector = new feature_detection::EdgeDetector(pic.cols, pic.rows, 25, IMG_LEVEL);
    //    GTEST_ASSERT_NE(detector, nullptr);

    //    feature_detection::AbstractDetector::features_t features, features1;
    //    features.clear(); features1.clear();
    //    printf("-- detector start!\n");

    //#define SHOW_EDGE
    //#ifdef  SHOW_EDGE
    //    detector->detect(frame, frame->getMeasure().measurement.imgPyr, 5, features);
    //    detector->detect(frame1, frame1->getMeasure().measurement.imgPyr, 5, features1);
    //    GTEST_ASSERT_NE(features.empty(), true);

    //    printf("file[%s],  lines: %d\n",__FILE__, __LINE__);

    //    for (auto it : features) {
    ////        std::cout << it->px << std::endl;
    ////        printf("\n");
    //        cv::Point2i point(int((it)->px(0)), int((it)->px(1)));
    //        cv::circle(pic, point, 4 * (it->level + 1), cv::Scalar(0, 255, 0), 1);
    //    }

    //    for (auto it : features1) {
    //        cv::Point2i point(int((it)->px(0)), int((it)->px(1)));
    //        cv::circle(pic1, point, 4 * (it->level + 1), cv::Scalar(0, 255, 0), 1);
    //    }

    //    cv::imshow("Edge",pic);
    //    cv::imshow("Edge1", pic1);
    //    cv::waitKey();

    //#else
    //    detector->detect(frame1, frame1->getMeasure().measurement.imgPyr, 5, features1);
    //    GTEST_ASSERT_NE(features1.empty(), true);
    //#endif

    //    printf("file: %s,  lines: %d\n",__FILE__, __LINE__);
    ////    delete detector;
    //    printf("file: %s,  lines: %d\n",__FILE__, __LINE__);

}

