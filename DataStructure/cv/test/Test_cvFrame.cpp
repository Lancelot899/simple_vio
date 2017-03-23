//
// Created by lancelot on 3/9/17.
//

#include "../cvFrame.h"
#include <opencv2/ts/ts.hpp>
#include "IO/camera/CameraIO.h"
#include "util/util.h"

TEST(cvFrame, cvFrame) {
    std::string camDatafile = "../testData/mav0/cam1/data.csv";
    std::string camParamfile ="../testData/mav0/cam1/sensor.yaml";
    CameraIO camTest(camDatafile,camParamfile);
    const CameraIO::pCamereParam cam = camTest.getCamera();
    cv::Mat pic_ = cv::imread("../testData/mav0/cam0/data/1403715273562142976.png", 0);
    cv::Mat pic = Undistort(pic_, cam);
    GTEST_ASSERT_NE(pic.empty(), true);
    std::shared_ptr<cvFrame> frame = std::make_shared<cvFrame>(cam, pic);

#ifdef SHOW_CVFRAME_
    for(unsigned i = 0; i < IMG_LEVEL; ++i) {
        cv::Mat img(frame->getHeight(i), frame->getWidth(i), CV_8UC1);
        for(int u = 0; u < img.cols; ++u) {
            for(int v = 0; v < img.rows; ++v) {
                img.at<u_char>(v, u) = u_char(frame->getMeasure().measurement.imgPyr[i][v * img.cols + u](0));
            }
        }

        char buf[32];
        memset(buf, 0, 32);
        sprintf(buf, "img%d", i);
        cv::imshow(buf, img);
    }

    cv::waitKey();
#endif

}

