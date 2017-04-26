#include "cv/Tracker/Tracker.h"
#include "../Triangulater.h"
#include "DataStructure/viFrame.h"
#include "DataStructure/cv/cvFrame.h"
#include "IO/camera/CameraIO.h"
#include "cv/FeatureDetector/Detector.h"
#include "DataStructure/cv/Feature.h"

#include "opencv2/ts/ts.hpp"

TEST(Triangulate, Triangulate)
{
    direct_tracker::Tracker tracker;
    cv::Mat pic_i_ = cv::imread("../testData/mav0/cam0/data/1403715281712143104.png", 0);
   // cv::Mat pic_j_ = cv::imread("../testData/mav0/cam0/data/1403715281712143104.png", 0);
    cv::Mat pic_j_ = cv::imread("../testData/mav0/cam0/data/1403715281512143104.png", 0);
    std::string camDatafile = "../testData/mav0/cam1/data.csv";
    std::string camParamfile ="../testData/mav0/cam1/sensor.yaml";
    CameraIO camTest(camDatafile,camParamfile);
    const CameraIO::pCamereParam cam = camTest.getCamera();
    cv::Mat pic_i = Undistort(pic_i_, cam);
    cv::Mat pic_j = Undistort(pic_j_, cam);
    std::shared_ptr<cvFrame> cvframe_i = std::make_shared<cvFrame>(cam, pic_i);
    std::shared_ptr<cvFrame> cvframe_j = std::make_shared<cvFrame>(cam, pic_j);

    cvMeasure::features_t fts;
    feature_detection::Detector detector(pic_i.cols, pic_i.rows, 25, IMG_LEVEL);
    detector.detect(cvframe_i, cvframe_i->getMeasure().measurement.imgPyr, fts);
    for(auto &ft : fts)
        cvframe_i->addFeature(ft);

    auto imuParam = std::make_shared<ImuParameters>();
    std::shared_ptr<viFrame> viframe_i = std::make_shared<viFrame>(1, cvframe_i, imuParam);
    std::shared_ptr<viFrame> viframe_j = std::make_shared<viFrame>(2, cvframe_j, imuParam);

    Sophus::SE3d Tij;
    Eigen::Matrix<double, 6, 6> info;
    bool isTracked = tracker.Tracking(viframe_i, viframe_j, Tij, info, 50);
    if(isTracked)
        printf("\t--pre Tracking successful!\n\n");
    else
        printf("\t--pre Tracking failed!\n\n");

    printf("pre Tracking was finished! Now begin Triangulate: \n\n");

    Triangulater triangula;
    int count = triangula.triangulate(viframe_i,viframe_j,Tij,info, 30);
    printf("add %d points' depth!\n",count);

}
