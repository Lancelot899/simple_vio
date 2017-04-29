//
// Created by lancelot on 4/12/17.
//

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
#include "cv/FeatureDetector/Detector.h"
#include "cv/Triangulater/Triangulater.h"
#include "cv/Tracker/Tracker.h"
#include "util/util.h"
#include "util/setting.h"
#include "./BA/BundleAdjustemt.h"

namespace vio {



system::system(std::string &imuDatafile, std::string &imuParamfile, std::string &camDatafile, std::string &camParamfile,
               std::string &imageFile, std::string &dataDirectory,  const int img_width, const int img_height) {
    std::shared_ptr<CameraIO> camIO = std::make_shared<CameraIO>(camDatafile, camParamfile);
    cam = camIO->getCamera();
    BARunning = false;
    BAResult = true;
    imgIO = std::make_shared<ImageIO>(imageFile, dataDirectory, cam);
    imuIO = std::make_shared<IMUIO>(imuDatafile, imuParamfile);
    imuParam  = imuIO->getImuParam();
    detector = std::make_shared<feature_detection::Detector>(img_width, img_width, 25, IMG_LEVEL);
    tracker = std::make_shared<direct_tracker::Tracker>();
    triangulater = std::make_shared<Triangulater>();
    imu = std::make_shared<IMU>();
    initialier = std::make_shared<Initialize>(detector, tracker, triangulater, imu);
    BA = std::make_shared<BundleAdjustemt>(SIMPLE_BA);
    BAThread = std::thread(&system::workLoop, this);
    id = -1;
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
    BundleAdjustemt::obsModeType obsModes;
    for(auto &keyframe : keyFrames) {
        for(auto &ft : keyframe->getCVFrame()->getMeasure().fts_) {
            auto it = obsModes.find(ft->point);
            if(it != obsModes.end())
                it->second.push_back(ft);
            else {
                std::list<std::shared_ptr<Feature>> fl;
                fl.push_back(ft);
                obsModes.insert(std::make_pair<std::shared_ptr<Point>,
                                std::list<std::shared_ptr<Feature>> >(std::shared_ptr<Point>(ft->point),
                                                                      std::move(fl)));
            }
        }
    }

    return BA->run(keyFrames, obsModes, imuFactors);
}

bool system::isInsertKeyframe(int num, Sophus::SE3d T)
{
    if(num < detectCellWidth*detectCellHeight*0.75)
        return true;
    if(T.translation().transpose() * T.translation() > KeyFrameTranslateThreadThold2)
        return true;



    return false;
}

void system::run() {
    okvis::Time pre_time;
    int lost = 0;
    while(!imgIO->isEmpty()) {
        id++;
        auto tImg = imgIO->popImageAndTimestamp();
        std::shared_ptr<cvFrame> frame = std::make_shared<cvFrame>(cam, tImg.second, tImg.first);

        if(id < 7) {
            if(id == 0) {
                initialier->setFirstFrame(frame, imuParam);
                pre_time = tImg.first;
            }

            else {
                std::shared_ptr<cvFrame> frame = std::make_shared<cvFrame>(cam, tImg.second, tImg.first);
                auto imuMeasure = imuIO->pop(tImg.first, pre_time);
                Sophus::SE3d T;
                IMUMeasure::SpeedAndBias spbs = IMUMeasure::SpeedAndBias::Zero();
                IMUMeasure::covariance_t var(9, 9);
                IMUMeasure::jacobian_t jac(15, 3);
                imu->propagation(imuMeasure, *imuParam, T, spbs, pre_time, tImg.first, &var, &jac);
                pre_time = tImg.first;
                std::shared_ptr<imuFactor> imufact = std::make_shared<imuFactor>(T, jac, spbs.block<3, 1>(0, 0), var);
                initialier->pushcvFrame(frame, imufact, imuParam);

                if(id == 6) {
                    initialier->init(imuParam);
                    std::swap(initialier->getInitialViframe(), keyFrames);
                    std::swap(initialier->getInitialImuFactor(), imuFactors);
                    BARunning = true;
                    callBA.notify_all();
                    auto &Viframes = initialier->getInitialViframe();
                    for(auto &f : Viframes)
                        QueKeyFrames.push_back(f);
                    curframe = QueKeyFrames.back();
                    isInsert = false;
                }
            }
        }

        else { //id > 7
            Sophus::SE3d T;
            Eigen::Matrix<double, 6, 6> info;
//            std::shared_ptr<viFrame> curViKF = std::make_shared<viFrame>(curframe,imuParam);
            std::shared_ptr<viFrame> newKF = std::make_shared<viFrame>(id,frame,imuParam);

            if(!tracker->Tracking(curframe, newKF, T, info)) {
                printf("lost!\n");
                lost++;
                if(lost > 5) {
                    printf("system has lost!!!\n");
                    exit(0);
                }
                continue;
            }

            if(isInsert) {
                isInsert = false;
                //! triangula
                int count = triangulater->triangulate(curframe, newKF, T, info);
                printf("triangulate %d's points!\n",count);
            }
            else {
                int cellnum = tracker->reProject(curframe, newKF, T, info);
                if(isInsertKeyframe(cellnum,T)) {
                    //! insert keyframe
                    //! detect
                    detector->detect(frame,frame->getMeasure().measurement.imgPyr,frame->getMeasure().fts_);
                    QueKeyFrames.pop_front();

                    QueKeyFrames.push_back(newKF);
                    isInsert = true;
                    callBA.notify_all();
                }
                curframe = newKF;
            }
        }
    }
}


}
