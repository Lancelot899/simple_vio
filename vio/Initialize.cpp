//
// Created by lancelot on 1/15/17.
//

#include "Initialize.h"
#include "DataStructure/cv/cvFrame.h"

Initialize::Initialize(IMU::IntegalType type) {
    imu_ = std::make_shared<IMU>(type);
}

bool Initialize::init(std::shared_ptr<viFrame> &firstFrame, std::shared_ptr<viFrame> &frame,
                      IMUMeasure::ImuMeasureDeque &imuMeasures, ImuParameters& imuParam) {
    firstFrame->cvframe->pose_ = Sophus::SE3d(Eigen::Quaternion<double>(1, 0, 0, 0), Eigen::Vector3d(0, 0, 0));
    IMUMeasure::Transformation transformation;
    IMUMeasure::SpeedAndBias spbs = IMUMeasure::SpeedAndBias::Zero();
    imu_->propagation(imuMeasures, imuParam, transformation, spbs,
                      firstFrame->cvframe->cvData.timeStamp, frame->cvframe->cvData.timeStamp, nullptr, nullptr);

    frame->cvframe->pose_ = transformation;
    frame->spbs = spbs;



    return true;
}


