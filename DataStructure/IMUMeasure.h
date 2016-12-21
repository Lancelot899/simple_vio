#ifndef IMUMEASURE_H
#define IMUMEASURE_H

#include <deque>

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <sophus/se3.hpp>

#include "DataStructure/Measurements.h"

struct IMUData {
    Eigen::Vector3d           acceleration;
    Eigen::Vector3d           gyroscopes;
    Eigen::Quaternion<double> orientation;
    Eigen::Vector3d           geomagnetism;
    double                    atmospheric_pressure;
    double                    time;
};

struct IMUMeasure : public MeasurementBase<IMUData> {
    IMUMeasure(int sensorId, double timeStamp, Eigen::Vector3d acceleration, Eigen::Vector3d gyroscopes,
               Eigen::Vector3d geomagnetism = Eigen::Vector3d(), Eigen::Quaternion<double> orientation = Eigen::Quaternion<double>()) {
        this->sensorId = sensorId;
        this->timeStamp = timeStamp;
        this->measurement.acceleration = acceleration;
        this->measurement.gyroscopes = gyroscopes;
        this->measurement.geomagnetism = geomagnetism;
        this->measurement.orientation = orientation;
        this->measurement.time = timeStamp;
    }

    typedef std::deque<IMUMeasure, Eigen::aligned_allocator<IMUMeasure>> ImuMeasureDeque;
    typedef Sophus::SE3d                                                 Transformation;
    typedef Eigen::Matrix<double, 9, 1>                                  SpeedAndBias;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>        covariance_t;
    typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>        jacobian_t;
    typedef Eigen::Matrix<double, Eigen::Dynamic, 1>                     Error_t;
};


struct ImuParamenters {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    IMUMeasure::Transformation T_BS; ///< Transformation from Body frame to IMU (sensor frame S).
    double a_max;  ///< Accelerometer saturation. [m/s^2]
    double g_max;  ///< Gyroscope saturation. [rad/s]
    double sigma_g_c;  ///< Gyroscope noise density.
    double sigma_bg;  ///< Initial gyroscope bias.
    double sigma_a_c;  ///< Accelerometer noise density.
    double sigma_ba;  ///< Initial accelerometer bias
    double sigma_gw_c; ///< Gyroscope drift noise density.
    double sigma_aw_c; ///< Accelerometer drift noise density.
    double tau;  ///< Reversion time constant of accerometer bias. [s]
    double g;    ///< Earth acceleration.
    Eigen::Vector3d a0;  ///< Mean of the prior accelerometer bias.
    int rate;  ///< IMU rate in Hz.
};




#endif // IMUMEASURE_H
