//
// Created by lancelot on 1/6/17.
//

#ifndef SIMPLE_VIO_PINHOLECAMERA_H
#define SIMPLE_VIO_PINHOLECAMERA_H

#include "AbstractCamera.h"
#include "util/setting.h"

class PinholeCamera : public AbstractCamera {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    PinholeCamera(double width, double height,
                  double fx, double fy, double cx, double cy,
                  double d0=0.0, double d1=0.0, double d2=0.0, double d3=0.0, double d4=0.0);

    ~PinholeCamera();
    void initUnistortionMap();

    virtual Eigen::Vector3d cam2world(const double& x, const double& y) const;

    virtual Eigen::Vector3d cam2world(const Eigen::Vector2d& px) const;

    virtual Eigen::Vector2d world2cam(const Eigen::Vector3d& xyz_c) const;

    virtual Eigen::Vector2d world2cam(const Eigen::Vector2d& uv) const;
    Eigen::Vector2d world2camUV(const Eigen::Vector2d& uv) const;

    Eigen::Vector2d focal_length() const {
        return Eigen::Vector2d(fx_[0], fy_[0]);
    }

    virtual double errorMultiplier2() const {
        return fabs(fx_[0]);
    }

    virtual double errorMultiplier() const {
        return fabs(4.0*fx_[0]*fy_[0]);
    }

    const Eigen::Matrix3d K(int level) const { return K_[level]; }
    const Eigen::Matrix3d K_inv(int level) const { return K_inv_[level]; }
    virtual double fx(int level) const { return fx_[level]; }
    virtual double fy(int level) const { return fy_[level]; }
    virtual double cx(int level) const { return cx_[level]; }
    virtual double cy(int level) const { return cy_[level]; }
    virtual double d(int i) const { return d_[i]; }
    void undistortImage(const cv::Mat& raw, cv::Mat& rectified);

private:
    double * fx_;
    double * fy_;
    double * cx_;
    double * cy_;
    bool distortion_;             //!< is it pure pinhole model or has it radial distortion?
    double d_[5];                 //!< distortion parameters, see http://docs.opencv.org/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
    cv::Mat cvK_[IMG_LEVEL], cvD_;
    cv::Mat undist_map1_, undist_map2_;
    bool use_optimization_;
    Eigen::Matrix3d K_[IMG_LEVEL];
    Eigen::Matrix3d K_inv_[IMG_LEVEL];

};

#endif //SIMPLE_VIO_PINHOLECAMERA_H
