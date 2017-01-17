//
// Created by lancelot on 1/6/17.
//

#ifndef SIMPLE_VIO_VIOPINHOLECAMERA_H
#define SIMPLE_VIO_VIOPINHOLECAMERA_H

#include "PinholeCamera.h"



class VIOPinholeCamera : public PinholeCamera {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    typedef Sophus::SE3d       pose_t;
public:
    VIOPinholeCamera(double width, double height,double fx, double fy,
              double cx, double cy, double d0, double d1,
              double d2, double d3, double d4, pose_t t_bs,
              int rate_,std::string cameraMode,std::string distortionMode)
            : PinholeCamera(width, height, fx, fy, cx, cy, d0, d1, d2, d3, d4),
              T_BS(t_bs),m_Rate(rate_),m_CamMode(cameraMode),
              m_DistortionMode(distortionMode)
    {}

    virtual Eigen::Vector3d cam2world(const double& x, const double& y) const;
    virtual Eigen::Vector3d cam2world(const Eigen::Vector2d& px) const;
    virtual Eigen::Vector2d world2cam(const Eigen::Vector3d& xyz_c) const;
    virtual Eigen::Vector2d world2cam(const Eigen::Vector2d& uv) const;

    const Sophus::SE3d&    getT_BS(void) {return  T_BS;}
    int                    getRate(void) {return m_Rate;}
    const std::string&     getCameraMode(void) {return m_CamMode;}
    const std::string&     getDistorMode(void) {return m_DistortionMode;}

protected:
    pose_t       T_BS;
    int          m_Rate;
    std::string  m_CamMode;
    std::string  m_DistortionMode;
};


#endif //SIMPLE_VIO_VIOPINHOLECAMERA_H
