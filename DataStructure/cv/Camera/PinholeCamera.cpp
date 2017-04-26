//
// Created by lancelot on 1/6/17.
//

#include "PinholeCamera.h"

using namespace Eigen;


PinholeCamera::PinholeCamera(double width, double height,
                             double fx, double fy,
                             double cx, double cy,
                             double d0, double d1, double d2, double d3, double d4) :
        AbstractCamera(width, height),distortion_(false),
        undist_map1_(height_, width_, CV_16SC2),
        undist_map2_(height_, width_, CV_16SC2),
        use_optimization_(false) {
    d_[0] = d0; d_[1] = d1; d_[2] = d2; d_[3] = d3; d_[4] = d4;

    fx_ = new double[IMG_LEVEL]; fy_ = new double[IMG_LEVEL];
    cx_ = new double[IMG_LEVEL]; cy_ = new double[IMG_LEVEL];

    fx_[0] = fx, fy_[0] = fy, cx_[0] = cx, cy_[0] = cy;
    cvK_[0] = (cv::Mat_<double>(3, 3) << fx_[0], 0.0, cx_[0], 0.0, fy_[0], cy_[0], 0.0, 0.0, 1.0);
    cvD_ = (cv::Mat_<double>(1, 5) << d_[0], d_[1], d_[2], d_[3], d_[4]);

    for (int i = 1; i < IMG_LEVEL; ++i) {
        fx_[i]  = fx_[i-1]/2; fy_[i] = fy_[i-1]/2;
        cx_[i]  = cx_[i-1]/2; cy_[i] = cy_[i-1]/2;
        cvK_[i] = (cv::Mat_<double>(3, 3) << fx_[i], 0.0, cx_[i], 0.0, fy_[i], cy_[i], 0.0, 0.0, 1.0);
    }


    cv::initUndistortRectifyMap(cvK_[0], cvD_, cv::Mat_<double>::eye(3,3), cvK_[0],
                                cv::Size(width_, height_), CV_16SC2, undist_map1_, undist_map2_);
    for (int var = 0; var < IMG_LEVEL; ++var) {
        K_[var] << fx_[var], 0.0, cx_[var], 0.0, fy_[var], cy_[var], 0.0, 0.0, 1.0;
        K_inv_[var] = K_[var].inverse();
    }

}

PinholeCamera::~PinholeCamera() {
    delete []fx_; delete []fy_;
    delete []cx_; delete []cy_;
}

Vector3d PinholeCamera::cam2world(const double& u, const double& v) const {
    Vector3d xyz;
    if(!distortion_) {
        xyz[0] = (u - cx_[0])/fx_[0];
        xyz[1] = (v - cy_[0])/fy_[0];
        xyz[2] = 1.0;
    }

    else {
        cv::Point2d uv(u,v), px;
        const cv::Mat src_pt(1, 1, CV_32FC2, &uv.x);
        cv::Mat dst_pt(1, 1, CV_32FC2, &px.x);
        cv::undistortPoints(src_pt, dst_pt, cvK_[0], cvD_);
        xyz[0] = px.x;
        xyz[1] = px.y;
        xyz[2] = 1.0;
    }
    return xyz;
}


Vector3d PinholeCamera::cam2world (const Vector2d& uv) const {
    return cam2world(uv[0], uv[1]);
}

Vector2d PinholeCamera::world2cam(const Vector3d& xyz) const {
    //return world2cam(vk::project2d(xyz));
    const Vector2d uv= xyz.head<2>()/xyz[2];
    return world2camUV(uv);
}

Vector2d PinholeCamera::world2cam(const Vector2d& uv) const {
    Vector2d px;
    if(!distortion_) {
        px[0] = fx_[0]*uv[0] + cx_[0];
        px[1] = fy_[0]*uv[1] + cy_[0];
    }

    else {
        double x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
        x = uv[0];
        y = uv[1];
        r2 = x*x + y*y;
        r4 = r2*r2;
        r6 = r4*r2;
        a1 = 2*x*y;
        a2 = r2 + 2*x*x;
        a3 = r2 + 2*y*y;
        cdist = 1 + d_[0]*r2 + d_[1]*r4 + d_[4]*r6;
        xd = x*cdist + d_[2]*a1 + d_[3]*a2;
        yd = y*cdist + d_[2]*a3 + d_[3]*a1;
        px[0] = xd*fx_[0] + cx_[0];
        px[1] = yd*fy_[0] + cy_[0];
    }
    return px;
}

Vector2d PinholeCamera::world2camUV(const Vector2d& uv) const {
    Vector2d px;
    if(!distortion_) {
        px[0] = fx_[0]*uv[0] + cx_[0];
        px[1] = fy_[0]*uv[1] + cy_[0];
    }

    else {
        double x, y, r2, r4, r6, a1, a2, a3, cdist, xd, yd;
        x = uv[0];
        y = uv[1];
        r2 = x*x + y*y;
        r4 = r2*r2;
        r6 = r4*r2;
        a1 = 2*x*y;
        a2 = r2 + 2*x*x;
        a3 = r2 + 2*y*y;
        cdist = 1 + d_[0]*r2 + d_[1]*r4 + d_[4]*r6;
        xd = x*cdist + d_[2]*a1 + d_[3]*a2;
        yd = y*cdist + d_[2]*a3 + d_[3]*a1;
        px[0] = xd*fx_[0] + cx_[0];
        px[1] = yd*fy_[0] + cy_[0];
    }
    return px;
}

void PinholeCamera::undistortImage(const cv::Mat& raw, cv::Mat& rectified) {
    if(distortion_)
        cv::remap(raw, rectified, undist_map1_, undist_map2_, CV_INTER_LINEAR);
    else
        rectified = raw.clone();
}
