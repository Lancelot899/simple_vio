#ifndef CAMERA_H
#define CAMERA_H

#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <sophus/se3.hpp>

class AbstractCamera {
public:
    AbstractCamera() {} // need this constructor for omni camera
    AbstractCamera(int width, int height) : width_(width), height_(height) {}

    virtual ~AbstractCamera() {}

    /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
    virtual Eigen::Vector3d cam2world(const double& x, const double& y) const = 0;

    /// Project from pixels to world coordiantes. Returns a bearing vector of unit length.
    virtual Eigen::Vector3d cam2world(const Eigen::Vector2d& px) const = 0;

    virtual Eigen::Vector2d world2cam(const Eigen::Vector3d& xyz_c) const = 0;

    /// projects unit plane coordinates to camera coordinates
    virtual Eigen::Vector2d world2cam(const Eigen::Vector2d& uv) const = 0;

    virtual double errorMultiplier2() const = 0;

    virtual double errorMultiplier() const = 0;

    inline int width() const { return width_; }

    inline int height() const { return height_; }

    inline bool isInFrame(const Eigen::Vector2i & obs, int boundary=0) const {
        if(obs[0]>=boundary && obs[0]<width()-boundary
                && obs[1]>=boundary && obs[1]<height()-boundary)
            return true;
        return false;
    }

    inline bool isInFrame(const Eigen::Vector2i &obs, int boundary, int level) const {
        if(obs[0] >= boundary && obs[0] < width()/(1<<level)-boundary
                && obs[1] >= boundary && obs[1] <height()/(1<<level)-boundary)
            return true;
        return false;
    }

protected:
    int width_;   // TODO cannot be const because of omni-camera model
    int height_;
};




#endif // CAMERA_H
