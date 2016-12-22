#ifndef FEATURE_H
#define FEATURE_H


#include <Eigen/Dense>
#include <memory>

class Point;
class cvFrame;

struct Feature
{
    enum FeatureType {
        CORNER,
        EDGELET
    };

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    FeatureType type;                 //!< Type can be corner or edgelet.
    std::shared_ptr<cvFrame> frame;   //!< Pointer to frame in which the feature was detected.
    Eigen::Vector2d px;               //!< Coordinates in pixels on pyramid level 0.
    Eigen::Vector3d f;                //!< Unit-bearing vector of the feature.
    int level;                        //!< Image pyramid level where feature was extracted.
    std::shared_ptr<Point>   point;   //!< Pointer to 3D point which corresponds to the feature.
    Eigen::Vector2d grad;             //!< Dominant gradient direction for edglets, normalized.

    Feature(std::shared_ptr<cvFrame>& _frame, const Eigen::Vector2d &_px, const Eigen::Vector2d &_grad, int _level):
        type(EDGELET),
        frame(_frame),
        px(_px),
        f(frame->cam_->cam2world(px)),
        level(_level),
        grad(_grad)
    {}

    Feature(std::shared_ptr<cvFrame>& _frame, const Eigen::Vector2d& _px, int _level) :
        type(CORNER),
        frame(_frame),
        px(_px),
        f(frame->cam_->cam2world(px)),
        level(_level),
        grad(1.0,0.0)
    {}

    Feature(std::shared_ptr<cvFrame>& _frame, const Eigen::Vector2d& _px, const Eigen::Vector3d& _f, int _level) :
        type(CORNER),
        frame(_frame),
        px(_px),
        f(_f),
        level(_level),
        grad(1.0,0.0)
    {}

    Feature(std::shared_ptr<cvFrame>& _frame, std::shared_ptr<Point>& _point, const Eigen::Vector2d& _px, const Eigen::Vector3d& _f, int _level) :
        type(CORNER),
        frame(_frame),
        px(_px),
        f(_f),
        level(_level),
        point(_point),
        grad(1.0,0.0)
    {}

    Feature(std::shared_ptr<cvFrame>& _frame, std::shared_ptr<Point>& _point, const Eigen::Vector2d& _px,
            const Eigen::Vector3d& _f, const Eigen::Vector2d &_grad,int _level) :
        type(EDGELET),
        frame(_frame),
        px(_px),
        f(_f),
        level(_level),
        point(_point),
        grad(_grad)
    {}
};


#endif // FEATURE_H
