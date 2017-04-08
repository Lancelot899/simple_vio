#ifndef POINT_H
#define POINT_H
#include <memory>
#include <list>
#include <Eigen/Dense>
#include <boost/noncopyable.hpp>
#include "boost/thread/shared_mutex.hpp"

#include "util/util.h"

class Feature;
class cvFrame;

class Point : boost::noncopyable
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    enum PointType {
        TYPE_DELETED,
        TYPE_CANDIDATE,
        TYPE_UNKNOWN,
        TYPE_GOOD
    };

    static int                             point_counter_;           //!< Counts the number of created points. Used to set the unique id.
    int                                    id_;                      //!< Unique ID of the point.
    Eigen::Vector3d                        pos_;                     //!< 3d pos of the point in the world coordinate frame.
    boost::shared_mutex                    pos_mutex;
    Eigen::Vector3d                        normal_;                  //!< Surface normal at point.
    Eigen::Matrix3d                        normal_information_;      //!< Inverse covariance matrix of normal estimation.
    bool                                   normal_set_;              //!< Flag whether the surface normal was estimated or not.
    std::list<std::shared_ptr<Feature>>    obs_;                     //!< References to keyframes which observe the point.
    size_t                                 n_obs_;                   //!< Number of obervations: Keyframes AND successful reprojections in intermediate frames.
    int                                    last_published_ts_;       //!< Timestamp of last publishing.
    int                                    last_projected_kf_id_;    //!< Flag for the reprojection: don't reproject a pt twice.
    PointType                              type_;                    //!< Quality of the point.
    int                                    n_failed_reproj_;         //!< Number of failed reprojections. Used to assess the quality of the point.
    int                                    n_succeeded_reproj_;      //!< Number of succeeded reprojections. Used to assess the quality of the point.
    int                                    last_structure_optim_;    //!< Timestamp of last point optimization

    Point(const Eigen::Vector3d& pos);
    Point(const Eigen::Vector3d& pos, std::shared_ptr<Feature>& ftr);
    ~Point();

    /// Add a reference to a frame.
    void addFrameRef(std::shared_ptr<Feature>& ftr);

    /// Remove reference to a frame.
    bool deleteFrameRef(std::shared_ptr<cvFrame>& frame);

    /// Initialize point normal. The inital estimate will point towards the frame.
    void initNormal();

    /// Check whether mappoint has reference to a frame.
    std::shared_ptr<Feature> findFrameRef(std::shared_ptr<cvFrame>& frame);

    /// Get Frame with similar viewpoint.
    bool getCloseViewObs(const Eigen::Vector3d& pos, std::shared_ptr<Feature>& obs);

    /// Get number of observations.
    inline size_t nRefs() const { return obs_.size(); }

    /// Optimize point position through minimizing the reprojection error.
    void optimize(const size_t n_iter);

    /// Jacobian of point projection on unit plane (focal length = 1) in frame (f).
    inline static void jacobian_xyz2uv(
            const Eigen::Vector3d& p_in_f,
            const Eigen::Matrix3d& R_f_w,
            Matrix23d& point_jac)
    {
        const double z_inv = 1.0/p_in_f[2];
        const double z_inv_sq = z_inv*z_inv;
        point_jac(0, 0) = z_inv;
        point_jac(0, 1) = 0.0;
        point_jac(0, 2) = -p_in_f[0] * z_inv_sq;
        point_jac(1, 0) = 0.0;
        point_jac(1, 1) = z_inv;
        point_jac(1, 2) = -p_in_f[1] * z_inv_sq;
        point_jac = - point_jac * R_f_w;
    }
};

#endif // POINT_H
