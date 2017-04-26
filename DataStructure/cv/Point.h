#ifndef POINT_H
#define POINT_H
#include <memory>
#include <atomic>
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
    double                                  normal_information_;      //!< Inverse covariance.
    boost::shared_mutex                    infoMutex;
    std::list<std::shared_ptr<Feature>>    obs_;                     //!< References to keyframes which observe the point.
    boost::shared_mutex                    obsMutex;
    std::atomic_int                        last_projected_kf_id_;    //!< Flag for the reprojection: don't reproject a pt twice.
    PointType                              type_;                    //!< Quality of the point.
    std::atomic<int>                       n_failed_reproj_;         //!< Number of failed reprojections. Used to assess the quality of the point.
    std::atomic<int>                       n_succeeded_reproj_;      //!< Number of succeeded  reprojections. Used to assess the quality of the point.

    Point(const Eigen::Vector3d& pos);
    Point(const Eigen::Vector3d& pos, std::shared_ptr<Feature>& ftr);
    ~Point();

    double getDepthInformation();
    double updateDepth(double depth, double information);

};

#endif // POINT_H
