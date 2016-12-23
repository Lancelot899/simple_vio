#ifndef viFrame_H
#define viFrame_H

#include <memory>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

#include <boost/noncopyable.hpp>

#include "util/setting.h"
#include "DataStructure/Measurements.h"
#include "Camera.h"

typedef Sophus::SE3d pose_t;

class Feature;

struct cvData {
    cv::Mat img[IMG_LEVEL];                                               //!< Image Pyramid.
    std::list<std::shared_ptr<Feature>>           fts_;                   //!< List of features in the image.
    std::vector<std::shared_ptr<Feature>>         key_pts_;               //!< Five features and associated 3D points which are used to detect if two frames have overlapping field of view.
};


struct cvMeasure : public MeasurementBase<cvData> {
    typedef cv::Mat                             Img_t;
    typedef std::list<std::shared_ptr<Feature>> features_t;
    typedef std::shared_ptr<AbstractCamera>     cam_t;
    int id;
};

class cvFrame : boost::noncopyable {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

public:
    typedef Eigen::Vector3d             position_t;
    typedef cvMeasure::cam_t            cam_t;
    typedef cvMeasure::features_t       features_t;
    typedef cvMeasure::Img_t            Img_t;
    typedef Eigen::Matrix<double, 6, 6> cov_t;

public:
    cvFrame(std::shared_ptr<AbstractCamera>& cam);
    ~cvFrame();

    int getID();
    double getTimestamp();
    int getSensorID();
    const cam_t getCam();
    const cov_t& getCovariance();
    const pose_t& getPose();
    void setPose(pose_t &pose);
    void setCovariance(cov_t& Cov);
    bool isKeyframe();
    void setKey();
    void cancelKeyframe();
    int getPublishTimestamp();
    const position_t& pos();

public:
    static int         frame_counter_;         //!< Counts the number of created frames. Used to set the unique id.

private:
    cvMeasure          cvData;
    pose_t             pose_;                  //!< Transform frame from world.
    cov_t              Cov_;                   //!< Covariance.
    cam_t              cam_;                   //!< Camera model.
    bool               is_keyframe_;           //!< Was this frames selected as keyframe?
    int                last_published_ts_;     //!< Timestamp of last publishing.
};

#endif // viFrame_H
