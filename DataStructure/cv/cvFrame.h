#ifndef viFrame_H
#define viFrame_H

#include <memory>
#include <array>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

#include <boost/noncopyable.hpp>

#include "util/setting.h"
#include "DataStructure/Measurements.h"
#include "Camera.h"

typedef Sophus::SE3d pose_t;

class Feature;
class AbstractDetector;

struct cvData {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef cv::Mat                                                  Pic_t;                  //! raw image
    typedef std::vector<Eigen::Vector3d>                             Img_t;
    typedef std::array<Img_t, IMG_LEVEL>                             ImgPyr_t;               //!< Image Pyramid.
    typedef std::vector<double, Eigen::aligned_allocator<double>>    GradNorm_t;
    typedef std::array<GradNorm_t, IMG_LEVEL>                        GradNormPyr_t;

public:
    int              width[IMG_LEVEL];
    int              height[IMG_LEVEL];

    Pic_t            pic;
    ImgPyr_t         imgPyr;
    GradNormPyr_t    gradNormPyr;
};


struct cvMeasure : public MeasurementBase<cvData> {
public:
    typedef cvData::ImgPyr_t                        ImgPyr_t;
    typedef cvData::Pic_t                           Pic_t;
    typedef std::list<std::shared_ptr<Feature>>     features_t;     //!< List of features in the image.
    typedef std::vector<std::shared_ptr<Feature>>   keyPoints_t;    //!< Five features and associated 3D points which are used to detect if two frames have overlapping field of view.
    typedef std::shared_ptr<AbstractCamera>         cam_t;

public:
    int id;
    features_t       fts_;
    keyPoints_t      key_pts_;
};

class cvFrame : boost::noncopyable {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    friend class AbstractDetector;

public:
    typedef Eigen::Vector3d             position_t;
    typedef cvMeasure::cam_t            cam_t;
    typedef cvMeasure::features_t       features_t;
    typedef cvMeasure::Pic_t            Pic_t;
    typedef Eigen::Matrix<double, 6, 6> cov_t;
    typedef Eigen::Vector2d             grad_t;

public:
    cvFrame(std::shared_ptr<AbstractCamera>& cam, Pic_t &pic);
    ~cvFrame();

    int getID();
    const okvis::Time& getTimestamp();
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
    const Pic_t&  getPicture();
    double getIntensity(int u, int v, int level = 0);
    bool getGrad(int u, int v, grad_t&  out, int level = 0);
    double getGradNorm(int u, int v, int level = 0);
    int getWidth(int level = 0);
    int getHeight(int level = 0);

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

typedef std::shared_ptr<cvFrame> cvframePtr_t;

#endif // viFrame_H
