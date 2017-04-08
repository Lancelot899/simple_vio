#ifndef cvFrame_H_
#define cvFrame_H_

#include <memory>
#include <array>
#include <sophus/se3.hpp>
#include <opencv2/opencv.hpp>

#include <boost/noncopyable.hpp>
#include <boost/thread/pthread/shared_mutex.hpp>

#include "util/setting.h"
#include "DataStructure/Measurements.h"
#include "DataStructure/cv/Camera/VIOPinholeCamera.h"

static const int cellNumbel[5]=
{
    0, 256, 320, 336, 340
};

static const int cellRowNumbel[5]=
{
    16, 8, 4, 2, 1,
};

typedef Sophus::SE3d pose_t;

class Feature;

namespace feature_detection {
    class AbstractDetector;
    class FastDetector;
}

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
    features_t                        fts_;
    keyPoints_t                       key_pts_;
};

class cvFrame : boost::noncopyable {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    friend class feature_detection::AbstractDetector;
    friend feature_detection::FastDetector;
    friend class Initialize;

public:
    typedef Eigen::Vector3d             position_t;
    typedef cvMeasure::cam_t            cam_t;
    typedef cvMeasure::features_t       features_t;
    typedef cvMeasure::Pic_t            Pic_t;
    typedef Eigen::Matrix<double, 6, 6> cov_t;
    typedef Eigen::Vector2d             grad_t;

public:
    cvFrame(const std::shared_ptr<AbstractCamera>& cam, Pic_t &pic);
    ~cvFrame();
    const std::shared_ptr<Feature>& addFeature(const std::shared_ptr<Feature>& ft);

    int getID();
    const okvis::Time& getTimestamp();
    int getSensorID();
    const cam_t& getCam();
    const cov_t& getCovariance();
    pose_t getPose();
    void setPose(pose_t &pose);
    void setCovariance(cov_t& Cov);
    bool isKeyframe();
    void setKey();
    void cancelKeyframe();
    int getPublishTimestamp();
    const position_t pos();
    const Pic_t&  getPicture();
    double getIntensity(int u, int v, int level = 0);
    double getIntensityBilinear(double u, double v, int level = 0);
    Eigen::Vector2d getGradBilinear(double u, double v, int level = 0);
    bool getGrad(int u, int v, grad_t&  out, int level = 0);
    double getGradNorm(int u, int v, int level = 0);
    int getWidth(int level = 0);
    int getHeight(int level = 0);
    const cvMeasure& getMeasure();
    bool checkOccupy(int u,int v);
    void setCellTrue(int u, int v);
    bool checkCell(int u, int v) {
        return cell[u + v * detectCellWidth];
    }
//    bool checkCellOccupy(int index,int level = 0);

public:
    static int         frame_counter_;         //!< Counts the number of created frames. Used to set the unique id.

private:
    cvMeasure           cvData;
    pose_t              pose_;                                               //!< Transform frame from world.
    boost::shared_mutex pose_mutex;
    cov_t               Cov_;                                                //!< Covariance.
    cam_t               cam_;                                                //!< Camera model.
    bool               is_keyframe_;                                         //!< Was this frames selected askeyframe?
    int                 last_published_ts_;                                  //!< Timestamp of last publishing.
    bool occupy[detectCellWidth * detectCellHeight * detectHeightGrid * detectWidthGrid];  //!< whether cell is occupy by features
    bool cell[detectCellWidth * detectCellHeight];                           //!< whether the big cell is occupied
};

typedef std::shared_ptr<cvFrame> cvframePtr_t;

#endif // cvFrame_H_
