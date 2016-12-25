#ifndef FEATURE_DETECTOR_H
#define FEATURE_DETECTOR_H

#include <vector>
#include "DataStructure/cv/cvFrame.h"

/// Implementation of various feature detectors.
namespace feature_detection {

/// Temporary container used for corner detection. Features are initialized from these.
struct Corner {
  int x;        //!< x-coordinate of corner in the image.
  int y;        //!< y-coordinate of corner in the image.
  int level;    //!< pyramid level of the corner.
  float score;  //!< shi-tomasi score of the corner.
  float angle;  //!< for gradient-features: dominant gradient angle.
  Corner(int x, int y, float score, int level, float angle) :
    x(x), y(y), level(level), score(score), angle(angle)
  {}
};

typedef std::vector<Corner> Corners;

/// All detectors should derive from this abstract class.
class AbstractDetector {
public:
    typedef cvMeasure::ImgPyr_t    ImgPyr_t;
    typedef cvMeasure::features_t  features_t;


public:
  AbstractDetector(
      const int img_width,
      const int img_height,
      const int cell_size,
      const int n_pyr_levels);

  virtual ~AbstractDetector() {}

  virtual void detect(
      cvframePtr_t frame,
      const ImgPyr_t& img_pyr,
      const double detection_threshold,
      features_t& fts) = 0;

  /// Flag the grid cell as occupied
  void setGridOccpuancy(const Eigen::Vector2d& px);

  /// Set grid cells of existing features as occupied
  void setExistingFeatures(const features_t& fts);

protected:
  static const int border_ = 8; //!< no feature should be within 8px of border.
  const int cell_size_;
  const int n_pyr_levels_;
  const int grid_n_cols_;
  const int grid_n_rows_;
  std::vector<bool> grid_occupancy_;

  void resetGrid();

  inline int getCellIndex(int x, int y, int level)
  {
    const int scale = (1<<level);
    return (scale * y) / cell_size_ * grid_n_cols_ + (scale * x) / cell_size_;
  }
};

typedef std::shared_ptr<AbstractDetector> DetectorPtr;

/// Edgelete detector
class EdgeDetector : public AbstractDetector {
public:
  EdgeDetector(
      const int img_width,
      const int img_height,
      const int cell_size,
      const int n_pyr_levels);

  virtual ~EdgeDetector() {}

  virtual void detect(
      cvframePtr_t frame,
      const ImgPyr_t& img_pyr,
      const double detection_threshold,
      features_t& fts);
};

/// FAST detector by Edward Rosten.
class FastDetector : public AbstractDetector
{
public:
  FastDetector(
      const int img_width,
      const int img_height,
      const int cell_size,
      const int n_pyr_levels);

  virtual ~FastDetector() {}

  virtual void detect(
      cvframePtr_t frame,
      const ImgPyr_t& img_pyr,
      const double detection_threshold,
      features_t& fts);
};

} // namespace feature_detection

#endif // FEATURE_DETECTOR_H
