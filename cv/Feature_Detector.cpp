#include <fast/fast.h>
#include <opencv2/opencv.hpp>
#include <fstream>

#include "Feature_Detector.h"
#include "DataStructure/cv/Feature.h"
#include "util/util.h"

namespace feature_detection {

using namespace Eigen;
using namespace std;

void saveMatToCsv(cv::Mat data, std::string filename) {
    std::ofstream outputFile(filename.c_str());
    outputFile << cv::format(data,"CSV") << std::endl;
    outputFile.close();
}

AbstractDetector::AbstractDetector(
    const int img_width,
    const int img_height,
    const int cell_size,
    const int n_pyr_levels) :
        cell_size_(cell_size),
        n_pyr_levels_(n_pyr_levels),
        grid_n_cols_(ceil(static_cast<double>(img_width)/cell_size_)),
        grid_n_rows_(ceil(static_cast<double>(img_height)/cell_size_)),
        grid_occupancy_(grid_n_cols_*grid_n_rows_, false)
{}

void AbstractDetector::resetGrid()
{
  std::fill(grid_occupancy_.begin(), grid_occupancy_.end(), false);
}

void AbstractDetector::setExistingFeatures(const features_t& fts)
{
  std::for_each(fts.begin(), fts.end(), [&](std::shared_ptr<Feature> i) {
    grid_occupancy_.at(
        static_cast<int>(i->px[1]/cell_size_)*grid_n_cols_
        + static_cast<int>(i->px[0]/cell_size_)) = true;
  });
}

void AbstractDetector::setGridOccpuancy(const Vector2d& px)
{
  grid_occupancy_.at(
      static_cast<int>(px[1]/cell_size_)*grid_n_cols_
    + static_cast<int>(px[0]/cell_size_)) = true;
}

FastDetector::FastDetector(
    const int img_width,
    const int img_height,
    const int cell_size,
    const int n_pyr_levels) :
        AbstractDetector(img_width, img_height, cell_size, n_pyr_levels)
{}

void FastDetector::detect(
    cvframePtr_t frame,
    const ImgPyr_t& img_pyr,
    const double detection_threshold,
    features_t& fts)
{
  Corners corners(grid_n_cols_*grid_n_rows_, Corner(0,0,detection_threshold,0,0.0f));
  //Corners corners(grid_n_cols_*grid_n_rows_, Corner(0,0,0,0,0.0f));
  for(int L=0; L<n_pyr_levels_; ++L)
  {
    const int scale = (1<<L);
    vector<fast::fast_xy> fast_corners;
#if __SSE2__
      fast::fast_corner_detect_10_sse2(
          (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
          img_pyr[L].rows, img_pyr[L].cols, 8, fast_corners);   // 20
#elif HAVE_FAST_NEON
      fast::fast_corner_detect_9_neon(
          (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
          img_pyr[L].rows, img_pyr[L].cols, 8, fast_corners);
#else
      fast::fast_corner_detect_10(
          (fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols,
          img_pyr[L].rows, img_pyr[L].cols, 8, fast_corners);
#endif
    vector<int> scores, nm_corners;
    fast::fast_corner_score_10((fast::fast_byte*) img_pyr[L].data, img_pyr[L].cols, fast_corners,8, scores);// 20
    fast::fast_nonmax_3x3(fast_corners, scores, nm_corners);

    for(auto it=nm_corners.begin(), ite=nm_corners.end(); it!=ite; ++it)
    {
      fast::fast_xy& xy = fast_corners.at(*it);
      const int k = static_cast<int>((xy.y*scale)/cell_size_)*grid_n_cols_
                  + static_cast<int>((xy.x*scale)/cell_size_);
      if(grid_occupancy_[k])
        continue;
      const float score = shiTomasiScore(img_pyr[L], xy.x, xy.y);
      if(score > corners.at(k).score)
        corners.at(k) = Corner(xy.x*scale, xy.y*scale, score, L, 0.0f);
    }
  }

  // Create feature for every corner that has high enough corner score
  std::for_each(corners.begin(), corners.end(), [&](Corner& c) {
    if(c.score > detection_threshold)
      fts.push_back(std::shared_ptr<Feature>(new Feature(frame, Vector2d(c.x, c.y), c.level)));
  });

  resetGrid();
}


EdgeDetector::EdgeDetector(
    const int img_width,
    const int img_height,
    const int cell_size,
    const int n_pyr_levels) :
    AbstractDetector(img_width, img_height, cell_size, n_pyr_levels)
{}

void EdgeDetector::detect(cvframePtr_t frame, const AbstractDetector::ImgPyr_t &img_pyr, const double detection_threshold, AbstractDetector::features_t &fts) {

}

class Pt
{
public:
    float grad;
    Vector2d xy;
    Pt()
    {
        xy[0]=0;
        xy[1]=0;
        grad = 0.0;
    }
    Pt(float grad_, Vector2d xy_): grad(grad_),xy(xy_){}

    bool operator < (const Pt &m)const{ return grad > m.grad; }
};

class Edgelete
{
public:
    float grad;
    Vector2d xy;
    Vector2d dir;
    Edgelete()
    {
        xy[0]=0;
        xy[1]=0;
        grad = 0.0;
    }
    Edgelete(float grad_, Vector2d xy_, Vector2d dir_): grad(grad_),xy(xy_), dir(dir_){}

    bool operator < (const Edgelete &m)const{ return grad > m.grad; }
};

} // namespace feature_detection


