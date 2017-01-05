#include <opencv2/opencv.hpp>
#include <fstream>

#include "AbstractDetector.h"
#include "DataStructure/cv/Feature.h"

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

} // namespace feature_detection
