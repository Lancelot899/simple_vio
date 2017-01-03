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
        AbstractDetector(img_width, img_height, cell_size, n_pyr_levels) {}



static inline bool test_gt_set(const cvData::Img_t::value_type& a_, double b, double& min_diff)
{
    double a = a_(0);
    if(a > b)
    {
        if(a-b < min_diff)
            min_diff = a-b;

        return 1;
    }
    return 0;
}

static inline bool test_gt_set(double a, const cvData::Img_t::value_type& b_, double& min_diff) {
    double b = b_(0);
    if(a > b){
        if(a-b < min_diff)
            min_diff = a-b;

        return 1;
    }
    return 0;
}

void FastDetector::fast_corner_detect_10(const cvData::Img_t &img_,
                                         int img_width, int img_height, int img_stride,
                                         double barrier, std::vector<fast_xy> &corners) {
    int y, cb, c_b;
    cvData::Img_t::const_iterator img = img_.begin();
    cvData::Img_t::const_iterator line_max, line_min;
    cvData::Img_t::const_iterator cache_0;

    int pixel[16] = {
            0 + img_stride * 3,
            1 + img_stride * 3,
            2 + img_stride * 2,
            3 + img_stride * 1,
            3 + img_stride * 0,
            3 + img_stride * -1,
            2 + img_stride * -2,
            1 + img_stride * -3,
            0 + img_stride * -3,
            -1 + img_stride * -3,
            -2 + img_stride * -2,
            -3 + img_stride * -1,
            -3 + img_stride * 0,
            -3 + img_stride * 1,
            -2 + img_stride * 2,
            -1 + img_stride * 3,
    };

    for(y = 3 ; y < img_height - 3; y++)
    {
        cache_0 = img + y*img_stride + 3;
        line_min = cache_0 - 3;
        line_max = img + y*img_stride + img_width - 3;
        // cache_0 = &i[y][3];
        // line_min = cache_0 - 3;
        // line_max = &i[y][i.size().x - 3];


        for(; cache_0 < line_max;cache_0++)
        {
            cb = (*cache_0)(0) + barrier;
            c_b= (*cache_0)(0) - barrier;

            if((*(cache_0 + pixel[0]))(0) > cb)
                if((*(cache_0 + pixel[8]))(0) > cb)
                    if((*(cache_0 + pixel[3]))(0) > cb)
                        if((*(cache_0 + pixel[5]))(0) > cb)
                            if((*(cache_0 + pixel[2]))(0) > cb)
                                if((*(cache_0 + pixel[6]))(0) > cb)
                                    if((*(cache_0 + 3))(0) > cb)
                                        if((*(cache_0 + pixel[7]))(0) > cb)
                                            if((*(cache_0 + pixel[1]))(0) > cb)
                                                if((*(cache_0 + pixel[9]))(0) > cb)
                                                    goto success;
                                                else
                                                if((*(cache_0 + pixel[15]))(0) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else if((*(cache_0 + pixel[1]))(0) < c_b)
                                                if((*(cache_0 + pixel[9]))(0) > cb)
                                                    if((*(cache_0 + pixel[10]))(0) > cb)
                                                        if((*(cache_0 + pixel[11]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[11]))(0) > cb)
                                                if((*(cache_0 + pixel[10]))(0) > cb)
                                                    if((*(cache_0 + pixel[9]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if((*(cache_0 + pixel[7]))(0) < c_b)
                                            if((*(cache_0 + pixel[1]))(0) > cb)
                                                if((*(cache_0 + pixel[13]))(0) > cb)
                                                    if((*(cache_0 + pixel[14]))(0) > cb)
                                                        if((*(cache_0 + pixel[15]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                            if((*(cache_0 + pixel[14]))(0) > cb)
                                                if((*(cache_0 + pixel[15]))(0) > cb)
                                                    if((*(cache_0 + pixel[1]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if((*(cache_0 + 3))(0) < c_b)
                                        if((*(cache_0 + pixel[10]))(0) > cb)
                                            if((*(cache_0 + pixel[11]))(0) > cb)
                                                if((*(cache_0 + -3))(0) > cb)
                                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                                        if((*(cache_0 + pixel[14]))(0) > cb)
                                                            if((*(cache_0 + pixel[1]))(0) > cb)
                                                                if((*(cache_0 + pixel[15]))(0) > cb)
                                                                    goto success;
                                                                else
                                                                if((*(cache_0 + pixel[7]))(0) > cb)
                                                                    if((*(cache_0 + pixel[9]))(0) > cb)
                                                                        goto success;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                            if((*(cache_0 + pixel[7]))(0) > cb)
                                                                if((*(cache_0 + pixel[9]))(0) > cb)
                                                                    goto success;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                    if((*(cache_0 + -3))(0) > cb)
                                        if((*(cache_0 + pixel[14]))(0) > cb)
                                            if((*(cache_0 + pixel[10]))(0) > cb)
                                                if((*(cache_0 + pixel[11]))(0) > cb)
                                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                                        if((*(cache_0 + pixel[1]))(0) > cb)
                                                            if((*(cache_0 + pixel[7]))(0) > cb)
                                                                if((*(cache_0 + pixel[9]))(0) > cb)
                                                                    goto success;
                                                                else
                                                                if((*(cache_0 + pixel[15]))(0) > cb)
                                                                    goto success;
                                                                else
                                                                    continue;
                                                            else
                                                            if((*(cache_0 + pixel[15]))(0) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else if((*(cache_0 + pixel[1]))(0) < c_b)
                                                            if((*(cache_0 + pixel[7]))(0) > cb)
                                                                if((*(cache_0 + pixel[9]))(0) > cb)
                                                                    goto success;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                        if((*(cache_0 + pixel[9]))(0) > cb)
                                                            if((*(cache_0 + pixel[7]))(0) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if((*(cache_0 + pixel[6]))(0) < c_b)
                                    if((*(cache_0 + -3))(0) > cb)
                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                            if((*(cache_0 + pixel[14]))(0) > cb)
                                                if((*(cache_0 + pixel[15]))(0) > cb)
                                                    if((*(cache_0 + pixel[1]))(0) > cb)
                                                        if((*(cache_0 + 3))(0) > cb)
                                                            goto success;
                                                        else
                                                        if((*(cache_0 + pixel[10]))(0) > cb)
                                                            if((*(cache_0 + pixel[11]))(0) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                    if((*(cache_0 + pixel[7]))(0) > cb)
                                                        if((*(cache_0 + pixel[9]))(0) > cb)
                                                            if(((*(cache_0 + pixel[10])))(0) > cb)
                                                                if((*(cache_0 + pixel[11]))(0) > cb)
                                                                    goto success;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                if((*(cache_0 + -3))(0) > cb)
                                    if((*(cache_0 + pixel[14]))(0) > cb)
                                        if((*(cache_0 + pixel[15]))(0) > cb)
                                            if((*(cache_0 + pixel[13]))(0) > cb)
                                                if((*(cache_0 + pixel[1]))(0) > cb)
                                                    if((*(cache_0 + 3))(0) > cb)
                                                        goto success;
                                                    else
                                                    if((*(cache_0 + pixel[10]))(0) > cb)
                                                        if((*(cache_0 + pixel[11]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if((*(cache_0 + pixel[1]))(0) < c_b)
                                                    if((*(cache_0 + pixel[7]))(0) > cb)
                                                        if((*(cache_0 + pixel[9]))(0) > cb)
                                                            if((*(cache_0 + pixel[10]))(0) > cb)
                                                                if((*(cache_0 + pixel[11]))(0) > cb)
                                                                    goto success;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + pixel[7]))(0) > cb)
                                                    if((*(cache_0 + pixel[10]))(0) > cb)
                                                        if((*(cache_0 + pixel[11]))(0) > cb)
                                                            if((*(cache_0 + pixel[9]))(0) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if((*(cache_0 + pixel[2]))(0) < c_b)
                                if((*(cache_0 + -3))(0) > cb)
                                    if((*(cache_0 + pixel[9]))(0) > cb)
                                        if((*(cache_0 + pixel[10]))(0) > cb)
                                            if((*(cache_0 + pixel[11]))(0) > cb)
                                                if((*(cache_0 + pixel[7]))(0) > cb)
                                                    if((*(cache_0 + pixel[6]))(0) > cb)
                                                        if((*(cache_0 + 3))(0) > cb)
                                                            goto success;
                                                        else
                                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                                            if((*(cache_0 + pixel[14]))(0) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                                        if((*(cache_0 + pixel[14]))(0) > cb)
                                                            if((*(cache_0 + pixel[15]))(0) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + pixel[1]))(0) > cb)
                                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                                        if((*(cache_0 + pixel[14]))(0) > cb)
                                                            if((*(cache_0 + pixel[15]))(0) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                            if((*(cache_0 + pixel[11]))(0) > cb)
                                if((*(cache_0 + pixel[10]))(0) > cb)
                                    if((*(cache_0 + -3))(0) > cb)
                                        if((*(cache_0 + pixel[9]))(0) > cb)
                                            if((*(cache_0 + pixel[7]))(0) > cb)
                                                if((*(cache_0 + pixel[6]))(0) > cb)
                                                    if(((*(cache_0 + 3)))(0) > cb)
                                                        goto success;
                                                    else if((*(cache_0 + 3))(0) < c_b)
                                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                                            if((*(cache_0 + pixel[14]))(0) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                    if((*(cache_0 + pixel[14]))(0) > cb)
                                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if((*(cache_0 + pixel[6]))(0) < c_b)
                                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                                        if((*(cache_0 + pixel[14]))(0) > cb)
                                                            if((*(cache_0 + pixel[15]))(0) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + pixel[14]))(0) > cb)
                                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                                        if((*(cache_0 + pixel[15]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if((*(cache_0 + pixel[7]))(0) < c_b)
                                                if((*(cache_0 + pixel[1]))(0) > cb)
                                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                                        if((*(cache_0 + pixel[14]))(0) > cb)
                                                            if((*(cache_0 + pixel[15]))(0) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[14]))(0) > cb)
                                                if((*(cache_0 + pixel[1]))(0) > cb)
                                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                                        if((*(cache_0 + pixel[15]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if((*(cache_0 + pixel[5]))(0) < c_b)
                            if((*(cache_0 + pixel[13]))(0) > cb)
                                if((*(cache_0 + pixel[11]))(0) > cb)
                                    if((*(cache_0 + -3))(0) > cb)
                                        if((*(cache_0 + pixel[14]))(0) > cb)
                                            if((*(cache_0 + pixel[15]))(0) > cb)
                                                if((*(cache_0 + pixel[10]))(0) > cb)
                                                    if((*(cache_0 + pixel[9]))(0) > cb)
                                                        if((*(cache_0 + pixel[1]))(0) > cb)
                                                            goto success;
                                                        else
                                                        if((*(cache_0 + pixel[7]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                    if((*(cache_0 + pixel[1]))(0) > cb)
                                                        if((*(cache_0 + pixel[2]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + pixel[1]))(0) > cb)
                                                    if((*(cache_0 + pixel[2]))(0) > cb)
                                                        if((*(cache_0 + 3))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                        if((*(cache_0 + -3))(0) > cb)
                            if((*(cache_0 + pixel[14]))(0) > cb)
                                if((*(cache_0 + pixel[11]))(0) > cb)
                                    if((*(cache_0 + pixel[15]))(0) > cb)
                                        if((*(cache_0 + pixel[10]))(0) > cb)
                                            if((*(cache_0 + pixel[13]))(0) > cb)
                                                if((*(cache_0 + pixel[1]))(0) > cb)
                                                    if((*(cache_0 + pixel[2]))(0) > cb)
                                                        goto success;
                                                    else
                                                    if((*(cache_0 + pixel[9]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + pixel[7]))(0) > cb)
                                                    if((*(cache_0 + pixel[9]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if((*(cache_0 + pixel[10]))(0) < c_b)
                                            if((*(cache_0 + pixel[1]))(0) > cb)
                                                if((*(cache_0 + pixel[2]))(0) > cb)
                                                    if((*(cache_0 + 3))(0) > cb)
                                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                        if((*(cache_0 + 3))(0) > cb)
                                            if((*(cache_0 + pixel[2]))(0) > cb)
                                                if((*(cache_0 + pixel[1]))(0) > cb)
                                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if((*(cache_0 + pixel[3]))(0) < c_b)
                        if((*(cache_0 + -3))(0) > cb)
                            if((*(cache_0 + pixel[10]))(0) > cb)
                                if((*(cache_0 + pixel[13]))(0) > cb)
                                    if((*(cache_0 + pixel[9]))(0) > cb)
                                        if((*(cache_0 + pixel[11]))(0) > cb)
                                            if((*(cache_0 + pixel[14]))(0) > cb)
                                                if((*(cache_0 + pixel[15]))(0) > cb)
                                                    if((*(cache_0 + pixel[7]))(0) > cb)
                                                        goto success;
                                                    else
                                                    if((*(cache_0 + pixel[1]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + pixel[5]))(0) > cb)
                                                    if((*(cache_0 + pixel[6]))(0) > cb)
                                                        if((*(cache_0 + pixel[7]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + 3))(0) > cb)
                                                if((*(cache_0 + pixel[5]))(0) > cb)
                                                    if((*(cache_0 + pixel[6]))(0) > cb)
                                                        if((*(cache_0 + pixel[7]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                    if((*(cache_0 + -3))(0) > cb)
                        if((*(cache_0 + pixel[10]))(0) > cb)
                            if((*(cache_0 + pixel[14]))(0) > cb)
                                if((*(cache_0 + pixel[11]))(0) > cb)
                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                        if((*(cache_0 + pixel[9]))(0) > cb)
                                            if((*(cache_0 + pixel[7]))(0) > cb)
                                                if((*(cache_0 + pixel[15]))(0) > cb)
                                                    goto success;
                                                else
                                                if((*(cache_0 + pixel[5]))(0) > cb)
                                                    if((*(cache_0 + pixel[6]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[1]))(0) > cb)
                                                if((*(cache_0 + pixel[15]))(0) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if((*(cache_0 + pixel[14]))(0) < c_b)
                                if((*(cache_0 + 3))(0) > cb)
                                    if((*(cache_0 + pixel[5]))(0) > cb)
                                        if((*(cache_0 + pixel[6]))(0) > cb)
                                            if((*(cache_0 + pixel[7]))(0) > cb)
                                                if((*(cache_0 + pixel[9]))(0) > cb)
                                                    if((*(cache_0 + pixel[11]))(0) > cb)
                                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                            if((*(cache_0 + 3))(0) > cb)
                                if((*(cache_0 + pixel[13]))(0) > cb)
                                    if((*(cache_0 + pixel[6]))(0) > cb)
                                        if((*(cache_0 + pixel[11]))(0) > cb)
                                            if((*(cache_0 + pixel[7]))(0) > cb)
                                                if((*(cache_0 + pixel[5]))(0) > cb)
                                                    if((*(cache_0 + pixel[9]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        continue;
                else if((*(cache_0 + pixel[8]))(0) < c_b)
                    if((*(cache_0 + pixel[11]))(0) > cb)
                        if((*(cache_0 + pixel[2]))(0) > cb)
                            if((*(cache_0 + pixel[15]))(0) > cb)
                                if((*(cache_0 + pixel[1]))(0) > cb)
                                    if((*(cache_0 + pixel[14]))(0) > cb)
                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                            if((*(cache_0 + pixel[3]))(0) > cb)
                                                if((*(cache_0 + -3))(0) > cb)
                                                    if((*(cache_0 + 3))(0) > cb)
                                                        goto success;
                                                    else
                                                    if((*(cache_0 + pixel[10]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + 3))(0) > cb)
                                                    if((*(cache_0 + pixel[5]))(0) > cb)
                                                        if((*(cache_0 + pixel[6]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[9]))(0) > cb)
                                                if((*(cache_0 + pixel[10]))(0) > cb)
                                                    if((*(cache_0 + -3))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                        if((*(cache_0 + pixel[3]))(0) > cb)
                                            if((*(cache_0 + 3))(0) > cb)
                                                if((*(cache_0 + pixel[5]))(0) > cb)
                                                    if((*(cache_0 + pixel[6]))(0) > cb)
                                                        if((*(cache_0 + pixel[7]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if((*(cache_0 + pixel[2]))(0) < c_b)
                            if((*(cache_0 + pixel[1]))(0) < c_b)
                                if((*(cache_0 + pixel[3]))(0) < c_b)
                                    if((*(cache_0 + 3))(0) < c_b)
                                        if((*(cache_0 + pixel[5]))(0) < c_b)
                                            if((*(cache_0 + pixel[6]))(0) < c_b)
                                                if((*(cache_0 + pixel[7]))(0) < c_b)
                                                    if((*(cache_0 + pixel[9]))(0) < c_b)
                                                        if((*(cache_0 + pixel[10]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if((*(cache_0 + pixel[11]))(0) < c_b)
                        if((*(cache_0 + pixel[6]))(0) > cb)
                            if((*(cache_0 + pixel[14]))(0) > cb)
                                if((*(cache_0 + pixel[3]))(0) > cb)
                                    if((*(cache_0 + pixel[1]))(0) > cb)
                                        if((*(cache_0 + pixel[2]))(0) > cb)
                                            if((*(cache_0 + 3))(0) > cb)
                                                if((*(cache_0 + pixel[5]))(0) > cb)
                                                    if((*(cache_0 + pixel[15]))(0) > cb)
                                                        if((*(cache_0 + pixel[7]))(0) > cb)
                                                            goto success;
                                                        else
                                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if((*(cache_0 + pixel[6]))(0) < c_b)
                            if((*(cache_0 + pixel[10]))(0) > cb)
                                if((*(cache_0 + pixel[1]))(0) > cb)
                                    if((*(cache_0 + pixel[2]))(0) > cb)
                                        if((*(cache_0 + pixel[3]))(0) > cb)
                                            if((*(cache_0 + 3))(0) > cb)
                                                if((*(cache_0 + pixel[5]))(0) > cb)
                                                    if((*(cache_0 + -3))(0) > cb)
                                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                                            if((*(cache_0 + pixel[14]))(0) > cb)
                                                                if((*(cache_0 + pixel[15]))(0) > cb)
                                                                    goto success;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if((*(cache_0 + pixel[10]))(0) < c_b)
                                if((*(cache_0 + pixel[5]))(0) > cb)
                                    if((*(cache_0 + pixel[7]))(0) > cb)
                                        if((*(cache_0 + pixel[1]))(0) > cb)
                                            if((*(cache_0 + pixel[2]))(0) > cb)
                                                if((*(cache_0 + pixel[3]))(0) > cb)
                                                    if((*(cache_0 + 3))(0) > cb)
                                                        if((*(cache_0 + -3))(0) > cb)
                                                            if((*(cache_0 + pixel[13]))(0) > cb)
                                                                if((*(cache_0 + pixel[14]))(0) > cb)
                                                                    if((*(cache_0 + pixel[15]))(0) > cb)
                                                                        goto success;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if((*(cache_0 + pixel[7]))(0) < c_b)
                                        if((*(cache_0 + pixel[14]))(0) > cb)
                                            if((*(cache_0 + -3))(0) > cb)
                                                if((*(cache_0 + pixel[1]))(0) > cb)
                                                    if((*(cache_0 + pixel[2]))(0) > cb)
                                                        if((*(cache_0 + pixel[3]))(0) > cb)
                                                            if((*(cache_0 + 3))(0) > cb)
                                                                if((*(cache_0 + pixel[13]))(0) > cb)
                                                                    if((*(cache_0 + pixel[15]))(0) > cb)
                                                                        goto success;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if((*(cache_0 + pixel[14]))(0) < c_b)
                                            if((*(cache_0 + pixel[9]))(0) < c_b)
                                                if((*(cache_0 + -3))(0) < c_b)
                                                    if((*(cache_0 + pixel[13]))(0) < c_b)
                                                        if((*(cache_0 + pixel[15]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                    if((*(cache_0 + -3))(0) > cb)
                                        if((*(cache_0 + pixel[1]))(0) > cb)
                                            if((*(cache_0 + pixel[2]))(0) > cb)
                                                if((*(cache_0 + pixel[3]))(0) > cb)
                                                    if((*(cache_0 + 3))(0) > cb)
                                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                                            if((*(cache_0 + pixel[14]))(0) > cb)
                                                                if((*(cache_0 + pixel[15]))(0) > cb)
                                                                    goto success;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if((*(cache_0 + pixel[5]))(0) < c_b)
                                    if((*(cache_0 + -3))(0) > cb)
                                        if((*(cache_0 + pixel[2]))(0) < c_b)
                                            if((*(cache_0 + pixel[3]))(0) < c_b)
                                                if((*(cache_0 + 3))(0) < c_b)
                                                    if((*(cache_0 + pixel[7]))(0) < c_b)
                                                        if((*(cache_0 + pixel[9]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if((*(cache_0 + -3))(0) < c_b)
                                        if((*(cache_0 + pixel[9]))(0) < c_b)
                                            if((*(cache_0 + 3))(0) > cb)
                                            if((*(cache_0 + 3))(0) > cb)
                                                if((*(cache_0 + pixel[7]))(0) < c_b)
                                                    if((*(cache_0 + pixel[13]))(0) < c_b)
                                                        if((*(cache_0 + pixel[14]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if((*(cache_0 + 3))(0) < c_b)
                                                if((*(cache_0 + pixel[7]))(0) < c_b)
                                                    if((*(cache_0 + pixel[13]))(0) < c_b)
                                                        goto success;
                                                    else
                                                    if((*(cache_0 + pixel[3]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[14]))(0) < c_b)
                                                if((*(cache_0 + pixel[13]))(0) < c_b)
                                                    if((*(cache_0 + pixel[7]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                    if((*(cache_0 + pixel[2]))(0) < c_b)
                                        if((*(cache_0 + pixel[7]))(0) < c_b)
                                            if((*(cache_0 + pixel[3]))(0) < c_b)
                                                if((*(cache_0 + pixel[9]))(0) < c_b)
                                                    if((*(cache_0 + 3))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                if((*(cache_0 + pixel[15]))(0) < c_b)
                                    if((*(cache_0 + pixel[14]))(0) < c_b)
                                        if((*(cache_0 + pixel[7]))(0) < c_b)
                                            if((*(cache_0 + pixel[9]))(0) < c_b)
                                                if((*(cache_0 + -3))(0) < c_b)
                                                    if((*(cache_0 + pixel[13]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                            if((*(cache_0 + -3))(0) > cb)
                                if((*(cache_0 + pixel[1]))(0) > cb)
                                    if((*(cache_0 + pixel[2]))(0) > cb)
                                        if((*(cache_0 + pixel[3]))(0) > cb)
                                            if((*(cache_0 + 3))(0) > cb)
                                                if((*(cache_0 + pixel[5]))(0) > cb)
                                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                                        if((*(cache_0 + pixel[14]))(0) > cb)
                                                            if((*(cache_0 + pixel[15]))(0) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                        if((*(cache_0 + -3))(0) > cb)
                            if((*(cache_0 + pixel[3]))(0) > cb)
                                if((*(cache_0 + pixel[1]))(0) > cb)
                                    if((*(cache_0 + pixel[2]))(0) > cb)
                                        if((*(cache_0 + 3))(0) > cb)
                                            if((*(cache_0 + pixel[5]))(0) > cb)
                                                if((*(cache_0 + pixel[13]))(0) > cb)
                                                    if((*(cache_0 + pixel[14]))(0) > cb)
                                                        if((*(cache_0 + pixel[15]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                    if((*(cache_0 + pixel[3]))(0) > cb)
                        if((*(cache_0 + pixel[5]))(0) > cb)
                            if((*(cache_0 + pixel[14]))(0) > cb)
                                if((*(cache_0 + pixel[15]))(0) > cb)
                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                        if((*(cache_0 + pixel[1]))(0) > cb)
                                            if((*(cache_0 + pixel[2]))(0) > cb)
                                                if((*(cache_0 + 3))(0) > cb)
                                                    if((*(cache_0 + pixel[6]))(0) > cb)
                                                        goto success;
                                                    else
                                                    if((*(cache_0 + -3))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if((*(cache_0 + pixel[13]))(0) < c_b)
                                        if((*(cache_0 + pixel[6]))(0) > cb)
                                            if((*(cache_0 + pixel[1]))(0) > cb)
                                                if((*(cache_0 + pixel[2]))(0) > cb)
                                                    if((*(cache_0 + 3))(0) > cb)
                                                        if((*(cache_0 + pixel[7]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                    if((*(cache_0 + pixel[7]))(0) > cb)
                                        if((*(cache_0 + pixel[1]))(0) > cb)
                                            if((*(cache_0 + pixel[2]))(0) > cb)
                                                if((*(cache_0 + 3))(0) > cb)
                                                    if((*(cache_0 + pixel[6]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if((*(cache_0 + pixel[3]))(0) < c_b)
                        if((*(cache_0 + pixel[1]))(0) < c_b)
                            if((*(cache_0 + pixel[10]))(0) < c_b)
                                if((*(cache_0 + pixel[2]))(0) < c_b)
                                    if((*(cache_0 + 3))(0) < c_b)
                                        if((*(cache_0 + pixel[5]))(0) < c_b)
                                            if((*(cache_0 + pixel[6]))(0) < c_b)
                                                if((*(cache_0 + pixel[7]))(0) < c_b)
                                                    if((*(cache_0 + pixel[9]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        continue;
                else
                if((*(cache_0 + pixel[3]))(0) > cb)
                    if((*(cache_0 + pixel[14]))(0) > cb)
                        if((*(cache_0 + -3))(0) > cb)
                            if((*(cache_0 + pixel[2]))(0) > cb)
                                if((*(cache_0 + 3))(0) > cb)
                                    if((*(cache_0 + pixel[15]))(0) > cb)
                                        if((*(cache_0 + pixel[1]))(0) > cb)
                                            if((*(cache_0 + pixel[13]))(0) > cb)
                                                if((*(cache_0 + pixel[11]))(0) > cb)
                                                    goto success;
                                                else
                                                if((*(cache_0 + pixel[5]))(0) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else if((*(cache_0 + pixel[13]))(0) < c_b)
                                                if((*(cache_0 + pixel[5]))(0) > cb)
                                                    if((*(cache_0 + pixel[6]))(0) > cb)
                                                        if((*(cache_0 + pixel[7]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[7]))(0) > cb)
                                                if((*(cache_0 + pixel[5]))(0) > cb)
                                                    if((*(cache_0 + pixel[6]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if((*(cache_0 + 3))(0) < c_b)
                                    if((*(cache_0 + pixel[1]))(0) > cb)
                                        if((*(cache_0 + pixel[10]))(0) > cb)
                                            if((*(cache_0 + pixel[11]))(0) > cb)
                                                if((*(cache_0 + pixel[13]))(0) > cb)
                                                    if((*(cache_0 + pixel[15]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                if((*(cache_0 + pixel[10]))(0) > cb)
                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                        if((*(cache_0 + pixel[11]))(0) > cb)
                                            if((*(cache_0 + pixel[15]))(0) > cb)
                                                if((*(cache_0 + pixel[1]))(0) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if((*(cache_0 + -3))(0) < c_b)
                            if((*(cache_0 + pixel[6]))(0) > cb)
                                if((*(cache_0 + pixel[1]))(0) > cb)
                                    if((*(cache_0 + pixel[2]))(0) > cb)
                                        if((*(cache_0 + 3))(0) > cb)
                                            if((*(cache_0 + pixel[5]))(0) > cb)
                                                if((*(cache_0 + pixel[15]))(0) > cb)
                                                    if((*(cache_0 + pixel[7]))(0) > cb)
                                                        goto success;
                                                    else
                                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                        if((*(cache_0 + pixel[6]))(0) > cb)
                            if((*(cache_0 + pixel[2]))(0) > cb)
                                if((*(cache_0 + pixel[5]))(0) > cb)
                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                        if((*(cache_0 + pixel[15]))(0) > cb)
                                            if((*(cache_0 + 3))(0) > cb)
                                                if((*(cache_0 + pixel[1]))(0) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if((*(cache_0 + pixel[13]))(0) < c_b)
                                        if((*(cache_0 + pixel[1]))(0) > cb)
                                            if((*(cache_0 + 3))(0) > cb)
                                                if((*(cache_0 + pixel[7]))(0) > cb)
                                                    if((*(cache_0 + pixel[15]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                    if((*(cache_0 + pixel[7]))(0) > cb)
                                        if((*(cache_0 + pixel[15]))(0) > cb)
                                            if((*(cache_0 + 3))(0) > cb)
                                                if((*(cache_0 + pixel[1]))(0) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        continue;
                else if((*(cache_0 + pixel[3]))(0) < c_b)
                    if((*(cache_0 + pixel[2]))(0) > cb)
                        if((*(cache_0 + pixel[9]))(0) > cb)
                            if((*(cache_0 + pixel[1]))(0) > cb)
                                if((*(cache_0 + pixel[10]))(0) > cb)
                                    if((*(cache_0 + pixel[11]))(0) > cb)
                                        if((*(cache_0 + -3))(0) > cb)
                                            if((*(cache_0 + pixel[13]))(0) > cb)
                                                if((*(cache_0 + pixel[14]))(0) > cb)
                                                    if((*(cache_0 + pixel[15]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        continue;
                else
                if((*(cache_0 + pixel[9]))(0) > cb)
                    if((*(cache_0 + pixel[2]))(0) > cb)
                        if((*(cache_0 + -3))(0) > cb)
                            if((*(cache_0 + pixel[14]))(0) > cb)
                                if((*(cache_0 + pixel[11]))(0) > cb)
                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                        if((*(cache_0 + pixel[15]))(0) > cb)
                                            if((*(cache_0 + pixel[10]))(0) > cb)
                                                if((*(cache_0 + pixel[1]))(0) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        continue;
                else
                    continue;
            else if((*(cache_0 + pixel[0]))(0) < c_b)
                if((*(cache_0 + pixel[8]))(0) > cb)
                    if((*(cache_0 + pixel[2]))(0) > cb)
                        if((*(cache_0 + pixel[10]))(0) > cb)
                            if((*(cache_0 + pixel[6]))(0) > cb)
                                if((*(cache_0 + pixel[7]))(0) > cb)
                                    if((*(cache_0 + pixel[9]))(0) > cb)
                                        if((*(cache_0 + pixel[5]))(0) > cb)
                                            if((*(cache_0 + pixel[11]))(0) > cb)
                                                if((*(cache_0 + 3))(0) > cb)
                                                    if((*(cache_0 + pixel[3]))(0) > cb)
                                                        goto success;
                                                    else
                                                    if((*(cache_0 + -3))(0) > cb)
                                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + -3))(0) > cb)
                                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                                        if((*(cache_0 + pixel[14]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[1]))(0) > cb)
                                                if((*(cache_0 + pixel[3]))(0) > cb)
                                                    if((*(cache_0 + 3))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if((*(cache_0 + pixel[5]))(0) < c_b)
                                            if((*(cache_0 + pixel[11]))(0) > cb)
                                                if((*(cache_0 + -3))(0) > cb)
                                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                                        if((*(cache_0 + pixel[14]))(0) > cb)
                                                            if((*(cache_0 + pixel[15]))(0) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                            if((*(cache_0 + pixel[11]))(0) > cb)
                                                if((*(cache_0 + -3))(0) > cb)
                                                    if((*(cache_0 + pixel[14]))(0) > cb)
                                                        if((*(cache_0 + pixel[15]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if((*(cache_0 + pixel[2]))(0) < c_b)
                        if((*(cache_0 + pixel[13]))(0) > cb)
                            if((*(cache_0 + pixel[6]))(0) > cb)
                                if((*(cache_0 + pixel[11]))(0) > cb)
                                    if((*(cache_0 + pixel[9]))(0) > cb)
                                        if((*(cache_0 + pixel[7]))(0) > cb)
                                            if((*(cache_0 + pixel[10]))(0) > cb)
                                                if((*(cache_0 + pixel[5]))(0) > cb)
                                                    if((*(cache_0 + -3))(0) > cb)
                                                        if((*(cache_0 + 3))(0) > cb)
                                                            goto success;
                                                        else
                                                        if((*(cache_0 + pixel[14]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + pixel[15]))(0) > cb)
                                                    if((*(cache_0 + -3))(0) > cb)
                                                        if((*(cache_0 + pixel[14]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if((*(cache_0 + pixel[6]))(0) < c_b)
                                if((*(cache_0 + pixel[7]))(0) < c_b)
                                    if((*(cache_0 + pixel[1]))(0) < c_b)
                                        if((*(cache_0 + pixel[3]))(0) < c_b)
                                            if((*(cache_0 + 3))(0) < c_b)
                                                if((*(cache_0 + pixel[5]))(0) < c_b)
                                                    if((*(cache_0 + pixel[14]))(0) < c_b)
                                                        if((*(cache_0 + pixel[15]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if((*(cache_0 + pixel[13]))(0) < c_b)
                            if((*(cache_0 + pixel[3]))(0) > cb)
                                if((*(cache_0 + pixel[10]))(0) > cb)
                                    if((*(cache_0 + pixel[7]))(0) > cb)
                                        if((*(cache_0 + 3))(0) > cb)
                                            if((*(cache_0 + pixel[5]))(0) > cb)
                                                if((*(cache_0 + pixel[6]))(0) > cb)
                                                    if((*(cache_0 + pixel[9]))(0) > cb)
                                                        if((*(cache_0 + pixel[11]))(0) > cb)
                                                            if((*(cache_0 + -3))(0) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if((*(cache_0 + pixel[10]))(0) < c_b)
                                    if((*(cache_0 + pixel[9]))(0) < c_b)
                                        if((*(cache_0 + pixel[1]))(0) < c_b)
                                            if((*(cache_0 + pixel[11]))(0) < c_b)
                                                if((*(cache_0 + -3))(0) < c_b)
                                                    if((*(cache_0 + pixel[14]))(0) < c_b)
                                                        if((*(cache_0 + pixel[15]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if((*(cache_0 + pixel[3]))(0) < c_b)
                                if((*(cache_0 + pixel[15]))(0) < c_b)
                                    if((*(cache_0 + pixel[1]))(0) < c_b)
                                        if((*(cache_0 + pixel[5]))(0) > cb)
                                            if((*(cache_0 + pixel[10]))(0) < c_b)
                                                if((*(cache_0 + pixel[14]))(0) < c_b)
                                                    if((*(cache_0 + pixel[11]))(0) < c_b)
                                                        if((*(cache_0 + -3))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + 3))(0) < c_b)
                                                if((*(cache_0 + pixel[11]))(0) < c_b)
                                                    if((*(cache_0 + -3))(0) < c_b)
                                                        if((*(cache_0 + pixel[14]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if((*(cache_0 + pixel[5]))(0) < c_b)
                                            if((*(cache_0 + 3))(0) < c_b)
                                                if((*(cache_0 + pixel[6]))(0) < c_b)
                                                    if((*(cache_0 + pixel[14]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + -3))(0) < c_b)
                                                    if((*(cache_0 + pixel[14]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[10]))(0) < c_b)
                                                if((*(cache_0 + pixel[11]))(0) < c_b)
                                                    if((*(cache_0 + -3))(0) < c_b)
                                                        if((*(cache_0 + pixel[14]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                        if((*(cache_0 + pixel[11]))(0) < c_b)
                                            if((*(cache_0 + pixel[10]))(0) > cb)
                                                if((*(cache_0 + 3))(0) < c_b)
                                                    if((*(cache_0 + -3))(0) < c_b)
                                                        if((*(cache_0 + pixel[14]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if((*(cache_0 + pixel[10]))(0) < c_b)
                                                if((*(cache_0 + pixel[14]))(0) < c_b)
                                                    if((*(cache_0 + -3))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + 3))(0) < c_b)
                                                if((*(cache_0 + pixel[14]))(0) < c_b)
                                                    if((*(cache_0 + -3))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                            if((*(cache_0 + pixel[9]))(0) < c_b)
                                if((*(cache_0 + pixel[11]))(0) < c_b)
                                    if((*(cache_0 + pixel[1]))(0) < c_b)
                                        if((*(cache_0 + pixel[10]))(0) < c_b)
                                            if((*(cache_0 + -3))(0) < c_b)
                                                if((*(cache_0 + pixel[14]))(0) < c_b)
                                                    if((*(cache_0 + pixel[15]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                        if((*(cache_0 + pixel[7]))(0) > cb)
                            if((*(cache_0 + pixel[3]))(0) > cb)
                                if((*(cache_0 + pixel[10]))(0) > cb)
                                    if((*(cache_0 + 3))(0) > cb)
                                        if((*(cache_0 + pixel[5]))(0) > cb)
                                            if((*(cache_0 + pixel[6]))(0) > cb)
                                                if((*(cache_0 + pixel[9]))(0) > cb)
                                                    if((*(cache_0 + pixel[11]))(0) > cb)
                                                        if((*(cache_0 + -3))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if((*(cache_0 + pixel[7]))(0) < c_b)
                            if((*(cache_0 + pixel[1]))(0) < c_b)
                                if((*(cache_0 + pixel[3]))(0) < c_b)
                                    if((*(cache_0 + 3))(0) < c_b)
                                        if((*(cache_0 + pixel[5]))(0) < c_b)
                                            if((*(cache_0 + pixel[6]))(0) < c_b)
                                                if((*(cache_0 + pixel[14]))(0) < c_b)
                                                    if((*(cache_0 + pixel[15]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                    if((*(cache_0 + -3))(0) > cb)
                        if((*(cache_0 + pixel[6]))(0) > cb)
                            if((*(cache_0 + pixel[11]))(0) > cb)
                                if((*(cache_0 + pixel[9]))(0) > cb)
                                    if((*(cache_0 + pixel[10]))(0) > cb)
                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                            if((*(cache_0 + pixel[7]))(0) > cb)
                                                if((*(cache_0 + pixel[5]))(0) > cb)
                                                    if((*(cache_0 + 3))(0) > cb)
                                                        goto success;
                                                    else
                                                    if((*(cache_0 + pixel[14]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + pixel[15]))(0) > cb)
                                                    if((*(cache_0 + pixel[14]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                        if((*(cache_0 + pixel[3]))(0) > cb)
                                            if((*(cache_0 + 3))(0) > cb)
                                                if((*(cache_0 + pixel[5]))(0) > cb)
                                                    if((*(cache_0 + pixel[7]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        continue;
                else if((*(cache_0 + pixel[8]))(0) < c_b)
                    if((*(cache_0 + 3))(0) > cb)
                        if((*(cache_0 + -3))(0) < c_b)
                            if((*(cache_0 + pixel[10]))(0) < c_b)
                                if((*(cache_0 + pixel[14]))(0) < c_b)
                                    if((*(cache_0 + pixel[15]))(0) < c_b)
                                        if((*(cache_0 + pixel[13]))(0) < c_b)
                                            if((*(cache_0 + pixel[1]))(0) < c_b)
                                                if((*(cache_0 + pixel[11]))(0) < c_b)
                                                    if((*(cache_0 + pixel[9]))(0) > cb)
                                                        if((*(cache_0 + pixel[2]))(0) < c_b)
                                                            if((*(cache_0 + pixel[3]))(0) < c_b)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else if((*(cache_0 + pixel[9]))(0) < c_b)
                                                        goto success;
                                                    else
                                                    if((*(cache_0 + pixel[3]))(0) < c_b)
                                                        if((*(cache_0 + pixel[2]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[7]))(0) < c_b)
                                                if((*(cache_0 + pixel[9]))(0) < c_b)
                                                    if((*(cache_0 + pixel[11]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                    if((*(cache_0 + pixel[5]))(0) < c_b)
                                        if((*(cache_0 + pixel[6]))(0) < c_b)
                                            if((*(cache_0 + pixel[7]))(0) < c_b)
                                                if((*(cache_0 + pixel[9]))(0) < c_b)
                                                    if((*(cache_0 + pixel[11]))(0) < c_b)
                                                        if((*(cache_0 + pixel[13]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if((*(cache_0 + 3))(0) < c_b)
                        if((*(cache_0 + pixel[2]))(0) > cb)
                            if((*(cache_0 + pixel[10]))(0) < c_b)
                                if((*(cache_0 + -3))(0) < c_b)
                                    if((*(cache_0 + pixel[11]))(0) < c_b)
                                        if((*(cache_0 + pixel[9]))(0) < c_b)
                                            if((*(cache_0 + pixel[13]))(0) < c_b)
                                                if((*(cache_0 + pixel[14]))(0) < c_b)
                                                    if((*(cache_0 + pixel[7]))(0) < c_b)
                                                        if((*(cache_0 + pixel[15]))(0) > cb)
                                                            if((*(cache_0 + pixel[5]))(0) < c_b)
                                                                if((*(cache_0 + pixel[6]))(0) < c_b)
                                                                    goto success;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else if((*(cache_0 + pixel[15]))(0) < c_b)
                                                            goto success;
                                                        else
                                                        if((*(cache_0 + pixel[6]))(0) < c_b)
                                                            if((*(cache_0 + pixel[5]))(0) < c_b)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                    if((*(cache_0 + pixel[1]))(0) < c_b)
                                                        if((*(cache_0 + pixel[15]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + pixel[5]))(0) < c_b)
                                                    if((*(cache_0 + pixel[6]))(0) < c_b)
                                                        if((*(cache_0 + pixel[7]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[3]))(0) < c_b)
                                                if((*(cache_0 + pixel[5]))(0) < c_b)
                                                    if((*(cache_0 + pixel[6]))(0) < c_b)
                                                        if((*(cache_0 + pixel[7]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if((*(cache_0 + pixel[2]))(0) < c_b)
                            if((*(cache_0 + pixel[6]))(0) > cb)
                                if((*(cache_0 + pixel[13]))(0) < c_b)
                                    if((*(cache_0 + pixel[14]))(0) < c_b)
                                        if((*(cache_0 + pixel[15]))(0) < c_b)
                                            if((*(cache_0 + -3))(0) < c_b)
                                                if((*(cache_0 + pixel[1]))(0) < c_b)
                                                    if((*(cache_0 + pixel[3]))(0) < c_b)
                                                        if((*(cache_0 + pixel[11]))(0) < c_b)
                                                            goto success;
                                                        else
                                                        if((*(cache_0 + pixel[5]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                    if((*(cache_0 + pixel[9]))(0) < c_b)
                                                        if((*(cache_0 + pixel[10]))(0) < c_b)
                                                            if((*(cache_0 + pixel[11]))(0) < c_b)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + pixel[7]))(0) < c_b)
                                                    if((*(cache_0 + pixel[9]))(0) < c_b)
                                                        if((*(cache_0 + pixel[10]))(0) < c_b)
                                                            if((*(cache_0 + pixel[11]))(0) < c_b)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if((*(cache_0 + pixel[6]))(0) < c_b)
                                if((*(cache_0 + pixel[3]))(0) > cb)
                                    if((*(cache_0 + pixel[9]))(0) < c_b)
                                        if((*(cache_0 + pixel[10]))(0) < c_b)
                                            if((*(cache_0 + pixel[11]))(0) < c_b)
                                                if((*(cache_0 + -3))(0) < c_b)
                                                    if((*(cache_0 + pixel[13]))(0) < c_b)
                                                        if((*(cache_0 + pixel[7]))(0) < c_b)
                                                            if((*(cache_0 + pixel[5]))(0) < c_b)
                                                                goto success;
                                                            else
                                                            if((*(cache_0 + pixel[14]))(0) < c_b)
                                                                if((*(cache_0 + pixel[15]))(0) < c_b)
                                                                    goto success;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                        if((*(cache_0 + pixel[1]))(0) < c_b)
                                                            if((*(cache_0 + pixel[14]))(0) < c_b)
                                                                if((*(cache_0 + pixel[15]))(0) < c_b)
                                                                    goto success;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if((*(cache_0 + pixel[3]))(0) < c_b)
                                    if((*(cache_0 + pixel[5]))(0) > cb)
                                        if((*(cache_0 + pixel[11]))(0) < c_b)
                                            if((*(cache_0 + -3))(0) < c_b)
                                                if((*(cache_0 + pixel[13]))(0) < c_b)
                                                    if((*(cache_0 + pixel[14]))(0) < c_b)
                                                        if((*(cache_0 + pixel[15]))(0) < c_b)
                                                            if((*(cache_0 + pixel[1]))(0) < c_b)
                                                                goto success;
                                                            else
                                                            if((*(cache_0 + pixel[7]))(0) < c_b)
                                                                if((*(cache_0 + pixel[9]))(0) < c_b)
                                                                    if((*(cache_0 + pixel[10]))(0) < c_b)
                                                                        goto success;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if((*(cache_0 + pixel[5]))(0) < c_b)
                                        if((*(cache_0 + pixel[7]))(0) > cb)
                                            if((*(cache_0 + pixel[1]))(0) < c_b)
                                                if((*(cache_0 + pixel[13]))(0) < c_b)
                                                    if((*(cache_0 + pixel[14]))(0) < c_b)
                                                        if((*(cache_0 + pixel[15]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if((*(cache_0 + pixel[7]))(0) < c_b)
                                            if((*(cache_0 + pixel[1]))(0) > cb)
                                                if((*(cache_0 + pixel[9]))(0) < c_b)
                                                    if((*(cache_0 + pixel[10]))(0) < c_b)
                                                        if((*(cache_0 + pixel[11]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if((*(cache_0 + pixel[1]))(0) < c_b)
                                                if((*(cache_0 + pixel[9]))(0) < c_b)
                                                    goto success;
                                                else
                                                if((*(cache_0 + pixel[15]))(0) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[11]))(0) < c_b)
                                                if((*(cache_0 + pixel[10]))(0) < c_b)
                                                    if((*(cache_0 + pixel[9]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                        if((*(cache_0 + pixel[13]))(0) < c_b)
                                            if((*(cache_0 + pixel[15]))(0) < c_b)
                                                if((*(cache_0 + pixel[14]))(0) < c_b)
                                                    if((*(cache_0 + pixel[1]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                    if((*(cache_0 + -3))(0) < c_b)
                                        if((*(cache_0 + pixel[14]))(0) < c_b)
                                            if((*(cache_0 + pixel[11]))(0) < c_b)
                                                if((*(cache_0 + pixel[13]))(0) < c_b)
                                                    if((*(cache_0 + pixel[15]))(0) < c_b)
                                                        if((*(cache_0 + pixel[1]))(0) > cb)
                                                            if((*(cache_0 + pixel[7]))(0) < c_b)
                                                                if((*(cache_0 + pixel[9]))(0) < c_b)
                                                                    if((*(cache_0 + pixel[10]))(0) < c_b)
                                                                        goto success;
                                                                    else
                                                                        continue;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else if((*(cache_0 + pixel[1]))(0) < c_b)
                                                            goto success;
                                                        else
                                                        if((*(cache_0 + pixel[9]))(0) < c_b)
                                                            if((*(cache_0 + pixel[7]))(0) < c_b)
                                                                if((*(cache_0 + pixel[10]))(0) < c_b)
                                                                    goto success;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                if((*(cache_0 + pixel[11]))(0) < c_b)
                                    if((*(cache_0 + pixel[13]))(0) < c_b)
                                        if((*(cache_0 + pixel[10]))(0) < c_b)
                                            if((*(cache_0 + pixel[9]))(0) < c_b)
                                                if((*(cache_0 + -3))(0) < c_b)
                                                    if((*(cache_0 + pixel[7]))(0) > cb)
                                                        if((*(cache_0 + pixel[1]))(0) < c_b)
                                                            if((*(cache_0 + pixel[14]))(0) < c_b)
                                                                if((*(cache_0 + pixel[15]))(0) < c_b)
                                                                    goto success;
                                                                else
                                                                    continue;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else if((*(cache_0 + pixel[7]))(0) < c_b)
                                                        if((*(cache_0 + pixel[5]))(0) < c_b)
                                                            goto success;
                                                        else
                                                        if((*(cache_0 + pixel[14]))(0) < c_b)
                                                            if((*(cache_0 + pixel[15]))(0) < c_b)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                    if((*(cache_0 + pixel[15]))(0) < c_b)
                                                        if((*(cache_0 + pixel[1]))(0) < c_b)
                                                            if((*(cache_0 + pixel[14]))(0) < c_b)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                            if((*(cache_0 + -3))(0) < c_b)
                                if((*(cache_0 + pixel[14]))(0) < c_b)
                                    if((*(cache_0 + pixel[15]))(0) < c_b)
                                        if((*(cache_0 + pixel[13]))(0) < c_b)
                                            if((*(cache_0 + pixel[11]))(0) > cb)
                                                if((*(cache_0 + pixel[1]))(0) < c_b)
                                                    if((*(cache_0 + pixel[3]))(0) < c_b)
                                                        if((*(cache_0 + pixel[5]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if((*(cache_0 + pixel[11]))(0) < c_b)
                                                if((*(cache_0 + pixel[1]))(0) > cb)
                                                    if((*(cache_0 + pixel[7]))(0) < c_b)
                                                        if((*(cache_0 + pixel[9]))(0) < c_b)
                                                            if((*(cache_0 + pixel[10]))(0) < c_b)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else if((*(cache_0 + pixel[1]))(0) < c_b)
                                                    if((*(cache_0 + pixel[3]))(0) > cb)
                                                        if((*(cache_0 + pixel[9]))(0) < c_b)
                                                            if((*(cache_0 + pixel[10]))(0) < c_b)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else if((*(cache_0 + pixel[3]))(0) < c_b)
                                                        goto success;
                                                    else
                                                    if((*(cache_0 + pixel[10]))(0) < c_b)
                                                        if((*(cache_0 + pixel[9]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + pixel[7]))(0) < c_b)
                                                    if((*(cache_0 + pixel[10]))(0) < c_b)
                                                        if((*(cache_0 + pixel[9]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[5]))(0) < c_b)
                                                if((*(cache_0 + pixel[3]))(0) < c_b)
                                                    if((*(cache_0 + pixel[1]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                        if((*(cache_0 + pixel[11]))(0) < c_b)
                            if((*(cache_0 + pixel[10]))(0) < c_b)
                                if((*(cache_0 + -3))(0) < c_b)
                                    if((*(cache_0 + pixel[9]))(0) < c_b)
                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                            if((*(cache_0 + pixel[3]))(0) < c_b)
                                                if((*(cache_0 + pixel[5]))(0) < c_b)
                                                    if((*(cache_0 + pixel[6]))(0) < c_b)
                                                        if((*(cache_0 + pixel[7]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else if((*(cache_0 + pixel[13]))(0) < c_b)
                                            if((*(cache_0 + pixel[7]))(0) < c_b)
                                                if((*(cache_0 + pixel[6]))(0) < c_b)
                                                    if((*(cache_0 + pixel[5]))(0) < c_b)
                                                        goto success;
                                                    else
                                                    if((*(cache_0 + pixel[14]))(0) < c_b)
                                                        if((*(cache_0 + pixel[15]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + pixel[14]))(0) < c_b)
                                                    if((*(cache_0 + pixel[15]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[1]))(0) < c_b)
                                                if((*(cache_0 + pixel[14]))(0) < c_b)
                                                    if((*(cache_0 + pixel[15]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                        if((*(cache_0 + pixel[3]))(0) < c_b)
                                            if((*(cache_0 + pixel[6]))(0) < c_b)
                                                if((*(cache_0 + pixel[7]))(0) < c_b)
                                                    if((*(cache_0 + pixel[5]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                    if((*(cache_0 + -3))(0) < c_b)
                        if((*(cache_0 + pixel[10]))(0) < c_b)
                            if((*(cache_0 + pixel[14]))(0) < c_b)
                                if((*(cache_0 + pixel[11]))(0) < c_b)
                                    if((*(cache_0 + pixel[13]))(0) < c_b)
                                        if((*(cache_0 + pixel[15]))(0) < c_b)
                                            if((*(cache_0 + pixel[9]))(0) > cb)
                                                if((*(cache_0 + pixel[1]))(0) < c_b)
                                                    if((*(cache_0 + pixel[2]))(0) < c_b)
                                                        if((*(cache_0 + pixel[3]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if((*(cache_0 + pixel[9]))(0) < c_b)
                                                if((*(cache_0 + pixel[1]))(0) < c_b)
                                                    goto success;
                                                else
                                                if((*(cache_0 + pixel[7]))(0) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[3]))(0) < c_b)
                                                if((*(cache_0 + pixel[2]))(0) < c_b)
                                                    if((*(cache_0 + pixel[1]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                        if((*(cache_0 + pixel[5]))(0) < c_b)
                                            if((*(cache_0 + pixel[6]))(0) < c_b)
                                                if((*(cache_0 + pixel[7]))(0) < c_b)
                                                    if((*(cache_0 + pixel[9]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        continue;
                else
                if((*(cache_0 + pixel[2]))(0) < c_b)
                    if((*(cache_0 + -3))(0) > cb)
                        if((*(cache_0 + pixel[6]))(0) < c_b)
                            if((*(cache_0 + pixel[14]))(0) < c_b)
                                if((*(cache_0 + pixel[7]))(0) > cb)
                                    if((*(cache_0 + pixel[1]))(0) < c_b)
                                        if((*(cache_0 + pixel[3]))(0) < c_b)
                                            if((*(cache_0 + 3))(0) < c_b)
                                                if((*(cache_0 + pixel[5]))(0) < c_b)
                                                    if((*(cache_0 + pixel[13]))(0) < c_b)
                                                        if((*(cache_0 + pixel[15]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if((*(cache_0 + pixel[7]))(0) < c_b)
                                    if((*(cache_0 + 3))(0) < c_b)
                                        if((*(cache_0 + pixel[5]))(0) < c_b)
                                            if((*(cache_0 + pixel[1]))(0) < c_b)
                                                if((*(cache_0 + pixel[3]))(0) < c_b)
                                                    if((*(cache_0 + pixel[15]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                if((*(cache_0 + pixel[13]))(0) < c_b)
                                    if((*(cache_0 + pixel[1]))(0) < c_b)
                                        if((*(cache_0 + pixel[3]))(0) < c_b)
                                            if((*(cache_0 + 3))(0) < c_b)
                                                if((*(cache_0 + pixel[5]))(0) < c_b)
                                                    if((*(cache_0 + pixel[15]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if((*(cache_0 + -3))(0) < c_b)
                        if((*(cache_0 + pixel[3]))(0) > cb)
                            if((*(cache_0 + pixel[9]))(0) < c_b)
                                if((*(cache_0 + pixel[11]))(0) < c_b)
                                    if((*(cache_0 + pixel[14]))(0) < c_b)
                                        if((*(cache_0 + pixel[13]))(0) < c_b)
                                            if((*(cache_0 + pixel[15]))(0) < c_b)
                                                if((*(cache_0 + pixel[1]))(0) < c_b)
                                                    if((*(cache_0 + pixel[10]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if((*(cache_0 + pixel[3]))(0) < c_b)
                            if((*(cache_0 + pixel[14]))(0) < c_b)
                                if((*(cache_0 + 3))(0) > cb)
                                    if((*(cache_0 + pixel[10]))(0) < c_b)
                                        if((*(cache_0 + pixel[15]))(0) < c_b)
                                            if((*(cache_0 + pixel[1]))(0) < c_b)
                                                if((*(cache_0 + pixel[11]))(0) < c_b)
                                                    if((*(cache_0 + pixel[13]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if((*(cache_0 + 3))(0) < c_b)
                                    if((*(cache_0 + pixel[15]))(0) < c_b)
                                        if((*(cache_0 + pixel[1]))(0) < c_b)
                                            if((*(cache_0 + pixel[13]))(0) > cb)
                                                if((*(cache_0 + pixel[5]))(0) < c_b)
                                                    if((*(cache_0 + pixel[6]))(0) < c_b)
                                                        if((*(cache_0 + pixel[7]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if((*(cache_0 + pixel[13]))(0) < c_b)
                                                if((*(cache_0 + pixel[5]))(0) < c_b)
                                                    goto success;
                                                else
                                                if((*(cache_0 + pixel[11]))(0) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[7]))(0) < c_b)
                                                if((*(cache_0 + pixel[6]))(0) < c_b)
                                                    if((*(cache_0 + pixel[5]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                if((*(cache_0 + pixel[10]))(0) < c_b)
                                    if((*(cache_0 + pixel[11]))(0) < c_b)
                                        if((*(cache_0 + pixel[15]))(0) < c_b)
                                            if((*(cache_0 + pixel[13]))(0) < c_b)
                                                if((*(cache_0 + pixel[1]))(0) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                        if((*(cache_0 + pixel[9]))(0) < c_b)
                            if((*(cache_0 + pixel[10]))(0) < c_b)
                                if((*(cache_0 + pixel[14]))(0) < c_b)
                                    if((*(cache_0 + pixel[11]))(0) < c_b)
                                        if((*(cache_0 + pixel[15]))(0) < c_b)
                                            if((*(cache_0 + pixel[1]))(0) < c_b)
                                                if((*(cache_0 + pixel[13]))(0) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                    if((*(cache_0 + pixel[6]))(0) < c_b)
                        if((*(cache_0 + pixel[14]))(0) < c_b)
                            if((*(cache_0 + 3))(0) < c_b)
                                if((*(cache_0 + pixel[13]))(0) > cb)
                                    if((*(cache_0 + pixel[7]))(0) < c_b)
                                        if((*(cache_0 + pixel[3]))(0) < c_b)
                                            if((*(cache_0 + pixel[1]))(0) < c_b)
                                                if((*(cache_0 + pixel[5]))(0) < c_b)
                                                    if((*(cache_0 + pixel[15]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if((*(cache_0 + pixel[13]))(0) < c_b)
                                    if((*(cache_0 + pixel[5]))(0) < c_b)
                                        if((*(cache_0 + pixel[15]))(0) < c_b)
                                            if((*(cache_0 + pixel[1]))(0) < c_b)
                                                if((*(cache_0 + pixel[3]))(0) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                if((*(cache_0 + pixel[7]))(0) < c_b)
                                    if((*(cache_0 + pixel[15]))(0) < c_b)
                                        if((*(cache_0 + pixel[3]))(0) < c_b)
                                            if((*(cache_0 + pixel[5]))(0) < c_b)
                                                if((*(cache_0 + pixel[1]))(0) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        continue;
                else
                    continue;
            else
            if((*(cache_0 + pixel[8]))(0) > cb)
                if((*(cache_0 + pixel[10]))(0) > cb)
                    if((*(cache_0 + 3))(0) > cb)
                        if((*(cache_0 + pixel[2]))(0) > cb)
                            if((*(cache_0 + pixel[6]))(0) > cb)
                                if((*(cache_0 + pixel[7]))(0) > cb)
                                    if((*(cache_0 + pixel[11]))(0) > cb)
                                        if((*(cache_0 + pixel[9]))(0) > cb)
                                            if((*(cache_0 + pixel[5]))(0) > cb)
                                                if((*(cache_0 + pixel[3]))(0) > cb)
                                                    goto success;
                                                else if((*(cache_0 + pixel[3]))(0) < c_b)
                                                    if((*(cache_0 + -3))(0) > cb)
                                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + pixel[13]))(0) > cb)
                                                    if((*(cache_0 + -3))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if((*(cache_0 + pixel[5]))(0) < c_b)
                                                if((*(cache_0 + -3))(0) > cb)
                                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                                        if((*(cache_0 + pixel[14]))(0) > cb)
                                                            if((*(cache_0 + pixel[15]))(0) > cb)
                                                                goto success;
                                                            else
                                                                continue;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[15]))(0) > cb)
                                                if((*(cache_0 + pixel[14]))(0) > cb)
                                                    if((*(cache_0 + -3))(0) > cb)
                                                        if((*(cache_0 + pixel[13]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                    if((*(cache_0 + pixel[1]))(0) > cb)
                                        if((*(cache_0 + pixel[3]))(0) > cb)
                                            if((*(cache_0 + pixel[5]))(0) > cb)
                                                if((*(cache_0 + pixel[9]))(0) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else if((*(cache_0 + pixel[2]))(0) < c_b)
                            if((*(cache_0 + pixel[11]))(0) > cb)
                                if((*(cache_0 + -3))(0) > cb)
                                    if((*(cache_0 + pixel[9]))(0) > cb)
                                        if((*(cache_0 + pixel[6]))(0) > cb)
                                            if((*(cache_0 + pixel[7]))(0) > cb)
                                                if((*(cache_0 + pixel[13]))(0) > cb)
                                                    if((*(cache_0 + pixel[5]))(0) > cb)
                                                        goto success;
                                                    else
                                                    if((*(cache_0 + pixel[14]))(0) > cb)
                                                        if((*(cache_0 + pixel[15]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + pixel[3]))(0) > cb)
                                                    if((*(cache_0 + pixel[5]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                        if((*(cache_0 + -3))(0) > cb)
                            if((*(cache_0 + pixel[6]))(0) > cb)
                                if((*(cache_0 + pixel[11]))(0) > cb)
                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                        if((*(cache_0 + pixel[7]))(0) > cb)
                                            if((*(cache_0 + pixel[9]))(0) > cb)
                                                if((*(cache_0 + pixel[5]))(0) > cb)
                                                    goto success;
                                                else if((*(cache_0 + pixel[5]))(0) < c_b)
                                                    if((*(cache_0 + pixel[14]))(0) > cb)
                                                        if((*(cache_0 + pixel[15]))(0) > cb)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                if((*(cache_0 + pixel[15]))(0) > cb)
                                                    if((*(cache_0 + pixel[14]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if((*(cache_0 + pixel[13]))(0) < c_b)
                                        if((*(cache_0 + pixel[3]))(0) > cb)
                                            if((*(cache_0 + pixel[5]))(0) > cb)
                                                if((*(cache_0 + pixel[7]))(0) > cb)
                                                    if((*(cache_0 + pixel[9]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                    if((*(cache_0 + pixel[3]))(0) > cb)
                                        if((*(cache_0 + pixel[7]))(0) > cb)
                                            if((*(cache_0 + pixel[9]))(0) > cb)
                                                if((*(cache_0 + pixel[5]))(0) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if((*(cache_0 + 3))(0) < c_b)
                        if((*(cache_0 + pixel[6]))(0) > cb)
                            if((*(cache_0 + pixel[14]))(0) > cb)
                                if((*(cache_0 + pixel[13]))(0) > cb)
                                    if((*(cache_0 + pixel[7]))(0) > cb)
                                        if((*(cache_0 + pixel[15]))(0) > cb)
                                            if((*(cache_0 + pixel[9]))(0) > cb)
                                                if((*(cache_0 + pixel[11]))(0) > cb)
                                                    if((*(cache_0 + -3))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                        if((*(cache_0 + pixel[5]))(0) > cb)
                                            if((*(cache_0 + pixel[9]))(0) > cb)
                                                if((*(cache_0 + pixel[11]))(0) > cb)
                                                    if((*(cache_0 + -3))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                    if((*(cache_0 + pixel[14]))(0) > cb)
                        if((*(cache_0 + pixel[6]))(0) > cb)
                            if((*(cache_0 + -3))(0) > cb)
                                if((*(cache_0 + pixel[5]))(0) > cb)
                                    if((*(cache_0 + pixel[11]))(0) > cb)
                                        if((*(cache_0 + pixel[9]))(0) > cb)
                                            if((*(cache_0 + pixel[7]))(0) > cb)
                                                if((*(cache_0 + pixel[13]))(0) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if((*(cache_0 + pixel[5]))(0) < c_b)
                                    if((*(cache_0 + pixel[15]))(0) > cb)
                                        if((*(cache_0 + pixel[7]))(0) > cb)
                                            if((*(cache_0 + pixel[9]))(0) > cb)
                                                if((*(cache_0 + pixel[11]))(0) > cb)
                                                    if((*(cache_0 + pixel[13]))(0) > cb)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                if((*(cache_0 + pixel[15]))(0) > cb)
                                    if((*(cache_0 + pixel[11]))(0) > cb)
                                        if((*(cache_0 + pixel[9]))(0) > cb)
                                            if((*(cache_0 + pixel[13]))(0) > cb)
                                                if((*(cache_0 + pixel[7]))(0) > cb)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        continue;
                else
                    continue;
            else if((*(cache_0 + pixel[8]))(0) < c_b)
                if((*(cache_0 + pixel[10]))(0) < c_b)
                    if((*(cache_0 + 3))(0) > cb)
                        if((*(cache_0 + pixel[14]))(0) < c_b)
                            if((*(cache_0 + pixel[6]))(0) < c_b)
                                if((*(cache_0 + -3))(0) < c_b)
                                    if((*(cache_0 + pixel[9]))(0) < c_b)
                                        if((*(cache_0 + pixel[11]))(0) < c_b)
                                            if((*(cache_0 + pixel[15]))(0) < c_b)
                                                if((*(cache_0 + pixel[13]))(0) < c_b)
                                                    if((*(cache_0 + pixel[7]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[5]))(0) < c_b)
                                                if((*(cache_0 + pixel[7]))(0) < c_b)
                                                    if((*(cache_0 + pixel[13]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else if((*(cache_0 + 3))(0) < c_b)
                        if((*(cache_0 + pixel[6]))(0) < c_b)
                            if((*(cache_0 + -3))(0) > cb)
                                if((*(cache_0 + pixel[2]))(0) < c_b)
                                    if((*(cache_0 + pixel[1]))(0) > cb)
                                        if((*(cache_0 + pixel[3]))(0) < c_b)
                                            if((*(cache_0 + pixel[5]))(0) < c_b)
                                                if((*(cache_0 + pixel[7]))(0) < c_b)
                                                    if((*(cache_0 + pixel[9]))(0) < c_b)
                                                        if((*(cache_0 + pixel[11]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if((*(cache_0 + pixel[1]))(0) < c_b)
                                        if((*(cache_0 + pixel[5]))(0) < c_b)
                                            if((*(cache_0 + pixel[9]))(0) < c_b)
                                                if((*(cache_0 + pixel[3]))(0) < c_b)
                                                    if((*(cache_0 + pixel[7]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                    if((*(cache_0 + pixel[11]))(0) < c_b)
                                        if((*(cache_0 + pixel[3]))(0) < c_b)
                                            if((*(cache_0 + pixel[5]))(0) < c_b)
                                                if((*(cache_0 + pixel[7]))(0) < c_b)
                                                    if((*(cache_0 + pixel[9]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else if((*(cache_0 + -3))(0) < c_b)
                                if((*(cache_0 + pixel[7]))(0) < c_b)
                                    if((*(cache_0 + pixel[11]))(0) > cb)
                                        if((*(cache_0 + pixel[1]))(0) < c_b)
                                            if((*(cache_0 + pixel[2]))(0) < c_b)
                                                if((*(cache_0 + pixel[3]))(0) < c_b)
                                                    if((*(cache_0 + pixel[5]))(0) < c_b)
                                                        if((*(cache_0 + pixel[9]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else if((*(cache_0 + pixel[11]))(0) < c_b)
                                        if((*(cache_0 + pixel[9]))(0) < c_b)
                                            if((*(cache_0 + pixel[5]))(0) > cb)
                                                if((*(cache_0 + pixel[13]))(0) < c_b)
                                                    if((*(cache_0 + pixel[14]))(0) < c_b)
                                                        if((*(cache_0 + pixel[15]))(0) < c_b)
                                                            goto success;
                                                        else
                                                            continue;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else if((*(cache_0 + pixel[5]))(0) < c_b)
                                                if((*(cache_0 + pixel[13]))(0) < c_b)
                                                    goto success;
                                                else
                                                if((*(cache_0 + pixel[3]))(0) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                            if((*(cache_0 + pixel[15]))(0) < c_b)
                                                if((*(cache_0 + pixel[14]))(0) < c_b)
                                                    if((*(cache_0 + pixel[13]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                    if((*(cache_0 + pixel[1]))(0) < c_b)
                                        if((*(cache_0 + pixel[2]))(0) < c_b)
                                            if((*(cache_0 + pixel[9]))(0) < c_b)
                                                if((*(cache_0 + pixel[3]))(0) < c_b)
                                                    if((*(cache_0 + pixel[5]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                            if((*(cache_0 + pixel[2]))(0) < c_b)
                                if((*(cache_0 + pixel[1]))(0) < c_b)
                                    if((*(cache_0 + pixel[3]))(0) < c_b)
                                        if((*(cache_0 + pixel[7]))(0) < c_b)
                                            if((*(cache_0 + pixel[9]))(0) < c_b)
                                                if((*(cache_0 + pixel[5]))(0) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                if((*(cache_0 + pixel[11]))(0) < c_b)
                                    if((*(cache_0 + pixel[3]))(0) < c_b)
                                        if((*(cache_0 + pixel[5]))(0) < c_b)
                                            if((*(cache_0 + pixel[7]))(0) < c_b)
                                                if((*(cache_0 + pixel[9]))(0) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                    if((*(cache_0 + pixel[14]))(0) < c_b)
                        if((*(cache_0 + pixel[6]))(0) < c_b)
                            if((*(cache_0 + -3))(0) < c_b)
                                if((*(cache_0 + pixel[5]))(0) > cb)
                                    if((*(cache_0 + pixel[9]))(0) < c_b)
                                        if((*(cache_0 + pixel[7]))(0) < c_b)
                                            if((*(cache_0 + pixel[11]))(0) < c_b)
                                                if((*(cache_0 + pixel[13]))(0) < c_b)
                                                    if((*(cache_0 + pixel[15]))(0) < c_b)
                                                        goto success;
                                                    else
                                                        continue;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else if((*(cache_0 + pixel[5]))(0) < c_b)
                                    if((*(cache_0 + pixel[13]))(0) < c_b)
                                        if((*(cache_0 + pixel[11]))(0) < c_b)
                                            if((*(cache_0 + pixel[7]))(0) < c_b)
                                                if((*(cache_0 + pixel[9]))(0) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                if((*(cache_0 + pixel[15]))(0) < c_b)
                                    if((*(cache_0 + pixel[13]))(0) < c_b)
                                        if((*(cache_0 + pixel[7]))(0) < c_b)
                                            if((*(cache_0 + pixel[9]))(0) < c_b)
                                                if((*(cache_0 + pixel[11]))(0) < c_b)
                                                    goto success;
                                                else
                                                    continue;
                                            else
                                                continue;
                                        else
                                            continue;
                                    else
                                        continue;
                                else
                                    continue;
                            else
                                continue;
                        else
                            continue;
                    else
                        continue;
                else
                    continue;
            else
                continue;

            success:
            corners.push_back(fast_xy(static_cast<short>(cache_0-line_min),
                                      static_cast<short>(y)));
        }
    }
}


void FastDetector::detect(
    cvframePtr_t frame,
    const ImgPyr_t& img_pyr,
    const double detection_threshold,
    features_t& fts)
{
  Corners corners(grid_n_cols_ * grid_n_rows_, Corner(0,0,detection_threshold,0,0.0f));
  //Corners corners(grid_n_cols_*grid_n_rows_, Corner(0,0,0,0,0.0f));
  for(int L=0; L<n_pyr_levels_; ++L)
  {
    const int scale = (1<<L);
    vector<fast_xy> fast_corners;

      fast_corner_detect_10(img_pyr[L], frame->getWidth(L),
          frame->getHeight(L), frame->getWidth(L), 8.0, fast_corners);

    vector<int> scores, nm_corners;
    fast_corner_score_10(img_pyr[L].begin(), frame->getWidth(L), fast_corners, 8, scores);// 20
    fast_nonmax_3x3(fast_corners, scores, nm_corners);

    for(auto it=nm_corners.begin(), ite=nm_corners.end(); it!=ite; ++it)
    {
      fast_xy& xy = fast_corners.at(*it);
      const int k = static_cast<int>((xy.y*scale)/cell_size_)*grid_n_cols_
                  + static_cast<int>((xy.x*scale)/cell_size_);
      if(grid_occupancy_[k])
        continue;
      const double score = shiTomasiScore(img_pyr[L], frame->getWidth(L), frame->getHeight(L), xy.x, xy.y);
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

inline double fast_corner_score(cvData::Img_t::const_iterator cache_0, const int offset[], double b) {
    b += 1.0;

    for(;;)
    {
        double cb = (*cache_0)(0) + b;
        double c_b= (*cache_0)(0) - b;
        double min_diff = double(INT_MAX);
        if(test_gt_set(*(cache_0 + offset[0]), cb, min_diff))
            if(test_gt_set(*(cache_0 + offset[8]), cb, min_diff))
                if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                b += min_diff;
                                            else
                                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                            if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                        if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                    if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                                b += min_diff;
                                                            else
                                                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                                if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                                    b += min_diff;
                                                                else
                                                                    break;
                                                            else
                                                                break;
                                                        else
                                                        if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                            if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                                b += min_diff;
                                                            else
                                                                break;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                            if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                                b += min_diff;
                                                            else
                                                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                                b += min_diff;
                                                            else
                                                                break;
                                                        else
                                                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                            if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                                b += min_diff;
                                                            else
                                                                break;
                                                        else
                                                            break;
                                                    else
                                                    if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                    if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                                            if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                                                b += min_diff;
                                                            else
                                                                break;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                            if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                                            if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                                                b += min_diff;
                                                            else
                                                                break;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                            if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                    b += min_diff;
                                                else if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                        if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                    if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                    if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                        if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                    if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                    if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                b += min_diff;
                                            else
                                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                            if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                    break;
            else if(test_gt_set(c_b, *(cache_0 + offset[8]), min_diff))
                if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                    if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                    if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                        if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                                b += min_diff;
                                                            else
                                                                break;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                                if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                                    b += min_diff;
                                                                else
                                                                    break;
                                                            else
                                                                break;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                    if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                                if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                                    b += min_diff;
                                                                else
                                                                    break;
                                                            else
                                                                break;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                                b += min_diff;
                                                            else
                                                                break;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                    b += min_diff;
                                                else
                                                if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                            if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                        if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                    if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                    if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                    if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                    break;
            else
            if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                                b += min_diff;
                                            else
                                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                            if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                        if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                    if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                    if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                    break;
            else if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                    break;
            else
            if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                    break;
            else
                break;
        else if(test_gt_set(c_b, *(cache_0 + offset[0]), min_diff))
            if(test_gt_set(*(cache_0 + offset[8]), cb, min_diff))
                if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                    if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                        if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                    if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                    if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                        if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                        if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                    if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                    if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                    break;
            else if(test_gt_set(c_b, *(cache_0 + offset[8]), min_diff))
                if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                    if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                                if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                                                        if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                    b += min_diff;
                                                else
                                                if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                    if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                        if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                            if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                                                b += min_diff;
                                                            else
                                                                break;
                                                        else
                                                            break;
                                                    else if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                        b += min_diff;
                                                    else
                                                    if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                                        if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                        if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                                        b += min_diff;
                                                    else
                                                    if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                                        if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                                        if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                            if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                        if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                            b += min_diff;
                                                        else
                                                        if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                            if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                                b += min_diff;
                                                            else
                                                                break;
                                                        else
                                                            break;
                                                    else
                                                    if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                        if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                            if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                                b += min_diff;
                                                            else
                                                                break;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                            b += min_diff;
                                                        else
                                                        if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                            if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                                if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                                                    b += min_diff;
                                                                else
                                                                    break;
                                                            else
                                                                break;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                    if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                        if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                b += min_diff;
                                            else
                                            if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                    if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                        if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                            if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                                if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                                                    b += min_diff;
                                                                else
                                                                    break;
                                                            else
                                                                break;
                                                        else
                                                            break;
                                                    else if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                        b += min_diff;
                                                    else
                                                    if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                        if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                            if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                                                b += min_diff;
                                                            else
                                                                break;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                            if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                                if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                        if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                            if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                                b += min_diff;
                                                            else
                                                                break;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                        b += min_diff;
                                                    else
                                                    if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                        if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                        if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                        if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                            if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                        if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                        if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                                    b += min_diff;
                                                else
                                                if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                    if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                    b += min_diff;
                                                else
                                                if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                    if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                    if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                        if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                b += min_diff;
                                            else
                                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                    if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                    break;
            else
            if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                    if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                            if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                    if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                            if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                        if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                b += min_diff;
                                            else
                                            if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                            if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                    if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                    if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                    break;
            else
                break;
        else
        if(test_gt_set(*(cache_0 + offset[8]), cb, min_diff))
            if(test_gt_set(*(cache_0 + offset[10]), cb, min_diff))
                if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[2]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                                b += min_diff;
                                            else if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                            if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                            b += min_diff;
                                                        else
                                                            break;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                        if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                    if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                b += min_diff;
                                            else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                    if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                    if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                if(test_gt_set(*(cache_0 + offset[3]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                    if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                    if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                if(test_gt_set(*(cache_0 + offset[14]), cb, min_diff))
                    if(test_gt_set(*(cache_0 + offset[6]), cb, min_diff))
                        if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                                if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                            if(test_gt_set(*(cache_0 + offset[15]), cb, min_diff))
                                if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                    if(test_gt_set(*(cache_0 + offset[9]), cb, min_diff))
                                        if(test_gt_set(*(cache_0 + offset[13]), cb, min_diff))
                                            if(test_gt_set(*(cache_0 + offset[7]), cb, min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                    break;
            else
                break;
        else if(test_gt_set(c_b, *(cache_0 + offset[8]), min_diff))
            if(test_gt_set(c_b, *(cache_0 + offset[10]), min_diff))
                if(test_gt_set(*(cache_0 + offset[4]), cb, min_diff))
                    if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else if(test_gt_set(c_b, *(cache_0 + offset[4]), min_diff))
                    if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                        if(test_gt_set(*(cache_0 + offset[12]), cb, min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                                if(test_gt_set(*(cache_0 + offset[1]), cb, min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                if(test_gt_set(*(cache_0 + offset[11]), cb, min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                        if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                    if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                        b += min_diff;
                                                    else
                                                        break;
                                                else
                                                    break;
                                            else
                                                break;
                                        else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                b += min_diff;
                                            else
                                            if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                        if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                        if(test_gt_set(c_b, *(cache_0 + offset[2]), min_diff))
                            if(test_gt_set(c_b, *(cache_0 + offset[1]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                            if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[3]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                if(test_gt_set(c_b, *(cache_0 + offset[14]), min_diff))
                    if(test_gt_set(c_b, *(cache_0 + offset[6]), min_diff))
                        if(test_gt_set(c_b, *(cache_0 + offset[12]), min_diff))
                            if(test_gt_set(*(cache_0 + offset[5]), cb, min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                                if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                                    b += min_diff;
                                                else
                                                    break;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else if(test_gt_set(c_b, *(cache_0 + offset[5]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                            if(test_gt_set(c_b, *(cache_0 + offset[15]), min_diff))
                                if(test_gt_set(c_b, *(cache_0 + offset[13]), min_diff))
                                    if(test_gt_set(c_b, *(cache_0 + offset[7]), min_diff))
                                        if(test_gt_set(c_b, *(cache_0 + offset[9]), min_diff))
                                            if(test_gt_set(c_b, *(cache_0 + offset[11]), min_diff))
                                                b += min_diff;
                                            else
                                                break;
                                        else
                                            break;
                                    else
                                        break;
                                else
                                    break;
                            else
                                break;
                        else
                            break;
                    else
                        break;
                else
                    break;
            else
                break;
        else
            break;

    }

    return b-1;
}

void FastDetector::fast_corner_score_10(cvData::Img_t::const_iterator img, const int img_stride,
                                        const std::vector<fast_xy> &corners, const double threshold,
                                        std::vector<int> &scores){
    scores.resize(corners.size());
    int pixel[16] = {
            0 + img_stride * 3,
            1 + img_stride * 3,
            2 + img_stride * 2,
            3 + img_stride * 1,
            3 + img_stride * 0,
            3 + img_stride * -1,
            2 + img_stride * -2,
            1 + img_stride * -3,
            0 + img_stride * -3,
            -1 + img_stride * -3,
            -2 + img_stride * -2,
            -3 + img_stride * -1,
            -3 + img_stride * 0,
            -3 + img_stride * 1,
            -2 + img_stride * 2,
            -1 + img_stride * 3,
    };
    for(unsigned int n=0; n < corners.size(); n++)
        scores[n] = (int)fast_corner_score(img + corners[n].y*img_stride + corners[n].x, pixel, threshold);
}

void FastDetector::fast_nonmax_3x3(const std::vector<fast_xy> &corners, const std::vector<int> &scores,
                                   std::vector<int> &nonmax_corners){
    nonmax_corners.clear();
    nonmax_corners.reserve(corners.size());

    if(corners.size() < 1)
        return;


    // Find where each row begins
    // (the corners are output in raster scan order). A beginning of -1 signifies
    // that there are no corners on that row.
    int last_row = corners.back().y;
    vector<int> row_start(last_row + 1, -1);

    int prev_row = -1;
    for(unsigned int i=0; i< corners.size(); i++)
        if(corners[i].y != prev_row)
        {
            row_start[corners[i].y] = i;
            prev_row = corners[i].y;
        }


    //Point above points (roughly) to the pixel above the one of interest, if there
    //is a feature there.
    int point_above = 0;
    int point_below = 0;

    const int sz = (int)corners.size();

    for(int i=0; i < sz; i++)
    {
        int score = scores[i];
        fast_xy pos = corners[i];

        //Check left
        if(i > 0)
            //if(corners[i-1] == pos-ImageRef(1,0) && (scores[i-1] >= score))
            if(corners[i-1].x == pos.x-1 && corners[i-1].y == pos.y && scores[i-1] >= score)
                continue;

        //Check right
        if(i < (sz - 1))
            //if(corners[i+1] == pos+ImageRef(1,0) &&  (scores[i+1] >= score))
            if(corners[i+1].x == pos.x+1 && corners[i+1].y == pos.y && scores[i+1] >= score)
                continue;

        //Check above (if there is a valid row above)
        if(pos.y != 0 && row_start[pos.y - 1] != -1)
        {
            //Make sure that current point_above is one
            //row above.
            if(corners[point_above].y < pos.y - 1)
                point_above = row_start[pos.y-1];

            //Make point_above point to the first of the pixels above the current point,
            //if it exists.
            for(; corners[point_above].y < pos.y && corners[point_above].x < pos.x - 1; point_above++)
            {}


            for(int i=point_above; corners[i].y < pos.y && corners[i].x <= pos.x + 1; i++)
            {
                int x = corners[i].x;
                if( (x == pos.x - 1 || x ==pos.x || x == pos.x+1) && (scores[i] >= score))
                    goto cont;
            }

        }

        if(pos.y != last_row && row_start[pos.y + 1] != -1 && point_below < sz) //Nothing below
        {
            if(corners[point_below].y < pos.y + 1)
                point_below = row_start[pos.y+1];

            for(; point_below < sz && corners[point_below].y == pos.y+1 && corners[point_below].x < pos.x - 1; point_below++)
            {}

            for(int i=point_below; i < sz && corners[i].y == pos.y+1 && corners[i].x <= pos.x + 1; i++)
            {
                int x = corners[i].x;
                if( (x == pos.x - 1 || x ==pos.x || x == pos.x+1) && (scores[i] >= score))
                    goto cont;
            }
        }

        nonmax_corners.push_back(i);

        cont:
        ;
    }
}

EdgeDetector::EdgeDetector(
    const int img_width,
    const int img_height,
    const int cell_size,
    const int n_pyr_levels) :
    AbstractDetector(img_width, img_height, cell_size, n_pyr_levels)
{}

void EdgeDetector::detect(cvframePtr_t frame,
                          const AbstractDetector::ImgPyr_t &img_pyr,
                          const double detection_threshold,
                          AbstractDetector::features_t &fts) {



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
