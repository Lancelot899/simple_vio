//
// Created by lancelot on 1/6/17.
//

#ifndef SIMPLE_VIO_MATCHER_H
#define SIMPLE_VIO_MATCHER_H

#include <memory>

#include <Eigen/Dense>
#include "ThirdParty/sophus/se3.hpp"
#include "DataStructure/cv/Camera/AbstractCamera.h"


class cvFrame;
class Feature;
class Point;

template<int HALF_PATCH_SIZE>
class ZMSSD {
public:

    static const int patch_size_ = 2 * HALF_PATCH_SIZE;
    static const int patch_area_ = patch_size_ * patch_size_;
    static const int threshold_  = 2000 * patch_area_;
    double* ref_patch_;
    double sumA_, sumAA_;

    ZMSSD(double* ref_patch) :
            ref_patch_(ref_patch)
    {
        double sumA_uint=0, sumAA_uint=0;
        for(int r = 0; r < patch_area_; r++)
        {
            double n = ref_patch_[r];
            sumA_uint += n;
            sumAA_uint += n * n;
        }
        sumA_ = sumA_uint;
        sumAA_ = sumAA_uint;
    }

    static int threshold() { return threshold_; }

    double computeScore(double* cur_patch) const
    {
        double sumB_uint = 0;
        double sumBB_uint = 0;
        double sumAB_uint = 0;
        for(int r = 0; r < patch_area_; r++) {
            const double cur_pixel = cur_patch[r];
            sumB_uint  += cur_pixel;
            sumBB_uint += cur_pixel*cur_pixel;
            sumAB_uint += cur_pixel * ref_patch_[r];
        }
        const double sumB = sumB_uint;
        const double sumBB = sumBB_uint;
        const double sumAB = sumAB_uint;
        return sumAA_ - 2*sumAB + sumBB - (sumA_*sumA_ - 2*sumA_*sumB + sumB*sumB)/patch_area_;
    }

    double computeScore(double* cur_patch, int stride) const
    {
        double sumB, sumBB, sumAB;
        {
            double sumB_uint = 0;
            double sumBB_uint = 0;
            double sumAB_uint = 0;
            for(int y=0, r=0; y < patch_size_; ++y)
            {
                double* cur_patch_ptr = cur_patch + y*stride;
                for(int x=0; x < patch_size_; ++x, ++r)
                {
                    const double cur_px = cur_patch_ptr[x];
                    sumB_uint  += cur_px;
                    sumBB_uint += cur_px * cur_px;
                    sumAB_uint += cur_px * ref_patch_[r];
                }
            }
            sumB = sumB_uint;
            sumBB = sumBB_uint;
            sumAB = sumAB_uint;
        }
        return sumAA_ - 2*sumAB + sumBB - (sumA_*sumA_ - 2*sumA_*sumB + sumB*sumB)/patch_area_;
    }
};



class Matcher {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    static const int halfpatch_size_ = 4;
    static const int patch_size_ = 8;
    typedef ZMSSD<halfpatch_size_> PatchScore;

    struct Options
    {
        bool align_1d;                             //!< in epipolar search: align patch 1D along epipolar line
        int align_max_iter;                        //!< number of iterations for aligning the feature patches in gauss newton
        double max_epi_length_optim;               //!< max length of epipolar line to skip epipolar search and directly go to img align
        size_t max_epi_search_steps;               //!< max number of evaluations along epipolar line
        bool subpix_refinement;                    //!< do gauss newton feature patch alignment after epipolar search
        bool epi_search_edgelet_filtering;
        double epi_search_edgelet_max_angle;
        Options() :
                align_1d(false),
                align_max_iter(10),
                max_epi_length_optim(2.0),
                max_epi_search_steps(1000),
                subpix_refinement(true),
                epi_search_edgelet_filtering(true),
                epi_search_edgelet_max_angle(0.7)
        {}
    };


public:
    static void getWarpMatrixAffine(
            const std::shared_ptr<AbstractCamera>& cam_ref,
            const std::shared_ptr<AbstractCamera>& cam_cur,
            const Eigen::Vector2d& px_ref,
            const Eigen::Vector3d& f_ref,
            const double idepth_ref,
            const Sophus::SE3d& T_cur_ref,
            const int level_ref,
            Eigen::Matrix2d& A_cur_ref);

    static int getBestSearchLevel(const Eigen::Matrix2d& A_cur_ref, const int max_level);
    static void warpAffine(
            const Eigen::Matrix2d& A_cur_ref,
            std::vector<Eigen::Vector3d>::const_iterator img_ref,
            const Eigen::Vector2d& px_ref,
            const int level_ref,
            const int search_level,
            const int halfpatch_size,
            int rows,
            int cols,
            double* patch);

    static bool align1D(std::vector<Eigen::Vector3d>::const_iterator& img_ref,
                    const Eigen::Vector2d &dir,
                    double *ref_patch_with_border,
                    double *ref_patch,
                    const int n_iter,
                    Eigen::Vector2d &cur_px_estimate,
                    double &h_inv);

public:
    Options options_;
    Matcher() = default;
    ~Matcher() = default;

    bool findMatchDirect(
            std::shared_ptr<Point>& pt,
            std::shared_ptr<cvFrame>& cur_frame,
            Eigen::Vector2d px_cur
    );

    void createPatchFromPatchWithBorder();


private:
    double patch_[patch_size_*patch_size_];
    double patch_with_border_[(patch_size_+2)*(patch_size_+2)];
    Eigen::Matrix2d A_cur_ref_;
    Eigen::Vector2d epi_dir_;
    double epi_length_;
    double h_inv_;
    int search_level_;
    bool reject_;
    std::shared_ptr<Feature> ref_ftr_;
    Eigen::Vector2d px_cur_;
};


#endif //SIMPLE_VIO_MATCHER_H
