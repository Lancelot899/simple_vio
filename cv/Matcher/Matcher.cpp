//
// Created by lancelot on 1/6/17.
//

#include "Matcher.h"
#include "util/util.h"
#include "DataStructure/cv/Point.h"
#include "DataStructure/cv/cvFrame.h"
#include "DataStructure/cv/Feature.h"

using namespace Eigen;


bool Matcher::findMatchDirect(std::shared_ptr<Point> &pt,
                              std::shared_ptr<cvFrame> &cur_frame,
                              Eigen::Vector2d px_cur) {
    if(!pt->getCloseViewObs(cur_frame->pos(), ref_ftr_))
        return false;

    if(!ref_ftr_->frame->getCam()->isInFrame(ref_ftr_->px.cast<int>() / (1 << ref_ftr_->level),
                                             halfpatch_size_+2, ref_ftr_->level))
        return false;

    this->getWarpMatrixAffine(ref_ftr_->frame->getCam(), cur_frame->getCam(),
                              ref_ftr_->px, ref_ftr_->f,
                              (ref_ftr_->frame->pos() - pt->pos_).norm(),
                               cur_frame->getPose() * ref_ftr_->frame->getPose().inverse(),
                               ref_ftr_->level, A_cur_ref_);
    search_level_ = getBestSearchLevel(A_cur_ref_, IMG_LEVEL - 1);
    this->warpAffine(A_cur_ref_, ref_ftr_->frame->getMeasure().measurement.imgPyr[ref_ftr_->level].begin(),
               ref_ftr_->px, ref_ftr_->level, search_level_, halfpatch_size_ + 1,
               ref_ftr_->frame->getMeasure().measurement.height[ref_ftr_->level],
               ref_ftr_->frame->getMeasure().measurement.width[ref_ftr_->level],
               patch_with_border_);
    createPatchFromPatchWithBorder();

    Vector2d px_scaled(px_cur / (1 << search_level_));
    bool success = false;
    if(ref_ftr_->type == Feature::EDGELET) {
        Vector2d dir_cur(A_cur_ref_ * ref_ftr_->grad);
        dir_cur.normalize();

    }
}

bool Matcher::align1D(std::vector<Eigen::Vector3d>::const_iterator &img_ref,
                      const Eigen::Vector2d &dir,
                      double *ref_patch_with_border, double *ref_patch, const int n_iter,
                      Eigen::Vector2d &cur_px_estimate, double &h_inv) {

}


void Matcher::createPatchFromPatchWithBorder() {
    double *ref_patch_ptr = patch_;
    for(int y=1; y<patch_size_+1; ++y, ref_patch_ptr += patch_size_) {
        double* ref_patch_border_ptr = patch_with_border_ + y*(patch_size_+2) + 1;
        for(int x=0; x<patch_size_; ++x)
            ref_patch_ptr[x] = ref_patch_border_ptr[x];
    }
}


void Matcher::warpAffine(
        const Eigen::Matrix2d& A_cur_ref,
        std::vector<Eigen::Vector3d>::const_iterator img_ref,
        const Eigen::Vector2d& px_ref,
        const int level_ref,
        const int search_level,
        const int halfpatch_size,
        int rows,
        int cols,
        double* patch) {
    const int patch_size = halfpatch_size * 2 ;
    const Matrix2d A_ref_cur = A_cur_ref.inverse();
    if(isnan(A_ref_cur(0,0))) {
        printf("Affine warp is NaN, probably camera has no translation\n"); // TODO
        return;
    }

    double *patch_ptr = patch;
    const Vector2d px_ref_pyr = px_ref / (1<<level_ref);
    for (int y=0; y<patch_size; ++y) {
        for (int x = 0; x < patch_size; ++x, ++patch_ptr) {
            Vector2d px_patch(x - halfpatch_size, y - halfpatch_size);
            px_patch *= (1<<search_level);
            const Vector2d px(A_ref_cur * px_patch + px_ref_pyr);
            if (px[0] < 0 || px[1] < 0 || px[0] >= cols - 1 || px[1] >= rows - 1)
                *patch_ptr = 0;
            else
                *patch_ptr =interpolate(img_ref, px[0], px[1], cols)(0);
        }
    }
}

void Matcher::getWarpMatrixAffine(const std::shared_ptr<AbstractCamera>& cam_ref,
                         const std::shared_ptr<AbstractCamera>& cam_cur,
                         const Eigen::Vector2d& px_ref,
                         const Eigen::Vector3d& f_ref,
                         const double idepth_ref,
                         const Sophus::SE3d& T_cur_ref,
                         const int level_ref,
                         Eigen::Matrix2d& A_cur_ref) {
    const int halfpatch_size = 5;
    const Vector3d xyz_ref(f_ref / idepth_ref);
    Vector3d xyz_du_ref(cam_ref->cam2world(px_ref + Vector2d(halfpatch_size, 0) * (1<<level_ref)));
    Vector3d xyz_dv_ref(cam_ref->cam2world(px_ref + Vector2d(0, halfpatch_size) * (1<<level_ref)));
    xyz_du_ref *= xyz_ref[2] / xyz_du_ref[2];
    xyz_dv_ref *= xyz_ref[2] / xyz_dv_ref[2];
    const Vector2d px_cur(cam_cur->world2cam(T_cur_ref * (xyz_ref)));
    const Vector2d px_du(cam_cur->world2cam(T_cur_ref * (xyz_du_ref)));
    const Vector2d px_dv(cam_cur->world2cam(T_cur_ref * (xyz_dv_ref)));
    A_cur_ref.col(0) = (px_du - px_cur) / halfpatch_size;
    A_cur_ref.col(1) = (px_dv - px_cur) / halfpatch_size;
}

int Matcher::getBestSearchLevel(const Eigen::Matrix2d& A_cur_ref, const int max_level) {
    int search_level = 0;
    double D = A_cur_ref.determinant();
    while(D > 3.0 && search_level < max_level) {
        search_level += 1;
        D *= 0.25;
    }

    return search_level;
}