#include "cv/Point.h"
#include "util.h"
#include "cv/cvFrame.h"
#include "cv/Feature.h"

using namespace Eigen;

int Point::point_counter_ = 0;

Point::Point(const Vector3d& pos) :
    id_(point_counter_++),
    pos_(pos),
    normal_set_(false),
    n_obs_(0),
    last_published_ts_(0),
    last_projected_kf_id_(-1),
    type_(TYPE_UNKNOWN),
    n_failed_reproj_(0),
    n_succeeded_reproj_(0),
    last_structure_optim_(0) {}

Point::Point(const Vector3d& pos, std::shared_ptr<Feature> &ftr) :
    id_(point_counter_++),
    pos_(pos),
    normal_set_(false),
    n_obs_(1),
    last_published_ts_(0),
    last_projected_kf_id_(-1),
    type_(TYPE_UNKNOWN),
    n_failed_reproj_(0),
    n_succeeded_reproj_(0),
    last_structure_optim_(0) {
    obs_.push_front(ftr);
}

Point::~Point() {}

void Point::addFrameRef(std::shared_ptr<Feature> &ftr) {
    obs_.push_front(ftr);
    ++n_obs_;
}

std::shared_ptr<Feature> Point::findFrameRef(std::shared_ptr<cvFrame>& frame) {
    for(auto it=obs_.begin(), ite=obs_.end(); it!=ite; ++it)
        if((*it)->frame == frame)
            return *it;
    return std::shared_ptr<Feature>();    // no keyframe found
}

bool Point::deleteFrameRef(std::shared_ptr<cvFrame>& frame) {
    for(auto it=obs_.begin(), ite=obs_.end(); it!=ite; ++it) {
        if((*it)->frame == frame) {
            obs_.erase(it);
            return true;
        }
    }
    return false;
}

void Point::initNormal() {
    assert(!obs_.empty());
    const std::shared_ptr<Feature>& ftr = obs_.back();
    assert(ftr->frame != NULL);
    normal_ = ftr->frame->getPose().so3().inverse().matrix() * (-ftr->f);
    normal_information_ = DiagonalMatrix<double,3,3>(pow(20 / (pos_ - ftr->frame->pos()).norm(), 2), 1.0, 1.0);
    normal_set_ = true;
}

bool Point::getCloseViewObs(const Vector3d& framepos, std::shared_ptr<Feature> &ftr) const {
    // TODO: get frame with same point of view AND same pyramid level!
    Vector3d obs_dir(framepos - pos_); obs_dir.normalize();
    auto min_it=obs_.begin();
    double min_cos_angle = 0;
    for(auto it=obs_.begin(), ite=obs_.end(); it!=ite; ++it) {
        Vector3d dir((*it)->frame->getPose().translation() - pos_); dir.normalize();
        double cos_angle = obs_dir.dot(dir);
        if(cos_angle > min_cos_angle) {
            min_cos_angle = cos_angle;
            min_it = it;
        }
    }
    ftr = *min_it;
    if(min_cos_angle < 0.5) // assume that observations larger than 60° are useless
        return false;
    return true;
}


void Point::optimize(const size_t n_iter) {
    Vector3d old_point = pos_;
    double chi2 = 0.0;
    Matrix3d A;
    Vector3d b;

    for(size_t i=0; i<n_iter; i++) {
        A.setZero();
        b.setZero();
        double new_chi2 = 0.0;

        // compute residuals
        for(auto it=obs_.begin(); it!=obs_.end(); ++it) {
            Matrix23d J;
            const Vector3d p_in_f((*it)->frame->getPose() * pos_);
            Point::jacobian_xyz2uv(p_in_f, (*it)->frame->getPose().so3().matrix(), J);
            const Vector2d e(project2d((*it)->f) - project2d(p_in_f));

            if((*it)->type == Feature::EDGELET) {
                float err_edge = (*it)->grad.transpose() * e;
                new_chi2 += err_edge * err_edge;
                A.noalias() += J.transpose()*(*it)->grad * (*it)->grad.transpose() * J;
                b.noalias() -= J.transpose() *(*it)->grad * err_edge;
            }

            else {
                new_chi2 += e.squaredNorm();
                A.noalias() += J.transpose() * J;
                b.noalias() -= J.transpose() * e;
            }

        }

        // solve linear system
        const Vector3d dp(A.ldlt().solve(b));

        // check if error increased
        if((i > 0 && new_chi2 > chi2) || (bool) std::isnan((double)dp[0])) {
            pos_ = old_point; // roll-back
            break;
        }

        // update the model
        Vector3d new_point = pos_ + dp;
        old_point = pos_;
        pos_ = new_point;
        chi2 = new_chi2;

        // stop when converged
        if(norm_max(dp) <= EPS)
            break;
    }
}
