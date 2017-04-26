#include "DataStructure/cv/Point.h"
#include "util/util.h"
#include "DataStructure/cv/cvFrame.h"
#include "DataStructure/cv/Feature.h"

using namespace Eigen;

int Point::point_counter_ = 0;

Point::Point(const Vector3d& pos) :
    id_(point_counter_++),
    pos_(pos),
    last_projected_kf_id_(-1),
    type_(TYPE_UNKNOWN),
    n_failed_reproj_(0),
    n_succeeded_reproj_(0){}

Point::Point(const Vector3d& pos, std::shared_ptr<Feature> &ftr) :
    id_(point_counter_++),
    pos_(pos),
    last_projected_kf_id_(-1),
    type_(TYPE_UNKNOWN),
    n_failed_reproj_(0),
    n_succeeded_reproj_(0) {
    obs_.push_front(ftr);
}

Point::~Point() {}

double Point::getDepthInformation() {
    inforMutex.lock_shared();
    double info = normal_information_;
    inforMutex.unlock_shared();
    return info;
}

double Point::updateDepth(double depth, double information) {
    pos_mutex.lock_shared();
    infoMutex.lock_shared();
    double depthNew = (normal_information_ + depth / pos_[2] * information) / (normal_information_ + information);
    double newInfo = normal_information_ + information;
	infoMutex.unlock_shared();
    pos_mutex.unlock_shared();

	infoMutex.lock();
    normal_information_ = newInfo;
	infoMutex.unlock();

    pos_mutex.lock();
    pos_ = depthNew * pos_;
    pos_mutex.unlock();

    return depthNew;
}