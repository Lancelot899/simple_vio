#include <ceres/ceres.h>

#include <boost/random.hpp>
#include "Triangulater.h"

#include "../DataStructure/viFrame.h"
#include "../DataStructure/cv/cvFrame.h"
#include "../DataStructure/cv/Feature.h"
#include "../DataStructure/cv/Point.h"

class depthErr : public ceres::SizedCostFunction<1, 1> {
public:
	virtual ~depthErr() {}

	depthErr(std::shared_ptr<viFrame> nextFrame, double I_k,
	         const cvMeasure::features_t::value_type &f) : f_(f) {
		this->nextFrame = nextFrame;
		this->I_k = I_k;
		f_->point->pos_mutex.lock_shared();
		normPoint = f_->point->pos_ / f_->point->pos_(2);
		f_->point->pos_mutex.unlock_shared();
	}

	virtual bool Evaluate(double const *const *parameters,
	                      double *residuals,
	                      double **jacobians) const {
		double d = **parameters;
		Eigen::Vector3d P_next = nextFrame->getCVFrame()->getPose() * (d * normPoint);
		auto &cam = nextFrame->getCam();
		auto uv = cam->world2cam(P_next);
		if (uv(0) < 0 || uv(0) >= nextFrame->getCVFrame()->getWidth() || uv(1) < 0 ||
		    uv(1) >= nextFrame->getCVFrame()->getHeight())
			return false;

		for (int i = 0; i < f_->level; ++i) {
			uv /= 2.0;
		}

		*residuals = nextFrame->getCVFrame()->getIntensityBilinear(uv(0), uv(1), f_->level) - I_k;
		if (jacobians && jacobians[0]) {
			Eigen::Vector2d grad = nextFrame->getCVFrame()->getGradBilinear(uv(0), uv(1), f_->level);
			double Ix, Iy;
			if (f_->type == Feature::EDGELET) {
				Eigen::Vector2d &dir = f_->grad;
				Ix = dir(1) * dir(0);
				Iy = Ix * grad(0) + dir(1) * dir(1) * grad(1);
				Ix *= grad(1);
				Ix += dir(0) * dir(0) * grad(0);
			} else {
				Ix = grad(0);
				Iy = grad(1);
			}

			Eigen::Matrix<double, 1, 3> Jac;
			Jac(0, 0) = Ix * cam->fx(f_->level) / P_next(2);
			Jac(0, 1) = Iy * cam->fy(f_->level) / P_next(2);
			Jac(0, 2) = -Ix * cam->fx(f_->level) * P_next(0) / P_next(2) / P_next(2) -
			            Iy * cam->fy(f_->level) * P_next(1) / P_next(2) / P_next(2);
			Jac = Jac * nextFrame->getCVFrame()->getPose().rotationMatrix();
			jacobians[0][0] = Jac * normPoint;
		}

	}

private:
	const cvMeasure::features_t::value_type f_;
	Eigen::Vector3d normPoint;
	std::shared_ptr<viFrame> nextFrame;
	double I_k;
};

Triangulater::Triangulater() {}

int Triangulater::triangulate(std::shared_ptr<viFrame> &keyFrame,
                              std::shared_ptr<viFrame> &nextFrame,
                              const Sophus::SE3d &T_kn,
                              Eigen::Matrix<double, 6, 6>&infomation,
                              int iter) {
	int newCreatPoint = 0;

	cvMeasure::features_t &fts = keyFrame->getCVFrame()->getMeasure().fts_;
	int width = nextFrame->getCVFrame()->getWidth();
	int height = nextFrame->getCVFrame()->getHeight();
	Sophus::SE3d _SPose_n = keyFrame->getT_BS().inverse() * keyFrame->getPose() * T_kn;
	nextFrame->getCVFrame()->setPose(_SPose_n);

	std::list<cvMeasure::features_t::iterator> toErase;

	for (cvMeasure::features_t::iterator it = fts.begin(); it != fts.end(); ++it) {
		auto &ft = *it;
		ft->point->pos_mutex.lock_shared();
		Eigen::Vector3d pos = ft->point->pos_;
		ft->point->pos_mutex.unlock_shared();

			 Eigen::Vector2d &uvi = ft->px;
			pos = _SPose_n * pos;
			if(pos[2] < 0.00000001 || std::isinf(pos[2])) {
				toErase.push_back(it);
				goto PROJECTFAILED;
			}

			pos /= pos[2];
			Eigen::Vector2d uvj = nextFrame->getCam()->world2cam(pos.block<2, 1>(0, 0));
			double Ij = nextFrame->getCVFrame()->getIntensity(uvj(0), uvj(1));
			double Ii = nextFrame->getCVFrame()->getIntensity(uvi(0), uvi(1));
			if (uvj(0) < width && uvj(1) < height && uvj(0) > 0 && uvj(1) > 0 && std::abs(Ii - Ij) < IuminanceErr) {
				if (ft->isBAed != true) {
					for(int i = 0; i < ft->level; ++i)
						uvi /= 2.0;

					Ii = keyFrame->getCVFrame()->getIntensityBilinear(uvi(0), uvi(1), ft->level);

					ft->point->pos_mutex.lock_shared();
					double initPth = ft->point->pos_[2];
					ft->point->pos_mutex.unlock_shared();
					if(initPth > 0.999999999999 && initPth < 1.0000000001) {
						initPth = init_depth();
						ft->point->infoMutex.lock();
						ft->point->normal_information_ = initVar +
								1.0 / (1.0 + keyFrame->getCVFrame()->getGradNorm(ft->px(0), ft->px(1), ft->level);

						ft->point->infoMutex.unlock();
					}

					ceres::Problem problem;
					ceres::CostFunction *func = new depthErr(nextFrame, Ii, ft);
					problem.AddResidualBlock(func, nullptr, &initPth);
					ceres::Solver::Options option;
					option.minimizer_type = ceres::LINE_SEARCH;
					option.linear_solver_type = ceres::DENSE_QR;
					ceres::Solver::Summary summary;
					ceres::Solve(option, &problem, &summary);
					if (summary.termination_type == ceres::CONVERGENCE) {
						ft->point->pos_mutex.lock_shared();
						Eigen::Vector3d normPoint = ft->point->pos_ / ft->point->pos_(2);
						ft->point->pos_mutex.unlock_shared();
						auto pose = nextFrame->getPose();
						Eigen::Vector3d Pj = pose * (initPth * normPoint);
						if(Pj[2] < 0.00000001 || std::isinf(Pj[2])) {
							toErase.push_back(it);
							goto PROJECTFAILED;
						}

						uvj = nextFrame->getCam()->world2cam(Pj);

						if(uvj(0) >= width || uvj(1) >= height || uvj(0) <= 0 || uvj(1) <= 0) {
							toErase.push_back(it);
							goto PROJECTFAILED;
						}

						Eigen::Matrix<double, 1, 6> Jac;
						Jac.block<1, 3>(0, 0) = normPoint.transpose() *
						                        Sophus::SO3d::hat(pose.so3().inverse() * (Pj - pose.translation()));

						Jac.block<1, 3>(0, 3) = normPoint.transpose() * pose.so3().inverse().matrix();
						Jac = Jac / (normPoint.transpose() * normPoint);

						double newJac = Jac * infomation.inverse() * Jac.transpose();
						ft->point->updateDepth(initPth, 1.0 / newJac);

						ft->point->pos_mutex.lock_shared();
						pos = ft->point->pos_;
						ft->point->pos_mutex.unlock_shared();
						pos = _SPose_n * pos;
						pos /= pos[2];
						uvj = nextFrame->getCam()->world2cam(pos.block<2, 1>(0, 0));

						for(int i = 0; i < ft->level; ++i)
							uvj /= 2.0;

					}
					std::shared_ptr<Feature> ft_ = std::make_shared<Feature>(nextFrame->getCVFrame(),
					                                                         ft->point, uvj, pos, ft->level);
					ft_->isBAed.exchange(ft->isBAed);
					ft_->point->obsMutex.lock();
					ft_->point->obs_.push_back(ft_);
					ft_->point->obsMutex.unlock();
					nextFrame->getCVFrame()->addFeature(ft_);

					ft->point->n_succeeded_reproj_++;
					continue;
				}

				else {
					std::shared_ptr<Feature> ft_ = std::make_shared<Feature>(nextFrame->getCVFrame(),
					                                                         ft->point, uvj, pos, ft->level);
					ft_->isBAed.exchange(ft->isBAed);
					ft_->point->obsMutex.lock();
					ft_->point->obs_.push_back(ft_);
					ft_->point->obsMutex.unlock();
					nextFrame->getCVFrame()->addFeature(ft_);
					ft->point->n_succeeded_reproj_++;
					continue;
				}
			}


PROJECTFAILED:

		if (ft->point->n_succeeded_reproj_ >= 1)
			ft->point->n_failed_reproj_++;
		else
			toErase.push_back(it);
	}

	for (auto &it : toErase)
		fts.erase(it);

	return newCreatPoint;
}