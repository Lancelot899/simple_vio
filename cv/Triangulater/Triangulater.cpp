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
	         const Sophus::SE3d &T_nk, const cvMeasure::features_t::value_type &f)
			: T_nk_(nextFrame->getT_BS().inverse() * T_nk * nextFrame->getT_BS()), f_(f) {
		this->nextFrame = nextFrame;
		this->I_k = I_k;
	}

	virtual bool Evaluate(double const *const *parameters,
	                      double *residuals,
	                      double **jacobians) const {
		double d = **parameters;
		Eigen::Vector3d P_next = T_nk_ * (d * f_->f);
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
			Jac(0, 0) = Ix * cam->fx() / P_next(2);
			Jac(0, 1) = Iy * cam->fy() / P_next(2);
			Jac(0, 2) = -Ix * cam->fx() * P_next(0) / P_next(2) / P_next(2) -
			            Iy * cam->fy() * P_next(1) / P_next(2) / P_next(2);
			Jac = Jac * T_nk_.rotationMatrix();
			jacobians[0][0] = Jac * f_->f;
		}

	}

private:
	const Sophus::SE3d T_nk_;
	const cvMeasure::features_t::value_type f_;
	std::shared_ptr<viFrame> nextFrame;
	double I_k;
};

Triangulater::Triangulater() {}

int Triangulater::triangulate(std::shared_ptr<viFrame> &keyFrame,
                              std::shared_ptr<viFrame> &nextFrame,
                              const Sophus::SE3d &T_nk, Eigen::Matrix<double, 6, 6>&infomation,
                              int iter) {
	int newCreatPoint = 0;

	cvMeasure::features_t &fts = keyFrame->getCVFrame()->getMeasure().fts_;
	int width = nextFrame->getCVFrame()->getWidth();
	int height = nextFrame->getCVFrame()->getHeight();
	int cellwidth = nextFrame->getCVFrame()->getWidth() / detectCellWidth;
	int chellheight = keyFrame->getCVFrame()->getHeight() / detectCellHeight;
	Sophus::SE3d _SPose_j = keyFrame->getT_BS().inverse() * keyFrame->getPose() * T_nk;
	nextFrame->getCVFrame()->setPose(_SPose_j);
	std::list<cvMeasure::features_t::iterator> toErase;

	ceres::Solver::Options option;
	option.max_num_iterations = iter;
	option.minimizer_type = ceres::LINE_SEARCH;
	option.linear_solver_type = ceres::DENSE_QR;

	boost::mt19937 gen;
	boost::uniform_real<>dist(0.9, 1.1);
	boost::variate_generator<boost::mt19937&,boost::uniform_real<>> scale_ (gen, dist);


	for (cvMeasure::features_t::iterator it = fts.begin(); it != fts.end(); ++it) {
		auto &ft = *it;
		ft->point->pos_mutex.lock_shared();
		Eigen::Vector3d pos = ft->point->pos_;
		ft->point->pos_mutex.unlock_shared();
		Eigen::Vector2d& uvi = ft->px;
		double Ii = keyFrame->getCVFrame()->getIntensityBilinear(uvi(0), uvi(1));
		if (uvi(0) < width && uvi(1) < height && uvi(0) > 0 && uvi(1) > 0) {
			Eigen::Vector2d uvj = nextFrame->getCam()->world2cam(_SPose_j * pos);
			double Ij = viframe_j->getCVFrame()->getIntensityBilinear(uvj(0), uvj(1));
			if (uvj(0) < width && uvj(1) < height && uvj(0) > 0 && uvj(1) > 0 && std::abs(Ii - Ij) < IuminanceErr) {
				if (ft->isBAed != true) {
					ft->point->pos_mutex.lock_shared();
					double initPth = ft->point->pos_[2];
					ft->point->pos_mutex.unlock_shared();
					ceres::Problem problem;
					ceres::CostFunction *func = new depthErr(viframe_j, Ii, ft);
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
						auto pose = viframe_j->getPose();
						Eigen::Vector3d Pj = pose * (initPth * normPoint);
						Eigen::Vector2d uv = viframe_i->getCam()->world2cam(Pj);
						Eigen::Matrix<double, 1, 6> Jac;
						Jac.block<1, 3>(0, 0) = normPoint.transpose() *
						                        Sophus::SO3d::hat(pose.so3().inverse() * (Pj - pose.translation()));

						Jac.block<1, 3>(0, 3) = normPoint.transpose() * pose.so3().inverse().matrix();
						Jac = Jac / (normPoint.transpose() * normPoint);

						double newJac = Jac * infomation.inverse() * Jac.transpose();
						ft->point->updateDepth(initPth, 1.0 / newJac);
					}
					std::shared_ptr<Feature> ft_ = std::make_shared<Feature>(viframe_j->getCVFrame(),
					                                                         ft->point, uvj, pos, ft->level);
					ft_->isBAed.exchange(ft->isBAed);
					ft_->point->obsMutex.lock();
					ft_->point->obs_.push_back(ft_);
					ft_->point->obsMutex.unlock();
					viframe_j->getCVFrame()->addFeature(ft_);

					int u = int(uvj(0) / cellwidth);
					int v = int(uvj(1) / chellheight);

					if (!viframe_j->getCVFrame()->checkCell(u, v)) {
						viframe_j->getCVFrame()->setCellTrue(u, v);
						cntCell++;
					}
					ft->point->n_succeeded_reproj_++;
					continue;
				}
			}
		}

		if (ft->point->n_succeeded_reproj_ >= 1)
			ft->point->n_failed_reproj_++;
		else
			toErase.push_back(it);
	}

	for (auto &it : toErase)
		fts.erase(it);

	return cntCell;
}


	for (auto &ftKey : fts) {
		ceres::Solver::Summary summary;
		ceres::Problem problem;
		double initDepth = scale * (init_depth + scale_() * std::abs((T_nk.so3() * T_nk.translation())(2)));
		auto cam = nextFrame->getCam();
		Eigen::Vector2d uv;
		if(ftKey->isBAed) continue;
		ftKey->point->pos_mutex.lock_shared();
		Eigen::Vector3d pos = ftKey->point->pos_;
		ftKey->point->pos_mutex.unlock_shared();
		if (pos[2] < 1.000000000001 && pos[2] > 0.99999999999) {
			auto pos_ = keyFrame->getCVFrame()->getPose() * pos;
			initDepth = pos_(2);
			uv = cam->world2cam(pos_);
		} else {
			uv(0) = ftKey->px(0);
			uv(1) = ftKey->px(1);
		}

		for (int level = 0; level < ftKey->level; level++)
			uv /= 2.0;

		double updateDepth = initDepth;
		Eigen::Vector3d tmpPoint;
		problem.AddResidualBlock(
				new depthErr(nextFrame, keyFrame->getCVFrame()->getIntensityBilinear(uv(0), uv(1), ftKey->level), T_nk,
				             ftKey),
				nullptr, &updateDepth);
		ceres::Solve(option, &problem, &summary);
		if(updateDepth < 0.00000000001)
			continue;

		if (summary.termination_type == ceres::CONVERGENCE) {
			newCreatPoint++;
			ftKey->point->pos_mutex.lock();
			if(ftKey->point->pos_[2] < 1.000000000001 && ftKey->point->pos_[2] > 0.99999999999)
				ftKey->point->pos_ *= updateDepth;
			else {
				double ratio = updateDepth / initDepth;
				ftKey->point->pos_ *= ratio;
			}
			ftKey->point->pos_mutex.unlock();
		}
	}
	return newCreatPoint;
}