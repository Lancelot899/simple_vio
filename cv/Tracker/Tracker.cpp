//
// Created by lancelot on 2/20/17.
//


#include <memory>

#include <ceres/ceres.h>
#include <boost/thread/shared_mutex.hpp>

#include "Tracker.h"
#include "DataStructure/cv/cvFrame.h"
#include "DataStructure/viFrame.h"
#include "DataStructure/cv/Feature.h"
#include "DataStructure/cv/Point.h"
#include "util/setting.h"

#define PHOTOMATRICERROR 40


namespace direct_tracker {

class TrackingErr : public ceres::SizedCostFunction<1, 6> {
public:
	TrackingErr(const std::shared_ptr<Feature> &ft, std::shared_ptr<viFrame> &viframe_i,
	            std::shared_ptr<viFrame> &viframe_j) {
		this->ft = ft;
		this->viframe_i = viframe_i;
		this->viframe_j = viframe_j;
		T_SB = viframe_i->getT_BS().inverse();
		sqrt_info = std::sqrt(ft->point->getDepthInformation());
		ft->point->pos_mutex.lock_shared();
		const Eigen::Vector3d p = ft->point->pos_;
		ft->point->pos_mutex.unlock_shared();
		const viFrame::cam_t &cam = viframe_j->getCam();
		Eigen::Vector3d pi = viframe_i->getCVFrame()->getPose() * p;
		Eigen::Vector2d px = cam->world2cam(pi);
		for (int i = 0; i < ft->level; ++i)
			px /= 2.0;

		I_i = viframe_i->getCVFrame()->getIntensityBilinear(px(0), px(1), ft->level);
	}

	virtual bool Evaluate(double const *const *parameters,
	                      double *residuals,
	                      double **jacobians) const {
		if (ft->isProjected == false) {
			*residuals = 0;
			//printf("projected failed!\n");
			if (jacobians && jacobians[0])
				memset(jacobians[0], 0, sizeof(double) * 6);
			return true;
		}

		Eigen::Vector3d so3, trans_ij;
		for (int i = 0; i < 3; ++i) {
			so3(i) = parameters[0][i];
			trans_ij(i) = parameters[0][3 + i];
		}

		Sophus::SO3d R_ij = Sophus::SO3d::exp(so3);
		ft->point->pos_mutex.lock_shared();
		const Eigen::Vector3d p = ft->point->pos_;
		ft->point->pos_mutex.unlock_shared();

		Sophus::SE3d T_Si = T_SB * viframe_i->getPose();
		Eigen::Vector3d pj = T_Si * (R_ij * p + trans_ij);

		if (pj(2) > 0.0000000001) {
			const viFrame::cam_t &cam = viframe_j->getCam();
			double u = cam->fx() * (pj(0) / pj(2)) + cam->cx();
			if (u >= 0 && u < viframe_j->getCVFrame()->getWidth()) {
				double v = cam->fy() * (pj(1) / pj(2)) + cam->cy();
				if (v >= 0 && v < viframe_j->getCVFrame()->getHeight()) {
					for (int i = 0; i < ft->level; ++i) {
						u /= 2.0;
						v /= 2.0;
					}

					double err = viframe_j->getCVFrame()->getIntensityBilinear(u, v, ft->level) - I_i;
					//std::cout << "err = " << err << std::endl;
					if (err < PHOTOMATRICERROR && err > -PHOTOMATRICERROR) {
						double w = 1.0 / viframe_j->getCVFrame()->getGradNorm(u, v, ft->level);

						if (w > 0.0000001 && !std::isinf(w)) {
							*residuals = sqrt_info * w * err;
							Eigen::Vector2d grad;

							if (viframe_j->getCVFrame()->getGrad(u, v, grad, ft->level)) {
								Eigen::Vector2d &dir = ft->grad;
								w = std::sqrt(w);
								if (jacobians && jacobians[0]) {
									double Ix, Iy;
									if (ft->type == Feature::EDGELET) {
										Ix = dir(1) * dir(0);
										Iy = Ix * grad(0) + dir(1) * dir(1) * grad(1);
										Ix *= grad(1);
										Ix += dir(0) * dir(0) * grad(0);
									} else {
										Ix = grad(0);
										Iy = grad(1);
									}
									Eigen::Matrix<double, 1, 3> Jac;
									Jac(0, 0) = Ix * cam->fx(ft->level) / pj(2);
									Jac(0, 1) = Iy * cam->fy(ft->level) / pj(2);
									Jac(0, 2) = -Ix * cam->fx(ft->level) * pj(0) / pj(2) / pj(2) -
									            Iy * cam->fy(ft->level) * pj(1) / pj(2) / pj(2);
									Jac = sqrt_info * w * Jac * T_Si.rotationMatrix();
									//	std::cout << "tracking dedt = \n" << Jac << std::endl;
									jacobians[0][3] = Jac(0, 0);
									jacobians[0][4] = Jac(0, 1);
									jacobians[0][5] = Jac(0, 2);
									Jac = -Jac * R_ij.matrix() * Sophus::SO3d::hat(p);
									jacobians[0][0] = Jac(0, 0);
									jacobians[0][1] = Jac(0, 1);
									jacobians[0][2] = Jac(0, 2);
									//	std::cout << "tracking dedphi = \n" << Jac << std::endl;
								}
								return true;
							}
						}
					}
				}
			}
		}
		*residuals = 0;
		ft->isProjected = false;
		if (jacobians && jacobians[0])
			memset(jacobians[0], 0, sizeof(double) * 6);
		return true;
	}

private:
	std::shared_ptr<Feature> ft;
	std::shared_ptr<viFrame> viframe_i;
	std::shared_ptr<viFrame> viframe_j;
	Sophus::SE3d T_SB;
	double sqrt_info;
	double I_i;
};

class CERES_EXPORT SE3Parameterization : public ceres::LocalParameterization {
public:
	virtual ~SE3Parameterization() {}

	virtual bool Plus(const double *x,
	                  const double *delta,
	                  double *x_plus_delta) const;

	virtual bool ComputeJacobian(const double *x,
	                             double *jacobian) const;

	virtual int GlobalSize() const { return 6; }

	virtual int LocalSize() const { return 6; }
};

bool SE3Parameterization::ComputeJacobian(const double *x, double *jacobian) const {
	ceres::MatrixRef(jacobian, 6, 6) = ceres::Matrix::Identity(6, 6);
	return true;
}

bool SE3Parameterization::Plus(const double *x,
                               const double *delta,
                               double *x_plus_delta) const {
	Eigen::Vector3d origin_x, delta_x;
	for (int i = 0; i < 3; ++i) {
		if (std::abs(delta[i]) > 0.001 || std::abs(delta[i + 3]) > 0.001) {
			memcpy(x_plus_delta, x, sizeof(double) * 6);

			for (int j = 0; j < 6; j++)
				x_plus_delta[j] += 0.001;
			return true;
		}
		origin_x(i) = x[i];
		delta_x(i) = delta[i];
		x_plus_delta[3 + i] = x[3 + i] + delta[3 + i];
	}

	Sophus::SO3d R = Sophus::SO3d::exp(origin_x);
	Sophus::SO3d delta_R = Sophus::SO3d::exp(delta_x);
	Eigen::Matrix<double, 3, 1> x_plus_delta_lie = (R * delta_R).log();

	for (int i = 0; i < 3; ++i) x_plus_delta[i] = x_plus_delta_lie(i, 0);
	return true;
}

Tracker::Tracker() {}

class depthErr : public ceres::SizedCostFunction<1, 1> {
public:
	virtual ~depthErr() {}

	depthErr(std::shared_ptr<viFrame> nextFrame, double I_k,
	         const cvMeasure::features_t::value_type &f, Sophus::SE3d &pose_) : f_(f), pose(pose_) {
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
		Eigen::Vector3d P_next = pose * (d * normPoint);
		auto &cam = nextFrame->getCam();
		auto uv = cam->world2cam(P_next);
		// std::cout << "uv = \n" << uv << std::endl;
		if (uv(0) < 0 || uv(0) >= nextFrame->getCVFrame()->getWidth() || uv(1) < 0 ||
		    uv(1) >= nextFrame->getCVFrame()->getHeight())
			return false;
		//	std::cout << "uv1 = \n" << uv << std::endl;

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
			Jac = Jac * pose.rotationMatrix();
			//		std::cout << "jac : \n" << Jac << std::endl;
			jacobians[0][0] = Jac * normPoint;
		}

	}

private:
	const cvMeasure::features_t::value_type f_;
	Eigen::Vector3d normPoint;
	std::shared_ptr<viFrame> nextFrame;
	double I_k;
	Sophus::SE3d &pose;
};


int Tracker::reProject(std::shared_ptr<viFrame> &viframe_i, std::shared_ptr<viFrame> &viframe_j,
                       Sophus::SE3d &Tij, Eigen::Matrix<double, 6, 6> &infomation) {
	cvMeasure::features_t &fts = viframe_i->getCVFrame()->getMeasure().fts_;
	int width = viframe_j->getCVFrame()->getWidth();
	int height = viframe_j->getCVFrame()->getHeight();
	int cellwidth = viframe_j->getCVFrame()->getWidth() / detectCellWidth;
	int chellheight = viframe_j->getCVFrame()->getHeight() / detectCellHeight;
	Sophus::SE3d _SPose_j = viframe_i->getT_BS().inverse() * viframe_i->getPose() * Tij;
	int cntCell = 0;
	std::list<cvMeasure::features_t::value_type> toErase;
	int cnt = 0;
	for (cvMeasure::features_t::iterator it = fts.begin(); it != fts.end(); ++it) {
		auto &ft = *it;
		ft->point->pos_mutex.lock_shared();
		Eigen::Vector3d pos = ft->point->pos_;
		ft->point->pos_mutex.unlock_shared();
		Eigen::Vector2d pos_;
		Eigen::Vector2d uvi, uvj;
		Eigen::Vector3d pi = viframe_i->getCVFrame()->getPose() * pos;
		if (pi[2] < 0.00000001 || std::isinf(pi[2]))
			toErase.push_back(*it);

		else {
			uvi = viframe_i->getCam()->world2cam(pi);
			if (uvi(0) < width && uvi(1) < height && uvi(0) > 0 && uvi(1) > 0) {
				pos = _SPose_j * pos;
				if (pos[2] < 0.00000001 || std::isinf(pos[2]))
					toErase.push_back(*it);

				else {
					pos /= pos[2];
					pos_ = pos.block<2, 1>(0, 0);
					uvj = viframe_j->getCam()->world2cam(pos_);
					if (uvj(0) < width && uvj(1) < height && uvj(0) > 0 && uvj(1) > 0) {
						int u = int(uvj(0) / cellwidth);
						int v = int(uvj(1) / chellheight);

						if (ft->isBAed != true) {
							for (int i = 0; i < ft->level; ++i)
								uvi /= 2.0;

							double Ii = viframe_i->getCVFrame()->getIntensityBilinear(uvi(0), uvi(1), ft->level);

							ft->point->pos_mutex.lock_shared();
							double initPth = ft->point->pos_[2];
							ft->point->pos_mutex.unlock_shared();
							ceres::Problem problem;
							// std::cout <<" Ii = " << Ii << std::endl;
							ceres::CostFunction *func = new depthErr(viframe_j, Ii, ft, _SPose_j);
							problem.AddResidualBlock(func, new ceres::HuberLoss(1), &initPth);
							ceres::Solver::Options option;
							option.minimizer_type = ceres::LINE_SEARCH;
							option.linear_solver_type = ceres::DENSE_QR;
							option.min_line_search_step_size = 0.001;
							ceres::Solver::Summary summary;
							ceres::Solve(option, &problem, &summary);
							//std::cout << summary.BriefReport() << std::endl;
							if (summary.termination_type == ceres::CONVERGENCE) {
								ft->point->pos_mutex.lock_shared();
								Eigen::Vector3d normPoint = ft->point->pos_ / ft->point->pos_(2);
								ft->point->pos_mutex.unlock_shared();
								Eigen::Vector3d Pj = _SPose_j * (initPth * normPoint);
								if (Pj[2] < 0.00000001 || std::isinf(Pj[2])) {
									toErase.push_back(*it);
									if (ft->point->n_succeeded_reproj_ >= 1)
										ft->point->n_failed_reproj_++;
									else
										toErase.push_back(*it);

									continue;
								}

								uvj = viframe_j->getCam()->world2cam(Pj);

								if (uvj(0) >= width || uvj(1) >= height || uvj(0) <= 0 || uvj(1) <= 0) {
									toErase.push_back(*it);
									if (ft->point->n_succeeded_reproj_ >= 1)
										ft->point->n_failed_reproj_++;
									else
										toErase.push_back(*it);
									continue;
								}

								Eigen::Matrix<double, 1, 6> Jac;
								Jac.block<1, 3>(0, 0) = normPoint.transpose() *
								                        Sophus::SO3d::hat(
										                        _SPose_j.so3().inverse() *
										                        (Pj - _SPose_j.translation()));

								Jac.block<1, 3>(0, 3) = normPoint.transpose() * _SPose_j.so3().inverse().matrix();
								Jac = Jac / (normPoint.transpose() * normPoint);

								double newJac = Jac * infomation.inverse() * Jac.transpose();
								ft->point->updateDepth(initPth, 1.0 / newJac);

								ft->point->pos_mutex.lock_shared();
								pos = ft->point->pos_;
								ft->point->pos_mutex.unlock_shared();
								pos = _SPose_j * pos;
								pos /= pos[2];
								pos_ = pos.block<2, 1>(0, 0);
								uvj = viframe_j->getCam()->world2cam(pos_);

								u = int(uvj(0) / cellwidth);
								v = int(uvj(1) / chellheight);

								for (int i = 0; i < ft->level; ++i)
									uvj /= 2.0;

							}
							std::shared_ptr<Feature> ft_ = std::make_shared<Feature>(viframe_j->getCVFrame(),
							                                                         ft->point, uvj, pos, ft->level);
							ft_->isBAed.exchange(ft->isBAed);
							ft_->point->obsMutex.lock();
							ft_->point->obs_.push_back(ft_);
							ft_->point->obsMutex.unlock();
							viframe_j->getCVFrame()->addFeature(ft_);

							if (!viframe_j->getCVFrame()->checkCell(u, v)) {
								viframe_j->getCVFrame()->setCellTrue(u, v);
								cntCell++;
							}
							ft->point->n_succeeded_reproj_++;
							continue;
						}

						else {
							std::shared_ptr<Feature> ft_ = std::make_shared<Feature>(viframe_j->getCVFrame(),
							                                                         ft->point, uvj, pos, ft->level);
							ft_->isBAed.exchange(ft->isBAed);
							ft_->point->obsMutex.lock();
							ft_->point->obs_.push_back(ft_);
							ft_->point->obsMutex.unlock();
							viframe_j->getCVFrame()->addFeature(ft_);
							if (!viframe_j->getCVFrame()->checkCell(u, v)) {
								viframe_j->getCVFrame()->setCellTrue(u, v);
								cntCell++;
							}

							ft->point->n_succeeded_reproj_++;
							continue;
						}
					}
				}
			}
		}

		if (ft->point->n_succeeded_reproj_ >= 1)
			ft->point->n_failed_reproj_++;
		else
			toErase.push_back(*it);
	}

	for (auto &it : toErase)
		fts.remove(it);

	return cntCell;
}

bool Tracker::Tracking(std::shared_ptr<viFrame> &viframe_i, std::shared_ptr<viFrame> &viframe_j,
                       Sophus::SE3d &T_ij_, Eigen::Matrix<double, 6, 6> &infomation, int n_iter) {

	ceres::Problem problem;
	ceres::Solver::Options options;
	ceres::Solver::Summary summary;

	cvMeasure::features_t &fts = viframe_i->getCVFrame()->getMeasure().fts_;
	std::cout << "the size of fts is : " << fts.size() << std::endl;
	Eigen::Vector3d so3 = T_ij_.so3().log();
	Eigen::Vector3d &tij = T_ij_.translation();
	double t_ij[6];
	for (int i = 0; i < 3; ++i) {
		t_ij[i] = so3(i);
		t_ij[3 + i] = tij(i);
	}
	int numOpt = 0;
	std::list<cvMeasure::features_t::iterator> toErase;
	auto &model = trackModel();
	auto T_SB = viframe_i->getT_BS().inverse();
	for (cvMeasure::features_t::iterator it = fts.begin(); it != fts.end(); it++) {
		auto ft = *it;

		ft->point->pos_mutex.lock_shared();
		const Eigen::Vector3d p = ft->point->pos_;
		ft->point->pos_mutex.unlock_shared();
		Eigen::Vector3d pi = viframe_i->getCVFrame()->getPose() * p;
		if (pi(2) > 0.0000000001) {
			Sophus::SE3d T_Si = T_SB * viframe_i->getPose();
			Eigen::Vector3d pj = T_Si * T_ij_ * p;

			if (pj(2) > 0.0000000001) {
				const viFrame::cam_t &cam = viframe_j->getCam();
				double u = cam->fx() * (pj(0) / pj(2)) + cam->cx();
				if (u >= 3 && u < viframe_j->getCVFrame()->getWidth() - 3) {
					double v = cam->fy() * (pj(1) / pj(2)) + cam->cy();
					if (v >= 3 && v < viframe_j->getCVFrame()->getHeight() - 3) {

						Eigen::Vector2d px = cam->world2cam(pi);
						if (px(0) >= 3 && px(0) < viframe_i->getCVFrame()->getWidth() - 3
						    && px(1) >= 3 && px(1) < viframe_i->getCVFrame()->getHeight() - 3) {

							for (int i = 0; i < ft->level; ++i) {
								u /= 2.0;
								v /= 2.0;
								px /= 2.0;
							}

							double err = 0.0;
							for (auto &step : model) {
								double u_ = u + double(step(0, 0));
								double v_ = v + double(step(1, 0));
								double px0 = px(0, 0) + double(step(0, 0));
								double px1 = px(1, 0) + double(step(1, 0));
								Eigen::Vector2d px_ = px;
								px_(0, 0) += step(0, 0);
								px_(1, 0) += step(1, 0);
								err += std::abs(viframe_j->getCVFrame()->getIntensityBilinear(u_, v_, ft->level) -
								                viframe_i->getCVFrame()->getIntensityBilinear(px0, px1, ft->level));
							}

							if (err < model.size() * IuminanceErr) {
								//std::cout << "err = " << err << std::endl;
								numOpt++;
								problem.AddResidualBlock(new TrackingErr(ft, viframe_i, viframe_j),
								                         new ceres::HuberLoss(0.5), t_ij);
								continue;
							}
						}
					}
				}
			}
		}
		(*it)->isProjected = false;
		toErase.push_back(it);
	}

	printf("the num of bad point : %lu\n", toErase.size());
	for (auto it : toErase) {
		if ((*it)->point->n_succeeded_reproj_ < 2)
			fts.erase(it);
	}

	if (numOpt < 20) {
		std::cout << "the num of project point is too small : " << numOpt << std::endl;
		return false;
	}

	problem.SetParameterization(t_ij, new SE3Parameterization);

	options.max_num_iterations = n_iter;
	options.minimizer_type = ceres::TRUST_REGION;
	options.trust_region_strategy_type = ceres::DOGLEG;
	options.linear_solver_type = ceres::DENSE_QR;
	//options.minimizer_progress_to_stdout = true;

	ceres::Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << std::endl;
	for (int i = 0; i < 3; ++i) {
		so3(i) = t_ij[i];
		tij(i) = t_ij[3 + i];
	}

	if (summary.termination_type != ceres::CONVERGENCE)
		return false;
	T_ij_.so3() = Sophus::SO3d::exp(so3);

	int cnt = 0;
	double sq_norm = 0.0;
	infomation = Eigen::Matrix<double, 6, 6>::Zero();
	for (auto &ft : fts) {
		if (ft->isProjected == false) continue;
		ft->point->pos_mutex.lock_shared();
		Eigen::Vector3d pj = T_ij_.so3().inverse() * (T_ij_ * ft->point->pos_);
		Eigen::Vector3d normP = ft->point->pos_ / ft->point->pos_(2);
		ft->point->pos_mutex.unlock_shared();
		if (pj(2) < 0.000000001 || std::isinf(pj(2)))
			continue;

		Eigen::Matrix<double, 1, 6> Jac;
		Jac.block<1, 3>(0, 0) = normP.transpose() * Sophus::SO3d::hat(pj);
		Jac.block<1, 3>(0, 3) = normP.transpose();
		sq_norm += Jac * Jac.transpose();
		infomation += Jac.transpose() * Jac * (1.0 / ft->point->getDepthInformation());
		cnt++;
	}

	if (cnt < 16) return false;

	infomation = infomation / sq_norm / sq_norm;
	infomation = infomation.inverse();
	return true;
}

}

