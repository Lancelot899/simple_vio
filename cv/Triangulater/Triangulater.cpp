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
                              std::shared_ptr<viFrame> &nextFrame, const Sophus::SE3d &T_nk, int iter) {
	int newCreatPoint = 0;
	int height_ = keyFrame->getCVFrame()->getHeight();
	int width_ = keyFrame->getCVFrame()->getWidth();

	const cvMeasure::features_t &fts = keyFrame->getCVFrame()->getMeasure().fts_;

	ceres::Solver::Options option;
	option.max_num_iterations = iter;
	option.minimizer_type = ceres::LINE_SEARCH;
	option.linear_solver_type = ceres::DENSE_QR;

	boost::mt19937 gen;
	boost::uniform_real<>dist(0.5, 1.5);
	boost::variate_generator<boost::mt19937&,boost::uniform_real<>> scale_ (gen, dist);


	for (auto &ftKey : fts) {
		ceres::Solver::Summary summary;
		ceres::Problem problem;
		double initDepth = scale * (init_depth + scale_() * (T_nk.so3() * T_nk.translation())(2));
		bool depthIsValid = false;
		auto cam = nextFrame->getCam();
		Eigen::Vector2d uv;
		if(ftKey->isBAed) continue;
		if (ftKey->point->pos_[2] < 1.000000000001 && ftKey->point->pos_[2] > 0.99999999999) {
			auto &pos = ftKey->point->pos_;
			initDepth = pos(2);
			uv = cam->world2cam(keyFrame->getCVFrame()->getPose() * pos);
		} else {
			uv(0) = ftKey->px(0);
			uv(1) = ftKey->px(1);
		}

		for (int level = 0; level < ftKey->level; level++)
			uv /= 2.0;

		Eigen::Vector3d tmpPoint;
		problem.AddResidualBlock(
				new depthErr(nextFrame, keyFrame->getCVFrame()->getIntensityBilinear(uv(0), uv(1), ftKey->level), T_nk,
				             ftKey),
				nullptr, &initDepth);
		problem.SetParameterBlockVariable(&initDepth);
		ceres::Solve(option, &problem, &summary);
		if (summary.termination_type == ceres::CONVERGENCE) {
			newCreatPoint++;
			ftKey->point->pos_[2] = initDepth;
		}
	}
	return newCreatPoint;
}

#if 0
/// select a depth init which make sence
///
double depthMin = 0, depthMax = 0;
auto cam = keyFrame->getCam();
double fx = cam->fx(),  fy = cam->fy(), cx = cam->cx(), cy = cam->cy();
auto K_ = cam->K(); auto K_inv = cam->K_inv();
double R[3][3], T[3];
for (int i = 0; i < 3; ++i) {
	T[i] = T_nk.translation()(i);
	for (int j = 0; j < 3; ++j) {
		R[i][j] = T_nk.rotationMatrix()(i,j);
	}
}
Eigen::Vector3d tmpPoint,XYZ_without_Depth; tmpPoint<<ftKey->px(0),ftKey->px(1),1.0;
XYZ_without_Depth = K_inv*tmpPoint;
double fac0 = fx*(R[0][0]*XYZ_without_Depth(0)+ R[0][1]*XYZ_without_Depth(1) + R[0][2]*XYZ_without_Depth[2]) +
			  cx

struct TriangulaterSolver
{
	TriangulaterSolver(int iter = 50)
	{
		m_options.max_num_iterations = iter;
		m_options.linear_solver_type = ceres::DENSE_QR;
		/// m_options.trust_region_strategy_type = ceres::DOGLEG;
		m_options.dogleg_type = ceres::SUBSPACE_DOGLEG;
		/// m_options.minimizer_progress_to_stdout = true;
		m_bInitParameters = false;
	}

	bool SolveParameters()
	{
		if(keyFrame == true || keyFrame == true)
			return false;

		ceres::Problem problem;
		double depthInit = 1.0;

		for(auto &ft: fts) {
			problem.AddResidualBlock(new TriangulaterPhotometrixResidual(keyFrame,nextFrame,T_nk,ft->px(0),ft->px(1)),
									 new ceres::HuberLoss(0.5),depthInit);
		}
		problem.SetParameterization(&depthInit,new DepthParameter);

		ceres::Solve(m_options,&problem,&m_summary);
		return true;
	}

	bool                               m_bInitParameters;
	double                             m_EllipsoidParameters[6];
	std::shared_ptr<cvFrame>           keyFrame,nextFrame;
	cvMeasure::features_t              fts;
	Sophus::SE3d                       T_nk;
	ceres::Solver::Options             m_options;
	ceres::Solver::Summary             m_summary;
};

#endif
