//
// Created by lancelot on 4/18/17.
//

#include <ceres/ceres.h>

#include "SimpleBA.h"
#include "DataStructure/cv/cvFrame.h"
#include "DataStructure/viFrame.h"
#include "DataStructure/imu/imuFactor.h"
#include "DataStructure/cv/Point.h"
#include "DataStructure/cv/Feature.h"
#include "../BundleAdjustemt.h"

class CERES_EXPORT VioPose : public ceres::LocalParameterization {
public:
	virtual ~VioPose() {}
	virtual bool Plus(const double* x,
	                  const double* delta,
	                  double* x_plus_delta) const;
	virtual bool ComputeJacobian(const double* x,
	                             double* jacobian) const;
	virtual int GlobalSize() const { return 15; }
	virtual int LocalSize() const { return 15; }
};

bool VioPose::ComputeJacobian(const double *x, double *jacobian) const {
	ceres::MatrixRef(jacobian, 15, 15) = ceres::Matrix::Identity(15, 15);
	return true;
}

bool VioPose::Plus(const double* x,
                   const double* delta,
                   double* x_plus_delta) const {
	Eigen::Vector3d origin_x, delta_x;
	for(int i = 0; i < 3; ++i) {
		origin_x(i) = x[i];
		delta_x(i) = delta[i];
	}

	Sophus::SO3d R = Sophus::SO3d::exp(origin_x);
	Sophus::SO3d delta_R = Sophus::SO3d::exp(delta_x);
	Eigen::Matrix<double, 3, 1> x_plus_delta_lie = (R * delta_R).log();

	for(int i = 0; i < 3; ++i) x_plus_delta[i] = x_plus_delta_lie(i, 0);
	for(int i = 3; i < 15; ++i) x_plus_delta[i] = x[i] + delta[i];
	return true;
}


class IMUErr : public ceres::SizedCostFunction<9, 15, 15> {
public:
	IMUErr(std::shared_ptr<imuFactor> &imufactor,
	       std::shared_ptr<viFrame> &viframe_i,
	       std::shared_ptr<viFrame> &viframe_j);

	virtual bool Evaluate(double const* const* parameters,
	                      double* residuals,
	                      double** jacobians) const;

private:
	std::shared_ptr<imuFactor> imufactor;
	std::shared_ptr<viFrame> viframe_i;
	std::shared_ptr<viFrame> viframe_j;
	Eigen::Matrix<double, 9, 9> L;
};

IMUErr::IMUErr(std::shared_ptr<imuFactor> &imufactor,
               std::shared_ptr<viFrame> &viframe_i,
               std::shared_ptr<viFrame> &viframe_j) {
	this->imufactor = imufactor;
	this->viframe_i = viframe_j;
	this->viframe_j = viframe_j;

	auto Var = imufactor->getVar();
	Eigen::Matrix<double, 9, 9> information = Var.inverse();
	Eigen::LLT<Eigen::Matrix<double, 9, 9>> llt(information);
	L = llt.matrixL();
}

bool IMUErr::Evaluate(double const *const *parameters,
                      double *residuals,
                      double **jacobians) const {
	Eigen::Map<const Eigen::Matrix<double, 15, 1>> posei(parameters[0]);
	Eigen::Map<const Eigen::Matrix<double, 15, 1>> posej(parameters[1]);

	Eigen::Vector3d dbias_g = posej.block<3, 1>(9, 0) - posei.block<3, 1>(9, 0);
	Eigen::Vector3d dbias_a = posej.block<3, 1>(12, 0) - posei.block<3, 1>(12, 0);


	auto imuParam = viframe_i->getImuParam();

	const Eigen::Vector3d &vi = posei.block<3, 1>(6, 0);
	const Eigen::Vector3d &vj = posej.block<3, 1>(6, 0);
	const Eigen::Vector3d &pi = posei.block<3, 1>(3, 0);
	const Eigen::Vector3d &pj = posej.block<3, 1>(3, 0);

	double dt = (viframe_j->getTimeStamp() - viframe_i->getTimeStamp()).toSec();

	Sophus::SO3d Ri = Sophus::SO3d::exp(posei.block<3, 1>(0, 0));
	Sophus::SO3d Rj = Sophus::SO3d::exp(posej.block<3, 1>(0, 0));
	imuFactor::FacJBias_t JBias = imufactor->getJBias();
	const Sophus::SE3d& T_ij = imufactor->getPoseFac();

	Eigen::Matrix<double, 9, 1> Err;
	Err.block<3, 1>(0, 0) = Sophus::SO3d::log((T_ij.so3() * Sophus::SO3d::exp(JBias.block<3, 3>(0, 0)
	                                           * dbias_g)).inverse() * (Ri.inverse() * Rj));

	Err.block<3, 1>(6, 0) = Ri.inverse() * (vj - vi - imuParam->g * dt)
	                         - (imufactor->getSpeedFac() + JBias.block<3, 3>(6, 0) * dbias_g
	                                                   + JBias.block<3, 3>(3, 0) * dbias_a);

	Err.block<3, 1>(3, 0) = Ri.inverse() * (pj - pi - vi * dt - 0.5 * imuParam->g * dt * dt)
	                        - (imufactor->getPoseFac().translation() + JBias.block<3, 3>(12, 0) * dbias_g
	                                                  +  JBias.block<3, 3>(9, 0) * dbias_a);
	Eigen::Matrix<double, 9, 1> err = L * Err;

	for(int i = 0; i < 9; ++i)
		residuals[i] = err(i, 0);

	if(jacobians) {
		if(jacobians[0]) {
			Eigen::Matrix<double, 9, 15> JacXi;
			JacXi.block<3, 3>(0, 0) = -rightJacobian(Err.segment<3>(0))
			                                     * (Rj.inverse() * Ri).matrix();
			JacXi.block<3, 3>(0, 3) = JacXi.block<3, 3>(0, 6) = JacXi.block<3, 3>(0, 9) = JacXi.block<3, 3>(0, 12) = Eigen::Matrix3d::Zero();
			JacXi.block<3, 3>(3, 0) = Sophus::SO3d::hat(Ri.inverse() * (vj - vi - imuParam->g * dt));
			JacXi.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero();
			JacXi.block<3, 3>(3, 6) = -Ri.inverse().matrix();
			JacXi.block<3, 3>(3, 9)  = Eigen::Matrix3d::Zero();
			JacXi.block<3, 3>(3, 12) = Eigen::Matrix3d::Zero();
			JacXi.block<3, 3>(6, 0) = Sophus::SO3d::hat(Ri.inverse() * (pj - pi - vi * dt - 0.5 * imuParam->g  * dt * dt));
			JacXi.block<3, 3>(6, 3)  = -Eigen::Matrix3d::Identity();
			JacXi.block<3, 3>(6, 6)  = -Ri.inverse().matrix() * dt;
			JacXi.block<3, 3>(6, 9)  = Eigen::Matrix3d::Zero();
			JacXi.block<3, 3>(6, 12) = Eigen::Matrix3d::Zero();

			JacXi = L * JacXi;

			int k = 0;
			for(int i = 0; i < 9; ++i) {
				for(int j = 0; j < 15; ++j)
					jacobians[0][k++] = JacXi(i, j);
			}
		}

		if(jacobians[1]) {
			Eigen::Matrix<double, 9, 15> JacXj;

			JacXj.block<3, 3>(0, 0) = rightJacobian(Err.segment<3>(0));
			JacXj.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
			JacXj.block<3, 3>(0, 6) = Eigen::Matrix3d::Zero();
			JacXj.block<3, 3>(0, 9) = -rightJacobian(Err.segment<3>(0)) * Sophus::SO3d::exp(Err.segment<3>(0)).inverse().matrix()
			                                     * rightJacobian(JBias.block<3, 3>(0, 0) * dbias_g) * JBias.block<3, 3>(0, 0);
			JacXj.block<3, 3>(0, 12) = Eigen::Matrix3d::Zero();
			JacXj.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
			JacXj.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero();
			JacXj.block<3, 3>(3, 6) = Ri.inverse().matrix();
			JacXj.block<3, 3>(3, 9)  = -JBias.block<3, 3>(6, 0);
			JacXj.block<3, 3>(3, 12) = -JBias.block<3, 3>(3, 0);
			JacXj.block<3, 3>(6, 0)  = Eigen::Matrix3d::Zero();
			JacXj.block<3, 3>(6, 3)  = (Ri.inverse() * Rj).matrix();
			JacXj.block<3, 3>(6, 6)  = Eigen::Matrix3d::Zero();
			JacXj.block<3, 3>(6, 9)  = -JBias.block<3, 3>(12, 0);
			JacXj.block<3, 3>(6, 12) = -JBias.block<3, 3>(9, 0);

			JacXj = L * JacXj;

			int k = 0;
			for(int i = 0; i < 9; ++i) {
				for(int j = 0; j < 15; ++j)
					jacobians[1][k++] = JacXj(i, j);
			}
		}
	}

	return true;
}

class PnPErr : public ceres::SizedCostFunction<2, 15, 3> {
public:
	PnPErr(std::shared_ptr<viFrame> &viframe,  std::shared_ptr<Feature> &ft);

	virtual bool Evaluate(double const* const* parameters,
	                      double* residuals,
	                      double** jacobians) const;
private:
	std::shared_ptr<viFrame> viframe;
	std::shared_ptr<Feature> ft;
	Sophus::SE3d T_SB;
};

PnPErr::PnPErr(std::shared_ptr<viFrame> &viframe, std::shared_ptr<Feature> &ft) {
	this->viframe = viframe;
	this->ft = ft;
	T_SB = viframe->getT_BS().inverse();
}

bool PnPErr::Evaluate(double const *const *parameters,
                      double *residuals,
                      double **jacobians) const {

	Eigen::Vector3d so3, trans;
	for (int i = 0; i < 3; ++i) {
		so3(i) = parameters[0][i];
		trans(i) = parameters[0][3 + i];
	}

	Sophus::SO3d R = Sophus::SO3d::exp(so3);
	Eigen::Map<const Eigen::Vector3d> p(parameters[1]);
	Eigen::Vector3d pi = R * p + trans;
	pi = T_SB * pi;
	auto cam = viframe->getCam();
	Eigen::Vector2d err = ft->px - cam->world2cam(pi);

	residuals[0] = err[0];
	residuals[1] = err[1];

	if(jacobians) {
		Eigen::Matrix<double, 2, 3> K;
		K(0, 0) = cam->fx() / pi(2);
		K(0, 1) = K(1, 0) = 0;
		K(0, 2) = - pi(0) * K(0, 0) / pi(2);
		K(1, 1) = cam->fy() / pi(2);
		K(1, 2) = - pi(1) * K(1, 1) / pi(2);

		if(jacobians[0]) {
			memset(jacobians[0], 0, sizeof(double) * 30);
			Eigen::Matrix3d dedphiR = T_SB.rotationMatrix() * R.matrix() * Sophus::SO3d::hat(p);
			Eigen::Matrix3d dedtR = -T_SB.rotationMatrix();

			Eigen::Matrix<double, 2, 3> dedphi = K * dedphiR;
			Eigen::Matrix<double, 2, 3> dedt = K * dedtR;
			int k = 0;
			for(int i = 0; i < 2; ++i) {
				for(int j = 0; j < 3; ++j) {
					jacobians[0][k] = dedphi(i, j);
					jacobians[0][k + 3] = dedt(i, j);
					k++;
				}
				k += 9;
			}
		}

		if(jacobians[1]) {
			Eigen::Matrix3d dedpR = -T_SB.rotationMatrix() * R.matrix();
			Eigen::Matrix<double, 2, 3> dedp = K * dedpR;
			int k = 0;
			for(int i = 0; i < 2; ++i) {
				for(int j = 0; j < 3; ++j)
					jacobians[1][k++] = dedp(i, j);
			}
		}
	}
	return true;

}


SimpleBA::SimpleBA() {

}

SimpleBA::~SimpleBA() {}

bool SimpleBA::run(std::vector <std::shared_ptr<viFrame>> &viframes,
                   typename BundleAdjustemt::obsModeType &obsModes,
                   std::vector <std::shared_ptr<imuFactor>> &imufactors, int iter_) {
	size_t poseNum = viframes.size();
	if(poseNum < widowSize)
		return false;

	double *poseData = (double *)malloc(sizeof(double) * poseNum * 15);
	for(size_t i = 0; i < poseNum; ++i) {
		auto pose = viframes[i]->getCVFrame()->getPose();
		Eigen::Vector3d phi = pose.so3().log();
		memcpy(poseData + i * 15, phi.data(), sizeof(double) * 3);
		memcpy(poseData + i * 15 + 3, pose.translation().data(), sizeof(double) * 3);
		memcpy(poseData + i * 15 + 6, viframes[i]->getSpeedAndBias().data(), sizeof(double) * 9);
	}

	std::map<std::shared_ptr<cvFrame>, int> memTabel;
	ceres::Problem problem;

	{
		size_t i = 0;
		for (i; i < imufactors.size(); ++i) {
			ceres::CostFunction *costFun = new IMUErr(imufactors[i], viframes[i], viframes[i + 1]);
			problem.AddResidualBlock(costFun, new ceres::HuberLoss(0.5), poseData + i * 15, poseData + i * 15 + 15);
			problem.SetParameterization(poseData + i * 15, new VioPose());
			memTabel.insert(std::make_pair(viframes[i]->getCVFrame(), i));
        }

		problem.SetParameterization(poseData + i * 15, new VioPose());
		memTabel.insert(std::make_pair(viframes[i]->getCVFrame(), i));
	}

	std::list<std::shared_ptr<SimpleBA::CopyPoint>> copy_points;

	for(auto it = obsModes.begin(); it != obsModes.end(); ++it) {
		if(it->second.size() < 3)
			continue;

		if(it->first->last_projected_kf_id_ == viframes[poseNum -1]->ID) {
			auto pt = std::make_shared<SimpleBA::CopyPoint>();
			pt->point = it->first;
			pt->pos_ = it->first->pos_;
			for(auto &ft : it->second) {
				int i = memTabel.find(ft->frame)->second;
				ceres::CostFunction *costFun = new PnPErr(viframes[i], ft);
				problem.AddResidualBlock(costFun, new ceres::HuberLoss(0.5), poseData + i * 15, pt->pos_.data());
				copy_points.push_back(pt);
			}
		}

		else {
			for(auto &ft : it->second) {
				int i = memTabel.find(ft->frame)->second;
				ceres::CostFunction *costFun = new PnPErr(viframes[i], ft);
				problem.AddResidualBlock(costFun, new ceres::HuberLoss(0.5), poseData + i * 15, it->first->pos_.data());
			}
		}
	}

	ceres::Solver::Options options;
	options.dynamic_sparsity = true;
	options.max_num_iterations = iter_;
	options.sparse_linear_algebra_library_type = ceres::SUITE_SPARSE;
	options.minimizer_type = ceres::TRUST_REGION;
	options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
	options.trust_region_strategy_type = ceres::DOGLEG;
	options.minimizer_progress_to_stdout = true;
	options.dogleg_type = ceres::SUBSPACE_DOGLEG;

	ceres::Solver::Summary summary;
	Solve(options, &problem, &summary);
	std::cout << summary.BriefReport() << std::endl;

	bool res = true;
	if(res = (summary.termination_type == ceres::CONVERGENCE)) {
		for(size_t i = 0; i < poseNum; ++i) {
			Eigen::Map<Eigen::Vector3d> phi(poseData + i * 15);
			Sophus::SO3d R = Sophus::SO3d::exp(phi);
			Eigen::Map<Eigen::Vector3d> trans(poseData + i * 15 + 3);
			Sophus::SE3d T(R, trans);
			viframes[i]->getCVFrame()->setPose(T);
			Eigen::Map<IMUMeasure::SpeedAndBias> spbs(poseData + i * 15 + 6);
			viframes[i]->getSpeedAndBias() = spbs;
		}

		for(auto &data : copy_points)
			data->point->pos_ = data->pos_;

	}

	free(poseData);

	return res;
}