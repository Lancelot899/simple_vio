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

class CERES_EXPORT SO3Parameterization : public ceres::LocalParameterization {
public:
	virtual ~SO3Parameterization() {}
	virtual bool Plus(const double* x,
	                  const double* delta,
	                  double* x_plus_delta) const;
	virtual bool ComputeJacobian(const double* x,
	                             double* jacobian) const;
	virtual int GlobalSize() const { return 3; }
	virtual int LocalSize() const { return 3; }
};

bool SO3Parameterization::ComputeJacobian(const double *x, double *jacobian) const {
	ceres::MatrixRef(jacobian, 3, 3) = ceres::Matrix::Identity(3, 3);
	return true;
}

bool SO3Parameterization::Plus(const double* x,
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
	Err.block<3, 1>(0, 0) = Sophus::SO3d::log((T_ij.so3() * Sophus::SO3d::exp(JBias.block<3, 3>(0, 0) * dbias_g)).inverse()
	                                          * (Ri.inverse() * Rj));
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
			Eigen::Matrix<double, 9, 15> _jacobianOplusXi;
			_jacobianOplusXi.block<3, 3>(0, 0) = -rightJacobian(Err.segment<3>(0))
			                                     * (Rj.inverse() * Ri).matrix();
			_jacobianOplusXi.block<3, 3>(0, 3) = _jacobianOplusXi.block<3, 3>(0, 6)
					= _jacobianOplusXi.block<3, 3>(0, 9)
					= _jacobianOplusXi.block<3, 3>(0, 12)
					= Eigen::Matrix3d::Zero();
			_jacobianOplusXi.block<3, 3>(3, 0) = Sophus::SO3d::hat(Ri.inverse() * (vj - vi - imuParam->g * dt));
			_jacobianOplusXi.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero();
			_jacobianOplusXi.block<3, 3>(3, 6) = -Ri.inverse().matrix();
			_jacobianOplusXi.block<3, 3>(3, 9)  = Eigen::Matrix3d::Zero();
			_jacobianOplusXi.block<3, 3>(3, 12) = Eigen::Matrix3d::Zero();
			_jacobianOplusXi.block<3, 3>(6, 0) = Sophus::SO3d::hat(Ri.inverse() * (pj - pi - vi * dt - 0.5 * imuParam->g  * dt * dt));
			_jacobianOplusXi.block<3, 3>(6, 3)  = -Eigen::Matrix3d::Identity();
			_jacobianOplusXi.block<3, 3>(6, 6)  = -Ri.inverse().matrix() * dt;
			_jacobianOplusXi.block<3, 3>(6, 9)  = Eigen::Matrix3d::Zero();
			_jacobianOplusXi.block<3, 3>(6, 12) = Eigen::Matrix3d::Zero();

			_jacobianOplusXi = L * _jacobianOplusXi;

			int k = 0;
			for(int i = 0; i < 9; ++i) {
				for(int j = 0; j < 15; ++j)
					jacobians[0][k++] = _jacobianOplusXi(i, j);
			}
		}

		if(jacobians[1]) {
			Eigen::Matrix<double, 9, 15> _jacobianOplusXj;

			_jacobianOplusXj.block<3, 3>(0, 0) = rightJacobian(Err.segment<3>(0));
			_jacobianOplusXj.block<3, 3>(0, 3) = Eigen::Matrix3d::Zero();
			_jacobianOplusXj.block<3, 3>(0, 6) = Eigen::Matrix3d::Zero();
			_jacobianOplusXj.block<3, 3>(0, 9) = -rightJacobian(Err.segment<3>(0)) * Sophus::SO3d::exp(Err.segment<3>(0)).inverse().matrix()
			                                     * rightJacobian(JBias.block<3, 3>(0, 0) * dbias_g) * JBias.block<3, 3>(0, 0);
			_jacobianOplusXj.block<3, 3>(0, 12) = Eigen::Matrix3d::Zero();
			_jacobianOplusXj.block<3, 3>(3, 0) = Eigen::Matrix3d::Zero();
			_jacobianOplusXj.block<3, 3>(3, 3) = Eigen::Matrix3d::Zero();
			_jacobianOplusXj.block<3, 3>(3, 6) = Ri.inverse().matrix();
			_jacobianOplusXj.block<3, 3>(3, 9)  = -JBias.block<3, 3>(6, 0);
			_jacobianOplusXj.block<3, 3>(3, 12) = -JBias.block<3, 3>(3, 0);
			_jacobianOplusXj.block<3, 3>(6, 0)  = Eigen::Matrix3d::Zero();
			_jacobianOplusXj.block<3, 3>(6, 3)  = (Ri.inverse() * Rj).matrix();
			_jacobianOplusXj.block<3, 3>(6, 6)  = Eigen::Matrix3d::Zero();
			_jacobianOplusXj.block<3, 3>(6, 9)  = -JBias.block<3, 3>(12, 0);
			_jacobianOplusXj.block<3, 3>(6, 12) = -JBias.block<3, 3>(9, 0);

			_jacobianOplusXj = L * _jacobianOplusXj;

			int k = 0;
			for(int i = 0; i < 9; ++i) {
				for(int j = 0; j < 15; ++j)
					jacobians[0][k++] = _jacobianOplusXj(i, j);
			}
		}
	}

	return true;
}

class PnPErr : public ceres::SizedCostFunction<2, 6, 3> {
public:
	PnPErr(std::shared_ptr<viFrame> &viframe, std::shared_ptr<Point> &point);

	virtual bool Evaluate(double const* const* parameters,
	                      double* residuals,
	                      double** jacobians) const;
private:
	std::shared_ptr<viFrame> viframe;
	std::shared_ptr<Feature> ft;
	std::shared_ptr<Point> point;
	Sophus::SE3d T_SB;
};

PnPErr::PnPErr(std::shared_ptr<viFrame> &viframe, std::shared_ptr<Point> &point) {
	this->viframe = viframe;
	this->point = point;
	ft = point->findFrameRef(viframe->getCVFrame());
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
				k += 3;
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
                   std::vector <std::shared_ptr<imuFactor>> &imufactors) {

}