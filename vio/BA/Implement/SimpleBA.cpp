//
// Created by lancelot on 4/18/17.
//

#include <ceres/ceres.h>

#include "SimpleBA.h"
#include "DataStructure/cv/cvFrame.h"
#include "DataStructure/viFrame.h"
#include "DataStructure/imu/imuFactor.h"
#include "DataStructure/cv/Point.h"


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
	Eigen::Matrix<double, 3, 1> x_plus_delta_lie = (delta_R * R).log();

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

};

IMUErr::IMUErr(std::shared_ptr<imuFactor> &imufactor,
               std::shared_ptr<viFrame> &viframe_i,
               std::shared_ptr<viFrame> &viframe_j) {
	this->imufactor = imufactor;
	this->viframe_i = viframe_j;
	this->viframe_j = viframe_j;
}

bool IMUErr::Evaluate(double const *const *parameters,
                      double *residuals,
                      double **jacobians) const {


	return true;
}

class PnPErr : public ceres::SizedCostFunction<1, 6, 3> {
public:
	PnPErr(std::shared_ptr<viFrame> &viframe, std::shared_ptr<Point> &point);

	virtual bool Evaluate(double const* const* parameters,
	                      double* residuals,
	                      double** jacobians) const;
private:
	std::shared_ptr<viFrame> viframe;
	std::shared_ptr<Point> point;

};

PnPErr::PnPErr(std::shared_ptr<viFrame> &viframe, std::shared_ptr<Point> &point) {
	this->viframe = viframe;
	this->point = point;
}

bool PnPErr::Evaluate(double const *const *parameters,
                      double *residuals,
                      double **jacobians) const {

}


SimpleBA::SimpleBA() {

}

SimpleBA::~SimpleBA() {}

bool SimpleBA::run(std::vector <std::shared_ptr<viFrame>> &viframes,
                   std::vector <std::shared_ptr<imuFactor>> &imufactors) {

}