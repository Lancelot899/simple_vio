//
// Created by lancelot on 4/18/17.
//

#ifndef SIMPLE_VIO_SIMPLEBA_H
#define SIMPLE_VIO_SIMPLEBA_H


#include <Eigen/Dense>
#include "BABase.h"


class SimpleBA : public BABase {
public:
	SimpleBA();
	~SimpleBA();
	bool run(std::vector<std::shared_ptr<viFrame>> &viframes,
	         typename BundleAdjustemt::obsModeType &obsModes,
	         std::vector<std::shared_ptr<imuFactor>> &imufactors, int iter_);

private:
	struct CopyPoint {
		std::shared_ptr<Point> point;
		Eigen::Vector3d pos_;
	};

};


#endif //SIMPLE_VIO_SIMPLEBA_H
