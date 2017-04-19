//
// Created by lancelot on 4/18/17.
//

#ifndef SIMPLE_VIO_BABASE_H
#define SIMPLE_VIO_BABASE_H

#include <memory>
#include <vector>
#include "../BundleAdjustemt.h"

class viFrame;
class imuFactor;
class Point;

class BABase {
public:
	BABase() {}
	virtual ~BABase() {}
	virtual  bool run(std::vector<std::shared_ptr<viFrame>> &viframes,
	                  typename BundleAdjustemt::obsModeType &obsModes,
	                  std::vector<std::shared_ptr<imuFactor>> &imufactors, int iter_) = 0;

};


#endif //SIMPLE_VIO_BABASE_H
