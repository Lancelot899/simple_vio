//
// Created by lancelot on 4/18/17.
//

#ifndef SIMPLE_VIO_BUNDLEADJUSTEMT_H
#define SIMPLE_VIO_BUNDLEADJUSTEMT_H

#include <memory>
#include <vector>

class BABase;
class viFrame;
class imuFactor;
class Point;

class BundleAdjustemt {
public:
	BundleAdjustemt(int type);
	~BundleAdjustemt() {}
	bool run(std::vector<std::shared_ptr<viFrame>> &viframes,
	         std::vector<std::shared_ptr<Point>> &points,
	         std::vector<std::shared_ptr<imuFactor>> &imufactors);

private:
	std::shared_ptr<BABase> impl;
};


#endif //SIMPLE_VIO_BUNDLEADJUSTEMT_H
