//
// Created by lancelot on 4/18/17.
//

#ifndef SIMPLE_VIO_BUNDLEADJUSTEMT_H
#define SIMPLE_VIO_BUNDLEADJUSTEMT_H

#include <memory>
#include <vector>
#include <map>
#include <list>

class BABase;
class viFrame;
class imuFactor;
class Point;
class Feature;

class BundleAdjustemt {
public:
	typedef std::map<std::shared_ptr<Point>, std::list<std::shared_ptr<Feature>>> obsModeType;

public:
	BundleAdjustemt(int type);
	~BundleAdjustemt() {}
	bool run(std::vector<std::shared_ptr<viFrame>> &viframes,
	         obsModeType &obsModes,
	         std::vector<std::shared_ptr<imuFactor>> &imufactors, int iter_ = 50);

private:
	std::shared_ptr<BABase> impl;
};


#endif //SIMPLE_VIO_BUNDLEADJUSTEMT_H
