//
// Created by lancelot on 4/18/17.
//

#ifndef SIMPLE_VIO_BABASE_H
#define SIMPLE_VIO_BABASE_H

#include <memory>
#include <vector>

class viFrame;
class imuFactor;

class BABase {
public:
	BABase() {}
	virtual ~BABase() {}
	virtual  bool run(std::vector<std::shared_ptr<viFrame>> &viframes,
	                  std::vector<std::shared_ptr<imuFactor>> &imufactors) = 0;

};


#endif //SIMPLE_VIO_BABASE_H
