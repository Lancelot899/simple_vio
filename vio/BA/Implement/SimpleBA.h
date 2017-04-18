//
// Created by lancelot on 4/18/17.
//

#ifndef SIMPLE_VIO_SIMPLEBA_H
#define SIMPLE_VIO_SIMPLEBA_H

#include "BABase.h"

class SimpleBA : public BABase {
public:
	SimpleBA();
	~SimpleBA();
	bool run(std::vector<std::shared_ptr<viFrame>> &viframes,
	         std::vector<std::shared_ptr<imuFactor>> &imufactors);

};


#endif //SIMPLE_VIO_SIMPLEBA_H
