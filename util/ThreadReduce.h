//
// Created by lancelot on 2/28/17.
//

#ifndef SIMPLE_VIO_THREADREDUCE_H
#define SIMPLE_VIO_THREADREDUCE_H

#include "setting.h"
#include "boost/thread.hpp"

class ThreadReduce {
public:
    ThreadReduce();


private:


private:
    boost::thread threads[ThreadNum];
};


#endif //SIMPLE_VIO_THREADREDUCE_H
