#ifndef IMUFACTOR_H
#define IMUFACTOR_H

#include <memory>

#include "IMUMeasure.h"

class viFrame;

class imuFactor {
public:
    typedef std::shared_ptr<viFrame> connection_t;

public:
    typedef Eigen::Matrix<double, 15, 3>   FacJBias_t;
    typedef Eigen::Vector3d                speed_t;
    typedef Eigen::Matrix<double, 9, 1>    bias_t;

public:
    imuFactor();

private:
    int                          id;
    IMUMeasure::Transformation   deltaPose;
    speed_t                      deltaSpeed;
    FacJBias_t                   JBias;
    connection_t                 from;
    connection_t                 to;
};

#endif // IMUFACTOR_H
