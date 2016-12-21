#ifndef IMUFACTOR_H
#define IMUFACTOR_H

#include <memory>

#include "IMUMeasure.h"

class viFrame;

class imuFactor {
public:
    typedef std::shared_ptr<viFrame>          connection_t;

public:
    typedef Eigen::Matrix<double, 15, 3>   FacJBias_t;
    typedef Eigen::Vector3d                speed_t;
    typedef Eigen::Matrix<double, 6, 1>    bias_t;

public:
    imuFactor();
    bool checkConnect(const connection_t &from, const connection_t &to);
    IMUMeasure::Transformation& getPoseFac() {
        return deltaPose;
    }

    speed_t& getSpeedFac() {
        return deltaSpeed;
    }

    FacJBias_t& getJBias() {
        return JBias;
    }

private:
    int                          id;
    IMUMeasure::Transformation   deltaPose;
    speed_t                      deltaSpeed;
    FacJBias_t                   JBias;
    connection_t                 from;
    connection_t                 to;
};

#endif // IMUFACTOR_H
