#ifndef IMUFACTOR_H
#define IMUFACTOR_H

#include <memory>

#include "IMUMeasure.h"

class viFrame;

class imuFactor {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
public:
    typedef std::shared_ptr<viFrame>          connection_t;

public:
    typedef Eigen::Matrix<double, 15, 3>   FacJBias_t;  ///< dRdb_g, dvdb_a dvdb_g dpdb_a dpdb_g
    typedef Eigen::Vector3d                speed_t;
    typedef Eigen::Matrix<double, 6, 1>    bias_t;      ///< b_g, b_a

public:
    imuFactor();
    bool checkConnect(const connection_t &from, const connection_t &to);
    const IMUMeasure::Transformation& getPoseFac() {
        return deltaPose;
    }

    const speed_t& getSpeedFac() {
        return deltaSpeed;
    }

    const FacJBias_t& getJBias() {
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
